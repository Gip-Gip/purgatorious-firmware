//! Temperature PID controller for all the heater zones

use std::{
    intrinsics::powf32, path::Path, sync::{Arc, Mutex}, thread::sleep, time::{Duration, Instant}
};
use watchdog::ADDR_ESTOP;

use i2c::*;
use pid::Pid;
use rppal::gpio::Gpio;
use screw::{ADDR_LINE_A, ADDR_LINE_V};
use shared::*;
use thermo::*;
use urap::*;

const GPIO_SSR_Z1_HEAT: u8 = 18;
const GPIO_SSR_Z2_HEAT: u8 = 27;
const GPIO_SSR_Z3_HEAT: u8 = 10;
const GPIO_SSR_Z4_HEAT: u8 = 11;
const GPIO_SSR_Z5_HEAT: u8 = 6;
const GPIO_SSR_Z6_HEAT: u8 = 13;

const GPIO_SSR_Z1_FAN: u8 = 17;
const GPIO_SSR_Z2_FAN: u8 = 22;
const GPIO_SSR_Z3_FAN: u8 = 9;
const GPIO_SSR_Z4_FAN: u8 = 7;

const URAP_REG_COUNT: usize = 0x29;

const URAP_WRITE_PROTECT: [bool; URAP_REG_COUNT] = [
    true, false, false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, true, false, false, false, false, false, false,
    false, false,
];

const PWM_PERIOD_MS: u64 = 100 * PWM_PERIOD_MIN_MS;
const PWM_PERIOD_MIN_MS: u64 = 1000 / 30;
const PWM_PERIOD_STAGGER_MS: u64 = 6000 / 6;

const CUR_MAX_A: f32 = 60.0 * 0.8;
const CUR_Z1_A: f32 = 10.0;
const CUR_Z2_A: f32 = 15.0;
const CUR_Z3_A: f32 = 12.5;
const CUR_Z4_A: f32 = 24.0;
const CUR_Z5_A: f32 = 6.0;
const CUR_Z6_A: f32 = 1.6;

const FAN_ENABLE_C: f32 = 10.0;

const LOOP_TIME_MS: u64 = PWM_PERIOD_MIN_MS;

struct LinearEquation {
    a: f32,
    b: f32,
}

impl LinearEquation {
    #[inline]
    fn f(&self, x: f32) -> f32 {
        x * self.a + self.b
    }
}

// Zone 3:
// P = (0.125, 123), (0.15, 169)
//  a = rise/run
//      = (0.15 - 0.125) / (169 - 123)
//      = 5.434782e-4
//  b = y - ax
//      = 0.125 - 1.630e-3 * 123
//      = 5.815217e-2
//
// I =

static LINEAR_P: [LinearEquation; 6] = [
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation {
        a: 5.434782e-4,
        b: 5.815217e-2,
    },
    LinearEquation {
        a: 5.434782e-4,
        b: 5.815217e-2,
    },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
];

static LINEAR_I: [LinearEquation; 6] = [
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
];

static LINEAR_D: [LinearEquation; 6] = [
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
    LinearEquation { a: 0.0, b: 0.0 },
];

fn main() {
    let mut urap_watchdog = UrapMaster::new(URAP_WATCHDOG_PATH).unwrap();

    let gpio = Gpio::new().unwrap();

    let mut ssr_z1_heat = gpio.get(GPIO_SSR_Z1_HEAT).unwrap().into_output_low();
    let mut ssr_z2_heat = gpio.get(GPIO_SSR_Z2_HEAT).unwrap().into_output_low();
    let mut ssr_z3_heat = gpio.get(GPIO_SSR_Z3_HEAT).unwrap().into_output_low();
    let mut ssr_z4_heat = gpio.get(GPIO_SSR_Z4_HEAT).unwrap().into_output_low();
    let mut ssr_z5_heat = gpio.get(GPIO_SSR_Z5_HEAT).unwrap().into_output_low();
    let mut ssr_z6_heat = gpio.get(GPIO_SSR_Z6_HEAT).unwrap().into_output_low();

    let mut ssr_z1_fan = gpio.get(GPIO_SSR_Z1_FAN).unwrap().into_output_low();
    let mut ssr_z2_fan = gpio.get(GPIO_SSR_Z2_FAN).unwrap().into_output_low();
    let mut ssr_z3_fan = gpio.get(GPIO_SSR_Z3_FAN).unwrap().into_output_low();
    let mut ssr_z4_fan = gpio.get(GPIO_SSR_Z4_FAN).unwrap().into_output_low();

    let registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; URAP_REG_COUNT]>> =
        Arc::new(Mutex::new([[0; URAP_REG_WIDTH]; URAP_REG_COUNT]));

    let mut registers_lk = registers.lock().unwrap();
    registers_lk[ADDR_ENBL_LINEAR_PID] = [1; URAP_REG_WIDTH];
    drop(registers_lk);

    UrapSlave::spawn(URAP_THERMO_PATH, registers.clone(), URAP_WRITE_PROTECT).unwrap();
    // Wait on the vPLCs to initialize
    let urap_i2c_path = Path::new(URAP_I2C_PATH);
    let urap_screw_path = Path::new(URAP_SCREW_PATH);

    while !urap_i2c_path.exists() && !urap_screw_path.exists() {
        sleep(Duration::from_secs(1));
    }

    let mut urap_i2c = UrapMaster::new(URAP_I2C_PATH).unwrap();
    let mut urap_screw = UrapMaster::new(URAP_SCREW_PATH).unwrap();

    let mut pid_z1 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z2 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z3 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z4 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z5 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z6 = Pid::<f32>::new(0.0, 1.0);

    let mut pid_z1_irange_c: f32 = 5.0;
    let mut pid_z2_irange_c: f32 = 5.0;
    let mut pid_z3_irange_c: f32 = 5.0;
    let mut pid_z4_irange_c: f32 = 5.0;
    let mut pid_z5_irange_c: f32 = 5.0;
    let mut pid_z6_irange_c: f32 = 5.0;

    let mut instant_z1_on = Instant::now();
    let mut instant_z1_off = instant_z1_on.clone();

    let mut instant_z2_on = Instant::now()
        .checked_add(Duration::from_millis(PWM_PERIOD_STAGGER_MS))
        .unwrap();
    let mut instant_z2_off = instant_z2_on.clone();

    let mut instant_z3_on = Instant::now()
        .checked_add(Duration::from_millis(2 * PWM_PERIOD_STAGGER_MS))
        .unwrap();
    let mut instant_z3_off = instant_z3_on.clone();

    let mut instant_z4_on = Instant::now()
        .checked_add(Duration::from_millis(3 * PWM_PERIOD_STAGGER_MS))
        .unwrap();
    let mut instant_z4_off = instant_z4_on.clone();

    let mut instant_z5_on = Instant::now()
        .checked_add(Duration::from_millis(4 * PWM_PERIOD_STAGGER_MS))
        .unwrap();
    let mut instant_z5_off = instant_z5_on.clone();

    let mut instant_z6_on = Instant::now()
        .checked_add(Duration::from_millis(5 * PWM_PERIOD_STAGGER_MS))
        .unwrap();
    let mut instant_z6_off = instant_z6_on.clone();

    let mut pwr_z1_mean: f32 = 0.0;
    let mut pwr_z2_mean: f32 = 0.0;
    let mut pwr_z3_mean: f32 = 0.0;
    let mut pwr_z4_mean: f32 = 0.0;
    let mut pwr_z5_mean: f32 = 0.0;
    let mut pwr_z6_mean: f32 = 0.0;

    let mut pwr_z1_samples: f32 = 0.0;
    let mut pwr_z2_samples: f32 = 0.0;
    let mut pwr_z3_samples: f32 = 0.0;
    let mut pwr_z4_samples: f32 = 0.0;
    let mut pwr_z5_samples: f32 = 0.0;
    let mut pwr_z6_samples: f32 = 0.0;

    loop {
        let now = Instant::now();
        let wakeup = now
            .checked_add(Duration::from_millis(LOOP_TIME_MS))
            .unwrap();

        let mut registers_lk = registers.lock().unwrap();

        // Increment the inchash
        let inchash = u32::from_ne_bytes(registers_lk[ADDR_INCHASH]) + 1;
        registers_lk[ADDR_INCHASH] = inchash.to_ne_bytes();

        // Normal Operation
        if urap_watchdog.read_u32(ADDR_ESTOP).unwrap_or(1) == 0 {
            // Values retrieved from sources that may fail
            let (
                mut t1_c,
                mut t2_c,
                mut t3_c,
                mut t4_c,
                mut t5_c,
                mut t6_c,
                mut t_amb_c,
                mut motor_line_a,
                mut line_v,
            ) = (
                0_f32, 0_f32, 0_f32, 0_f32, 0_f32, 0_f32, 0_f32, 0_f32, 0_f32,
            );

            let temps: [(&mut f32, u16); 7] = [
                (&mut t1_c, ADDR_T1_C as u16),
                (&mut t2_c, ADDR_T2_C as u16),
                (&mut t3_c, ADDR_T3_C as u16),
                (&mut t4_c, ADDR_T4_C as u16),
                (&mut t5_c, ADDR_T5_C as u16),
                (&mut t6_c, ADDR_T6_C as u16),
                (&mut t_amb_c, ADDR_AMBIENT_C as u16),
            ];

            let motor_stats: [(&mut f32, u16); 2] = [
                (&mut motor_line_a, ADDR_LINE_A as u16),
                (&mut line_v, ADDR_LINE_V as u16),
            ];

            for (temp, addr) in temps {
                *temp = match urap_i2c.read_f32(addr) {
                    Ok(val) => val,
                    Err(_) => {
                        urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
                        break;
                    }
                };
            }

            for (motor_stat, addr) in motor_stats {
                *motor_stat = match urap_screw.read_f32(addr) {
                    Ok(val) => val,
                    Err(_) => {
                        urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
                        break;
                    }
                };
            }

            let (t1_c, t2_c, t3_c, t4_c, t5_c, t6_c, t_amb_c, motor_line_a, line_v) = (
                t1_c,
                t2_c,
                t3_c,
                t4_c,
                t5_c,
                t6_c,
                t_amb_c,
                motor_line_a,
                line_v,
            );

            pid_z1.setpoint(f32::from_ne_bytes(registers_lk[ADDR_SET_Z1_C]));
            pid_z2.setpoint(f32::from_ne_bytes(registers_lk[ADDR_SET_Z2_C]));
            pid_z3.setpoint(f32::from_ne_bytes(registers_lk[ADDR_SET_Z3_C]));
            pid_z4.setpoint(f32::from_ne_bytes(registers_lk[ADDR_SET_Z4_C]));
            pid_z5.setpoint(f32::from_ne_bytes(registers_lk[ADDR_SET_Z5_C]));
            pid_z6.setpoint(f32::from_ne_bytes(registers_lk[ADDR_SET_Z6_C]));

            if (t1_c - pid_z1.setpoint).abs() > pid_z1_irange_c {
                pid_z1.reset_integral_term();
            }
            if (t2_c - pid_z2.setpoint).abs() > pid_z2_irange_c {
                pid_z2.reset_integral_term();
            }
            if (t3_c - pid_z3.setpoint).abs() > pid_z3_irange_c {
                pid_z3.reset_integral_term();
            }
            if (t4_c - pid_z4.setpoint).abs() > pid_z4_irange_c {
                pid_z4.reset_integral_term();
            }
            if (t5_c - pid_z5.setpoint).abs() > pid_z5_irange_c {
                pid_z5.reset_integral_term();
            }
            if (t6_c - pid_z6.setpoint).abs() > pid_z6_irange_c {
                pid_z6.reset_integral_term();
            }

            if registers_lk[ADDR_ENBL_LINEAR_PID] != [0; URAP_REG_WIDTH] {
                for (i, (pid, addr_p, addr_i, addr_d)) in [
                    (&mut pid_z1, ADDR_P_Z1, ADDR_I_Z1, ADDR_D_Z1),
                    (&mut pid_z2, ADDR_P_Z2, ADDR_I_Z2, ADDR_D_Z2),
                    (&mut pid_z3, ADDR_P_Z3, ADDR_I_Z3, ADDR_D_Z3),
                    (&mut pid_z4, ADDR_P_Z4, ADDR_I_Z4, ADDR_D_Z4),
                    (&mut pid_z5, ADDR_P_Z5, ADDR_I_Z5, ADDR_D_Z5),
                    (&mut pid_z6, ADDR_P_Z6, ADDR_I_Z6, ADDR_D_Z6),
                ]
                .iter_mut()
                .enumerate()
                {
                    pid.p(LINEAR_P[i].f(pid.setpoint - t_amb_c).max(0.0), 1.0);
                    pid.i(LINEAR_I[i].f(pid.setpoint - t_amb_c).max(0.0), 1.0);
                    pid.d(LINEAR_D[i].f(pid.setpoint - t_amb_c).max(0.0), 1.0);

                    registers_lk[*addr_p] = pid.kp.to_ne_bytes();
                    registers_lk[*addr_i] = pid.ki.to_ne_bytes();
                    registers_lk[*addr_d] = pid.kd.to_ne_bytes();
                }
            }

            if registers_lk[ADDR_RELOAD_PID] != [0; URAP_REG_WIDTH] {
                registers_lk[ADDR_RELOAD_PID] = [0; URAP_REG_WIDTH];

                for (pid, addr_p, addr_i, addr_d, addr_i_range, i_range) in [
                    (
                        &mut pid_z1,
                        ADDR_P_Z1,
                        ADDR_I_Z1,
                        ADDR_D_Z1,
                        ADDR_I_RANGE_Z1_C,
                        &mut pid_z1_irange_c,
                    ),
                    (
                        &mut pid_z2,
                        ADDR_P_Z2,
                        ADDR_I_Z2,
                        ADDR_D_Z2,
                        ADDR_I_RANGE_Z2_C,
                        &mut pid_z2_irange_c,
                    ),
                    (
                        &mut pid_z3,
                        ADDR_P_Z3,
                        ADDR_I_Z3,
                        ADDR_D_Z3,
                        ADDR_I_RANGE_Z3_C,
                        &mut pid_z3_irange_c,
                    ),
                    (
                        &mut pid_z4,
                        ADDR_P_Z4,
                        ADDR_I_Z4,
                        ADDR_D_Z4,
                        ADDR_I_RANGE_Z4_C,
                        &mut pid_z4_irange_c,
                    ),
                    (
                        &mut pid_z5,
                        ADDR_P_Z5,
                        ADDR_I_Z5,
                        ADDR_D_Z5,
                        ADDR_I_RANGE_Z5_C,
                        &mut pid_z5_irange_c,
                    ),
                    (
                        &mut pid_z6,
                        ADDR_P_Z6,
                        ADDR_I_Z6,
                        ADDR_D_Z6,
                        ADDR_I_RANGE_Z6_C,
                        &mut pid_z6_irange_c,
                    ),
                ] {
                    pid.p(f32::from_ne_bytes(registers_lk[addr_p]).max(0.0), 1.0);
                    pid.i(f32::from_ne_bytes(registers_lk[addr_i]).max(0.0), 1.0);
                    pid.d(f32::from_ne_bytes(registers_lk[addr_d]).max(0.0), 1.0);

                    *i_range = f32::from_ne_bytes(registers_lk[addr_i_range]);

                    registers_lk[addr_p] = pid.kp.to_ne_bytes();
                    registers_lk[addr_i] = pid.ki.to_ne_bytes();
                    registers_lk[addr_d] = pid.kd.to_ne_bytes();
                }
            }

            // Fan Logic
            match t1_c > pid_z1.setpoint - FAN_ENABLE_C {
                true => ssr_z1_fan.set_high(),
                false => ssr_z1_fan.set_low(),
            }

            match t2_c > pid_z2.setpoint - FAN_ENABLE_C {
                true => ssr_z2_fan.set_high(),
                false => ssr_z2_fan.set_low(),
            }

            match t3_c > pid_z3.setpoint - FAN_ENABLE_C {
                true => ssr_z3_fan.set_high(),
                false => ssr_z3_fan.set_low(),
            }

            match t4_c > pid_z4.setpoint - FAN_ENABLE_C {
                true => ssr_z4_fan.set_high(),
                false => ssr_z4_fan.set_low(),
            }

            let (pwr_z1, pwr_z2, pwr_z3, pwr_z4, pwr_z5, pwr_z6) =
                if registers_lk[ADDR_ENBL_PWR_OVERRIDE] != [0; URAP_REG_WIDTH] {
                    (
                        f32::from_ne_bytes(registers_lk[ADDR_Z1_PWR]).max(0.0),
                        f32::from_ne_bytes(registers_lk[ADDR_Z2_PWR]).max(0.0),
                        f32::from_ne_bytes(registers_lk[ADDR_Z3_PWR]).max(0.0),
                        f32::from_ne_bytes(registers_lk[ADDR_Z4_PWR]).max(0.0),
                        f32::from_ne_bytes(registers_lk[ADDR_Z5_PWR]).max(0.0),
                        f32::from_ne_bytes(registers_lk[ADDR_Z6_PWR]).max(0.0),
                    )
                } else {
                    (
                        pid_z1.next_control_output(t1_c).output.max(0.0),
                        pid_z2.next_control_output(t2_c).output.max(0.0),
                        pid_z3.next_control_output(t3_c).output.max(0.0),
                        pid_z4.next_control_output(t4_c).output.max(0.0),
                        pid_z5.next_control_output(t5_c).output.max(0.0),
                        pid_z6.next_control_output(t6_c).output.max(0.0),
                    )
                };

            let cur_ideal_a = {
                pwr_z1 * CUR_Z1_A
                    + pwr_z2 * CUR_Z2_A
                    + pwr_z3 * CUR_Z3_A
                    + pwr_z4 * CUR_Z4_A
                    + pwr_z5 * CUR_Z5_A
                    + pwr_z6 * CUR_Z6_A
            };

            pwr_z1_samples += 1.0;
            pwr_z2_samples += 1.0;
            pwr_z3_samples += 1.0;
            pwr_z4_samples += 1.0;
            pwr_z5_samples += 1.0;
            pwr_z6_samples += 1.0;

            let pwr_z1_weight = 1.0 / pwr_z1_samples;
            let pwr_z2_weight = 1.0 / pwr_z2_samples;
            let pwr_z3_weight = 1.0 / pwr_z3_samples;
            let pwr_z4_weight = 1.0 / pwr_z4_samples;
            let pwr_z5_weight = 1.0 / pwr_z5_samples;
            let pwr_z6_weight = 1.0 / pwr_z6_samples;

            pwr_z1_mean = pwr_z1 * pwr_z1_weight + pwr_z1_mean * 1.0 - pwr_z1_weight;
            pwr_z2_mean = pwr_z1 * pwr_z2_weight + pwr_z2_mean * 1.0 - pwr_z2_weight;
            pwr_z3_mean = pwr_z1 * pwr_z3_weight + pwr_z3_mean * 1.0 - pwr_z3_weight;
            pwr_z4_mean = pwr_z1 * pwr_z4_weight + pwr_z4_mean * 1.0 - pwr_z4_weight;
            pwr_z5_mean = pwr_z1 * pwr_z5_weight + pwr_z5_mean * 1.0 - pwr_z5_weight;
            pwr_z6_mean = pwr_z1 * pwr_z6_weight + pwr_z6_mean * 1.0 - pwr_z6_weight;

            // When the motor is stopped the current reads negative, ignore this.
            let cur_total_a = cur_ideal_a + motor_line_a.min(0.0);

            let cur_multiplier = 1.0_f32.min(CUR_MAX_A / cur_total_a);

            registers_lk[ADDR_Z1_PWR] = pwr_z1.to_ne_bytes();
            registers_lk[ADDR_Z2_PWR] = pwr_z2.to_ne_bytes();
            registers_lk[ADDR_Z3_PWR] = pwr_z3.to_ne_bytes();
            registers_lk[ADDR_Z4_PWR] = pwr_z4.to_ne_bytes();
            registers_lk[ADDR_Z5_PWR] = pwr_z5.to_ne_bytes();
            registers_lk[ADDR_Z6_PWR] = pwr_z6.to_ne_bytes();

            registers_lk[ADDR_THERMO_PWR_W] = (cur_ideal_a * cur_multiplier * line_v).to_ne_bytes();

            drop(registers_lk);

            let pwr_z1 = pwr_z1 * cur_multiplier;
            let pwr_z2 = pwr_z2 * cur_multiplier;
            let pwr_z3 = pwr_z3 * cur_multiplier;
            let pwr_z4 = pwr_z4 * cur_multiplier;
            let pwr_z5 = pwr_z5 * cur_multiplier;
            let pwr_z6 = pwr_z6 * cur_multiplier;

            for (instant_on, instant_off, ssr_heat, pwr_mean, pwr_samples) in [
                (&mut instant_z1_on, &mut instant_z1_off, &mut ssr_z1_heat, &mut pwr_z1_mean, &mut pwr_z1_samples),
                (&mut instant_z2_on, &mut instant_z2_off, &mut ssr_z2_heat, &mut pwr_z2_mean, &mut pwr_z2_samples),
                (&mut instant_z3_on, &mut instant_z3_off, &mut ssr_z3_heat, &mut pwr_z3_mean, &mut pwr_z3_samples),
                (&mut instant_z4_on, &mut instant_z4_off, &mut ssr_z4_heat, &mut pwr_z4_mean, &mut pwr_z4_samples),
                (&mut instant_z5_on, &mut instant_z5_off, &mut ssr_z5_heat, &mut pwr_z5_mean, &mut pwr_z5_samples),
                (&mut instant_z6_on, &mut instant_z6_off, &mut ssr_z6_heat, &mut pwr_z6_mean, &mut pwr_z6_samples),
            ] {
                if instant_on.saturating_duration_since(now) == Duration::ZERO {
                    let period_ms = *pwr_mean * PWM_PERIOD_MS as f32;

                    *pwr_mean = 0.0;
                    *pwr_samples = 0.0;

                    *instant_on = now
                        .checked_add(Duration::from_millis(PWM_PERIOD_MS))
                        .unwrap();
                    *instant_off = now
                        .checked_add(Duration::from_millis(period_ms as u64))
                        .unwrap();

                    if period_ms > PWM_PERIOD_MIN_MS as f32 {
                        ssr_heat.set_high();
                    } else {
                        ssr_heat.set_low();
                    }
                } else if instant_off.saturating_duration_since(now) == Duration::ZERO {
                    ssr_heat.set_low();
                }
            }
        }
        // E-Stop Code
        else {
            registers_lk[ADDR_SET_Z1_C] = 0_f32.to_ne_bytes();
            registers_lk[ADDR_SET_Z2_C] = 0_f32.to_ne_bytes();
            registers_lk[ADDR_SET_Z3_C] = 0_f32.to_ne_bytes();
            registers_lk[ADDR_SET_Z4_C] = 0_f32.to_ne_bytes();
            registers_lk[ADDR_SET_Z5_C] = 0_f32.to_ne_bytes();
            registers_lk[ADDR_SET_Z6_C] = 0_f32.to_ne_bytes();

            drop(registers_lk);

            ssr_z1_fan.set_high();
            ssr_z2_fan.set_high();
            ssr_z3_fan.set_high();
            ssr_z4_fan.set_high();

            ssr_z1_heat.set_low();
            ssr_z2_heat.set_low();
            ssr_z3_heat.set_low();
            ssr_z4_heat.set_low();
            ssr_z5_heat.set_low();
            ssr_z6_heat.set_low();

            // Reopen any URAP connections that may have been lost
            if !urap_i2c.is_healthy() {
                urap_i2c = UrapMaster::new(URAP_I2C_PATH).unwrap_or(urap_i2c);
            }
            if !urap_screw.is_healthy() {
                urap_screw = UrapMaster::new(URAP_SCREW_PATH).unwrap_or(urap_screw);
            }
        }

        sleep_till(wakeup);
    }
}
