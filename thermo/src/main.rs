#![doc = include_str!("../README.md")]

use lazy_static::lazy_static;
use std::{
    path::Path,
    sync::{Arc, Mutex},
    thread::sleep,
    time::{Duration, Instant},
};
use watchdog::ADDR_ESTOP;

use i2c::*;
use pid::Pid;
use rppal::gpio::Gpio;
use screw::{ADDR_LINE_A, ADDR_LINE_V};
use shared::*;
use thermo::*;
use urap::{usockets::*, URAP_DATA_WIDTH};

/// GPIO pin that controls the heater ssr of Zone 1
const GPIO_SSR_Z1_HEAT: u8 = 18;
/// GPIO pin that controls the heater ssr of Zone 2
const GPIO_SSR_Z2_HEAT: u8 = 27;
/// GPIO pin that controls the heater ssr of Zone 3
const GPIO_SSR_Z3_HEAT: u8 = 10;
/// GPIO pin that controls the heater ssr of Zone 4
const GPIO_SSR_Z4_HEAT: u8 = 11;
/// GPIO pin that controls the heater ssr of Zone 5
const GPIO_SSR_Z5_HEAT: u8 = 6;
/// GPIO pin that controls the heater ssr of Zone 6
const GPIO_SSR_Z6_HEAT: u8 = 13;

/// GPIO pin that controls the fan ssr of Zone 6
const GPIO_SSR_Z1_FAN: u8 = 17;
/// GPIO pin that controls the fan ssr of Zone 7
const GPIO_SSR_Z2_FAN: u8 = 22;
/// GPIO pin that controls the fan ssr of Zone 8
const GPIO_SSR_Z3_FAN: u8 = 9;
/// GPIO pin that controls the fan ssr of Zone 9
const GPIO_SSR_Z4_FAN: u8 = 7;

/// Number of registers the URAP secondary has
const URAP_REG_COUNT: usize = 0x29;

/// Write protect flags of all registers
const URAP_WRITE_PROTECT: [bool; URAP_REG_COUNT] = [
    true, false, false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, true, false, false, false, false, false, false,
    false, false,
];

/// PWM Period of all heaters
const PWM_PERIOD_MS: u64 = 100 * PWM_PERIOD_MIN_MS;
/// Shortest PWM period allowed with our SSRs, equal to two cycles
const PWM_PERIOD_MIN_MS: u64 = 1000 / 30;
/// Time stagger of all heaters. Heaters are heated staggered so
/// they don't all draw power at the same time
const PWM_PERIOD_STAGGER_MS: u64 = PWM_PERIOD_MS / 6;

/// Maximum current draw in amps
const CUR_MAX_A: f32 = 60.0 * 0.8;
/// Current drawn at full power by Zone 1
const CUR_Z1_A: f32 = 10.0;
/// Current drawn at full power by Zone 2
const CUR_Z2_A: f32 = 15.0;
/// Current drawn at full power by Zone 3
const CUR_Z3_A: f32 = 12.5;
/// Current drawn at full power by Zone 4
const CUR_Z4_A: f32 = 24.0;
/// Current drawn at full power by Zone 5
const CUR_Z5_A: f32 = 6.0;
/// Current drawn at full power by Zone 6
const CUR_Z6_A: f32 = 1.6;

/// Temperature below the setpoint to enable the fan, in celcius
const FAN_ENABLE_C: f32 = 10.0;
/// Temperature that the fans are cut off at
const FAN_CUTOFF_C: f32 = 60.0;
/// Thermocouple noise floor in celcius
const NOISE_FLOOR_C: f32 = 1.0;

/// Time per loop in ms
const LOOP_TIME_MS: u64 = 10;
/// Rate at which the PID is updated in ms
const POLL_PID_MS: u64 = 100;

/// Constant structure used for calculating new PID values at
/// different temperatures
struct PidRule {
    dp: f32,
    di: f32,
    dd: f32,
}

impl PidRule {
    /// Calculate Kpid values with the given parameters, following the rule
    fn get_pid(&self, ku: f32, pu_s: f32) -> (f32, f32, f32) {
        let kp = ku / self.dp;
        let ki = kp / (pu_s / self.di);
        let kd = kp * (pu_s / self.dd);

        (kp, ki, kd)
    }
}

/// Linear equation, used to help calculate PID values
struct LinearEquation {
    a: f32,
    b: f32,
}

impl LinearEquation {
    /// Create a linear equation from two points
    fn from_points(p1: (f32, f32), p2: (f32, f32)) -> Self {
        let (x1, y1) = p1;
        let (x2, y2) = p2;
        let a = (y2 - y1) / (x2 - x1);

        let b = y1 - a * x1;

        Self { a, b }
    }
    #[inline]
    /// Calculate and return `ax + b`
    fn f(&self, x: f32) -> f32 {
        x * self.a + self.b
    }
}

/// Chains multiple linear equations to approximate more complex functions
struct LinearProgression {
    ranges: Vec<(f32, LinearEquation)>,
}

impl LinearProgression {
    /// Create a linear progression from multiple points
    fn new(points: &[(f32, f32)]) -> Self {
        let mut ranges = Vec::with_capacity(points.len() - 1);

        for i in 1..points.len() {
            let p1 = points[i - 1];
            let p2 = points[i];

            let (limit, _) = match i == points.len() - 1 {
                true => (f32::INFINITY, 0.0),
                false => p2,
            };

            let equation = LinearEquation::from_points(p1, p2);

            ranges.push((limit, equation));
        }

        Self { ranges }
    }

    /// Calculate and return `ax + b` for a specific value
    fn f(&self, x: f32) -> f32 {
        for (limit, equation) in &self.ranges {
            if x < *limit {
                return equation.f(x);
            }
        }

        return f32::NAN;
    }
}

static KU_PROG_POINTS_Z1: [(f32, f32); 5] = [
    (100.0-29.0, 2.2036837e-1),
    (125.0-32.0, 2.2975750e-1),
    (150.0-35.0, 2.5202265e-1),
    (175.0-37.0, 2.5359127e-1),
    (200.0-40.0, 2.6514316e-1),
];

static PU_S_PROG_POINTS_Z1: [(f32, f32); 5] = [
    (100.0-29.0, 387.8200),
    (125.0-32.0, 709.9025),
    (150.0-35.0, 712.1970),
    (175.0-37.0, 557.6140),
    (200.0-40.0, 457.0730),
];

static RULE_Z1: PidRule = PidRule {
    dp: 4.291611,
    di: 1.428715e4,
    dd: 4.918557e-1,
};

/// Used to calculate the Kpid based on temperature delta for Zone 1
fn pid_z1(t_delta_c: f32) -> (f32, f32, f32) {
    lazy_static! {
        static ref KU_PROGRESSION_Z1: LinearProgression = LinearProgression::new(&KU_PROG_POINTS_Z1);
        static ref PU_S_PROGRESSION_Z1: LinearProgression = LinearProgression::new(&PU_S_PROG_POINTS_Z1);
    };

    let ku = KU_PROGRESSION_Z1.f(t_delta_c);
    let pu_s = PU_S_PROGRESSION_Z1.f(t_delta_c);

    RULE_Z1.get_pid(ku, pu_s)
}

static KU_PROG_POINTS_Z2: [(f32, f32); 5] = [
    (100.0-35.0, 3.7114674e-1),
    (125.0-38.0, 3.8927070e-1),
    (150.0-38.0, 2.8034633e-1),
    (175.0-36.0, 3.3035403e-1),
    (200.0-40.0, 2.7591646e-1),
];

static PU_S_PROG_POINTS_Z2: [(f32, f32); 5] = [
    (100.0-35.0, 374.7200),
    (125.0-38.0, 349.8690),
    (150.0-38.0, 315.6045),
    (175.0-36.0, 303.7525),
    (200.0-40.0, 306.3730),
];

static RULE_Z2: PidRule = PidRule {
    dp: 3.179031,
    di: 9.339003e3,
    dd: 5.883713e-1,
};

/// Used to calculate the Kpid based on temperature delta for Zone 2
fn pid_z2(t_delta_c: f32) -> (f32, f32, f32) {
    lazy_static! {
        static ref KU_PROGRESSION_Z2: LinearProgression = LinearProgression::new(&KU_PROG_POINTS_Z2);
        static ref PU_S_PROGRESSION_Z2: LinearProgression = LinearProgression::new(&PU_S_PROG_POINTS_Z2);
    };

    let ku = KU_PROGRESSION_Z2.f(t_delta_c);
    let pu_s = PU_S_PROGRESSION_Z2.f(t_delta_c);

    RULE_Z2.get_pid(ku, pu_s)
}

/// Used to calculate the Kpid based on temperature delta for Zone 3
fn pid_z3(t_delta_c: f32) -> (f32, f32, f32) {
    (0.0, 0.0, 0.0)
}

/// Used to calculate the Kpid based on temperature delta for Zone 4
fn pid_z4(t_delta_c: f32) -> (f32, f32, f32) {
    (0.0, 0.0, 0.0)
}

/// Used to calculate the Kpid based on temperature delta for Zone 5
fn pid_z5(t_delta_c: f32) -> (f32, f32, f32) {
    (0.0, 0.0, 0.0)
}


static KU_PROG_POINTS_Z6: [(f32, f32); 5] = [
    (100.0-32.0, 2.406e-1),
    (125.0-32.0, 2.122e-1),
    (150.0-32.0, 1.719e-1),
    (175.0-32.0, 1.822e-1),
    (200.0-32.0, 1.630e-1),
];

static PU_S_PROG_POINTS_Z6: [(f32, f32); 5] = [
    (100.0-32.0, 351.5),
    (125.0-32.0, 278.5),
    (150.0-32.0, 278.2),
    (175.0-32.0, 292.7),
    (200.0-32.0, 581.3),
];

static RULE_Z6: PidRule = PidRule {
    dp: 1.513,
    di: 2.249e-1,
    dd: 3.650,
};

/// Used to calculate the Kpid based on temperature delta for Zone 6
fn pid_z6(t_delta_c: f32) -> (f32, f32, f32) {
    lazy_static! {
        static ref KU_PROGRESSION_Z6: LinearProgression = LinearProgression::new(&KU_PROG_POINTS_Z6);
        static ref PU_S_PROGRESSION_Z6: LinearProgression = LinearProgression::new(&PU_S_PROG_POINTS_Z6);
    };

    let ku = KU_PROGRESSION_Z6.f(t_delta_c);
    let pu_s = PU_S_PROGRESSION_Z6.f(t_delta_c);

    RULE_Z6.get_pid(ku, pu_s)
}

/// Map of all the pid calculation functions
const PID_FUNCS: [fn(f32) -> (f32, f32, f32); 6] = [pid_z1, pid_z2, pid_z3, pid_z4, pid_z5, pid_z6];

fn main() {
    // # 1: Program Initialization

    // ## 1.1: Connect to watchdog
    let mut urap_watchdog = UrapPrimary::new(URAP_WATCHDOG_PATH).unwrap();

    // ## 1.2: Aquire GPIO
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

    // ## 1.3: Initalize URAP secondary
    let registers: Arc<Mutex<[[u8; URAP_DATA_WIDTH]; URAP_REG_COUNT]>> =
        Arc::new(Mutex::new([[0; URAP_DATA_WIDTH]; URAP_REG_COUNT]));

    let mut registers_lk = registers.lock().unwrap();
    registers_lk[ADDR_ENBL_FUNCIONAL_PID as usize] = [1; URAP_DATA_WIDTH];
    drop(registers_lk);

    UrapSecondary::spawn(URAP_THERMO_PATH, registers.clone(), URAP_WRITE_PROTECT).unwrap();

    // ## 1.4: Connect to other vPLCs
    let urap_i2c_path = Path::new(URAP_I2C_PATH);
    let urap_screw_path = Path::new(URAP_SCREW_PATH);

    while !urap_i2c_path.exists() && !urap_screw_path.exists() {
        sleep(Duration::from_secs(1));
    }

    let mut urap_i2c = UrapPrimary::new(URAP_I2C_PATH).unwrap();
    let mut urap_screw = UrapPrimary::new(URAP_SCREW_PATH).unwrap();

    // ## 1.5: Initialize mutable variables
    let mut pid_z1 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z2 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z3 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z4 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z5 = Pid::<f32>::new(0.0, 1.0);
    let mut pid_z6 = Pid::<f32>::new(0.0, 1.0);

    let mut pid_z1_irange_c: f32 = 10.0;
    let mut pid_z2_irange_c: f32 = 10.0;
    let mut pid_z3_irange_c: f32 = 10.0;
    let mut pid_z4_irange_c: f32 = 10.0;
    let mut pid_z5_irange_c: f32 = 10.0;
    let mut pid_z6_irange_c: f32 = 10.0;

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

    let mut poll_pid = Instant::now();

    let mut pwr_z1_mean: f32 = 0.0;
    let mut pwr_z2_mean: f32 = 0.0;
    let mut pwr_z3_mean: f32 = 0.0;
    let mut pwr_z4_mean: f32 = 0.0;
    let mut pwr_z5_mean: f32 = 0.0;
    let mut pwr_z6_mean: f32 = 0.0;

    let mut pwr_z1: f32 = 0.0;
    let mut pwr_z2: f32 = 0.0;
    let mut pwr_z3: f32 = 0.0;
    let mut pwr_z4: f32 = 0.0;
    let mut pwr_z5: f32 = 0.0;
    let mut pwr_z6: f32 = 0.0;

    let mut pwr_z1_samples: f32 = 0.0;
    let mut pwr_z2_samples: f32 = 0.0;
    let mut pwr_z3_samples: f32 = 0.0;
    let mut pwr_z4_samples: f32 = 0.0;
    let mut pwr_z5_samples: f32 = 0.0;
    let mut pwr_z6_samples: f32 = 0.0;

    // # 2: Execution loop
    loop {
        // ## 2.1: Calculate instant to resume loop
        let now = Instant::now();
        let wakeup = now
            .checked_add(Duration::from_millis(LOOP_TIME_MS))
            .unwrap();

        // ## 2.2: Aquire lock on registers
        let mut registers_lk = registers.lock().unwrap();

        // ## 2.3: Increment the inchash
        let inchash = u32::from_le_bytes(registers_lk[ADDR_INCHASH as usize]) + 1;
        registers_lk[ADDR_INCHASH as usize] = inchash.to_le_bytes();

        // ## 2.4a: Peform normal operations if estop is not active
        if urap_watchdog.read_u32(ADDR_ESTOP).unwrap_or(1) == 0 {
            // ### 2.4a.1: Retrieve values from other vPLCs
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
                (&mut t1_c, ADDR_T1_C),
                (&mut t2_c, ADDR_T2_C),
                (&mut t3_c, ADDR_T3_C),
                (&mut t4_c, ADDR_T4_C),
                (&mut t5_c, ADDR_T5_C),
                (&mut t6_c, ADDR_T6_C),
                (&mut t_amb_c, ADDR_AMBIENT_C),
            ];

            let motor_stats: [(&mut f32, u16); 2] =
                [(&mut motor_line_a, ADDR_LINE_A), (&mut line_v, ADDR_LINE_V)];

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

            // ### 2.4a.2: Update target temperature from register values
            pid_z1.setpoint(f32::from_le_bytes(registers_lk[ADDR_SET_Z1_C as usize]));
            pid_z2.setpoint(f32::from_le_bytes(registers_lk[ADDR_SET_Z2_C as usize]));
            pid_z3.setpoint(f32::from_le_bytes(registers_lk[ADDR_SET_Z3_C as usize]));
            pid_z4.setpoint(f32::from_le_bytes(registers_lk[ADDR_SET_Z4_C as usize]));
            pid_z5.setpoint(f32::from_le_bytes(registers_lk[ADDR_SET_Z5_C as usize]));
            pid_z6.setpoint(f32::from_le_bytes(registers_lk[ADDR_SET_Z6_C as usize]));

            // ### 2.4a.3: Reset/clear integral term if outside of the Ki range
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

            // ### 2.4a.4: Update Kpid values if functional pid is enabled
            if registers_lk[ADDR_ENBL_FUNCIONAL_PID as usize] != [0; URAP_DATA_WIDTH] {
                for (i, (pid, addr_p, addr_i, addr_d)) in [
                    (
                        &mut pid_z1,
                        ADDR_P_Z1 as usize,
                        ADDR_I_Z1 as usize,
                        ADDR_D_Z1 as usize,
                    ),
                    (
                        &mut pid_z2,
                        ADDR_P_Z2 as usize,
                        ADDR_I_Z2 as usize,
                        ADDR_D_Z2 as usize,
                    ),
                    (
                        &mut pid_z3,
                        ADDR_P_Z3 as usize,
                        ADDR_I_Z3 as usize,
                        ADDR_D_Z3 as usize,
                    ),
                    (
                        &mut pid_z4,
                        ADDR_P_Z4 as usize,
                        ADDR_I_Z4 as usize,
                        ADDR_D_Z4 as usize,
                    ),
                    (
                        &mut pid_z5,
                        ADDR_P_Z5 as usize,
                        ADDR_I_Z5 as usize,
                        ADDR_D_Z5 as usize,
                    ),
                    (
                        &mut pid_z6,
                        ADDR_P_Z6 as usize,
                        ADDR_I_Z6 as usize,
                        ADDR_D_Z6 as usize,
                    ),
                ]
                .iter_mut()
                .enumerate()
                {
                    let (kp, ki, kd) = PID_FUNCS[i](pid.setpoint - t_amb_c);

                    pid.p(kp.max(0.0), 1.0);
                    pid.i(ki.max(0.0), 1.0);
                    pid.d(kd.max(0.0), 1.0);

                    registers_lk[*addr_p] = pid.kp.to_le_bytes();
                    registers_lk[*addr_i] = pid.ki.to_le_bytes();
                    registers_lk[*addr_d] = pid.kd.to_le_bytes();
                }
            }

            // ### 2.4a.5: Update Kpid values from registers if the reload pid flag is not zero
            if registers_lk[ADDR_RELOAD_PID as usize] != [0; URAP_DATA_WIDTH] {
                registers_lk[ADDR_RELOAD_PID as usize] = [0; URAP_DATA_WIDTH];

                for (pid, addr_p, addr_i, addr_d, addr_i_range, i_range) in [
                    (
                        &mut pid_z1,
                        ADDR_P_Z1 as usize,
                        ADDR_I_Z1 as usize,
                        ADDR_D_Z1 as usize,
                        ADDR_I_RANGE_Z1_C as usize,
                        &mut pid_z1_irange_c,
                    ),
                    (
                        &mut pid_z2,
                        ADDR_P_Z2 as usize,
                        ADDR_I_Z2 as usize,
                        ADDR_D_Z2 as usize,
                        ADDR_I_RANGE_Z2_C as usize,
                        &mut pid_z2_irange_c,
                    ),
                    (
                        &mut pid_z3,
                        ADDR_P_Z3 as usize,
                        ADDR_I_Z3 as usize,
                        ADDR_D_Z3 as usize,
                        ADDR_I_RANGE_Z3_C as usize,
                        &mut pid_z3_irange_c,
                    ),
                    (
                        &mut pid_z4,
                        ADDR_P_Z4 as usize,
                        ADDR_I_Z4 as usize,
                        ADDR_D_Z4 as usize,
                        ADDR_I_RANGE_Z4_C as usize,
                        &mut pid_z4_irange_c,
                    ),
                    (
                        &mut pid_z5,
                        ADDR_P_Z5 as usize,
                        ADDR_I_Z5 as usize,
                        ADDR_D_Z5 as usize,
                        ADDR_I_RANGE_Z5_C as usize,
                        &mut pid_z5_irange_c,
                    ),
                    (
                        &mut pid_z6,
                        ADDR_P_Z6 as usize,
                        ADDR_I_Z6 as usize,
                        ADDR_D_Z6 as usize,
                        ADDR_I_RANGE_Z6_C as usize,
                        &mut pid_z6_irange_c,
                    ),
                ] {
                    pid.p(f32::from_le_bytes(registers_lk[addr_p]).max(0.0), 1.0);
                    pid.i(f32::from_le_bytes(registers_lk[addr_i]).max(0.0), 1.0);
                    pid.d(f32::from_le_bytes(registers_lk[addr_d]).max(0.0), 1.0);

                    *i_range = f32::from_le_bytes(registers_lk[addr_i_range]);

                    registers_lk[addr_p] = pid.kp.to_le_bytes();
                    registers_lk[addr_i] = pid.ki.to_le_bytes();
                    registers_lk[addr_d] = pid.kd.to_le_bytes();
                }
            }

            // ### 2.4a.6: Toggle fans on or off
            for (temp_c, setpoint_c, ssr_fan) in [
                (t1_c, pid_z1.setpoint, &mut ssr_z1_fan),
                (t2_c, pid_z2.setpoint, &mut ssr_z2_fan),
                (t3_c, pid_z3.setpoint, &mut ssr_z3_fan),
                (t4_c, pid_z4.setpoint, &mut ssr_z4_fan),
            ] {
                if temp_c > (setpoint_c - FAN_ENABLE_C + NOISE_FLOOR_C) && temp_c > (FAN_CUTOFF_C + NOISE_FLOOR_C) {
                    ssr_fan.set_high();
                }

                if temp_c < (setpoint_c - FAN_ENABLE_C - NOISE_FLOOR_C) || temp_c < (FAN_CUTOFF_C - NOISE_FLOOR_C) {
                    ssr_fan.set_low();
                }
            }

            // ### 2.4a.7a: Update power values from registers if power override is not zero
            (pwr_z1, pwr_z2, pwr_z3, pwr_z4, pwr_z5, pwr_z6) =
                if registers_lk[ADDR_ENBL_PWR_OVERRIDE as usize] != [0; URAP_DATA_WIDTH] {
                    (
                        f32::from_le_bytes(registers_lk[ADDR_Z1_PWR as usize]),
                        f32::from_le_bytes(registers_lk[ADDR_Z2_PWR as usize]),
                        f32::from_le_bytes(registers_lk[ADDR_Z3_PWR as usize]),
                        f32::from_le_bytes(registers_lk[ADDR_Z4_PWR as usize]),
                        f32::from_le_bytes(registers_lk[ADDR_Z5_PWR as usize]),
                        f32::from_le_bytes(registers_lk[ADDR_Z6_PWR as usize]),
                    )
                }
                // ### 2.4a.7b: Otherwise, update power values from PID calcualtions
                else if poll_pid.saturating_duration_since(now).is_zero() {
                    poll_pid = now.checked_add(Duration::from_millis(POLL_PID_MS)).unwrap();

                    (
                        pid_z1.next_control_output(t1_c).output,
                        pid_z2.next_control_output(t2_c).output,
                        pid_z3.next_control_output(t3_c).output,
                        pid_z4.next_control_output(t4_c).output,
                        pid_z5.next_control_output(t5_c).output,
                        pid_z6.next_control_output(t6_c).output,
                    )
                } else {
                    (pwr_z1, pwr_z2, pwr_z3, pwr_z4, pwr_z5, pwr_z6)
                };

            // ### 2.4a.8: Ensure power values are safe and not negative

            let pwr_z1 = pwr_z1.max(0.0);
            let pwr_z2 = pwr_z2.max(0.0);
            let pwr_z3 = pwr_z3.max(0.0);
            let pwr_z4 = pwr_z4.max(0.0);
            let pwr_z5 = pwr_z5.max(0.0);
            let pwr_z6 = pwr_z6.max(0.0);

            // ### 2.4a.9: Calculate current usage
            let cur_ideal_a = {
                pwr_z1 * CUR_Z1_A
                    + pwr_z2 * CUR_Z2_A
                    + pwr_z3 * CUR_Z3_A
                    + pwr_z4 * CUR_Z4_A
                    + pwr_z5 * CUR_Z5_A
                    + pwr_z6 * CUR_Z6_A
            };

            // When the motor is stopped the current reads negative, ignore this.
            let cur_total_a = cur_ideal_a + motor_line_a.min(0.0);

            // #### 2.4a.9.1 Limit power so that total extruder current doesn't
            // exceed limit
            let cur_multiplier = 1.0_f32.min(CUR_MAX_A / cur_total_a);

            let pwr_z1 = pwr_z1 * cur_multiplier;
            let pwr_z2 = pwr_z2 * cur_multiplier;
            let pwr_z3 = pwr_z3 * cur_multiplier;
            let pwr_z4 = pwr_z4 * cur_multiplier;
            let pwr_z5 = pwr_z5 * cur_multiplier;
            let pwr_z6 = pwr_z6 * cur_multiplier;

            // ### 2.4a.10: Average out power in between PWM periods
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

            pwr_z1_mean = pwr_z1 * pwr_z1_weight + pwr_z1_mean * (1.0 - pwr_z1_weight);
            pwr_z2_mean = pwr_z2 * pwr_z2_weight + pwr_z2_mean * (1.0 - pwr_z2_weight);
            pwr_z3_mean = pwr_z3 * pwr_z3_weight + pwr_z3_mean * (1.0 - pwr_z3_weight);
            pwr_z4_mean = pwr_z4 * pwr_z4_weight + pwr_z4_mean * (1.0 - pwr_z4_weight);
            pwr_z5_mean = pwr_z5 * pwr_z5_weight + pwr_z5_mean * (1.0 - pwr_z5_weight);
            pwr_z6_mean = pwr_z6 * pwr_z6_weight + pwr_z6_mean * (1.0 - pwr_z6_weight);

            // #### 2.4a.10.1: Write total average power consumption to register
            
            let cur_actual_a = {
                pwr_z1_mean * CUR_Z1_A
                    + pwr_z2_mean * CUR_Z2_A
                    + pwr_z3_mean * CUR_Z3_A
                    + pwr_z4_mean * CUR_Z4_A
                    + pwr_z5_mean * CUR_Z5_A
                    + pwr_z6_mean * CUR_Z6_A
            };

            registers_lk[ADDR_THERMO_PWR_W as usize] =
                (cur_actual_a * line_v).to_le_bytes();

            // ### 2.4a.11: Control heater PWM
            for (instant_on, instant_off, ssr_heat, pwr_mean, pwr_samples, pwr_addr) in [
                (
                    &mut instant_z1_on,
                    &mut instant_z1_off,
                    &mut ssr_z1_heat,
                    &mut pwr_z1_mean,
                    &mut pwr_z1_samples,
                    ADDR_Z1_PWR as usize,
                ),
                (
                    &mut instant_z2_on,
                    &mut instant_z2_off,
                    &mut ssr_z2_heat,
                    &mut pwr_z2_mean,
                    &mut pwr_z2_samples,
                    ADDR_Z2_PWR as usize,
                ),
                (
                    &mut instant_z3_on,
                    &mut instant_z3_off,
                    &mut ssr_z3_heat,
                    &mut pwr_z3_mean,
                    &mut pwr_z3_samples,
                    ADDR_Z3_PWR as usize,
                ),
                (
                    &mut instant_z4_on,
                    &mut instant_z4_off,
                    &mut ssr_z4_heat,
                    &mut pwr_z4_mean,
                    &mut pwr_z4_samples,
                    ADDR_Z4_PWR as usize,
                ),
                (
                    &mut instant_z5_on,
                    &mut instant_z5_off,
                    &mut ssr_z5_heat,
                    &mut pwr_z5_mean,
                    &mut pwr_z5_samples,
                    ADDR_Z5_PWR as usize,
                ),
                (
                    &mut instant_z6_on,
                    &mut instant_z6_off,
                    &mut ssr_z6_heat,
                    &mut pwr_z6_mean,
                    &mut pwr_z6_samples,
                    ADDR_Z6_PWR as usize,
                ),
            ] {
                // #### 2.4a.11.1a: If start of a new period, calculate on and off duration
                if instant_on.saturating_duration_since(now).is_zero() {
                    let period_ms = *pwr_mean * PWM_PERIOD_MS as f32;

                    *instant_on = now
                        .checked_add(Duration::from_millis(PWM_PERIOD_MS))
                        .unwrap();
                    *instant_off = now
                        .checked_add(Duration::from_millis(period_ms as u64))
                        .unwrap();

                    // ##### 2.4a.11.1.1: If power override is disabled, write average power to power register
                    if registers_lk[ADDR_ENBL_PWR_OVERRIDE as usize] == [0; URAP_DATA_WIDTH] {
                        registers_lk[pwr_addr] = pwr_mean.to_le_bytes();
                    }

                    // ##### 2.4a.11.1.2: Reset power average
                    *pwr_mean = 0.0;
                    *pwr_samples = 0.0;

                    // ##### 2.4a.11.1.3a: If the duration the heater is to be on
                    // is greater than the minimum period time, turn on the heater
                    if period_ms > PWM_PERIOD_MIN_MS as f32 {
                        ssr_heat.set_high();
                    }
                    // ##### 2.4a.11.1.3b: Otherwise, ensure the heater is off
                    else {
                        ssr_heat.set_low();
                    }
                }
                // #### 2.4a.11.1b: Otherwise, if it is time to turn off the heater,
                // turn off the heater.
                else if instant_off.saturating_duration_since(now).is_zero() {
                    ssr_heat.set_low();
                }
            }

            // ### 2.4a.12: Drop lock on registers so secondary can access them
            drop(registers_lk);
        }
        // ## 2.4b: Otherwise, perform estop functions
        else {
            // ### 2.4b.1: Set target temperature and power registers to zero
            registers_lk[ADDR_SET_Z1_C as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_SET_Z2_C as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_SET_Z3_C as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_SET_Z4_C as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_SET_Z5_C as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_SET_Z6_C as usize] = 0_f32.to_le_bytes();

            registers_lk[ADDR_Z1_PWR as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_Z2_PWR as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_Z3_PWR as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_Z4_PWR as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_Z5_PWR as usize] = 0_f32.to_le_bytes();
            registers_lk[ADDR_Z6_PWR as usize] = 0_f32.to_le_bytes();

            // ### 2.4b.2: Drop lock on registers so secondary can access them
            drop(registers_lk);

            // ### 2.4b.3: Turn on all fans and turn off all heaters so extruder
            // can cool down
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

            // ### 2.4b.4: Reopen lost urap connections
            if !urap_i2c.is_healthy() {
                urap_i2c = UrapPrimary::new(URAP_I2C_PATH).unwrap_or(urap_i2c);
            }
            if !urap_screw.is_healthy() {
                urap_screw = UrapPrimary::new(URAP_SCREW_PATH).unwrap_or(urap_screw);
            }
        }

        // ## 2.5: Sleep until the loop restarts
        sleep_till(wakeup);
    }
}
