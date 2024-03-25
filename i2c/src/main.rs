//! Provides access to all the i2c peripherals

mod ads1113;
mod mcp9600;

use ads1113::{Ads1113, CalState};
use i2c::*;
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
use watchdog::ADDR_ESTOP;

use mcp9600::Mcp9600;
use rppal::{gpio::Gpio, i2c::I2c};
use shared::*;
use urap::*;

static SOCKETPATH: &str = "/var/firmware/i2c";

const I2C_BUS: u8 = 1;

const I2CADDR_TC7: u8 = 0x61;
const I2CADDR_TC6: u8 = 0x62;
const I2CADDR_TC5: u8 = 0x63;
const I2CADDR_TC4: u8 = 0x64;
const I2CADDR_TC3: u8 = 0x65;
const I2CADDR_TC2: u8 = 0x66;
const I2CADDR_TC1: u8 = 0x67;
const I2CADDR_BARREL_PRESSURE: u8 = 0x48;

const URAP_REG_COUNT: usize = 0x11;

const POLLRATE_MS: u64 = POLLSTAGGER_MS;
const POLLRATE_TC_MS: u64 = 100;
const POLLSTAGGER_MS: u64 = POLLRATE_TC_MS / 10;

const CAL_TIME_MS: u64 = 10000;
const CAL_START_TIME_MS: u64 = 1000;
const TC_TIMEOUT_MS: u64 = 3000;

const CAL_RELAY_PIN: u8 = 21;

const MAX_TEMP_C: f32 = 250.0;

fn main() {
    let mut urap_watchdog = UrapMaster::new(URAP_WATCHDOG_PATH).unwrap();

    let registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; URAP_REG_COUNT]>> =
        Arc::new(Mutex::new([[0; URAP_REG_WIDTH]; URAP_REG_COUNT]));

    UrapSlave::spawn(SOCKETPATH, registers.clone(), [true; URAP_REG_COUNT]).unwrap();

    // Calibration relay gpio
    let gpio = Gpio::new().unwrap();
    let mut cal_relay = gpio.get(CAL_RELAY_PIN).unwrap().into_output_low();

    let mut i2c = I2c::with_bus(I2C_BUS).unwrap();

    // Initialize all of the thermocouple chips
    let mcp9600s: [u8; 7] = [
        I2CADDR_TC1,
        I2CADDR_TC2,
        I2CADDR_TC3,
        I2CADDR_TC4,
        I2CADDR_TC5,
        I2CADDR_TC6,
        I2CADDR_TC7,
    ];

    for mcp9600 in mcp9600s {
        let mut mcp9600 = Mcp9600::new(&mut i2c, mcp9600).unwrap();

        loop {
            let result = mcp9600.init();

            if let Ok(_) = result {
                break;
            }
        }
    }

    let mut barrel_cal = CalState::default();
    let mut barrel_pres = Ads1113::new(&mut i2c, I2CADDR_BARREL_PRESSURE, &mut barrel_cal).unwrap();
    barrel_pres
        .init()
        .unwrap_or(barrel_pres.init().unwrap_or(barrel_pres.init().unwrap()));

    let now = Instant::now();

    let mut poll = Instant::now()
        .checked_add(Duration::from_millis(POLLRATE_MS))
        .unwrap();

    let mut poll_tc1 = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS))
        .unwrap();
    let mut poll_tc2 = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS + POLLSTAGGER_MS))
        .unwrap();
    let mut poll_tc3 = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS + 2 * POLLRATE_TC_MS))
        .unwrap();
    let mut poll_tc4 = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS + 3 * POLLRATE_TC_MS))
        .unwrap();
    let mut poll_tc5 = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS + 4 * POLLRATE_TC_MS))
        .unwrap();
    let mut poll_tc6 = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS + 5 * POLLRATE_TC_MS))
        .unwrap();
    let mut poll_tc7 = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS + 6 * POLLRATE_TC_MS))
        .unwrap();
    let mut poll_barrel_pres = now
        .checked_add(Duration::from_millis(POLLRATE_TC_MS + 7 * POLLRATE_TC_MS))
        .unwrap();

    let cal_start = now
        .checked_add(Duration::from_millis(CAL_START_TIME_MS))
        .unwrap();
    let cal_mid = cal_start
        .checked_add(Duration::from_millis(CAL_TIME_MS))
        .unwrap();
    let cal_inter = cal_mid
        .checked_add(Duration::from_millis(CAL_START_TIME_MS))
        .unwrap();
    let cal_end = cal_inter
        .checked_add(Duration::from_millis(CAL_TIME_MS))
        .unwrap();

    let mut t1_c: f32 = 0.0;
    let mut t2_c: f32 = 0.0;
    let mut t3_c: f32 = 0.0;
    let mut t4_c: f32 = 0.0;
    let mut t5_c: f32 = 0.0;
    let mut t6_c: f32 = 0.0;
    let mut t7_c: f32 = 0.0;

    let mut ta1_c: f32 = 0.0;
    let mut ta2_c: f32 = 0.0;
    let mut ta3_c: f32 = 0.0;
    let mut ta4_c: f32 = 0.0;
    let mut ta5_c: f32 = 0.0;
    let mut ta6_c: f32 = 0.0;
    let mut ta7_c: f32 = 0.0;

    let mut barrel_kpa: f32 = 0.0;

    loop {
        sleep_till(poll);
        let now = Instant::now();

        poll = Instant::now()
            .checked_add(Duration::from_millis(POLLRATE_MS))
            .unwrap();

        let mut registers = registers.lock().unwrap();

        // Increment the inchash
        let inchash = u32::from_ne_bytes(registers[ADDR_INCHASH]) + 1;
        registers[ADDR_INCHASH] = inchash.to_ne_bytes();

        // Transfer all the temperatures to the registers
        registers[ADDR_T1_C] = t1_c.to_ne_bytes();
        registers[ADDR_T2_C] = t2_c.to_ne_bytes();
        registers[ADDR_T3_C] = t3_c.to_ne_bytes();
        registers[ADDR_T4_C] = t4_c.to_ne_bytes();
        registers[ADDR_T5_C] = t5_c.to_ne_bytes();
        registers[ADDR_T6_C] = t6_c.to_ne_bytes();
        registers[ADDR_T7_C] = t7_c.to_ne_bytes();
        registers[ADDR_TA1_C] = ta1_c.to_ne_bytes();
        registers[ADDR_TA2_C] = ta2_c.to_ne_bytes();
        registers[ADDR_TA3_C] = ta3_c.to_ne_bytes();
        registers[ADDR_TA4_C] = ta4_c.to_ne_bytes();
        registers[ADDR_TA5_C] = ta5_c.to_ne_bytes();
        registers[ADDR_TA6_C] = ta6_c.to_ne_bytes();
        registers[ADDR_TA7_C] = ta7_c.to_ne_bytes();

        // Ambient temperature is the average of all the ambient temperatures
        registers[ADDR_AMBIENT_C] =
            ((ta1_c + ta2_c + ta3_c + ta4_c + ta5_c + ta6_c + ta7_c) / 7.0).to_ne_bytes();

        registers[ADDR_BARREL_KPA] = barrel_kpa.to_ne_bytes();

        // Poll thermocouples

        let thermos: [(&mut Instant, u8, &mut f32, &mut f32); 7] = [
            (&mut poll_tc1, I2CADDR_TC1, &mut t1_c, &mut ta1_c),
            (&mut poll_tc2, I2CADDR_TC2, &mut t2_c, &mut ta2_c),
            (&mut poll_tc3, I2CADDR_TC3, &mut t3_c, &mut ta3_c),
            (&mut poll_tc4, I2CADDR_TC4, &mut t4_c, &mut ta4_c),
            (&mut poll_tc5, I2CADDR_TC5, &mut t5_c, &mut ta5_c),
            (&mut poll_tc6, I2CADDR_TC6, &mut t6_c, &mut ta6_c),
            (&mut poll_tc7, I2CADDR_TC7, &mut t7_c, &mut ta7_c),
        ];

        for (poll, addr, temp_c, tamb_c) in thermos {
            if poll.saturating_duration_since(now).is_zero() {
                let mut tc = Mcp9600::new(&mut i2c, addr).unwrap();
                (*temp_c, *tamb_c) = match tc.get_temp_c() {
                    Ok(val) => {
                        *poll = now
                            .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                            .unwrap();

                        val
                    }
                    Err(_) => {
                        if now.saturating_duration_since(*poll)
                            >= Duration::from_millis(TC_TIMEOUT_MS)
                        {
                            if urap_watchdog.read_u32(ADDR_ESTOP).unwrap_or(1) == 0 {
                                println!(
                                    "**FAULT** Timeout on communicating with Mcp9600 at address {}",
                                    addr
                                );
                            }
                            urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap();
                        }

                        (*temp_c, *tamb_c)
                    }
                };
            }

            if *temp_c >= MAX_TEMP_C {
                if urap_watchdog.read_u32(ADDR_ESTOP).unwrap_or(1) == 0 {
                    println!(
                        "**FAULT** Overtemp of {} read from Mcp9600 at address {}",
                        *temp_c, addr
                    );
                }
                urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap();
            }
        }

        if poll_barrel_pres.saturating_duration_since(now) == Duration::ZERO {
            let mut barrel_pres =
                Ads1113::new(&mut i2c, I2CADDR_BARREL_PRESSURE, &mut barrel_cal).unwrap();

            if cal_start.saturating_duration_since(now) == Duration::ZERO
                && cal_mid.saturating_duration_since(now) != Duration::ZERO
            {
                barrel_kpa = barrel_pres.push_calibration_low_sample().unwrap_or(
                    barrel_pres
                        .push_calibration_low_sample()
                        .unwrap_or(barrel_pres.push_calibration_low_sample().unwrap()),
                );

                // Keep estop active while calibrating
                urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
            } else if cal_mid.saturating_duration_since(now) == Duration::ZERO
                && cal_inter.saturating_duration_since(now) != Duration::ZERO
            {
                cal_relay.set_high();
            } else if cal_inter.saturating_duration_since(now) == Duration::ZERO
                && cal_end.saturating_duration_since(now) != Duration::ZERO
            {
                barrel_kpa = barrel_pres.push_calibration_high_sample().unwrap_or(
                    barrel_pres
                        .push_calibration_high_sample()
                        .unwrap_or(barrel_pres.push_calibration_high_sample().unwrap()),
                );

                // Keep estop active while calibrating
                urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
            } else if cal_end.saturating_duration_since(now) == Duration::ZERO {
                cal_relay.set_low();
                barrel_kpa = barrel_pres.get_pressure_kpa().unwrap_or(
                    barrel_pres
                        .get_pressure_kpa()
                        .unwrap_or(barrel_pres.get_pressure_kpa().unwrap()),
                );

                // Sound estop if we're at the safety limit
                if barrel_pres.is_at_safety_limit(barrel_kpa) {
                    urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
                }
            }

            poll_barrel_pres = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
    }
}
