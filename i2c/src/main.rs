//! Provides access to all the i2c peripherals

mod mcp9600;

use i2c::*;
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use mcp9600::Mcp9600;
use rppal::i2c::I2c;
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

const URAP_REG_COUNT: usize = 16;

const POLLRATE_MS: u64 = POLLSTAGGER_MS;
const POLLRATE_TC_MS: u64 = 100;
const POLLSTAGGER_MS: u64 = POLLRATE_TC_MS / 10;

fn main() {
    let registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; URAP_REG_COUNT]>> =
        Arc::new(Mutex::new([[0; URAP_REG_WIDTH]; URAP_REG_COUNT]));

    UrapSlave::spawn(SOCKETPATH, registers.clone(), [true; URAP_REG_COUNT]).unwrap();

    let mut i2c = I2c::with_bus(I2C_BUS).unwrap();

    // Initialize all of the thermocouple chips
    let mut tc1 = Mcp9600::new(&mut i2c, I2CADDR_TC1).unwrap();
    tc1.init()
        .unwrap_or(tc1.init().unwrap_or(tc1.init().unwrap()));
    drop(tc1);

    let mut tc2 = Mcp9600::new(&mut i2c, I2CADDR_TC2).unwrap();
    tc2.init()
        .unwrap_or(tc2.init().unwrap_or(tc2.init().unwrap()));
    drop(tc2);

    let mut tc3 = Mcp9600::new(&mut i2c, I2CADDR_TC3).unwrap();
    tc3.init()
        .unwrap_or(tc3.init().unwrap_or(tc3.init().unwrap()));
    drop(tc3);

    let mut tc4 = Mcp9600::new(&mut i2c, I2CADDR_TC4).unwrap();
    tc4.init()
        .unwrap_or(tc4.init().unwrap_or(tc4.init().unwrap()));
    drop(tc4);

    let mut tc5 = Mcp9600::new(&mut i2c, I2CADDR_TC5).unwrap();
    tc5.init()
        .unwrap_or(tc5.init().unwrap_or(tc5.init().unwrap()));
    drop(tc5);

    let mut tc6 = Mcp9600::new(&mut i2c, I2CADDR_TC6).unwrap();
    tc6.init()
        .unwrap_or(tc6.init().unwrap_or(tc6.init().unwrap()));
    drop(tc6);

    let mut tc7 = Mcp9600::new(&mut i2c, I2CADDR_TC7).unwrap();
    tc7.init()
        .unwrap_or(tc7.init().unwrap_or(tc7.init().unwrap()));
    drop(tc7);

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

        // Poll thermocouples
        if poll_tc1.saturating_duration_since(now) == Duration::ZERO {
            let mut tc1 = Mcp9600::new(&mut i2c, I2CADDR_TC1).unwrap();
            (t1_c, ta1_c) = tc1.get_temp_c().unwrap_or((t1_c, ta1_c));

            poll_tc1 = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
        if poll_tc2.saturating_duration_since(now) == Duration::ZERO {
            let mut tc2 = Mcp9600::new(&mut i2c, I2CADDR_TC2).unwrap();
            (t2_c, ta2_c) = tc2.get_temp_c().unwrap_or((t2_c, t2_c));

            poll_tc2 = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
        if poll_tc3.saturating_duration_since(now) == Duration::ZERO {
            let mut tc3 = Mcp9600::new(&mut i2c, I2CADDR_TC3).unwrap();
            (t3_c, ta3_c) = tc3.get_temp_c().unwrap_or((t3_c, ta3_c));

            poll_tc3 = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
        if poll_tc4.saturating_duration_since(now) == Duration::ZERO {
            let mut tc4 = Mcp9600::new(&mut i2c, I2CADDR_TC4).unwrap();
            (t4_c, ta4_c) = tc4.get_temp_c().unwrap_or((t4_c, ta4_c));

            poll_tc4 = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
        if poll_tc5.saturating_duration_since(now) == Duration::ZERO {
            let mut tc5 = Mcp9600::new(&mut i2c, I2CADDR_TC5).unwrap();
            (t5_c, ta5_c) = tc5.get_temp_c().unwrap_or((t5_c, ta5_c));

            poll_tc5 = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
        if poll_tc6.saturating_duration_since(now) == Duration::ZERO {
            let mut tc6 = Mcp9600::new(&mut i2c, I2CADDR_TC6).unwrap();
            (t6_c, ta6_c) = tc6.get_temp_c().unwrap_or((t6_c, ta6_c));

            poll_tc6 = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
        if poll_tc7.saturating_duration_since(now) == Duration::ZERO {
            let mut tc7 = Mcp9600::new(&mut i2c, I2CADDR_TC7).unwrap();
            (t7_c, ta7_c) = tc7.get_temp_c().unwrap_or((t7_c, ta7_c));

            poll_tc7 = now
                .checked_add(Duration::from_millis(POLLRATE_TC_MS))
                .unwrap();
        }
    }
}
