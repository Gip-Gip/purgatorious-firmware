//! Interface with the screw motor controller

use std::{
    f32::NAN, sync::{Arc, Mutex}, time::{Duration, Instant}
};

use quantumiii::QuantumIII;
use screw::*;
use shared::{sleep_till, URAP_SCREW_PATH, URAP_WATCHDOG_PATH};
use urap::*;
use watchdog::ADDR_ESTOP;

mod quantumiii;

const Q3_ADDR: u8 = 1;
static UART_PATH: &str = "/dev/ttyAMA0";

const URAP_REG_COUNT: usize = 0x09;

const POLL_MS: u64 = 500;
const PROPOGATION_POLL_MS: u64 = 2000;

const URAP_WRITE_PROTECT: [bool; URAP_REG_COUNT] =
    [true, false, true, true, true, true, true, true, true];

fn main() {
    let mut urap_watchdog = UrapMaster::new(URAP_WATCHDOG_PATH).unwrap();
    let mut quantumiii = QuantumIII::new(UART_PATH, Q3_ADDR).unwrap();

    quantumiii.init().unwrap();

    let registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; URAP_REG_COUNT]>> =
        Arc::new(Mutex::new([[0; URAP_REG_WIDTH]; URAP_REG_COUNT]));

    UrapSlave::spawn(URAP_SCREW_PATH, registers.clone(), URAP_WRITE_PROTECT).unwrap();

    // Set to NAN so the first instinct is to set the drive speed to zero
    let mut old_set_screw_rpm = NAN;

    let mut propogation_check = Instant::now();

    loop {
        let now = Instant::now();
        let wakeup = now.checked_add(Duration::from_millis(POLL_MS)).unwrap();

        let mut registers_lk = registers.lock().unwrap();

        // Increment the inchash
        let inchash = u32::from_ne_bytes(registers_lk[ADDR_INCHASH]) + 1;
        registers_lk[ADDR_INCHASH] = inchash.to_ne_bytes();

        // Normal Operation
        if urap_watchdog.read_u32(ADDR_ESTOP).unwrap_or(1) == 0 {

            let set_screw_rpm = f32::from_ne_bytes(registers_lk[ADDR_SET_SCREW_RPM]);

            drop(registers_lk);

            // Only update the screw if it has been set to a new value, the RS485 connection
            // runs at a snail's pace of 9600 baud(1 char/ms approx)
            // Takes 18ms (17bytes out, 1 byte returned) on a good day
            if set_screw_rpm != old_set_screw_rpm {
                old_set_screw_rpm = set_screw_rpm;

                // Retry thrice...
                quantumiii.set_screw_rpm(set_screw_rpm).unwrap_or_else(|_| {
                    quantumiii.fault_reset().unwrap();
                    quantumiii.set_screw_rpm(set_screw_rpm).unwrap_or_else(|_| {
                        quantumiii.fault_reset().unwrap();
                        quantumiii.set_screw_rpm(set_screw_rpm).unwrap();
                    })
                });
            }

            // Try thrice, hopefully this doesn't fail but we can't afford to just
            // ignore a triple fault when communicating with the drive motor.
            // Something must be up, and the watchdog will handle the unwrap.
            // Takes 22ms (10 bytes transmitted, 12 bytes returned) for the first
            // transmit, 117ms (9*1 byte transmitted, 9*12 bytes returned) for the
            // subsequent queries.
            let zeropage = quantumiii.read_zero_page().unwrap_or_else(|_| {
                quantumiii.fault_reset().unwrap();
                quantumiii
                    .read_zero_page()
                    .unwrap_or_else(|_| {
                        quantumiii.fault_reset().unwrap();
                        quantumiii.read_zero_page().unwrap()
                    })
            });

            // Luck permitting all queries should take about 157ms (18ms + 22ms + 117ms)

            let mut registers_lk = registers.lock().unwrap();

            // Set speed takes time to propogate, wait a second or two before
            // setting our screw's set rpm to the drive's
            if propogation_check.saturating_duration_since(now) == Duration::ZERO {
                registers_lk[ADDR_SET_SCREW_RPM] = match zeropage.drive_enabled() {
                    true => zeropage.get_set_screw_rpm(),
                    false => 0.0,
                }
                .to_ne_bytes();

                propogation_check = propogation_check.checked_add(Duration::from_millis(PROPOGATION_POLL_MS)).unwrap();
            }


            registers_lk[ADDR_ACT_SCREW_RPM] = zeropage.get_act_screw_rpm().to_ne_bytes();
            registers_lk[ADDR_ACT_MOTOR_RPM] = zeropage.get_act_motor_rpm().to_ne_bytes();
            registers_lk[ADDR_MOTOR_A] = zeropage.get_motor_a().to_ne_bytes();
            registers_lk[ADDR_MOTOR_V] = zeropage.get_motor_v().to_ne_bytes();
            registers_lk[ADDR_DRIVE_ENBL] = match zeropage.drive_enabled() {
                true => 1_u32,
                false => 0_u32,
            }
            .to_ne_bytes();
            registers_lk[ADDR_LINE_V] = zeropage.get_line_v().to_ne_bytes();

            let watts = zeropage.get_motor_a() * zeropage.get_motor_v();
            let line_a = watts / zeropage.get_line_v();

            registers_lk[ADDR_LINE_A] = line_a.to_ne_bytes();

            drop(registers_lk);

            // If the drive has tripped, send the estop signal!
            if !zeropage.drive_ok() {
                urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();

                let trip_history = zeropage.get_trip_history();

                println!("*FAULT* QuantumIII Tripped! Past 3 trips: {:?} {:?} {:?}", trip_history[0], trip_history[1], trip_history[2]);
            }
        }
        // E-Stop Code
        else {
            drop(registers_lk);

            quantumiii.set_screw_rpm(0.0).unwrap_or_else(|_| {
                quantumiii.fault_reset().unwrap();
            });

            quantumiii.attempt_trip_reset().unwrap_or_else(|_| {
                quantumiii.fault_reset().unwrap();
            });
        }

        sleep_till(wakeup);
    }
}
