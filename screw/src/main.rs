//! Interface with the screw motor controller

use std::{
    f32::NAN,
    fs::File,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use q3_backup::{Param, PARAMS};
use quantumiii::QuantumIII;
use screw::*;
use shared::{retry_thrice, sleep_till, ADDR_INCHASH, URAP_SCREW_PATH, URAP_WATCHDOG_PATH};
use urap::{usockets::*, URAP_REG_WIDTH};
use watchdog::ADDR_ESTOP;

use crate::q3_backup::ParamD;

mod q3_backup;
mod quantumiii;

const Q3_ADDR: u8 = 1;
static UART_PATH: &str = "/dev/ttyAMA0";
static BACKUP_FILE: &str = "/opt/firmware/q3_backup.json";

const URAP_REG_COUNT: usize = 0x0B;

const POLL_MS: u64 = 500;
const PROPOGATION_POLL_MS: u64 = 2000;

const URAP_WRITE_PROTECT: [bool; URAP_REG_COUNT as usize] = [
    true, false, true, true, true, true, true, true, true, false, false,
];

const DELAY_Q3_BOOT_S: u64 = 5;
const DELAY_MOTOR_FAN_SHUTOFF_S: u64 = 15 * 60;

fn main() {
    let mut urap_watchdog = UrapPrimary::new(URAP_WATCHDOG_PATH).unwrap();
    let mut quantumiii = QuantumIII::new(UART_PATH, Q3_ADDR).unwrap();
    
    retry_thrice(|| {
        let r = quantumiii.set_motor_fan(false);
        if r.is_err() {
            quantumiii.fault_reset().unwrap();
        }

        r
    })
    .unwrap();

    let registers: Arc<Mutex<[[u8; URAP_REG_WIDTH as usize]; URAP_REG_COUNT as usize]>> = Arc::new(
        Mutex::new([[0; URAP_REG_WIDTH as usize]; URAP_REG_COUNT as usize]),
    );

    UrapSecondary::spawn(URAP_SCREW_PATH, registers.clone(), URAP_WRITE_PROTECT).unwrap();

    // Set to NAN so the first instinct is to set the drive speed to zero
    let mut old_set_screw_rpm = NAN;

    let mut propogation_check = Instant::now();

    let mut backup_buffer: Option<Vec<Param>> = None;
    let mut restore_buffer: Option<Vec<ParamD>> = None;

    #[derive(Debug, PartialEq)]
    enum Q3States {
        Booting(Instant),
        Standby,
        Okay,
        Tripped,
    }

    let mut q3state = Q3States::Standby;

    let mut motor_fan_shutdown_instant: Option<Instant> = None;
    let mut motor_fan_state: bool = false;

    loop {
        let now = Instant::now();
        let wakeup = now.checked_add(Duration::from_millis(POLL_MS)).unwrap();

        let mut registers_lk = registers.lock().unwrap();

        // Increment the inchash
        let inchash = u32::from_ne_bytes(registers_lk[ADDR_INCHASH as usize]) + 1;
        registers_lk[ADDR_INCHASH as usize] = inchash.to_ne_bytes();

        // Backup the quantum 3's parameters if requested
        if registers_lk[ADDR_BACKUP_Q3 as usize] != [0; URAP_REG_WIDTH as usize] {
            q3state = Q3States::Standby;
            urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap();

            if backup_buffer.is_none() {
                backup_buffer = Some(Vec::with_capacity(PARAMS.len()));
                registers_lk[ADDR_BACKUP_Q3 as usize] = 0_u32.to_ne_bytes();
                retry_thrice(|| {
                    let r = quantumiii.elevate_perms();
                    if r.is_err() {
                        quantumiii.fault_reset().unwrap();
                    }

                    r
                })
                .unwrap();
            }

            let i = u32::from_ne_bytes(registers_lk[ADDR_BACKUP_Q3 as usize]) as usize;

            if i == PARAMS.len() {
                registers_lk[ADDR_BACKUP_Q3 as usize] = 0_u32.to_ne_bytes();

                let backup_file = File::create(BACKUP_FILE).unwrap();

                serde_json::to_writer(backup_file, backup_buffer.as_ref().unwrap()).unwrap();

                backup_buffer = None;

                retry_thrice(|| {
                    let r = quantumiii.reduce_perms();
                    if r.is_err() {
                        quantumiii.fault_reset().unwrap();
                    }

                    r
                })
                .unwrap();
            } else {
                if let Some(backup_buffer) = &mut backup_buffer {
                    let mut param = PARAMS[i as usize].clone();

                    param.val = Some(
                        retry_thrice(|| {
                            let r = quantumiii.read_param(param.id);
                            if r.is_err() {
                                quantumiii.fault_reset().unwrap();
                            }

                            r
                        })
                        .unwrap(),
                    );

                    backup_buffer.push(param);
                }
                registers_lk[ADDR_BACKUP_Q3 as usize] = ((i as u32) + 1).to_ne_bytes();
            }
        }

        // Restore the quantum 3's parameters if requested
        if registers_lk[ADDR_RESTORE_Q3 as usize] != [0; URAP_REG_WIDTH as usize] {
            q3state = Q3States::Standby;
            urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap();

            if restore_buffer.is_none() {
                let file = File::open(BACKUP_FILE).unwrap();
                restore_buffer = Some(serde_json::from_reader(file).unwrap());

                registers_lk[ADDR_RESTORE_Q3 as usize] = 0_u32.to_ne_bytes();
                retry_thrice(|| {
                    let r = quantumiii.elevate_perms();
                    if r.is_err() {
                        quantumiii.fault_reset().unwrap();
                    }

                    r
                })
                .unwrap();
            }

            let i = u32::from_ne_bytes(registers_lk[ADDR_RESTORE_Q3 as usize]) as usize;

            if let Some(restore_buffer) = &restore_buffer {
                if i == restore_buffer.len() {
                    registers_lk[ADDR_RESTORE_Q3 as usize] = 0_u32.to_ne_bytes();

                    retry_thrice(|| {
                        let r = quantumiii.reduce_perms();
                        if r.is_err() {
                            quantumiii.fault_reset().unwrap();
                        }

                        r
                    })
                    .unwrap();
                } else {
                    let param = &restore_buffer[i as usize];
                    retry_thrice(|| {
                        let r = quantumiii.write_param(&param.id, param.val.unwrap_or(0));
                        if r.is_err() {
                            quantumiii.fault_reset().unwrap();
                        }

                        r
                    })
                    .unwrap();
                    registers_lk[ADDR_RESTORE_Q3 as usize] = ((i as u32) + 1).to_ne_bytes();
                }
            }
        }

        // Normal Operation
        if urap_watchdog.read_u32(ADDR_ESTOP).unwrap_or(1) == 0 {
            // Ensure the quantumiii is ready from any previous stoppages
            match q3state {
                Q3States::Standby => {
                    retry_thrice(|| {
                        let r = quantumiii.init();
                        if r.is_err() {
                            quantumiii.fault_reset().unwrap();
                        }

                        r
                    })
                    .unwrap();

                    q3state = Q3States::Okay;
                }
                Q3States::Tripped => {
                    retry_thrice(|| {
                        let r = quantumiii.reset_drive();
                        if r.is_err() {
                            quantumiii.fault_reset().unwrap();
                        }

                        r
                    })
                    .unwrap();
                    q3state = Q3States::Booting(
                        now.checked_add(Duration::from_secs(DELAY_Q3_BOOT_S))
                            .unwrap(),
                    );
                }

                _ => {}
            }

            if q3state != Q3States::Okay {
                urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
                continue;
            }

            // Check the estop button
            let estop_depressed = retry_thrice(|| {
                let r = quantumiii.estop_depressed();
                if r.is_err() {
                    quantumiii.fault_reset().unwrap();
                }

                r
            })
            .unwrap();

            if estop_depressed {
                retry_thrice(|| {
                    let r = quantumiii.trip();
                    if r.is_err() {
                        quantumiii.fault_reset().unwrap();
                    }

                    r
                })
                .unwrap();

                q3state = Q3States::Tripped;
                continue;
            }

            let set_screw_rpm = f32::from_ne_bytes(registers_lk[ADDR_SET_SCREW_RPM as usize]);

            drop(registers_lk);

            // Only update the screw if it has been set to a new value, the RS485 connection
            // runs at a snail's pace of 9600 baud(1 char/ms approx)
            // Takes 18ms (17bytes out, 1 byte returned) on a good day
            if set_screw_rpm != old_set_screw_rpm {
                old_set_screw_rpm = set_screw_rpm;

                // Retry thrice...
                retry_thrice(|| {
                    let r = quantumiii.set_screw_rpm(set_screw_rpm);
                    if r.is_err() {
                        quantumiii.fault_reset().unwrap();
                    }

                    r
                })
                .unwrap()
            }

            // Try thrice, hopefully this doesn't fail but we can't afford to just
            // ignore a triple fault when communicating with the drive motor.
            // Something must be up, and the watchdog will handle the unwrap.
            // Takes 22ms (10 bytes transmitted, 12 bytes returned) for the first
            // transmit, 117ms (9*1 byte transmitted, 9*12 bytes returned) for the
            // subsequent queries.
            let zeropage = retry_thrice(|| {
                let r = quantumiii.read_zero_page();
                if r.is_err() {
                    quantumiii.fault_reset().unwrap();
                }

                r
            })
            .unwrap();

            // Luck permitting all queries should take about 157ms (18ms + 22ms + 117ms)

            let mut registers_lk = registers.lock().unwrap();

            // Enable cooling fan if drive is enabled 
            if zeropage.drive_enabled() {
                retry_thrice(|| {
                    let r = quantumiii.set_motor_fan(true);
                    if r.is_err() {
                        quantumiii.fault_reset().unwrap();
                    }

                    r
                })
                .unwrap();
                motor_fan_state = true;
            }
            else {
                if motor_fan_state {
                    motor_fan_shutdown_instant = Some(now.checked_add(Duration::from_secs(DELAY_MOTOR_FAN_SHUTOFF_S)).unwrap());
                    motor_fan_state = false;
                }
                else if let Some(shutoff) = motor_fan_shutdown_instant {
                    if shutoff.saturating_duration_since(now).is_zero() {
                        motor_fan_shutdown_instant = None;
                        retry_thrice(|| {
                            let r = quantumiii.set_motor_fan(false);
                            if r.is_err() {
                                quantumiii.fault_reset().unwrap();
                            }

                            r
                        })
                        .unwrap();
                    }
                }
            }

            // Set speed takes time to propogate, wait a second or two before
            // setting our screw's set rpm to the drive's
            if propogation_check.saturating_duration_since(now).is_zero() {
                registers_lk[ADDR_SET_SCREW_RPM as usize] = match zeropage.drive_enabled() {
                    true => zeropage.get_set_screw_rpm(),
                    false => 0.0,
                }
                .to_ne_bytes();

                propogation_check = propogation_check
                    .checked_add(Duration::from_millis(PROPOGATION_POLL_MS))
                    .unwrap();
            }

            registers_lk[ADDR_ACT_SCREW_RPM as usize] = zeropage.get_act_screw_rpm().to_ne_bytes();
            registers_lk[ADDR_ACT_MOTOR_RPM as usize] = zeropage.get_act_motor_rpm().to_ne_bytes();
            registers_lk[ADDR_MOTOR_A as usize] = zeropage.get_motor_a().to_ne_bytes();
            registers_lk[ADDR_MOTOR_V as usize] = zeropage.get_motor_v().to_ne_bytes();
            registers_lk[ADDR_DRIVE_ENBL as usize] = match zeropage.drive_enabled() {
                true => 1_u32,
                false => 0_u32,
            }
            .to_ne_bytes();
            registers_lk[ADDR_LINE_V as usize] = zeropage.get_line_v().to_ne_bytes();

            let watts = zeropage.get_motor_a() * zeropage.get_motor_v();
            let line_a = watts / zeropage.get_line_v();

            registers_lk[ADDR_LINE_A as usize] = line_a.to_ne_bytes();

            drop(registers_lk);

            // If the drive has tripped, send the estop signal!
            if !zeropage.drive_ok() {
                urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();

                let trip_history = zeropage.get_trip_history();

                println!(
                    "*FAULT* QuantumIII Tripped! Past 3 trips: {:?} {:?} {:?}",
                    trip_history[0], trip_history[1], trip_history[2]
                );

                q3state = Q3States::Tripped;
            }
        }
        // E-Stop Code
        else {
            drop(registers_lk);

            match q3state {
                Q3States::Okay => {
                    retry_thrice(|| {
                        let r = quantumiii.trip();
                        if r.is_err() {
                            quantumiii.fault_reset().unwrap();
                        }

                        r
                    })
                    .unwrap();

                    q3state = Q3States::Tripped;
                }
                Q3States::Booting(dur) => {
                    if dur.saturating_duration_since(now).is_zero() {
                        q3state = Q3States::Standby;
                    }
                }
                _ => {}
            }
        }

        sleep_till(wakeup);
    }
}
