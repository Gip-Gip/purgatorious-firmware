use std::{thread::sleep, time::Instant};

pub static URAP_I2C_PATH: &str = "/var/firmware/i2c";
pub static URAP_THERMO_PATH: &str = "/var/firmware/thermo";
pub static URAP_GUI_PATH: &str = "/var/firmware/gui";
pub static URAP_SCREW_PATH: &str = "/var/firmware/screw";
pub static URAP_WATCHDOG_PATH: &str = "/var/firmware/watchdog";

#[inline]
pub fn sleep_till(instant: Instant) {
    let time_now = Instant::now();

    let sleep_dur = instant.saturating_duration_since(time_now);

    sleep(sleep_dur)
}
