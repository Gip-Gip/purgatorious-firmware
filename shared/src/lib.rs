use std::{thread::sleep, time::Instant};

pub static URAP_I2C_PATH: &str = "/var/firmware/i2c";
pub static URAP_THERMO_PATH: &str = "/var/firmware/thermo";
pub static URAP_GUI_PATH: &str = "/var/firmware/gui";
pub static URAP_SCREW_PATH: &str = "/var/firmware/screw";
pub static URAP_WATCHDOG_PATH: &str = "/var/firmware/watchdog";
pub const ADDR_INCHASH: u16 = 0x0000;

#[inline]
pub fn sleep_till(instant: Instant) {
    let time_now = Instant::now();

    let sleep_dur = instant.saturating_duration_since(time_now);

    sleep(sleep_dur)
}

#[inline]
pub fn retry_thrice<O, E, F>(mut x: F) -> Result<O, E>
where
    F: FnMut() -> Result<O, E>,
{
    for _ in 0..2 {
        match x() {
            Ok(o) => {
                return Ok(o);
            }
            Err(_) => {}
        }
    }

    x()
}
