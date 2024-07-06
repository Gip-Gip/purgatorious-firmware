//! Driver for the ADS1113 ADC for the pressure sensor

use rppal::i2c::{Error as I2CError, I2c};

/// FSO of the PT412 in kPa
const FSO_KPA: f32 = 68_947.57;

/// FSO% used during calibration
const CAL_FSO_KPA: f32 = 0.8 * FSO_KPA;
const SAFETY_LIMIT_KPA: f32 = 0.8 * FSO_KPA;

/// Config address
const ADDR_CFG: u8 = 0x01;

/// Config flags
const CONFIG: u16 = 0b0_00_0000_0_100_0_0000;

pub struct Ads1113<'a> {
    i2c: &'a mut I2c,
    calstate: &'a mut CalState,
}

#[derive(Default)]
pub struct CalState {
    conversion_multiplier: f32,
    zero_base: f32,
    cal_samples_hi: u32,
    cal_samples_lo: u32,
}

impl<'a> Ads1113<'a> {
    pub fn new(i2c: &'a mut I2c, addr: u8, calstate: &'a mut CalState) -> Result<Self, I2CError> {
        i2c.set_slave_address(addr as u16)?;

        Ok(Self { i2c, calstate })
    }

    pub fn write_u16(&mut self, addr: u8, val: u16) -> Result<usize, I2CError> {
        let val_bytes = val.to_be_bytes();
        let buffer: [u8; 3] = [addr, val_bytes[0], val_bytes[1]];

        self.i2c.write(&buffer)
    }

    pub fn read_i16(&mut self, addr: u8) -> Result<i16, I2CError> {
        let mut buffer: [u8; 2] = [0; 2];

        self.i2c.write_read(&[addr], &mut buffer)?;

        Ok(i16::from_be_bytes(buffer))
    }

    pub fn init(&mut self) -> Result<(), I2CError> {
        self.write_u16(ADDR_CFG, CONFIG)?;

        Ok(())
    }

    pub fn push_calibration_high_sample(&mut self) -> Result<f32, I2CError> {
        self.calstate.cal_samples_hi += 1;
        let raw_value = self.read_i16(0)? as f32 - self.calstate.zero_base;

        let sample = CAL_FSO_KPA / raw_value;

        let sample_weight = 1.0 / (self.calstate.cal_samples_hi as f32);

        self.calstate.conversion_multiplier =
            sample * sample_weight + self.calstate.conversion_multiplier * (1.0 - sample_weight);

        Ok(raw_value * self.calstate.conversion_multiplier)
    }

    pub fn push_calibration_low_sample(&mut self) -> Result<f32, I2CError> {
        self.calstate.cal_samples_lo += 1;
        let raw_value = self.read_i16(0)? as f32;

        let sample_weight = 1.0 / (self.calstate.cal_samples_lo as f32);

        self.calstate.zero_base =
            raw_value * sample_weight + self.calstate.zero_base * (1.0 - sample_weight);

        Ok(raw_value - self.calstate.zero_base)
    }

    pub fn get_pressure_kpa(&mut self) -> Result<f32, I2CError> {
        let raw_value = self.read_i16(0)? as f32 - self.calstate.zero_base;

        Ok(raw_value * self.calstate.conversion_multiplier)
    }

    pub fn is_at_safety_limit(&mut self, pressure_kpa: f32) -> bool {
        pressure_kpa >= SAFETY_LIMIT_KPA
    }
}
