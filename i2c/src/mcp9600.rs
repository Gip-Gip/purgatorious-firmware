use rppal::i2c::Error as I2CError;
use rppal::i2c::I2c;

/// Conversion rate between the raw thermocouple value and degrees celcius.
const TEMP_CONV_C: f32 = 0.0625;

/// Hot junction register
const REG_HOT_JUNC: u8 = 0;

/// Cold junction register
const REG_COLD_JUNC: u8 = 2;

/// Sensor config address
const ADDR_CFG_SENSOR: u8 = 0b0000_0101;

/// Device config address
const ADDR_CFG_DEVICE: u8 = 0b0000_0110;

/// Senser config flags
/// Use type J thermocouple and 4 filter pass
const CFG_SENSOR: u8 = 0b0001_0010;

/// Device config flags 
const CFG_DEVICE: u8 = 0b1_01_000_00;

/// Driver for the MCP9600 thermocouple chip.
pub struct Mcp9600<'a> {
    /// I2C bus to use.
    i2c: &'a mut I2c,
}

impl<'a> Mcp9600<'a> {
    /// Create a new driver given an i2c bus and the address of the chip.
    pub fn new(i2c: &'a mut I2c, addr: u8) -> Result<Self, I2CError> {
        let thermocouple = Self { i2c };

        thermocouple.i2c.set_slave_address(addr as u16)?;

        Ok(thermocouple)
    }

    /// Initialize the MCP9600
    pub fn init(&mut self) -> Result<(), I2CError> {
        self.write_u8(ADDR_CFG_SENSOR, CFG_SENSOR)?;
        self.write_u8(ADDR_CFG_DEVICE, CFG_DEVICE)?;

        Ok(())
    }

    /// Write a single byte to the MCP9600.
    pub fn write_u8(&mut self, addr: u8, val: u8) -> Result<usize, I2CError> {
        let buffer: [u8; 2] = [addr, val];

        self.i2c.write(&buffer)
    }

    /// Read two bytes from the MCP9600.
    pub fn read_i16(&mut self, addr: u8) -> Result<i16, I2CError> {
        let mut buffer: [u8; 2] = [0; 2];

        self.i2c.write_read(&[addr], &mut buffer)?;

        Ok(i16::from_be_bytes(buffer))
    }

    /// Get the hot end and cold junction temperature reading in celcius from the MCP9600.
    pub fn get_temp_c(&mut self) -> Result<(f32, f32), I2CError> {
        let (temp1_raw, temp2_raw) = (self.read_i16(REG_HOT_JUNC)?, self.read_i16(REG_COLD_JUNC)?);

        Ok((
            (temp1_raw as f32) * TEMP_CONV_C,
            (temp2_raw as f32) * TEMP_CONV_C,
        ))
    }
}
