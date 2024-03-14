use rppal::i2c::Error as I2CError;
use rppal::i2c::I2c;

/// Conversion rate between the raw thermocouple value and degrees celcius.
const TEMP_CONV_C: f32 = 0.0625;

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
        // !TODO! make this not hardcoded
        let sensor_cfg = 0b0001_0010; // Use type J thermocouple and 4 filter pass
        let device_cfg = 0b1_01_000_00;

        self.write_u8(0b0000_0101, sensor_cfg)?;
        self.write_u8(0b0000_0110, device_cfg)?;

        Ok(())
    }

    /// Write a single byte to the MCP9600.
    pub fn write_u8(&mut self, addr: u8, val: u8) -> Result<usize, I2CError> {
        let buffer: [u8; 2] = [addr, val];

        self.i2c.write(&buffer)
    }

    /// Read two bytes from the MCP9600.
    pub fn read_u16(&mut self, addr: u8) -> Result<i16, I2CError> {
        let mut buffer: [u8; 2] = [0; 2];

        self.i2c.write_read(&[addr], &mut buffer)?;

        Ok(i16::from_be_bytes(buffer))
    }

    /// Get the hot end and cold end temperature reading in celcius from the MCP9600.
    pub fn get_temp_c(&mut self) -> Result<(f32, f32), I2CError> {
        let (temp1_raw, temp2_raw) = (self.read_u16(0)?, self.read_u16(2)?);

        Ok((
            (temp1_raw as f32) * TEMP_CONV_C,
            (temp2_raw as f32) * TEMP_CONV_C,
        ))
    }
}
