//! Driver for Quantum III motor drivers over UART/RS-485

use std::{
    error::Error as ErrorT, fmt::Display, num::ParseIntError, thread::sleep, time::Duration,
};

use enum_primitive_derive::Primitive;
use num_traits::*;

use rppal::uart::{self, Parity, Uart};

const BCC_START_INDEX: usize = 1;
const DATA_START_INDEX: usize = 5;

// 100 speed = 175 rpm motor, 175rpm motor = 15.625rpm screw
// 100 speed = 15.625rpm screw
const TO_RPM_MULT: f32 = 15.625 / 100.0;
const MOTOR_TO_SCREW_RPM_MULT: f32 = 15.625 / 175.0;
const TO_SPEED_MULT: f32 = 100.0 / 15.625;

static PARAM_SET_SPEED: &str = "0001";
static PARAM_ZERO_PAGE_START: &str = "0001";
static PARAM_PERMISSIONS: &str = "0100";
static PARAM_EXTERNAL_TRIP: &str = "1034";
static PARAM_ESTOP_BUTTON: &str = "1521";
static PARAM_MOTOR_FAN: &str = "1522";
static PARAM_RESET: &str = "1035";
const PARAM_RESET_VAL: i16 = 255;

const UART_TIMEOUT_MS: u64 = 100;
const RESET_WAIT_MS: u64 = 100;

fn bcc(data: &[u8]) -> u8 {
    let mut xor: u8 = 0;

    for byte in data {
        xor ^= byte;
    }

    if xor < 32 {
        xor += 32;
    }

    xor
}

#[repr(u8)]
#[derive(Primitive, Clone, Debug, PartialEq)]
pub enum Q3TripCode {
    NoFault = 0,
    Unknown = 1,
    HardwareFault = 100,
    PhaseSequence = 101,
    ExternalTrip = 102,
    ExternalPowerSupply = 103,
    CurrentLoopOpen = 104,
    SerialLost = 105,
    FieldOvercurrent = 106,
    DriveOvertemp = 107,
    FieldOn = 108,
    FeedbackReversal = 109,
    ThermalShort = 110,
    FieldLoss = 118,
    FeedbackLoss = 119,
    SupplyLoss = 120,
    ArmatureOvercurrent = 121,
    CurrentOverTime = 122,
    MotorOvertemp = 123,
    P1Watchdog = 124,
    PowerSupply = 125,
    ArmatureOpen = 126,
    P2Watchdog = 131,
    EEprom = 132,
}

pub struct ZeroPage {
    offset: i16,
    rpm: i16,
    dc_voltage: i16,
    dc_current: i16,
    line_voltage: i16,
    drive_enabled: bool,
    trip_history: [Q3TripCode; 3],
    drive_ok: bool,
}

impl ZeroPage {
    #[inline]
    pub fn drive_enabled(&self) -> bool {
        self.drive_enabled
    }

    #[inline]
    pub fn drive_ok(&self) -> bool {
        self.drive_ok
    }

    #[inline]
    pub fn get_trip_history(&self) -> Vec<Q3TripCode> {
        self.trip_history.to_vec()
    }

    #[inline]
    pub fn get_set_screw_rpm(&self) -> f32 {
        self.offset as f32 * TO_RPM_MULT
    }

    #[inline]
    pub fn get_act_screw_rpm(&self) -> f32 {
        self.rpm as f32 * MOTOR_TO_SCREW_RPM_MULT
    }

    #[inline]
    pub fn get_act_motor_rpm(&self) -> f32 {
        self.rpm as f32
    }

    #[inline]
    pub fn get_motor_v(&self) -> f32 {
        self.dc_voltage as f32
    }

    #[inline]
    pub fn get_motor_a(&self) -> f32 {
        self.dc_current as f32
    }

    #[inline]
    pub fn get_line_v(&self) -> f32 {
        self.line_voltage as f32
    }
}

#[derive(Debug)]
pub enum Error {
    Uart(uart::Error),
    ParseError(ParseIntError),
    Garbage(u8),
    BadChecksum(u8, u8),
    Corrupt,
    Nak,
}

impl Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Uart(e) => e.fmt(f),
            Self::ParseError(e) => e.fmt(f),
            Self::Garbage(resp) => write!(f, "Garbage response {}", resp),
            Self::BadChecksum(calc, rcvd) => {
                write!(f, "Bad checksum, calc'd {} got {}", calc, rcvd)
            }
            Self::Corrupt => write!(f, "Incorrectly formatted or corrupt data"),
            Self::Nak => write!(f, "NAK recieved"),
        }
    }
}

impl ErrorT for Error {}

impl From<uart::Error> for Error {
    fn from(value: uart::Error) -> Self {
        Self::Uart(value)
    }
}

impl From<ParseIntError> for Error {
    fn from(value: ParseIntError) -> Self {
        Self::ParseError(value)
    }
}

pub struct QuantumIII {
    uart: Uart,
    address: String,
}

impl QuantumIII {
    pub fn new(path: &str, address: u8) -> Result<Self, uart::Error> {
        let mut address_string = String::with_capacity(4);

        for digit in format!("{:02}", address).chars().into_iter() {
            address_string.push(digit);
            address_string.push(digit);
        }

        assert_eq!(address_string.len(), 4);

        let mut uart = Uart::with_path(path, 9_600, Parity::Even, 7, 1)?;

        uart.set_read_mode(0, Duration::from_millis(UART_TIMEOUT_MS))?;
        uart.set_write_mode(true)?;

        uart.flush(uart::Queue::Both)?;

        Ok(Self {
            address: address_string,
            uart,
        })
    }

    pub fn init(&mut self) -> Result<(), Error> {
        self.reduce_perms()
    }

    pub fn write_param(&mut self, param: &str, data: i16) -> Result<(), Error> {
        let mut buf: Vec<u8> = Vec::with_capacity(20);

        buf.push(0x04); // EOT

        buf.extend_from_slice(self.address.as_bytes()); // Address

        buf.push(0x02); // STX

        // BCC is calculated past this point
        let bcc_index = buf.len();

        assert_eq!(param.len(), 4);

        buf.extend_from_slice(param.as_bytes());

        let data_str = format!("{:+}", data);
        buf.extend_from_slice(data_str.as_bytes());

        buf.push(0x03); // ETX

        let checksum = bcc(&buf[bcc_index..]);

        buf.push(checksum);

        self.uart.write(&buf)?;

        let mut buf: [u8; 1] = [0; 1];

        self.uart.read(&mut buf)?;

        match buf[0] {
            0x15 => Err(Error::Nak),
            0x06 => Ok(()),
            _ => Err(Error::Garbage(buf[0])),
        }
    }

    pub fn read_param(&mut self, param: &str) -> Result<i16, Error> {
        let mut buf: Vec<u8> = Vec::with_capacity(10);

        buf.push(0x04); // EOT

        buf.extend_from_slice(self.address.as_bytes());

        buf.extend_from_slice(param.as_bytes());

        buf.push(0x05); // ENQ

        self.uart.write(&buf)?;

        self.parse_param()
    }

    pub fn read_next_param(&mut self) -> Result<i16, Error> {
        let buf: [u8; 1] = [0x06; 1]; // ACK

        self.uart.write(&buf)?;

        self.parse_param()
    }

    fn parse_param(&mut self) -> Result<i16, Error> {
        let mut buf: [u8; 12] = [0; 12];

        self.uart.read(&mut buf)?;

        match buf[0] {
            0x15 => {
                return Err(Error::Nak);
            }
            0x02 => {} // STX
            _ => {
                return Err(Error::Garbage(buf[0]));
            }
        }

        let data_end_index: usize = match &buf.iter().position(|x| *x == 0x03) {
            Some(i) => *i,
            None => {
                return Err(Error::Corrupt);
            }
        };

        // Include ETX
        let checksum_calc = bcc(&buf[BCC_START_INDEX..=data_end_index]);

        let checksum_rcvd = buf[data_end_index + 1];

        if checksum_calc != checksum_rcvd {
            Err(Error::BadChecksum(checksum_calc, checksum_rcvd))
        } else {
            let data: String =
                String::from_utf8_lossy(&buf[DATA_START_INDEX..data_end_index]).to_string();
            Ok(data.parse()?)
        }
    }

    // When there's a fault you need let the drive think about what it's done,
    // and reset the buffer...
    pub fn fault_reset(&mut self) -> Result<(), Error> {
        sleep(Duration::from_millis(RESET_WAIT_MS));
        self.uart.flush(uart::Queue::Both)?;

        Ok(())
    }

    #[inline]
    pub fn set_screw_rpm(&mut self, rpm: f32) -> Result<(), Error> {
        let speed = rpm * TO_SPEED_MULT;

        self.write_param(PARAM_SET_SPEED, speed as i16)
    }

    pub fn read_zero_page(&mut self) -> Result<ZeroPage, Error> {
        Ok(ZeroPage {
            offset: self.read_param(PARAM_ZERO_PAGE_START)?,
            rpm: self.read_next_param()?,
            dc_voltage: self.read_next_param()?,
            dc_current: self.read_next_param()?,
            line_voltage: self.read_next_param()?,
            drive_enabled: self.read_next_param()? > 0,
            trip_history: [
                Q3TripCode::from_i16(self.read_next_param()?).unwrap_or(Q3TripCode::Unknown),
                Q3TripCode::from_i16(self.read_next_param()?).unwrap_or(Q3TripCode::Unknown),
                Q3TripCode::from_i16(self.read_next_param()?).unwrap_or(Q3TripCode::Unknown),
            ],
            drive_ok: self.read_next_param()? > 0,
        })
    }

    #[inline]
    pub fn estop_depressed(&mut self) -> Result<bool, Error> {
        Ok(self.read_param(PARAM_ESTOP_BUTTON)? != 0)
    }

    #[inline]
    pub fn elevate_perms(&mut self) -> Result<(), Error> {
        self.write_param(PARAM_PERMISSIONS, 200)
    }

    #[inline]
    pub fn reduce_perms(&mut self) -> Result<(), Error> {
        self.write_param(PARAM_PERMISSIONS, 149)
    }

    pub fn trip(&mut self) -> Result<(), Error> {
        // Only trip if we need to,
        if self.read_zero_page()?.drive_ok {
            self.elevate_perms()?;
            self.write_param(PARAM_EXTERNAL_TRIP, 1)?;
            self.reduce_perms()?;
        }

        Ok(())
    }

    /// Ensure to not contact the drive for a minimum of 5 seconds after resetting
    pub fn reset_drive(&mut self) -> Result<(), Error> {
        self.elevate_perms()?;
        // Do not forget to clear the trip
        self.write_param(PARAM_EXTERNAL_TRIP, 0)?;
        self.write_param(PARAM_RESET, PARAM_RESET_VAL)
        // We will now no longer expect communication from the drive
    }

    #[inline]
    pub fn set_motor_fan(&mut self, on: bool) -> Result<(), Error> {
        let val = match on {
            true => 1,
            false => 0,
        };

        self.write_param(PARAM_MOTOR_FAN, val)
    }
}
