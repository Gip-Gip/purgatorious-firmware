#![cfg_attr(not(feature = "std"), no_std)]

#![cfg_attr(feature="usockets", feature(unix_socket_peek))]
#[cfg(feature = "usockets")]
pub mod usockets;

use core::fmt::Display;

use embedded_io::{ErrorType, Read, ReadExactError, Write};
use crc::{Algorithm, Crc};

pub const URAP_REG_WIDTH: usize = 4;
pub const URAP_CRC_WIDTH: usize = 1;
pub const URAP_ADDR_WIDTH: usize = 2;
pub const URAP_ACK_WIDTH: usize = 1;
pub const URAP_WRITE_OR: u16 = 0x8000;

pub static CRC_8F_6: Algorithm<u8> = Algorithm {
    width: 8,
    poly: 0x9b & 1, // The notes specify that the polynominals imply the 1 bit is set
    init: 0x00,
    refin: false,
    refout: false,
    xorout: 0x00,
    check: 0x0C,
    residue: 0x00,
};


pub static ACK: [u8; 1] = [0xAA];
pub static NAK: [u8; 1] = [0x00];

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy)]
pub enum Error<E> {
    Io(E),
    Nak,
    BadCrc(u8, u8),
    OutOfBounds(u16),
    IncompletePacket,
    IndexWriteProtected(u16),
}

impl<E> Display for Error<E>
where E: Display {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{}", e),
            Error::Nak => write!(f, "NAK Recieved"),
            Error::BadCrc(calculated, provided) => write!(f, "Bad Crc, calc'd {:x} provided {:x}", calculated, provided),
            Error::OutOfBounds(index) => write!(f, "Attempted to access index {}, which is out of bounds", index),
            Error::IncompletePacket => write!(f, "Incomplete Packet"),
            Error::IndexWriteProtected(index) => write!(f, "Attempted to write to index {}, which is protected", index),
        }
    }
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Error::Io(value)
    }
}

impl<E> From<ReadExactError<E>> for Error<E> {
    fn from(value: ReadExactError<E>) -> Self {
        match value {
            ReadExactError::UnexpectedEof => Error::IncompletePacket,
            ReadExactError::Other(e) => Error::Io(e),
        }
    }
}

#[cfg(feature = "std")]
pub struct StdIo<IO>
where IO: std::io::Read + std::io::Write {
    io: IO,
}

impl<IO> StdIo<IO>
where IO: std::io::Read + std::io::Write {
    #[inline]
    pub fn get_inner(&mut self) -> &IO {
        &self.io
    }

    #[inline]
    pub fn get_inner_mut(&mut self) -> &mut IO {
        &mut self.io
    }
}

impl<IO> From<IO> for StdIo<IO>
where IO: std::io::Read + std::io::Write {
    #[inline]
    fn from(value: IO) -> Self {
        Self { io: value }
    }
}

impl<IO> ErrorType for StdIo<IO>
where IO: std::io::Read + std::io::Write {
    type Error = std::io::Error;
}

impl<IO> Read for StdIo<IO>
where IO: std::io::Read + std::io::Write {
    #[inline]
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.io.read(buf)
    }
}

impl<IO> Write for StdIo<IO>
where IO: std::io::Read + std::io::Write {
    #[inline]
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.io.write(buf)
    }

    #[inline]
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.io.flush()
    }
}

pub struct UrapSlave<'a, 'b, 'c, IO, const REGCNT: usize>
where IO: Read + Write {
    io: &'a mut IO,
    registers: &'b mut [[u8; URAP_REG_WIDTH]; REGCNT],
    writeprotect: &'c [bool; REGCNT],
}

impl<'a, 'b, 'c, IO, const REGCNT: usize> UrapSlave<'a, 'b, 'c, IO, REGCNT>
where IO: Read + Write {
    pub fn new(io: &'a mut IO, registers: &'b mut [[u8; URAP_REG_WIDTH]; REGCNT], writeprotect: &'c [bool; REGCNT]) -> Self {
        Self { io, registers, writeprotect }
    }

    pub fn poll(&mut self) -> Result<(), Error<IO::Error>> {
        let mut crc_in: [u8; URAP_CRC_WIDTH] = [0; URAP_CRC_WIDTH];
        let mut reg_buffer: [u8; URAP_ADDR_WIDTH] = [0; URAP_ADDR_WIDTH];

        self.io.read_exact(&mut crc_in)?;
        self.io.read_exact(&mut reg_buffer)?;
        let crc_alg = Crc::<u8>::new(&CRC_8F_6);
        let mut crc_calc = crc_alg.digest();

        crc_calc.update(&reg_buffer);

        let register = u16::from_le_bytes(reg_buffer);

        let registers = &mut self.registers;

        let write = (register & URAP_WRITE_OR) != 0;
        let register = (register & (URAP_WRITE_OR ^ u16::MAX)) as usize;

        if register >= registers.len() {
            self.clear()?;
            self.nak()?;
            return Err(Error::OutOfBounds(register as u16));
        }

        match write {
            true => {
                let mut data_buffer: [u8; URAP_REG_WIDTH] = [0; URAP_REG_WIDTH];
                match self.io.read_exact(&mut data_buffer) {
                    Ok(_) => { }
                    Err(e) => {
                        self.clear()?;
                        self.nak()?;
                        return Err(e.into());
                    }
                }

                crc_calc.update(&data_buffer);

                let crc_calc = crc_calc.finalize();

                if crc_in[0] != crc_calc {
                        self.clear()?;
                        self.nak()?;
                        return Err(Error::BadCrc(crc_calc, crc_in[0]));
                }

                if !self.writeprotect[register] {
                    registers[register] = data_buffer;
                    self.ack()?;
                }
                else {
                    self.nak()?;
                    return Err(Error::IndexWriteProtected(register as u16));
                }
            }
            false => {
                let crc_calc = crc_calc.finalize();
                if crc_in[0] != crc_calc {
                        self.clear()?;
                        self.nak()?;
                        return Err(Error::BadCrc(crc_calc, crc_in[0]));
                }

                let register_val = registers[register].clone();
                let out_crc = Crc::<u8>::new(&CRC_8F_6).checksum(&register_val);

                self.ack()?;
                self.io.write_all(&[out_crc])?;
                self.io.write_all(&register_val)?;
            } 
        }
        Ok(())
    }

    fn clear(&mut self) -> Result<(), IO::Error> {
        let mut buffer: [u8; URAP_ADDR_WIDTH] = [0; URAP_ADDR_WIDTH];
        while self.io.read(&mut buffer).unwrap_or(0) == buffer.len() { }
        Ok(())
    }

    fn ack(&mut self) -> Result<(), IO::Error> {
        self.io.write_all(&ACK)
    }

    fn nak(&mut self) -> Result<(), IO::Error> {
        self.io.write_all(&NAK)
    }
}

pub struct UrapMaster<'a, IO>
where IO: Read + Write {
    io: &'a mut IO,
}

impl<'a, IO> UrapMaster<'a, IO>
where IO: Read + Write {
    pub fn new(io: &'a mut IO) -> Self {
        Self{ io }
    }

    pub fn read_4u8(&mut self, register: u16) -> Result<[u8; 4], Error<IO::Error>> {
        assert_eq!(register & URAP_WRITE_OR, 0);
        let register = register.to_le_bytes();
        let crc = Crc::<u8>::new(&CRC_8F_6).checksum(&register);

        self.io.write_all(&[crc])?;
        self.io.write_all(&register)?;

        let mut ack_or_nak: [u8; 1] = [0; 1];
        let mut crc_buf: [u8; 1] = [0; 1];
        let mut buffer: [u8; 4] = [0; 4];

        self.io.read_exact(&mut ack_or_nak)?;

        if ack_or_nak != ACK {
            return Err(Error::Nak);
        }

        self.io.read_exact(&mut crc_buf)?;
        self.io.read_exact(&mut buffer)?;

        let crc_calc = Crc::<u8>::new(&CRC_8F_6).checksum(&buffer);

        if crc_calc != crc_buf[0] {
            return Err(Error::BadCrc(crc_calc, crc_buf[0]));
        }

        return Ok(buffer)
    }

    #[inline]
    pub fn read_f32(&mut self, register: u16) -> Result<f32, Error<IO::Error>> {
        Ok(f32::from_le_bytes(self.read_4u8(register)?))
    }

    #[inline]
    pub fn read_u32(&mut self, register: u16) -> Result<u32, Error<IO::Error>> {
        Ok(u32::from_le_bytes(self.read_4u8(register)?))
    }
    
    #[inline]
    pub fn read_i32(&mut self, register: u16) -> Result<i32, Error<IO::Error>> {
        Ok(i32::from_le_bytes(self.read_4u8(register)?))
    }

    pub fn write_4u8(&mut self, register: u16, buffer: &[u8; URAP_REG_WIDTH]) -> Result<(), Error<IO::Error>> {
        assert_eq!(register & URAP_WRITE_OR, 0);
        let register = (register | URAP_WRITE_OR).to_le_bytes();

        let crc_alg = Crc::<u8>::new(&CRC_8F_6);
        let mut crc_digest = crc_alg.digest();

        crc_digest.update(&register);
        crc_digest.update(buffer);

        let crc: [u8; 1] = [crc_digest.finalize()];

        self.io.write_all(&crc)?;
        self.io.write_all(&register)?;
        self.io.write_all(buffer)?;

        let mut ack_or_nak: [u8; 1] = [0];

        self.io.read_exact(&mut ack_or_nak)?;

        if ack_or_nak != ACK {
            return Err(Error::Nak);
        }

        Ok(())
    }

    #[inline]
    pub fn write_f32(&mut self, register: u16, num: f32) -> Result<(), Error<IO::Error>> {
        self.write_4u8(register, &num.to_le_bytes())
    }
    
    #[inline]
    pub fn write_u32(&mut self, register: u16, num: u32) -> Result<(), Error<IO::Error>> {
        self.write_4u8(register, &num.to_le_bytes())
    }
    
    #[inline]
    pub fn write_i32(&mut self, register: u16, num: i32) -> Result<(), Error<IO::Error>> {
        self.write_4u8(register, &num.to_le_bytes())
    }

    #[inline]
    pub fn is_healthy(&mut self) -> bool {
        match self.read_4u8(0) {
            Ok(_) => true,
            Err(_) => false,
        }
    }
}
