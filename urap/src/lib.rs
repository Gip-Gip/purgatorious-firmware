use std::{
    io::{Read, Write},
    net::Shutdown,
    os::unix::net::{UnixListener, UnixStream},
    sync::{Arc, Mutex},
    thread::{self, JoinHandle},
};

pub const URAP_REG_WIDTH: usize = 4;
pub const URAP_ADDR_WIDTH: usize = 2;
pub const URAP_WRITE_OR: u16 = 0x8000;

pub const ADDR_INCHASH: usize = 0x0000;

pub struct UrapSlave {}

impl UrapSlave {
    pub fn spawn<const REGCNT: usize>(
        address: &str,
        registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; REGCNT]>>,
        writeprotect: [bool; REGCNT],
    ) -> Result<JoinHandle<Result<(), std::io::Error>>, std::io::Error> {
        let listener = UnixListener::bind(address)?;

        Ok(thread::spawn(move || loop {
            for stream in listener.incoming() {
                match stream {
                    Ok(mut stream) => {
                        let regcopy = registers.clone();
                        thread::spawn(move || {
                            let mut buffer: [u8; URAP_ADDR_WIDTH] = [0; URAP_ADDR_WIDTH];

                            while stream.read(&mut buffer).unwrap_or(0) == buffer.len() {
                                let register = u16::from_ne_bytes(buffer);

                                let mut registers = regcopy.lock().unwrap();

                                match (register & URAP_WRITE_OR) != 0 {
                                    true => {
                                        let register = (register ^ URAP_WRITE_OR) as usize;

                                        let mut buf: [u8; URAP_REG_WIDTH] = [0; URAP_REG_WIDTH];
                                        stream.read_exact(&mut buf).unwrap_or_default();

                                        if !writeprotect[register] {
                                            registers[register] = buf;
                                        }
                                    }
                                    false => {
                                        let register = register as usize;
                                        stream.write_all(&registers[register]).unwrap_or_default();
                                    }
                                }
                            }
                        });
                    }
                    Err(_) => {}
                }
            }
        }))
    }
}

pub struct UrapMaster {
    socket: UnixStream,
}

impl UrapMaster {
    pub fn new(name: &str) -> Result<Self, std::io::Error> {
        let socket = UnixStream::connect(name)?;

        Ok(Self { socket })
    }

    pub fn read_4u8(&mut self, register: u16) -> Result<[u8; 4], std::io::Error> {
        let mut buf: [u8; URAP_REG_WIDTH] = [0; URAP_REG_WIDTH];

        self.socket.write_all(&register.to_ne_bytes())?;

        self.socket.read_exact(&mut buf)?;

        Ok(buf)
    }

    #[inline]
    pub fn read_f32(&mut self, register: u16) -> Result<f32, std::io::Error> {
        Ok(f32::from_ne_bytes(self.read_4u8(register)?))
    }

    #[inline]
    pub fn read_u32(&mut self, register: u16) -> Result<u32, std::io::Error> {
        Ok(u32::from_ne_bytes(self.read_4u8(register)?))
    }

    pub fn write_4u8(&mut self, register: u16, data: [u8; 4]) -> Result<(), std::io::Error> {
        let mut buf: [u8; URAP_ADDR_WIDTH + URAP_REG_WIDTH] = [0; URAP_ADDR_WIDTH + URAP_REG_WIDTH];

        buf[..URAP_ADDR_WIDTH].copy_from_slice(&(register | URAP_WRITE_OR).to_ne_bytes());
        buf[URAP_ADDR_WIDTH..].copy_from_slice(&data);

        self.socket.write_all(&buf)?;

        Ok(())
    }

    #[inline]
    pub fn write_f32(&mut self, register: u16, data: f32) -> Result<(), std::io::Error> {
        self.write_4u8(register, data.to_ne_bytes())
    }

    #[inline]
    pub fn write_u32(&mut self, register: u16, data: u32) -> Result<(), std::io::Error> {
        self.write_4u8(register, data.to_ne_bytes())
    }
}

impl Drop for UrapMaster {
    fn drop(&mut self) {
        self.socket.shutdown(Shutdown::Both).unwrap_or_default();
    }
}
