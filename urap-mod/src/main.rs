//! Utility for interfacing with vPLCs via URAP

use std::{thread::sleep, time::Duration};

use clap::{Parser, ValueEnum};
use urap::usockets::*;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    /// Socket to connect to
    socket: String,

    /// The register to address
    #[arg(short, long, value_name = "register")]
    register: u16,

    /// Allow for continuous monitoring
    #[arg(short, long)]
    continuous: bool,

    /// Write data to the register
    #[arg(short, long, value_name = "data")]
    write: Option<String>,

    /// Set the type of the register
    #[arg(short, long, value_name = "type")]
    type_: UrapTypes,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
enum UrapTypes {
    U32,
    F32,
}

fn main() {
    let cli = Cli::parse();

    let mut urap = UrapMaster::new(&cli.socket).unwrap();

    if let Some(data) = cli.write {
        let data_str = match cli.type_ {
            UrapTypes::U32 => urap.read_u32(cli.register).unwrap().to_string(),
            UrapTypes::F32 => urap.read_f32(cli.register).unwrap().to_string(),
        };

        println!("{:#04x}->{}", cli.register, data_str);

        let buf = match cli.type_ {
            UrapTypes::U32 => {
                let data: u32 = data.parse().unwrap();

                data.to_ne_bytes()
            }
            UrapTypes::F32 => {
                let data: f32 = data.parse().unwrap();

                data.to_ne_bytes()
            }
        };

        println!("{:#04x}<-{}", cli.register, data);
        urap.write_4u8(cli.register, &buf).unwrap();
    }

    loop {
        let data_str = match cli.type_ {
            UrapTypes::U32 => urap.read_u32(cli.register).unwrap().to_string(),
            UrapTypes::F32 => urap.read_f32(cli.register).unwrap().to_string(),
        };

        print!("\r{:#04x}->{}", cli.register, data_str);

        if !cli.continuous {
            break;
        }

        sleep(Duration::from_millis(100));
    }

    println!("");
}
