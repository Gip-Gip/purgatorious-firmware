#![doc = include_str!("../README.md")]

use std::{
    fs::{read_dir, remove_file, File},
    io::Write,
    path::Path,
    process::{Child, Command},
    sync::{Arc, Mutex},
    thread::{self, sleep, JoinHandle},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use shared::{URAP_WATCHDOG_PATH, ADDR_INCHASH};
use urap::{URAP_REG_WIDTH, usockets::*};
use watchdog::*;

static EXECDIR: &str = "/opt/firmware/active";
static PIPEDIR: &str = "/var/firmware/";
static LOGFILE: &str = "/opt/firmware/watchdog";

const MAX_LATENCY_MS: u64 = 1000;
const STARTUP_DELAY_MS: u64 = 5000;

fn main() {
    // # 1: Program Initialization
    
    // # 1.1: Initialized URAP Slave

    // remove any broken unix sockets
    for pipe in read_dir(PIPEDIR).unwrap() {
        let pipe = pipe.unwrap().path();
        remove_file(pipe).unwrap();
    }

    let registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; 1]>> =
        Arc::new(Mutex::new([[0; URAP_REG_WIDTH]]));

    let mut urap_slave = UrapSlave::spawn(URAP_WATCHDOG_PATH, registers.clone(), [false]).unwrap();
    
    // ## 1.2: Create Logfile For Watchdog
    let mut logfile = File::create(format!(
        "{}{}.txt",
        LOGFILE,
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs()
    ))
    .unwrap();

    // ## 1.3: Start vPLCs
    let mut execlist = Vec::<Arc<String>>::new();
    let mut pipelist = Vec::<Arc<String>>::new();
    let mut proclist = Vec::<Child>::new();
    let mut hashlist = Vec::<u32>::new();

    for prog in read_dir(EXECDIR).unwrap() {
        let prog = prog.unwrap().path();

        let progpath = prog.to_str().unwrap().to_owned();

        execlist.push(progpath.into());

        let progname = prog.file_name().unwrap().to_str().unwrap();
        pipelist.push(format!("{}/{}", PIPEDIR, progname).into());

        let logname = format!("/opt/firmware/{}", progname);

        let logfile = File::create(format!(
            "{}{}.txt",
            logname,
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs()
        ))
        .unwrap();

        let elogfile = File::create(format!(
            "{}{}_err.txt",
            logname,
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs()
        ))
        .unwrap();

        proclist.push(
            Command::new(prog)
                .stdout(logfile)
                .stderr(elogfile)
                .spawn()
                .unwrap(),
        );

        hashlist.push(0);
    }

    let hashlist = Arc::new(Mutex::new(hashlist));

    // ### 1.3.1: Wait for vPLCs to init
    sleep(Duration::from_millis(STARTUP_DELAY_MS));

    // # 2: Execution Loop
    loop {
        // ## 2.1: Check for frozen vPLCs
        let mut threads =
            Vec::<JoinHandle<Result<(), urap::Error<std::io::Error>>>>::with_capacity(pipelist.len());

        // ### 2.1.1: Spawn threads to check in on vPLCs
        for (i, prog) in pipelist.iter().enumerate() {
            let progname = prog.clone();

            let hlclone = hashlist.clone();

            threads.push(thread::spawn(move || {
                let mut socket = UrapMaster::new(&progname)?;

                let mut hashlist = hlclone.lock().unwrap();

                let inchash = socket.read_u32(ADDR_INCHASH as u16)?;

                if hashlist[i] == inchash {
                    return Err(urap::Error::Io(std::io::Error::new(
                        std::io::ErrorKind::Other,
                        "Process hasn't updated in time!",
                    )));
                }

                hashlist[i] = inchash;

                Ok(())
            }));
        }

        // ### 2.1.2: Delay to ensure all vPLCs have time to update and respond
        sleep(Duration::from_millis(MAX_LATENCY_MS));

        // ### 2.1.3: Check if the threads have finished
        for (i, thread) in threads.iter().enumerate() {
            if !thread.is_finished() {
                let mut registers_lk = registers.lock().unwrap();

                registers_lk[ADDR_ESTOP as usize] = [1; 4];

                drop(registers_lk);

                writeln!(
                    &mut logfile,
                    "*FAULT* HIGH LATENCY IN {}, RESTARTING vPLC!",
                    execlist[i]
                )
                .unwrap();
                proclist[i].kill().unwrap_or_else(|e| {
                    writeln!(&mut logfile, "*ERR* Unable to kill process! {}", e).unwrap();
                });

                if Path::new(pipelist[i].as_str()).exists() {
                    remove_file(pipelist[i].as_str()).unwrap_or_else(|e| {
                        writeln!(&mut logfile, "*ERR* Unable to remove pipe! {}", e).unwrap();
                    });
                }

                let logname = format!(
                    "/opt/firmware/{}",
                    Path::new(execlist[i].as_str())
                        .file_name()
                        .unwrap()
                        .to_str()
                        .unwrap()
                );

                let logfile = File::create(format!(
                    "{}{}.txt",
                    logname,
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs()
                ))
                .unwrap();

                let elogfile = File::create(format!(
                    "{}{}_err.txt",
                    logname,
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs()
                ))
                .unwrap();

                proclist[i] = Command::new(execlist[i].as_str())
                    .stdout(logfile)
                    .stderr(elogfile)
                    .spawn()
                    .unwrap();

                sleep(Duration::from_millis(STARTUP_DELAY_MS));
            }
        }

        // ### 2.1.4 Check to see if any of the threads threw an error
        let threads: Vec<Result<(), urap::Error<std::io::Error>>> = threads
            .into_iter()
            .map(|thread| thread.join().unwrap())
            .collect();

        for (i, thread) in threads.iter().enumerate() {
            match thread {
                Ok(()) => {}
                Err(e) => {
                    let mut registers_lk = registers.lock().unwrap();

                    registers_lk[ADDR_ESTOP as usize] = [1; 4];

                    drop(registers_lk);

                    writeln!(
                        &logfile,
                        "{}\n*FAULT* UNABLE TO CHECK IN ON {}, RESTARTING vPLC!",
                        e, execlist[i]
                    )
                    .unwrap();

                    proclist[i].kill().unwrap_or_else(|e| {
                        writeln!(&mut logfile, "*ERR* Unable to kill process! {}", e).unwrap();
                    });

                    if Path::new(pipelist[i].as_str()).exists() {
                        remove_file(pipelist[i].as_str()).unwrap_or_else(|e| {
                            writeln!(&mut logfile, "*ERR* Unable to remove pipe! {}", e).unwrap();
                        });
                    }

                    let logname = format!(
                        "/opt/firmware/{}",
                        Path::new(execlist[i].as_str())
                            .file_name()
                            .unwrap()
                            .to_str()
                            .unwrap()
                    );

                    let logfile = File::create(format!(
                        "{}{}.txt",
                        logname,
                        SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap()
                            .as_secs()
                    ))
                    .unwrap();

                    let elogfile = File::create(format!(
                        "{}{}_err.txt",
                        logname,
                        SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap()
                            .as_secs()
                    ))
                    .unwrap();

                    proclist[i] = Command::new(execlist[i].as_str())
                        .stdout(logfile)
                        .stderr(elogfile)
                        .spawn()
                        .unwrap();

                    sleep(Duration::from_millis(STARTUP_DELAY_MS));
                }
            }
        }

        // ## 2.2 Check the URAP slave for errors
        if let Some(e) = urap_slave.pop_error() {
            let mut registers_lk = registers.lock().unwrap();
            registers_lk[ADDR_ESTOP as usize] = [1; 4];
            drop(registers_lk);

            writeln!(logfile, "*FAULT* URAP encountered error {}", e).unwrap();
        }
    }
}
