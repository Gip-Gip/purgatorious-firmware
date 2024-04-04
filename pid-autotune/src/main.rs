//! Derived from https://github.com/hirschmann/pid-autotune/blob/master/autotune.py

use clap::{Parser, ValueEnum};
use pid::Pid;
use std::{
    f32::consts::PI,
    time::{Duration, Instant},
};

use i2c::*;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};
use shared::*;
use thermo::*;
use urap::UrapMaster;

const PEAK_AMPLITUDE_TOLERANCE: f32 = 0.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
enum Zones {
    Z1,
    Z2,
    Z3,
    Z4,
    Z5,
    Z6,
}

impl Into<(u16, u16)> for Zones {
    fn into(self) -> (u16, u16) {
        match self {
            Self::Z1 => (ADDR_T1_C as u16, ADDR_Z1_PWR as u16),
            Self::Z2 => (ADDR_T2_C as u16, ADDR_Z2_PWR as u16),
            Self::Z3 => (ADDR_T3_C as u16, ADDR_Z3_PWR as u16),
            Self::Z4 => (ADDR_T4_C as u16, ADDR_Z4_PWR as u16),
            Self::Z5 => (ADDR_T5_C as u16, ADDR_Z5_PWR as u16),
            Self::Z6 => (ADDR_T6_C as u16, ADDR_Z6_PWR as u16),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
enum PIDAutotuneState {
    RelayStepUp,
    RelayStepDown,
    Succeeded,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
enum PeakType {
    Maxima,
    Minima,
    Neither,
}

#[derive(Debug, Clone, Copy)]
enum TuningRule {
    ZeiglerNichols,
    TyreusLuyben,
    CianconeMarlin,
    PessenIntergral,
    SomeOvershoot,
    NoOvershoot,
    Brewing,
    HighMass,
    Zone6,
    Marlin,
    Flat,
}

impl Into<(f32, f32, f32)> for TuningRule {
    fn into(self) -> (f32, f32, f32) {
        match self {
            TuningRule::ZeiglerNichols => (34.0, 40.0, 160.0),
            TuningRule::TyreusLuyben => (44.0, 9.0, 126.0),
            TuningRule::CianconeMarlin => (66.0, 88.0, 162.0),
            TuningRule::PessenIntergral => (28.0, 50.0, 133.0),
            TuningRule::SomeOvershoot => (60.0, 40.0, 60.0),
            TuningRule::NoOvershoot => (100.0, 40.0, 60.0),
            TuningRule::Brewing => (2.5, 6.0, 380.0),
            TuningRule::HighMass => (1.839, 8.645, 6.349),
            TuningRule::Zone6 => (1.839, 8.645, 6.349),
            TuningRule::Marlin => (1.7, 0.5, 8.0),
            TuningRule::Flat => (1.0, 1.0, 1.0),
        }
    }
}

struct PIDAutotuner {
    inputs: Vec<f32>,
    sample_time_ms: u64,
    lookback_ms: u64,
    setpoint: f32,
    output_step: f32,
    noiseband: f32,
    out_min: f32,
    out_max: f32,
    state: PIDAutotuneState,
    /// (time, peak)
    peaks: Vec<(Instant, f32)>,
    output: f32,
    last_run_timestamp: Instant,
    peak_type: PeakType,
    peak_count: u32,
    initial_output: f32,
    induced_amplitude: f32,
    ku: f32,
    pu_ms: f32,
}

impl PIDAutotuner {
    pub fn new(
        setpoint: f32,
        output_step: f32,
        sample_time_ms: u64,
        lookback_ms: u64,
        out_min: f32,
        out_max: f32,
        noiseband: f32,
    ) -> Self {
        let (inputs, peaks, output, peak_count, initial_output, induced_amplitude, ku, pu_ms) =
            Default::default();

        Self {
            setpoint,
            output_step,
            sample_time_ms,
            lookback_ms,
            out_min,
            out_max,
            noiseband,
            inputs,
            peaks,
            peak_type: PeakType::Neither,
            state: PIDAutotuneState::RelayStepUp,
            output,
            last_run_timestamp: Instant::now(),
            peak_count,
            initial_output,
            induced_amplitude,
            ku,
            pu_ms,
        }
    }

    #[inline]
    pub fn get_state(&self) -> PIDAutotuneState {
        self.state
    }

    #[inline]
    pub fn get_output(&self) -> f32 {
        self.output
    }

    pub fn run(&mut self, input_val: f32) -> PIDAutotuneState {
        let now = Instant::now();

        let next_sample = self
            .last_run_timestamp
            .checked_add(Duration::from_millis(self.sample_time_ms))
            .unwrap();

        if !next_sample.saturating_duration_since(now).is_zero() {
            return self.get_state();
        }

        self.last_run_timestamp = now;

        if self.state == PIDAutotuneState::RelayStepUp && input_val > self.setpoint + self.noiseband
        {
            self.state = PIDAutotuneState::RelayStepDown;
        } else if self.state == PIDAutotuneState::RelayStepDown
            && input_val < self.setpoint - self.noiseband
        {
            self.state = PIDAutotuneState::RelayStepUp;
        }

        self.output = match self.state {
            PIDAutotuneState::RelayStepUp => self.initial_output + self.output_step,
            PIDAutotuneState::RelayStepDown => self.initial_output - self.output_step,
            _ => {
                return self.get_state();
            }
        };

        self.output = self.out_min.max(self.out_max.min(self.output));

        self.inputs.push(input_val);

        if self.inputs.len() != (self.lookback_ms / self.sample_time_ms) as usize {
            return self.get_state();
        }

        self.inputs.remove(0);

        let (mut is_max, mut is_min) = (true, true);

        for val in &self.inputs {
            is_max = is_max && input_val >= *val;
            is_min = is_min && input_val <= *val;
        }

        let mut infliction = false;

        if is_max {
            if self.peak_type == PeakType::Minima {
                infliction = true;
            }
            self.peak_type = PeakType::Maxima
        } else if is_min {
            if self.peak_type == PeakType::Maxima {
                infliction = true;
            }
            self.peak_type = PeakType::Minima
        }

        if infliction {
            self.peak_count += 1;
            self.peaks.push((now, input_val));
        }

        self.induced_amplitude = 0.0;

        if infliction && self.peak_count > 4 {
            let (_, mut abs_max) = self.peaks[self.peaks.len() - 2];
            let mut abs_min = abs_max;

            for i in 0..self.peaks.len() - 2 {
                let (_, peak1) = self.peaks[i];
                let (_, peak2) = self.peaks[i + 1];

                self.induced_amplitude += (peak1 - peak2).abs();

                abs_max = peak1.max(abs_max);
                abs_min = peak1.min(abs_min);
            }

            self.induced_amplitude /= 6.0;

            let amplitude_dev =
                (0.5 * (abs_max - abs_min) - self.induced_amplitude) / self.induced_amplitude;

            if amplitude_dev < PEAK_AMPLITUDE_TOLERANCE {
                self.state = PIDAutotuneState::Succeeded;
            }
        }

        if self.peaks.len() >= 20 {
            self.output = 0.0;
            self.state = PIDAutotuneState::Failed;
        }

        if self.state == PIDAutotuneState::Succeeded {
            self.output = 0.0;

            self.ku = 4.0 * self.output_step / (self.induced_amplitude * PI);

            let (peak1, _) = self.peaks[1];
            let (peak2, _) = self.peaks[2];
            let (peak3, _) = self.peaks[3];
            let (peak4, _) = self.peaks[4];

            let period1 = peak3.checked_duration_since(peak1).unwrap();
            let period2 = peak4.checked_duration_since(peak2).unwrap();

            self.pu_ms = 0.5 * (period1.as_millis() as f32 + period2.as_millis() as f32);
        }

        self.get_state()
    }

    pub fn get_kpid(&self, tuning_rule: TuningRule) -> (f32, f32, f32) {
        let (pdiv, idiv, ddiv): (f32, f32, f32) = tuning_rule.into();

        let pu_s = self.pu_ms / 1000.0;

        let kp = self.ku / pdiv;
        let ki = kp / (pu_s / idiv);
        let kd = kp * (pu_s / ddiv);

        (kp, ki, kd)
    }
}

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    /// Zone to tune
    zone: Zones,
    /// The setpoint to tune at in c
    #[arg(short, long, value_name = "setpoint_c")]
    setpoint_c: f32,
    /// The output step
    #[arg(short, long, value_name = "output_step")]
    output_step: f32,
    /// The sample time in ms
    #[arg(short, long, value_name = "sample_time_ms")]
    sample_time_ms: u64,
    /// The lookback time in ms
    #[arg(short, long, value_name = "lookback_ms")]
    lookback_ms: u64,
    /// The noiseband in c
    #[arg(short, long, value_name = "noiseband_c")]
    noiseband_c: f32,

    /// Perform genetic fine tuning
    #[arg(short, long, value_name = "genetic")]
    genetic: bool,

    /// Simply score the provided PID value and don't perform genetic fine tuning
    #[arg(short, long, value_name = "test")]
    test: bool,

    /// Supply your own kp, ki, and kd and skip the rough autotuning
    #[arg(short, long, value_name = "p")]
    p: Option<f32>,
    /// Supply your own kp, ki, and kd and skip the rough autotuning
    #[arg(short, long, value_name = "i")]
    i: Option<f32>,
    /// Supply your own kp, ki, and kd and skip the rough autotuning
    #[arg(short, long, value_name = "d")]
    d: Option<f32>,
}

fn main() {
    let cli = Cli::parse();

    let mut urap_i2c = UrapMaster::new(URAP_I2C_PATH).unwrap();
    let mut urap_thermo = UrapMaster::new(URAP_THERMO_PATH).unwrap();

    let mut autotuner = PIDAutotuner::new(
        cli.setpoint_c,
        cli.output_step,
        cli.sample_time_ms,
        cli.lookback_ms,
        0.0,
        1.0,
        cli.noiseband_c,
    );

    let (addr_temp, addr_pwr): (u16, u16) = cli.zone.into();

    // Disable any internal PID control in thermo
    urap_thermo
        .write_u32(ADDR_ENBL_PWR_OVERRIDE as u16, 1)
        .unwrap();

    if cli.p.is_none() {
        loop {
            let temp_c = urap_i2c.read_f32(addr_temp).unwrap();

            let status = autotuner.run(temp_c);
            let power = autotuner.get_output();

            print!("\r{:?} @ {:.0}C & {}pwr", status, temp_c, power);

            urap_thermo.write_f32(addr_pwr, power).unwrap();

            match status {
                PIDAutotuneState::Succeeded => {
                    let tuning_rules: [TuningRule; 10] = [
                        TuningRule::ZeiglerNichols,
                        TuningRule::TyreusLuyben,
                        TuningRule::CianconeMarlin,
                        TuningRule::PessenIntergral,
                        TuningRule::SomeOvershoot,
                        TuningRule::NoOvershoot,
                        TuningRule::Brewing,
                        TuningRule::HighMass,
                        TuningRule::Marlin,
                        TuningRule::Flat,
                    ];

                    println!("\nku={}, pu={}", autotuner.ku, autotuner.pu_ms / 1000.0);

                    for tuning_rule in tuning_rules {
                        let (kp, ki, kd) = autotuner.get_kpid(tuning_rule);

                        println!("{:?}: kp={:e}, ki={:e}, kd={:e}", tuning_rule, kp, ki, kd);
                    }
                    break;
                }

                PIDAutotuneState::Failed => {
                    return;
                }

                _ => {}
            }
            std::thread::sleep(Duration::from_millis(cli.sample_time_ms));
        }
    }

    if cli.genetic == false {
        return;
    }

    let choice = TuningRule::HighMass;

    println!("\n\nPerforming Genetic Fine Tuning using {:?}", choice);

    let (kp, ki, kd): (f32, f32, f32) = match cli.p {
        None => autotuner.get_kpid(choice),
        Some(val) => {
            let ki = cli.i.unwrap();
            let kd = cli.d.unwrap();

            (val, ki, kd)
        }
    };

    let mut ideal_pid = Pid::<f32>::new(cli.setpoint_c, 1.0);

    ideal_pid.p(kp, 1.0);
    ideal_pid.i(ki, 1.0);
    ideal_pid.d(kd, 1.0);

    if cli.test {
        urap_thermo.write_f32(addr_pwr, cli.output_step).unwrap();

        loop {
            let temp_c = urap_i2c.read_f32(addr_temp).unwrap();
            if temp_c >= cli.setpoint_c + cli.noiseband_c {
                break;
            }

            print!("\rWarmup @ {:.0}C\t", temp_c);
            std::thread::sleep(Duration::from_millis(cli.sample_time_ms));
        }

        urap_thermo.write_f32(addr_pwr, 0.0).unwrap();
        loop {
            let temp_c = urap_i2c.read_f32(addr_temp).unwrap();
            if temp_c <= cli.setpoint_c - 10.0 {
                break;
            }

            print!("\rCooldown @ {:.0}C\t", temp_c);
            std::thread::sleep(Duration::from_millis(cli.sample_time_ms));
        }

        let sample_length_ms = cli.lookback_ms * 3;

        let test_end = Instant::now()
            .checked_add(Duration::from_millis(sample_length_ms))
            .unwrap();

        let mut points: Vec<f32> =
            Vec::with_capacity((sample_length_ms / cli.sample_time_ms) as usize);

        loop {
            let temp_c = urap_i2c.read_f32(addr_temp).unwrap();
            let pwr = ideal_pid.next_control_output(temp_c).output;

            urap_thermo.write_f32(addr_pwr, pwr).unwrap();

            points.push(temp_c);

            if test_end.saturating_duration_since(Instant::now()).is_zero() {
                break;
            }

            print!("\rCollecting Points @ {:.0}C\t", temp_c);
            std::thread::sleep(Duration::from_millis(cli.sample_time_ms));
        }

        let mut delta_mean: f64 = 0.0;
        let delta_mean_weight: f64 = 1.0 / points.len() as f64;

        for point in points {
            delta_mean += (cli.setpoint_c - point).abs() as f64 * delta_mean_weight;
        }

        let score = delta_mean.powi(2);

        let kp = ideal_pid.kp;
        let ki = ideal_pid.ki;
        let kd = ideal_pid.kd;

        println!(
            "\n\nPID of values kp={:.3e}, ki={:.3e}, & kd={:.3e} produced {} score",
            kp, ki, kd, score
        );

        urap_thermo.write_f32(addr_pwr, 0.0).unwrap();

        return;
    }

    let rng = Normal::new(1.0, 0.25).unwrap();

    let mut last_score = f64::INFINITY;
    let mut last_score_mid = f64::INFINITY;

    loop {
        let mut children: Vec<(f64, Pid<f32>)> = Vec::with_capacity(10);

        for _ in 0..10 {
            urap_thermo.write_f32(addr_pwr, cli.output_step).unwrap();

            loop {
                let temp_c = urap_i2c.read_f32(addr_temp).unwrap();
                if temp_c >= cli.setpoint_c + cli.noiseband_c {
                    break;
                }

                print!("\rWarmup @ {:.0}C\t", temp_c);
                std::thread::sleep(Duration::from_millis(cli.sample_time_ms));
            }

            urap_thermo.write_f32(addr_pwr, 0.0).unwrap();
            loop {
                let temp_c = urap_i2c.read_f32(addr_temp).unwrap();
                if temp_c <= cli.setpoint_c - 10.0 {
                    break;
                }

                print!("\rCooldown @ {:.0}C\t", temp_c);
                std::thread::sleep(Duration::from_millis(cli.sample_time_ms));
            }

            let mut child = ideal_pid;

            let kp = ideal_pid.kp * rng.sample(&mut thread_rng());
            let ki = ideal_pid.ki * rng.sample(&mut thread_rng());
            let kd = ideal_pid.kd * rng.sample(&mut thread_rng());

            child.p(kp, 1.0);
            child.i(ki, 1.0);
            child.d(kd, 1.0);

            let sample_length_ms = cli.lookback_ms * 3;

            let mut points: Vec<f32> =
                Vec::with_capacity((sample_length_ms / cli.sample_time_ms) as usize);

            let mut delta_mean: f64 = 0.0;

            loop {
                let temp_c = urap_i2c.read_f32(addr_temp).unwrap();
                let pwr = child.next_control_output(temp_c).output;

                urap_thermo.write_f32(addr_pwr, pwr).unwrap();

                let delta_mean_weight: f64 = 1.0 / points.capacity() as f64;

                delta_mean += (temp_c - cli.setpoint_c).abs() as f64 * delta_mean_weight;

                points.push(temp_c);

                if points.len() >= points.capacity() {
                    break;
                }

                let score_mid = (delta_mean * 2.0).powi(2);

                if points.len() == (points.capacity() / 2) && score_mid > (1.25 * last_score_mid) {
                    delta_mean *= 2.0;
                    break;
                } else {
                    last_score_mid = (delta_mean * 2.0).min(last_score_mid);
                }

                print!("\rCollecting Points @ {:.0}C\t", temp_c);
                std::thread::sleep(Duration::from_millis(cli.sample_time_ms));
            }

            let score = delta_mean.powi(2);

            let kp = child.kp;
            let ki = child.ki;
            let kd = child.kd;

            println!(
                "\n\nChild of values kp={:.3e}, ki={:.3e}, & kd={:.3e} produced {} score",
                kp, ki, kd, score
            );

            children.push((score, child));
        }

        children.sort_by(|a, b| {
            let (ascore, _) = a;
            let (bscore, _) = b;

            ascore.partial_cmp(bscore).unwrap()
        });

        let (parent_score, parent_pid) = children[0];
        
        ideal_pid = parent_pid;
        ideal_pid.reset_integral_term();

        if last_score > parent_score {
            println!(
                "New Ideal kPID @ {:.0}C:\nkp={:.3e}\nki={:.3e}\nkd={:.3e}",
                urap_i2c.read_f32(ADDR_AMBIENT_C as u16).unwrap_or(0.0),
                ideal_pid.kp,
                ideal_pid.ki,
                ideal_pid.kd
            );
        }

        // Keep going until we can't acheive a 1% improvement
        if last_score != f64::INFINITY && (parent_score / last_score) >= 0.99 {
            break;
        } else {
            last_score = parent_score;
        }
    }

    println!("\n\nCalibration done!");

    urap_thermo.write_f32(addr_pwr, 0.0).unwrap();
}
