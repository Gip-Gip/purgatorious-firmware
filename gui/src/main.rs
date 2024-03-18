use chrono::{DateTime, Local};
use egui::{Button, Color32, Id, InnerResponse, Label, Response, RichText, Sense};
use i2c::*;
use watchdog::ADDR_ESTOP;
use std::{
    collections::LinkedList,
    path::Path,
    str::FromStr,
    sync::{Arc, Mutex},
    thread::sleep,
    time::{Duration, Instant, SystemTime},
};
use thermo::*;

use egui_plot::{Line, Plot, PlotBounds, PlotPoints};

use native_dialog::{MessageDialog, MessageType};

use eframe::egui;
use screw::*;
use shared::*;
use urap::*;

// Run at 30fps
const FRAME_TIME_MS: u64 = 1000 / 30;

// Update the plot every 18 seconds
const POLL_PLOT_S: u64 = 18;

const MAX_TEMP_C: f32 = 250.0;
const MIN_TEMP_C: f32 = 0.0;
const MIN_RPM: f32 = 0.0;
const MAX_RPM: f32 = 100.0;
const MAX_DIGITS_TEMP: usize = 3;
const MAX_DIGITS_SPEED: usize = 3;

const IDEAL_ZONE_TOL_C: f32 = 1.0;
const MAX_ZONE_TOL_C: f32 = 3.0;

fn main() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder {
            decorations: Some(false),
            maximized: Some(true),
            fullscreen: Some(false),
            titlebar_shown: Some(false),
            title_shown: Some(false),
            close_button: Some(false),
            minimize_button: Some(false),
            maximize_button: Some(false),
            titlebar_buttons_shown: Some(false),
            ..Default::default()
        },
        ..Default::default()
    };
    eframe::run_native(
        "Purgatorious",
        options,
        Box::new(|ctx| {
            setup_custom_fonts(&ctx.egui_ctx);

            Box::<MyApp>::default()
        }),
    )
    .unwrap();
}

struct MyApp {
    registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; 1]>>,
    previous_frame_ms: u64,
    urap_i2c: UrapMaster,
    urap_thermo: UrapMaster,
    urap_screw: UrapMaster,
    urap_watchdog: UrapMaster,
    keypad: bool,
    keypad_submit: bool,
    keypad_str: String,
    keypad_editing: Zone,
    plot_poll: Instant,
    plot_t1_c: LinkedList<f32>,
    plot_t1_xy: Vec<[f64; 2]>,
    plot_t2_c: LinkedList<f32>,
    plot_t2_xy: Vec<[f64; 2]>,
    plot_t3_c: LinkedList<f32>,
    plot_t3_xy: Vec<[f64; 2]>,
    plot_t4_c: LinkedList<f32>,
    plot_t4_xy: Vec<[f64; 2]>,
    plot_t5_c: LinkedList<f32>,
    plot_t5_xy: Vec<[f64; 2]>,
    plot_t6_c: LinkedList<f32>,
    plot_t6_xy: Vec<[f64; 2]>,
    plot_tm_c: LinkedList<f32>,
    plot_tm_xy: Vec<[f64; 2]>,
}

#[derive(PartialEq, Clone, Copy)]
enum Zone {
    Z1,
    Z2,
    Z3,
    Z4,
    Z5,
    Z6,
    Screw,
}

impl Default for MyApp {
    fn default() -> Self {
        let urap_watchdog = UrapMaster::new(URAP_WATCHDOG_PATH).unwrap();

        let registers: Arc<Mutex<[[u8; URAP_REG_WIDTH]; 1]>> =
            Arc::new(Mutex::new([[0; URAP_REG_WIDTH]]));

        UrapSlave::spawn(URAP_GUI_PATH, registers.clone(), [true]).unwrap();
        // Wait on the vPLCs to initialize
        let urap_i2c_path = Path::new(URAP_I2C_PATH);
        let urap_thermo_path = Path::new(URAP_THERMO_PATH);
        let urap_screw_path = Path::new(URAP_SCREW_PATH);

        while !urap_i2c_path.exists() && !urap_thermo_path.exists() && !urap_screw_path.exists() {
            sleep(Duration::from_secs(1));
        }

        let urap_i2c = UrapMaster::new(URAP_I2C_PATH).unwrap();
        let urap_thermo = UrapMaster::new(URAP_THERMO_PATH).unwrap();
        let urap_screw = UrapMaster::new(URAP_SCREW_PATH).unwrap();

        Self {
            registers,
            previous_frame_ms: 0,
            urap_i2c,
            urap_thermo,
            urap_screw,
            urap_watchdog,
            keypad_submit: false,
            keypad: false,
            keypad_str: String::new(),
            keypad_editing: Zone::Z1,
            plot_poll: Instant::now(),
            plot_t1_c: [0.0; 100].into(),
            plot_t1_xy: Vec::new().into(),
            plot_t2_c: [0.0; 100].into(),
            plot_t2_xy: Vec::new().into(),
            plot_t3_c: [0.0; 100].into(),
            plot_t3_xy: Vec::new().into(),
            plot_t4_c: [0.0; 100].into(),
            plot_t4_xy: Vec::new().into(),
            plot_t5_c: [0.0; 100].into(),
            plot_t5_xy: Vec::new().into(),
            plot_t6_c: [0.0; 100].into(),
            plot_t6_xy: Vec::new().into(),
            plot_tm_c: [0.0; 100].into(),
            plot_tm_xy: Vec::new().into(),
        }
    }
}

fn setup_custom_fonts(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();

    fonts.font_data.insert(
        "my_font".to_owned(),
        egui::FontData::from_static(include_bytes!("/usr/share/fonts/TTF/FiraCode-Regular.ttf")),
    );

    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "my_font".to_owned());

    fonts
        .families
        .entry(egui::FontFamily::Monospace)
        .or_default()
        .insert(0, "my_font".to_owned());

    ctx.set_fonts(fonts);
}

impl<'a> eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let sleep_time = now
            .checked_add(Duration::from_millis(FRAME_TIME_MS))
            .unwrap();

        // Increment the hash so the watchdog doesn't kill us
        let mut registers = self.registers.lock().unwrap();

        let inchash = u32::from_ne_bytes(registers[ADDR_INCHASH]) + 1;

        registers[ADDR_INCHASH] = inchash.to_ne_bytes();

        drop(registers);
        let (
            mut t1_c,
            mut t2_c,
            mut t3_c,
            mut t4_c,
            mut t5_c,
            mut t6_c,
            mut t7_c,
            mut ta_c,
            mut barrel_kpa,
            mut ts1_c,
            mut ts2_c,
            mut ts3_c,
            mut ts4_c,
            mut ts5_c,
            mut ts6_c,
            mut thermo_pwr_w,
            mut set_screw_rpm,
            mut act_screw_rpm,
            mut act_motor_rpm,
            mut motor_a,
            mut motor_v,
            mut line_v
        ) = (
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
            0_f32,
        );
        
        let temps: [(&mut f32, u16); 9] = [
            (&mut t1_c, ADDR_T1_C as u16),
            (&mut t2_c, ADDR_T2_C as u16),
            (&mut t3_c, ADDR_T3_C as u16),
            (&mut t4_c, ADDR_T4_C as u16),
            (&mut t5_c, ADDR_T5_C as u16),
            (&mut t6_c, ADDR_T6_C as u16),
            (&mut t7_c, ADDR_T7_C as u16),
            (&mut ta_c, ADDR_AMBIENT_C as u16),
            (&mut barrel_kpa, ADDR_BARREL_KPA as u16),
        ];
        
        let thermo_vars: [(&mut f32, u16); 7] = [
            (&mut ts1_c, ADDR_SET_Z1_C as u16),
            (&mut ts2_c, ADDR_SET_Z2_C as u16),
            (&mut ts3_c, ADDR_SET_Z3_C as u16),
            (&mut ts4_c, ADDR_SET_Z4_C as u16),
            (&mut ts5_c, ADDR_SET_Z5_C as u16),
            (&mut ts6_c, ADDR_SET_Z6_C as u16),
            (&mut thermo_pwr_w, ADDR_THERMO_PWR_W as u16),
        ];

        let motor_stats: [(&mut f32, u16); 6] = [
            (&mut set_screw_rpm, ADDR_SET_SCREW_RPM as u16),
            (&mut act_screw_rpm, ADDR_ACT_SCREW_RPM as u16),
            (&mut act_motor_rpm, ADDR_ACT_MOTOR_RPM as u16),
            (&mut motor_a, ADDR_MOTOR_A as u16),
            (&mut motor_v, ADDR_MOTOR_V as u16),
            (&mut line_v, ADDR_LINE_V as u16)
        ];
        
        for (temp, addr) in temps {
            *temp = match self.urap_i2c.read_f32(addr) {
                Ok(val) => val,
                Err(_) => {
                    self.urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
                    break;
                }
            };
        }
        
        for (motor_stat, addr) in motor_stats {
            *motor_stat = match self.urap_screw.read_f32(addr) {
                Ok(val) => val,
                Err(_) => {
                    self.urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
                    break;
                }
            };
        }

        for (thermo_var, addr) in thermo_vars {
            *thermo_var = match self.urap_thermo.read_f32(addr) {
                Ok(val) => val,
                Err(_) => {
                    self.urap_watchdog.write_u32(ADDR_ESTOP, 1).unwrap_or_default();
                    break;
                }
            };
        }

        let (
            t1_c,
            t2_c,
            t3_c,
            t4_c,
            t5_c,
            t6_c,
            t7_c,
            ta_c,
            barrel_kpa,
            ts1_c,
            ts2_c,
            ts3_c,
            ts4_c,
            ts5_c,
            ts6_c,
            thermo_pwr_w,
            set_screw_rpm,
            act_screw_rpm,
            act_motor_rpm,
            motor_a,
            motor_v,
            line_v
        ) = (
            t1_c,
            t2_c,
            t3_c,
            t4_c,
            t5_c,
            t6_c,
            t7_c,
            ta_c,
            barrel_kpa,
            ts1_c,
            ts2_c,
            ts3_c,
            ts4_c,
            ts5_c,
            ts6_c,
            thermo_pwr_w,
            set_screw_rpm,
            act_screw_rpm,
            act_motor_rpm,
            motor_a,
            motor_v,
            line_v
        );

        // True if not zero
        let drive_enabled = self
            .urap_screw
            .read_u32(ADDR_DRIVE_ENBL as u16)
            .unwrap_or(0)
            > 0;

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical(|ui| {
                // Misc. Stats
                ui.horizontal(|ui| {
                    let time = SystemTime::now();
                    let datetime: DateTime<Local> = time.into();
                    ui.label(datetime.format("%v %T").to_string());
                    ui.separator();
                    ui.label(format!("Ambient {:.0}°C", ta_c));
                    ui.separator();
                    ui.label(format!("Heaters: {:.0}W", thermo_pwr_w));
                    ui.separator();
                    ui.label(format!("Motor: {:.0}W", motor_a * motor_v));
                    ui.separator();
                    ui.label(format!("Line: {}V", line_v));
                    ui.separator();
                    ui.label(format!("Frame: {:03}ms", self.previous_frame_ms));
                });

                // Heater Zones
                ui.horizontal(|ui| {
                    let zones: [(&str, f32, f32, &Vec<[f64; 2]>); 6] = [
                        ("Zone 1", t1_c, ts1_c, &self.plot_t1_xy),
                        ("Zone 2", t2_c, ts2_c, &self.plot_t2_xy),
                        ("Zone 3", t3_c, ts3_c, &self.plot_t3_xy),
                        ("Zone 4", t4_c, ts4_c, &self.plot_t4_xy),
                        ("Zone 5", t5_c, ts5_c, &self.plot_t5_xy),
                        ("Zone 6", t6_c, ts6_c, &self.plot_t6_xy),
                    ];

                    let mut responses: Vec<InnerResponse<Response>> =
                        Vec::with_capacity(zones.len());

                    for (i, (name, act_c, set_c, act_xy)) in zones.iter().enumerate() {
                        let response = ui.vertical(|ui| {
                            let header_rt = RichText::new(*name).strong().size(18.0);

                            let actual_set_delta = (act_c - set_c).abs();

                            let actual_temp_color = if actual_set_delta < IDEAL_ZONE_TOL_C {
                                Color32::GREEN
                            } else if actual_set_delta < MAX_ZONE_TOL_C {
                                Color32::YELLOW
                            } else {
                                Color32::RED
                            };

                            let act_temp_rt = RichText::new(format!("Act: {:03.0}°C", act_c))
                                .color(actual_temp_color)
                                .monospace()
                                .size(18.0);
                            ui.add(Label::new(header_rt).wrap(true).selectable(false));
                            ui.add(Label::new(act_temp_rt).wrap(true).selectable(false));
                            ui.horizontal(|ui| {
                                let set_temp_rt = RichText::new("Set: ")
                                    .color(Color32::LIGHT_BLUE)
                                    .monospace()
                                    .size(18.0);
                                let set_temp_val_rt = RichText::new(format!("{:03.0}", set_c))
                                    .background_color(Color32::WHITE)
                                    .color(Color32::BLACK)
                                    .size(18.0)
                                    .monospace();
                                let set_temp_post_rt = RichText::new("°C")
                                    .color(Color32::LIGHT_BLUE)
                                    .monospace()
                                    .size(18.0);

                                ui.add(Label::new(set_temp_rt).wrap(true).selectable(false));
                                ui.add(Label::new(set_temp_val_rt).wrap(true).selectable(false));
                                ui.add(Label::new(set_temp_post_rt).wrap(true).selectable(false));
                            });

                            let plotpoints: PlotPoints = (*act_xy).clone().into();

                            let line = Line::new(plotpoints);

                            let set_c = *set_c as f64;

                            Plot::new(format!("plot{}", i))
                                .width(ui.min_rect().width())
                                .auto_bounds(false.into())
                                .show_x(false)
                                .show_y(false)
                                .show_axes(false)
                                .view_aspect(4.0)
                                .show(ui, |ui| {
                                    ui.set_plot_bounds(PlotBounds::from_min_max(
                                        [-100.0, set_c - 10.0],
                                        [0.0, set_c + 10.0],
                                    ));
                                    ui.line(line)
                                });

                            ui.interact(ui.min_rect(), Id::NULL, Sense::click())
                        });

                        ui.separator();

                        responses.push(response);
                    }

                    // Melt temperature and pressure
                    ui.vertical_centered(|ui| {
                        let zmelt_head = RichText::new("Melt Temperature").strong().size(18.0);
                        let zmelt_act_temp = RichText::new(format!("{:03.0}°C", t7_c))
                            .color(Color32::WHITE)
                            .monospace()
                            .size(18.0);
                        ui.add(Label::new(zmelt_head).wrap(true).selectable(false));
                        ui.add(Label::new(zmelt_act_temp).wrap(true).selectable(false));

                        let plotpoints: PlotPoints = (self.plot_tm_xy).clone().into();

                        let line = Line::new(plotpoints);

                        let act_c = t7_c as f64;

                        Plot::new("plotmelt")
                            .auto_bounds(false.into())
                            .show_x(false)
                            .show_y(false)
                            .show_axes(false)
                            .view_aspect(6.0)
                            .show(ui, |ui| {
                                ui.set_plot_bounds(PlotBounds::from_min_max(
                                    [-100.0, act_c - 10.0],
                                    [0.0, act_c + 10.0],
                                ));
                                ui.line(line)
                            });

                        let pres_head_rt = RichText::new("Melt Pressure").strong().size(18.0);
                        let barrel_pres_rt = RichText::new(format!("{:.0}kPa", barrel_kpa)).color(Color32::WHITE).monospace().size(18.0);
                        
                        ui.add(Label::new(pres_head_rt).wrap(true).selectable(false));
                        ui.add(Label::new(barrel_pres_rt).wrap(true).selectable(false));
                    });

                    // Zone interaction logic
                    let zones: [Zone; 6] =
                        [Zone::Z1, Zone::Z2, Zone::Z3, Zone::Z4, Zone::Z5, Zone::Z6];

                    for (i, zone) in zones.iter().enumerate() {
                        if responses[i].inner.clicked() {
                            self.keypad_str = String::with_capacity(MAX_DIGITS_TEMP);
                            self.keypad_editing = *zone;
                            self.keypad = true;
                        }
                    }
                });

                ui.separator();

                ui.horizontal(|ui| {
                    if drive_enabled {
                        if ui
                            .vertical(|ui| {
                                let screwhead_rt = RichText::new("Screw").strong().size(18.0);

                                let act_speed_rt =
                                    RichText::new(format!("Act: {:03.0}RPM", act_screw_rpm))
                                        .color(Color32::LIGHT_BLUE)
                                        .size(18.0);

                                ui.add(Label::new(screwhead_rt).wrap(true).selectable(false));
                                ui.add(Label::new(act_speed_rt).wrap(true).selectable(false));

                                ui.horizontal(|ui| {
                                    let set_speed_rt = RichText::new("Set: ")
                                        .color(Color32::LIGHT_BLUE)
                                        .monospace()
                                        .size(18.0);
                                    let set_speed_val_rt =
                                        RichText::new(format!("{:03.0}", set_screw_rpm))
                                            .background_color(Color32::WHITE)
                                            .color(Color32::BLACK)
                                            .size(18.0)
                                            .monospace();
                                    let set_speed_post_rt = RichText::new("RPM")
                                        .color(Color32::LIGHT_BLUE)
                                        .monospace()
                                        .size(18.0);

                                    ui.add(Label::new(set_speed_rt).wrap(true).selectable(false));
                                    ui.add(
                                        Label::new(set_speed_val_rt).wrap(true).selectable(false),
                                    );
                                    ui.add(
                                        Label::new(set_speed_post_rt).wrap(true).selectable(false),
                                    );
                                });

                                let current_rt = RichText::new(format!("Current: {}A", motor_a))
                                    .color(Color32::GOLD)
                                    .size(18.0);
                                let voltage_rt = RichText::new(format!("Voltage: {}V", motor_v))
                                    .color(Color32::GOLD)
                                    .size(18.0);

                                let act_motor_rt =
                                    RichText::new(format!("Motor Speed: {}RPM", act_motor_rpm))
                                        .color(Color32::GRAY)
                                        .size(18.0);

                                ui.add(Label::new(current_rt).wrap(true).selectable(false));
                                ui.add(Label::new(voltage_rt).wrap(true).selectable(false));
                                ui.add(Label::new(act_motor_rt).wrap(true).selectable(false));

                                ui.interact(ui.min_rect(), Id::NULL, Sense::click())
                            })
                            .inner
                            .clicked()
                        {
                            self.keypad_str = String::with_capacity(MAX_DIGITS_SPEED);
                            self.keypad_editing = Zone::Screw;
                            self.keypad = true;
                        }
                    } else {
                        let drive_disabled_rt = RichText::new("DRIVE DISABLED").strong().size(18.0).color(Color32::RED);

                        ui.add(Label::new(drive_disabled_rt).wrap(true).selectable(false));
                    }
                });

                ui.separator();

                if self.urap_watchdog.read_u32(ADDR_ESTOP).unwrap_or(1) != 0 {
                    let estop_rt = RichText::new("ESTOP ACTIVE").size(18.0).color(Color32::WHITE).strong();

                    let estop_button = Button::new(estop_rt).fill(Color32::RED);

                    if ui.add(estop_button).clicked() {
                        // Write 0 to clear estop
                        self.urap_watchdog.write_u32(ADDR_ESTOP, 0).unwrap_or_default();
            
                        let urap_i2c = UrapMaster::new(URAP_I2C_PATH);
                        let urap_thermo = UrapMaster::new(URAP_THERMO_PATH);
                        let urap_screw = UrapMaster::new(URAP_SCREW_PATH);

                        if let Ok(val) = urap_i2c {
                            self.urap_i2c = val;
                        }

                        if let Ok(val) = urap_thermo {
                            self.urap_thermo = val;
                        }

                        if let Ok(val) = urap_screw {
                            self.urap_screw = val;
                        }
                    }
                }
            });
        });

        // Keypad logic
        if self.keypad {
            ctx.show_viewport_immediate(
                egui::ViewportId::from_hash_of("keypad"),
                egui::ViewportBuilder {
                    title: Some("Enter New Value".to_owned()),
                    close_button: Some(false),
                    minimize_button: Some(false),
                    maximize_button: Some(false),
                    titlebar_buttons_shown: Some(false),
                    ..Default::default()
                },
                |ctx, _class| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        let layout: [[char; 4]; 4] = [
                            ['7', '8', '9', 'X'],
                            ['4', '5', '6', '←'],
                            ['1', '2', '3', 'C'],
                            ['.', '0', '-', '↲'],
                        ];

                        ui.vertical(|ui| {
                            let label_rt = RichText::new(format!("{:03}", self.keypad_str))
                                .background_color(Color32::WHITE)
                                .color(Color32::BLACK)
                                .size(36.0)
                                .monospace();

                            ui.label(label_rt);

                            for row in layout {
                                ui.horizontal(|ui| {
                                    for key in row {
                                        let key_rt = RichText::new(format!(" {} ", key))
                                            .monospace()
                                            .size(48.0);
                                        if ui.button(key_rt).clicked() {
                                            match key {
                                                'X' => self.keypad = false,
                                                '←' => {
                                                    self.keypad_str.pop();
                                                }
                                                'C' => self.keypad_str.clear(),
                                                '↲' => {
                                                    self.keypad = false;
                                                    self.keypad_submit = true;
                                                }
                                                _ => self.keypad_str.push(key),
                                            }
                                        }
                                    }
                                });
                            }
                        })
                    });
                },
            );
        }

        if self.keypad_submit {
            let zone_addr = match self.keypad_editing {
                Zone::Z1 => ADDR_SET_Z1_C as u16,
                Zone::Z2 => ADDR_SET_Z2_C as u16,
                Zone::Z3 => ADDR_SET_Z3_C as u16,
                Zone::Z4 => ADDR_SET_Z4_C as u16,
                Zone::Z5 => ADDR_SET_Z5_C as u16,
                Zone::Z6 => ADDR_SET_Z6_C as u16,
                Zone::Screw => ADDR_SET_SCREW_RPM as u16,
            };

            let (urap, min, max) = match self.keypad_editing {
                Zone::Z1 | Zone::Z2 | Zone::Z3 | Zone::Z4 | Zone::Z5 | Zone::Z6 => {
                    (&mut self.urap_thermo, MIN_TEMP_C, MAX_TEMP_C)
                }
                Zone::Screw => (&mut self.urap_screw, MIN_RPM, MAX_RPM),
            };

            let val: f32 = self
                .keypad_str
                .parse()
                .unwrap_or_else(|e: <f32 as FromStr>::Err| {
                    MessageDialog::new()
                        .set_type(MessageType::Error)
                        .set_title("Error")
                        .set_text(&e.to_string())
                        .show_alert()
                        .unwrap_or_default();
                    0.0
                });

            let val = match val > max || val < min {
                true => {
                    MessageDialog::new()
                        .set_type(MessageType::Error)
                        .set_title("Error")
                        .set_text(&format!(
                            "Value {} does not fit within {}-{}",
                            val, min, max
                        ))
                        .show_alert()
                        .unwrap_or_default();
                    min
                }
                false => val,
            };

            urap.write_f32(zone_addr, val).unwrap_or_default();

            self.keypad_submit = false;
        }

        // Plot refresh logic
        let mut zones: [(&mut LinkedList<f32>, &mut Vec<[f64; 2]>, f32); 7] = [
            (&mut self.plot_t1_c, &mut self.plot_t1_xy, t1_c),
            (&mut self.plot_t2_c, &mut self.plot_t2_xy, t2_c),
            (&mut self.plot_t3_c, &mut self.plot_t3_xy, t3_c),
            (&mut self.plot_t4_c, &mut self.plot_t4_xy, t4_c),
            (&mut self.plot_t5_c, &mut self.plot_t5_xy, t5_c),
            (&mut self.plot_t6_c, &mut self.plot_t6_xy, t6_c),
            (&mut self.plot_tm_c, &mut self.plot_tm_xy, t7_c),
        ];

        if self.plot_poll.saturating_duration_since(now) == Duration::ZERO {
            for (plot_t_c, plot_t_xy, act_c) in zones.iter_mut() {
                plot_t_c.pop_back();
                plot_t_c.push_front(*act_c);
                **plot_t_xy = plot_t_c
                    .iter()
                    .enumerate()
                    .map(|(i, temp_c)| {
                        let x = -(i as f32);

                        [x as f64, *temp_c as f64]
                    })
                    .collect();
            }

            self.plot_poll = now.checked_add(Duration::from_secs(POLL_PLOT_S)).unwrap();
        }

        self.previous_frame_ms = Instant::now().saturating_duration_since(now).as_millis() as u64;

        sleep_till(sleep_time);
        ctx.request_repaint();
    }
}
