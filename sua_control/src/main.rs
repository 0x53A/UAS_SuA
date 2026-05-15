mod gamepad;
mod serial_comm;

use eframe::egui;
use egui::{Color32, RichText};
use egui_plot::{Line, Plot, PlotPoints};
use serial_comm::{Command, SharedEvent, SharedWriter, Telemetry};
use std::collections::VecDeque;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::Instant;

const HISTORY_SECONDS: f64 = 60.0;
const HISTORY_CAPACITY: usize = 60 * 50;
const ANGLE_MIN_DEG: f64 = 2.0;
const ANGLE_MAX_DEG: f64 = 178.0;

struct AngleHistory {
    motor: VecDeque<(f64, f64)>,
    amr: VecDeque<(f64, f64)>,
    target: VecDeque<(f64, f64)>,
}

impl AngleHistory {
    fn new() -> Self {
        Self {
            motor: VecDeque::with_capacity(HISTORY_CAPACITY),
            amr: VecDeque::with_capacity(HISTORY_CAPACITY),
            target: VecDeque::with_capacity(HISTORY_CAPACITY),
        }
    }

    fn push(&mut self, t: f64, motor: f64, amr: f64, target: f64) {
        let cutoff = t - HISTORY_SECONDS;
        while self.motor.front().is_some_and(|(ts, _)| *ts < cutoff) {
            self.motor.pop_front();
            self.amr.pop_front();
            self.target.pop_front();
        }
        self.motor.push_back((t, motor));
        self.amr.push_back((t, amr));
        self.target.push_back((t, target));
    }
}

#[derive(PartialEq, Clone, Copy)]
enum ControlMode {
    Velocity,
    Position,
}

struct SuaApp {
    telemetry: Arc<Mutex<Telemetry>>,
    serial_connected: Arc<Mutex<bool>>,
    gamepad_input: Arc<Mutex<gamepad::GamepadInput>>,
    serial_writer: SharedWriter,
    serial_event: SharedEvent,

    history: AngleHistory,
    start_time: Instant,

    control_mode: ControlMode,
    target_angle: f64,
    max_speed_pct: f64,
    velocity_pct: f64,
    enabled: bool,
    calibration_angle: f64,

    prev_west: bool,
    prev_south: bool,
    prev_north: bool,
}

impl SuaApp {
    fn new(port_name: String) -> Self {
        let telemetry = Arc::new(Mutex::new(Telemetry::default()));
        let serial_connected = Arc::new(Mutex::new(false));
        let gamepad_input = Arc::new(Mutex::new(gamepad::GamepadInput::default()));
        let serial_writer: SharedWriter = Arc::new(Mutex::new(None));
        let serial_event: SharedEvent = Arc::new(Mutex::new(String::new()));

        {
            let t = telemetry.clone();
            let c = serial_connected.clone();
            let w = serial_writer.clone();
            let e = serial_event.clone();
            let pn = port_name.clone();
            std::thread::spawn(move || serial_comm::run_reader(&pn, t, c, w, e));
        }

        {
            let gp = gamepad_input.clone();
            std::thread::spawn(move || gamepad::run(gp));
        }

        Self {
            telemetry,
            serial_connected,
            gamepad_input,
            serial_writer,
            serial_event,
            history: AngleHistory::new(),
            start_time: Instant::now(),
            control_mode: ControlMode::Velocity,
            target_angle: 90.0,
            max_speed_pct: 30.0,
            velocity_pct: 0.0,
            enabled: false,
            calibration_angle: 2.0,
            prev_west: false,
            prev_south: false,
            prev_north: false,
        }
    }

    fn send_command(&mut self, cmd: Command) {
        let wire = cmd.to_wire();
        if let Some(ref mut port) = *self.serial_writer.lock().unwrap() {
            if let Err(e) = port.write_all(wire.as_bytes()) {
                eprintln!("Serial write error: {e}");
            }
        } else {
            eprintln!("Serial writer not available");
        }
    }

    fn process_gamepad(&mut self) {
        let gp = self.gamepad_input.lock().unwrap().clone();
        if !gp.connected { return; }

        // X: toggle enable
        if gp.west && !self.prev_west {
            self.enabled = !self.enabled;
            self.send_command(Command::Enable(self.enabled));
        }
        self.prev_west = gp.west;

        // A: calibrate
        if gp.south && !self.prev_south {
            let angle_cdeg = (self.calibration_angle * 100.0) as i32;
            self.send_command(Command::Calibrate(angle_cdeg));
        }
        self.prev_south = gp.south;

        // Y: toggle mode
        if gp.north && !self.prev_north {
            self.control_mode = match self.control_mode {
                ControlMode::Velocity => ControlMode::Position,
                ControlMode::Position => ControlMode::Velocity,
            };
        }
        self.prev_north = gp.north;

        const DEADZONE: f32 = 0.1;

        match self.control_mode {
            ControlMode::Velocity => {
                let raw = -gp.left_y;
                let v = if raw.abs() < DEADZONE { 0.0 } else { raw };
                let pct = (v as f64 * 100.0).clamp(-100.0, 100.0);
                if (pct - self.velocity_pct).abs() > 0.5 {
                    self.velocity_pct = pct;
                    let vel_cmd = (pct * 100.0) as i16;
                    self.send_command(Command::Velocity(vel_cmd));
                }
            }
            ControlMode::Position => {
                let raw = gp.right_x;
                if raw.abs() > DEADZONE {
                    self.target_angle = (self.target_angle + raw as f64 * 0.5).clamp(ANGLE_MIN_DEG, ANGLE_MAX_DEG);
                }
                if gp.dpad_right { self.target_angle = (self.target_angle + 1.0).min(ANGLE_MAX_DEG); }
                if gp.dpad_left { self.target_angle = (self.target_angle - 1.0).max(ANGLE_MIN_DEG); }
                if gp.dpad_up { self.target_angle = (self.target_angle + 0.1).min(ANGLE_MAX_DEG); }
                if gp.dpad_down { self.target_angle = (self.target_angle - 0.1).max(ANGLE_MIN_DEG); }

                let trigger_speed = 10.0 + gp.right_trigger as f64 * 90.0;
                self.max_speed_pct = trigger_speed;

                if gp.east {
                    let pos = (self.target_angle * 100.0) as i32;
                    let spd = (self.max_speed_pct * 100.0) as u16;
                    self.send_command(Command::Position(pos, spd));
                }
            }
        }
    }
}

impl eframe::App for SuaApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(33));

        self.process_gamepad();

        let telem = self.telemetry.lock().unwrap().clone();
        let serial_ok = *self.serial_connected.lock().unwrap();
        let serial_event = self.serial_event.lock().unwrap().clone();
        let gp = self.gamepad_input.lock().unwrap().clone();

        let now = self.start_time.elapsed().as_secs_f64();
        self.history.push(now, telem.motor_deg(), telem.amr_deg(), telem.target_deg());

        self.enabled = telem.en != 0;

        egui::Panel::top("status_bar").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                let serial_color = if serial_ok { Color32::from_rgb(0, 200, 0) } else { Color32::from_rgb(200, 0, 0) };
                let gp_color = if gp.connected { Color32::from_rgb(0, 200, 0) } else { Color32::from_rgb(200, 0, 0) };
                ui.colored_label(serial_color, if serial_ok { "Serial: OK" } else { "Serial: ---" });
                ui.separator();
                ui.colored_label(gp_color, if gp.connected { "Gamepad: OK" } else { "Gamepad: ---" });
                ui.separator();
                let mode_text = match telem.mode.as_str() {
                    "V" => "VELOCITY",
                    "P" => "POSITION",
                    "C" => "CALIBRATING",
                    _ => "IDLE",
                };
                ui.label(RichText::new(format!("MCU: {mode_text}")).strong());
                ui.separator();
                if telem.cal != 0 {
                    ui.colored_label(Color32::from_rgb(0, 200, 0), "CAL OK");
                } else {
                    ui.colored_label(Color32::from_rgb(200, 160, 0), "NOT CAL");
                }
                if !serial_event.is_empty() {
                    ui.separator();
                    ui.label(format!("Last command: {serial_event}"));
                }
            });
        });

        egui::Panel::left("controls").exact_size(280.0).show_inside(ui, |ui| {
            ui.heading("Controls");
            ui.add_space(8.0);

            let en_text = if self.enabled { "Disable Motor [X]" } else { "Enable Motor [X]" };
            let en_color = if self.enabled { Color32::from_rgb(200, 60, 60) } else { Color32::from_rgb(60, 200, 60) };
            if ui.add(egui::Button::new(RichText::new(en_text).color(en_color)).min_size(egui::vec2(250.0, 30.0))).clicked() {
                self.enabled = !self.enabled;
                self.send_command(Command::Enable(self.enabled));
            }
            ui.add_space(8.0);

            ui.horizontal(|ui| {
                ui.label("Mode [Y]:");
                ui.selectable_value(&mut self.control_mode, ControlMode::Velocity, "Velocity");
                ui.selectable_value(&mut self.control_mode, ControlMode::Position, "Position");
            });
            ui.add_space(8.0);
            ui.separator();

            match self.control_mode {
                ControlMode::Velocity => {
                    ui.heading("Velocity");
                    ui.add_space(4.0);
                    let mut v = self.velocity_pct;
                    if ui.add(egui::Slider::new(&mut v, -100.0..=100.0).suffix("%").text("Speed")).changed() {
                        self.velocity_pct = v;
                        let vel_cmd = (v * 100.0) as i16;
                        self.send_command(Command::Velocity(vel_cmd));
                    }
                    if ui.button("Stop").clicked() {
                        self.velocity_pct = 0.0;
                        self.send_command(Command::Velocity(0));
                    }
                }
                ControlMode::Position => {
                    ui.heading("Position");
                    ui.add_space(4.0);
                    ui.add(egui::Slider::new(&mut self.target_angle, ANGLE_MIN_DEG..=ANGLE_MAX_DEG).suffix("°").text("Target"));
                    ui.add(egui::Slider::new(&mut self.max_speed_pct, 1.0..=100.0).suffix("%").text("Max Spd"));
                    if ui.button("Go [B]").clicked() {
                        let pos = (self.target_angle * 100.0) as i32;
                        let spd = (self.max_speed_pct * 100.0) as u16;
                        self.send_command(Command::Position(pos, spd));
                    }
                }
            }

            ui.add_space(16.0);
            ui.separator();
            ui.heading("Calibration");
            ui.add(egui::Slider::new(&mut self.calibration_angle, ANGLE_MIN_DEG..=ANGLE_MAX_DEG).suffix("°").text("Zero Pt"));
            if ui.button("Calibrate [A]").clicked() {
                let angle_cdeg = (self.calibration_angle * 100.0) as i32;
                self.send_command(Command::Calibrate(angle_cdeg));
            }

            ui.add_space(16.0);
            ui.separator();
            ui.heading("Gamepad");
            if gp.connected {
                ui.label(format!("LX:{:.2} LY:{:.2}", gp.left_x, gp.left_y));
                ui.label(format!("RX:{:.2} RY:{:.2}", gp.right_x, gp.right_y));
                ui.label(format!("LT:{:.2} RT:{:.2}", gp.left_trigger, gp.right_trigger));
            } else {
                ui.colored_label(Color32::GRAY, "No gamepad detected");
            }
        });

        egui::Frame::central_panel(ui.style()).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label(RichText::new("Motor").small());
                    ui.label(RichText::new(format!("{:.2}°", telem.motor_deg())).heading().strong()
                        .color(Color32::from_rgb(255, 140, 0)));
                });
                ui.add_space(24.0);
                ui.vertical(|ui| {
                    ui.label(RichText::new("AMR").small());
                    ui.label(RichText::new(format!("{:.2}°", telem.amr_deg())).heading().strong()
                        .color(Color32::from_rgb(0, 180, 255)));
                });
                ui.add_space(24.0);
                ui.vertical(|ui| {
                    ui.label(RichText::new("Delta").small());
                    let delta = telem.motor_deg() - telem.amr_deg();
                    let color = if delta.abs() < 1.0 { Color32::from_rgb(0, 200, 0) } else { Color32::from_rgb(200, 160, 0) };
                    ui.label(RichText::new(format!("{delta:+.2}°")).heading().strong().color(color));
                });
                ui.add_space(24.0);
                match telem.mode.as_str() {
                    "V" => {
                        ui.vertical(|ui| {
                            ui.label(RichText::new("Speed").small());
                            ui.label(RichText::new(format!("{:.1}°/s", telem.deg_per_sec())).heading());
                        });
                        ui.add_space(16.0);
                        ui.vertical(|ui| {
                            ui.label(RichText::new("Steps/s").small());
                            ui.label(RichText::new(format!("{}", telem.sps)).heading());
                        });
                    }
                    "P" => {
                        ui.vertical(|ui| {
                            ui.label(RichText::new("Target").small());
                            ui.label(RichText::new(format!("{:.2}°", telem.target_deg())).heading()
                                .color(Color32::from_rgb(255, 60, 60)));
                        });
                        ui.add_space(16.0);
                        ui.vertical(|ui| {
                            ui.label(RichText::new("Remaining").small());
                            ui.label(RichText::new(format!("{}", telem.rem)).heading());
                        });
                    }
                    _ => {}
                }
            });

            ui.add_space(8.0);
            ui.separator();
            ui.add_space(4.0);

            let motor_points: PlotPoints = self.history.motor.iter().map(|(t, v)| [*t, *v]).collect();
            let amr_points: PlotPoints = self.history.amr.iter().map(|(t, v)| [*t, *v]).collect();
            let target_points: PlotPoints = self.history.target.iter().map(|(t, v)| [*t, *v]).collect();

            let plot = Plot::new("angles")
                .height(ui.available_height())
                .y_axis_label("°")
                .legend(egui_plot::Legend::default());

            plot.show(ui, |plot_ui| {
                plot_ui.line(
                    Line::new("Motor", motor_points)
                        .stroke((2.0, Color32::from_rgb(255, 140, 0)))
                );
                plot_ui.line(
                    Line::new("AMR", amr_points)
                        .stroke((2.0, Color32::from_rgb(0, 180, 255)))
                );
                plot_ui.line(
                    Line::new("Target", target_points)
                        .stroke((1.5, Color32::from_rgb(255, 60, 60)))
                );
            });
        });
    }
}

fn main() -> eframe::Result<()> {
    let port = std::env::args().nth(1)
        .or_else(serial_comm::find_serial_port)
        .unwrap_or_else(|| {
            eprintln!("No serial port found. Usage: sua-control [/dev/ttyACMx]");
            if let Ok(ports) = serialport::available_ports() {
                for p in ports { eprintln!("  {}", p.port_name); }
            }
            std::process::exit(1);
        });

    eprintln!("Using serial port: {port}");

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 700.0])
            .with_title("SuA Stepper Control"),
        ..Default::default()
    };

    eframe::run_native(
        "SuA Stepper Control",
        options,
        Box::new(move |_cc| Ok(Box::new(SuaApp::new(port)))),
    )
}
