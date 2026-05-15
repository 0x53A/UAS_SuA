use serde::Deserialize;
use std::io::{BufRead, BufReader};
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Debug, Clone, Default, Deserialize)]
pub struct Telemetry {
    #[serde(default)]
    pub mode: String,
    #[serde(default)]
    pub motor: i32,   // 0.01 deg
    #[serde(default)]
    pub amr: i32,     // 0.01 deg
    #[serde(default)]
    pub target: i32,  // 0.01 deg
    #[serde(default)]
    pub vel: i16,     // %*100
    #[serde(default)]
    pub sps: i32,     // steps/sec
    #[serde(default)]
    pub en: u8,
    #[serde(default)]
    pub cal: u8,
    #[serde(default)]
    pub rem: i32,     // steps remaining
    #[serde(default)]
    pub msg: String,
}

impl Telemetry {
    pub fn motor_deg(&self) -> f64 { self.motor as f64 / 100.0 }
    pub fn amr_deg(&self) -> f64 { self.amr as f64 / 100.0 }
    pub fn target_deg(&self) -> f64 { self.target as f64 / 100.0 }
    pub fn vel_pct(&self) -> f64 { self.vel as f64 / 100.0 }
    pub fn deg_per_sec(&self) -> f64 { self.sps as f64 * (360.0 / 12800.0) }
}

pub enum Command {
    Velocity(i16),                   // %*100
    Position(i32, u16),              // 0.01deg, max_speed %*100
    Calibrate(i32),                  // 0.01deg
    Enable(bool),
}

impl Command {
    pub fn to_wire(&self) -> String {
        match self {
            Command::Velocity(v) => format!("|SuA|V|{v}|\n"),
            Command::Position(pos, spd) => format!("|SuA|P|{pos}|{spd}|\n"),
            Command::Calibrate(angle) => format!("|SuA|C|{angle}|\n"),
            Command::Enable(en) => format!("|SuA|E|{}|\n", if *en { 1 } else { 0 }),
        }
    }
}

pub fn find_serial_port() -> Option<String> {
    let ports = serialport::available_ports().ok()?;
    for p in &ports {
        let name = &p.port_name;
        if name.contains("ttyACM") || name.contains("ttyUSB") {
            return Some(name.clone());
        }
    }
    ports.first().map(|p| p.port_name.clone())
}

pub type SharedWriter = Arc<Mutex<Option<Box<dyn serialport::SerialPort>>>>;

pub fn run_reader(
    port_name: &str,
    telemetry: Arc<Mutex<Telemetry>>,
    connected: Arc<Mutex<bool>>,
    writer: SharedWriter,
) {
    loop {
        let port = serialport::new(port_name, 115200)
            .timeout(Duration::from_millis(100))
            .open();

        let port = match port {
            Ok(p) => {
                *connected.lock().unwrap() = true;
                match p.try_clone() {
                    Ok(mut w) => {
                        let _ = w.set_timeout(Duration::from_secs(1));
                        *writer.lock().unwrap() = Some(w);
                    }
                    Err(e) => eprintln!("Serial clone failed: {e}"),
                }
                eprintln!("Serial connected: {port_name}");
                p
            }
            Err(e) => {
                *connected.lock().unwrap() = false;
                eprintln!("Serial open failed: {e}");
                std::thread::sleep(Duration::from_secs(2));
                continue;
            }
        };

        let reader = BufReader::new(port);
        for line in reader.lines() {
            match line {
                Ok(line) => {
                    if let Ok(t) = serde_json::from_str::<Telemetry>(&line) {
                        *telemetry.lock().unwrap() = t;
                    }
                }
                Err(e) => {
                    if e.kind() != std::io::ErrorKind::TimedOut {
                        eprintln!("Serial read error: {e}");
                        break;
                    }
                }
            }
        }

        *writer.lock().unwrap() = None;
        *connected.lock().unwrap() = false;
        eprintln!("Serial disconnected, reconnecting...");
        std::thread::sleep(Duration::from_secs(1));
    }
}
