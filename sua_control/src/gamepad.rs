use evdev::{AbsoluteAxisType, Device, InputEventKind};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Default)]
pub struct GamepadInput {
    pub connected: bool,
    pub left_x: f32,
    pub left_y: f32,
    pub right_x: f32,
    pub right_y: f32,
    pub left_trigger: f32,
    pub right_trigger: f32,
    pub dpad_up: bool,
    pub dpad_down: bool,
    pub dpad_left: bool,
    pub dpad_right: bool,
    pub south: bool,  // A
    pub east: bool,   // B
    pub north: bool,  // Y
    pub west: bool,   // X — enable/disable toggle
    pub left_bumper: bool,
    pub right_bumper: bool,
}

fn find_gamepad() -> Option<Device> {
    for (path, dev) in evdev::enumerate() {
        let Some(axes) = dev.supported_absolute_axes() else { continue };
        if axes.contains(AbsoluteAxisType::ABS_X) && axes.contains(AbsoluteAxisType::ABS_Z) {
            match Device::open(&path) {
                Ok(d) => return Some(d),
                Err(e) => eprintln!("Failed to open {path:?}: {e}"),
            }
        }
    }
    None
}

fn normalize_stick(value: i32, min: i32, max: i32) -> f32 {
    let mid = (min + max) / 2;
    let half_range = (max - min) as f32 / 2.0;
    if half_range == 0.0 { return 0.0; }
    ((value - mid) as f32 / half_range).clamp(-1.0, 1.0)
}

fn normalize_trigger(value: i32, min: i32, max: i32) -> f32 {
    let range = (max - min) as f32;
    if range == 0.0 { return 0.0; }
    ((value - min) as f32 / range).clamp(0.0, 1.0)
}

struct AxisRange { min: i32, max: i32 }

impl AxisRange {
    fn from_device(dev: &Device, axis: AbsoluteAxisType) -> Self {
        let info = dev.get_abs_state().unwrap();
        let state = &info[axis.0 as usize];
        Self { min: state.minimum, max: state.maximum }
    }
}

pub fn run(input: Arc<Mutex<GamepadInput>>) {
    loop {
        let mut dev = match find_gamepad() {
            Some(d) => d,
            None => {
                input.lock().unwrap().connected = false;
                std::thread::sleep(std::time::Duration::from_secs(2));
                continue;
            }
        };

        let name = dev.name().unwrap_or("Unknown").to_string();
        eprintln!("Gamepad connected: {name}");

        let lx = AxisRange::from_device(&dev, AbsoluteAxisType::ABS_X);
        let ly = AxisRange::from_device(&dev, AbsoluteAxisType::ABS_Y);
        let rx = AxisRange::from_device(&dev, AbsoluteAxisType::ABS_RX);
        let ry = AxisRange::from_device(&dev, AbsoluteAxisType::ABS_RY);
        let lz = AxisRange::from_device(&dev, AbsoluteAxisType::ABS_Z);
        let rz = AxisRange::from_device(&dev, AbsoluteAxisType::ABS_RZ);

        input.lock().unwrap().connected = true;

        'device: loop {
            let events = match dev.fetch_events() {
                Ok(e) => e,
                Err(_) => {
                    eprintln!("Gamepad disconnected: {name}");
                    *input.lock().unwrap() = GamepadInput::default();
                    break 'device;
                }
            };

            let mut g = input.lock().unwrap();
            for event in events {
                match event.kind() {
                    InputEventKind::AbsAxis(axis) => {
                        let v = event.value();
                        match axis {
                            AbsoluteAxisType::ABS_X => g.left_x = normalize_stick(v, lx.min, lx.max),
                            AbsoluteAxisType::ABS_Y => g.left_y = normalize_stick(v, ly.min, ly.max),
                            AbsoluteAxisType::ABS_RX => g.right_x = normalize_stick(v, rx.min, rx.max),
                            AbsoluteAxisType::ABS_RY => g.right_y = normalize_stick(v, ry.min, ry.max),
                            AbsoluteAxisType::ABS_Z => g.left_trigger = normalize_trigger(v, lz.min, lz.max),
                            AbsoluteAxisType::ABS_RZ => g.right_trigger = normalize_trigger(v, rz.min, rz.max),
                            AbsoluteAxisType::ABS_HAT0X => {
                                g.dpad_left = v < 0;
                                g.dpad_right = v > 0;
                            }
                            AbsoluteAxisType::ABS_HAT0Y => {
                                g.dpad_up = v < 0;
                                g.dpad_down = v > 0;
                            }
                            _ => {}
                        }
                    }
                    InputEventKind::Key(key) => {
                        let pressed = event.value() != 0;
                        match key.0 {
                            304 => g.south = pressed,
                            305 => g.east = pressed,
                            307 => g.north = pressed,
                            308 => g.west = pressed,
                            310 => g.left_bumper = pressed,
                            311 => g.right_bumper = pressed,
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(100));
    }
}
