// NC-style test panel for the DRV8833 stepper firmware on the XIAO ESP32-S3.
// Talks the firmware's G-code-ish protocol over the board's USB serial port.
//
// The firmware works in raw half-steps. This app adds an axis on top: zero at
// the motor face (found by homing), optional direction flip so "away from the
// motor" is positive, soft limits, and mm units from a measured travel.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use eframe::egui;
use egui::{Color32, Key, RichText};
use serde::{Deserialize, Serialize};
use std::io::{Read, Write};
use std::sync::mpsc::{Receiver, Sender, TryRecvError, channel};
use std::thread;
use std::time::{Duration, Instant};

const STATUS_POLL: Duration = Duration::from_millis(100);
const TEST_POLL: Duration = Duration::from_millis(20);
const JOG_FAR: i64 = 1_000_000;
const MAX_LOG_LINES: usize = 2000;

enum ToDevice {
    Line(String),
    Realtime(u8),
}

enum FromDevice {
    Line(String),
    Closed(String),
}

struct Connection {
    tx: Sender<ToDevice>,
    rx: Receiver<FromDevice>,
}

impl Connection {
    fn open(name: &str, ctx: egui::Context) -> Result<Self, serialport::Error> {
        let mut port = serialport::new(name, 115_200)
            .timeout(Duration::from_millis(10))
            .open()?;
        // RTS low keeps the ESP32-S3's USB-Serial-JTAG from holding the chip in reset.
        let _ = port.write_request_to_send(false);
        let _ = port.write_data_terminal_ready(true);

        let (to_dev_tx, to_dev_rx) = channel::<ToDevice>();
        let (from_dev_tx, from_dev_rx) = channel::<FromDevice>();

        thread::spawn(move || {
            let mut pending = Vec::new();
            let mut buf = [0u8; 256];
            let result: Result<(), String> = 'io: loop {
                loop {
                    let bytes = match to_dev_rx.try_recv() {
                        Ok(ToDevice::Line(l)) => format!("{l}\n").into_bytes(),
                        Ok(ToDevice::Realtime(b)) => vec![b],
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => break 'io Ok(()),
                    };
                    if let Err(e) = port.write_all(&bytes) {
                        break 'io Err(e.to_string());
                    }
                }
                match port.read(&mut buf) {
                    Ok(n) => {
                        pending.extend_from_slice(&buf[..n]);
                        while let Some(i) = pending.iter().position(|&b| b == b'\n') {
                            let line: Vec<u8> = pending.drain(..=i).collect();
                            let line = String::from_utf8_lossy(&line).trim().to_string();
                            if !line.is_empty() {
                                let _ = from_dev_tx.send(FromDevice::Line(line));
                            }
                        }
                        ctx.request_repaint();
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(e) => break 'io Err(e.to_string()),
                }
            };
            if let Err(e) = result {
                let _ = from_dev_tx.send(FromDevice::Closed(e));
                ctx.request_repaint();
            }
        });

        Ok(Self {
            tx: to_dev_tx,
            rx: from_dev_rx,
        })
    }
}

#[derive(Default, Clone)]
struct Status {
    running: bool,
    pos: i64,
    feed: u32,
    hold: bool,
    absolute: bool,
}

// Parses "<Idle|MPos:123|F:200|Hold:0|Abs:1>".
fn parse_status(line: &str) -> Option<Status> {
    let inner = line.strip_prefix('<')?.strip_suffix('>')?;
    let mut parts = inner.split('|');
    let mut s = Status {
        running: parts.next()? == "Run",
        ..Default::default()
    };
    for part in parts {
        let (k, v) = part.split_once(':')?;
        match k {
            "MPos" => s.pos = v.parse().ok()?,
            "F" => s.feed = v.parse().ok()?,
            "Hold" => s.hold = v == "1",
            "Abs" => s.absolute = v == "1",
            _ => {}
        }
    }
    Some(s)
}

#[derive(PartialEq)]
enum LogKind {
    Sent,
    Received,
    Error,
    Info,
}

/// Remembered between runs.
#[derive(Serialize, Deserialize)]
#[serde(default)]
struct Settings {
    port: String,
    feed: u32,
    use_mm: bool,
    increment_steps: i64,
    increment_mm: f64,
    /// Flip so that moving away from the motor is positive.
    invert: bool,
    /// Measured travel between the hard stops, in steps and in mm.
    travel_steps: i64,
    travel_mm: f64,
    limits: bool,
    limit_min: i64,
    limit_max: i64,
    home_feed: u32,
    home_backoff: i64,
    test_a_mm: f64,
    test_b_mm: f64,
    test_dwell_ms: u32,
    test_feed: u32,
    /// 0 = run until stopped.
    test_cycles: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            port: String::new(),
            feed: 200,
            use_mm: true,
            increment_steps: 10,
            increment_mm: 0.1,
            // On this rig the motor face reads higher raw steps than the far end.
            invert: true,
            travel_steps: 700,
            // Estimate from the ~5 mm screw; measure and correct.
            travel_mm: 5.0,
            limits: true,
            limit_min: 0,
            // Stay clear of the far end, where the nut barely catches the thread.
            limit_max: 680,
            home_feed: 150,
            home_backoff: 10,
            test_a_mm: 0.25,
            test_b_mm: 4.75,
            test_dwell_ms: 500,
            test_feed: 200,
            test_cycles: 0,
        }
    }
}

impl Settings {
    fn sign(&self) -> i64 {
        if self.invert { -1 } else { 1 }
    }

    fn mm_per_step(&self) -> f64 {
        self.travel_mm / self.travel_steps.max(1) as f64
    }
}

enum Homing {
    /// Creeping into the motor face; waiting for the move to finish.
    Seeking { since: Instant, saw_run: bool },
}

/// A <-> B bounce test. A cycle is A -> dwell -> B -> dwell.
struct TestRun {
    to_b: bool,
    dwell_until: Option<Instant>,
    cycle: u32,
    started: Instant,
    cycle_started: Instant,
    last_cycle: Option<Duration>,
}

struct App {
    settings: Settings,
    ports: Vec<String>,
    conn: Option<Connection>,
    status: Option<Status>,
    last_status: Instant,
    last_poll: Instant,

    homed: bool,
    homing: Option<Homing>,
    test: Option<TestRun>,
    /// Last commanded target in axis steps, so quick repeated jogs stack up.
    target: i64,
    goto_target: f64,
    jogging: Option<i64>,

    mdi: String,
    history: Vec<String>,
    history_pos: usize,
    log: Vec<(LogKind, String)>,
    show_ok: bool,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let settings = cc
            .storage
            .and_then(|s| eframe::get_value(s, eframe::APP_KEY))
            .unwrap_or_default();
        let mut app = Self {
            settings,
            ports: Vec::new(),
            conn: None,
            status: None,
            last_status: Instant::now(),
            last_poll: Instant::now(),
            homed: false,
            homing: None,
            test: None,
            target: 0,
            goto_target: 0.0,
            jogging: None,
            mdi: String::new(),
            history: Vec::new(),
            history_pos: 0,
            log: Vec::new(),
            show_ok: false,
        };
        app.refresh_ports();
        app
    }

    // --- units ---------------------------------------------------------

    /// Current position in axis steps (direction-corrected).
    fn pos(&self) -> i64 {
        self.status.as_ref().map_or(0, |s| s.pos * self.settings.sign())
    }

    fn to_steps(&self, v: f64) -> i64 {
        if self.settings.use_mm {
            (v / self.settings.mm_per_step()).round() as i64
        } else {
            v.round() as i64
        }
    }

    fn from_steps(&self, steps: i64) -> f64 {
        if self.settings.use_mm {
            steps as f64 * self.settings.mm_per_step()
        } else {
            steps as f64
        }
    }

    fn unit(&self) -> &'static str {
        if self.settings.use_mm { "mm" } else { "steps" }
    }

    fn limits_active(&self) -> bool {
        self.settings.limits && self.homed
    }

    fn clamp(&self, steps: i64) -> i64 {
        if self.limits_active() {
            steps.clamp(self.settings.limit_min, self.settings.limit_max)
        } else {
            steps
        }
    }

    // --- connection ----------------------------------------------------

    fn refresh_ports(&mut self) {
        self.ports = serialport::available_ports()
            .map(|ps| ps.into_iter().map(|p| p.port_name).collect())
            .unwrap_or_default();
        if !self.ports.contains(&self.settings.port) {
            self.settings.port = self.ports.first().cloned().unwrap_or_default();
        }
    }

    fn log(&mut self, kind: LogKind, text: impl Into<String>) {
        self.log.push((kind, text.into()));
        if self.log.len() > MAX_LOG_LINES {
            self.log.drain(..self.log.len() - MAX_LOG_LINES);
        }
    }

    fn connect(&mut self, ctx: &egui::Context) {
        let port = self.settings.port.clone();
        match Connection::open(&port, ctx.clone()) {
            Ok(conn) => {
                self.conn = Some(conn);
                self.log(LogKind::Info, format!("Connected to {port}"));
                self.last_status = Instant::now();
                self.send("$");
            }
            Err(e) => self.log(LogKind::Error, format!("Can't open {port}: {e}")),
        }
    }

    fn disconnect(&mut self, reason: &str) {
        if self.conn.take().is_some() {
            self.log(LogKind::Info, reason.to_string());
        }
        self.status = None;
        self.jogging = None;
        self.homing = None;
        self.test = None;
        // The board may reboot while we're away, so the reference is lost.
        self.homed = false;
    }

    fn send(&mut self, line: &str) {
        let Some(conn) = &self.conn else { return };
        if conn.tx.send(ToDevice::Line(line.to_string())).is_ok() {
            self.log(LogKind::Sent, line);
        }
    }

    fn realtime(&mut self, b: u8) {
        if let Some(conn) = &self.conn {
            let _ = conn.tx.send(ToDevice::Realtime(b));
        }
    }

    // --- motion --------------------------------------------------------

    fn stop(&mut self) {
        self.realtime(b'!');
        self.jogging = None;
        if self.homing.take().is_some() {
            self.log(LogKind::Info, "Homing cancelled");
        }
        if let Some(t) = self.test.take() {
            self.log(LogKind::Info, format!("Test stopped after {} cycles", t.cycle));
        }
        self.target = self.pos();
        self.log(LogKind::Sent, "! (stop)");
    }

    fn move_to_steps(&mut self, steps: i64, feed: u32) {
        let steps = self.clamp(steps);
        self.target = steps;
        let raw = steps * self.settings.sign();
        self.send(&format!("G90 G0 X{raw} F{feed}"));
    }

    fn jog_steps(&mut self, delta: i64) {
        let running = self.status.as_ref().is_some_and(|s| s.running);
        let base = if running { self.target } else { self.pos() };
        self.move_to_steps(base + delta, self.settings.feed);
    }

    fn jog_increment(&mut self, dir: i64) {
        let inc = if self.settings.use_mm {
            self.to_steps(self.settings.increment_mm).max(1)
        } else {
            self.settings.increment_steps
        };
        self.jog_steps(dir * inc);
    }

    fn start_continuous_jog(&mut self, dir: i64) {
        let end = if self.limits_active() {
            if dir > 0 { self.settings.limit_max } else { self.settings.limit_min }
        } else {
            self.pos() + dir * JOG_FAR
        };
        self.move_to_steps(end, self.settings.feed);
    }

    fn start_homing(&mut self) {
        // Run toward the motor face for more than the full travel; the motor
        // just skips steps against the stop at this low speed.
        let distance = self.settings.travel_steps + 50;
        let raw = -distance * self.settings.sign();
        self.send(&format!("G91 G0 X{raw} F{}", self.settings.home_feed));
        self.homing = Some(Homing::Seeking {
            since: Instant::now(),
            saw_run: false,
        });
        self.log(LogKind::Info, "Homing toward motor face...");
    }

    fn update_homing(&mut self) {
        let running = self.status.as_ref().is_some_and(|s| s.running);
        let Some(Homing::Seeking { since, saw_run }) = &mut self.homing else { return };
        if running {
            *saw_run = true;
            return;
        }
        if !*saw_run && since.elapsed() < Duration::from_secs(1) {
            return;
        }
        self.homing = None;
        self.send("G92 X0");
        self.homed = true;
        self.target = 0;
        self.log(LogKind::Info, "Homed: motor face is X0");
        if self.settings.home_backoff > 0 {
            self.move_to_steps(self.settings.home_backoff, self.settings.home_feed);
        }
    }

    /// Faster while testing so arrival and dwell timing stay tight.
    fn poll_interval(&self) -> Duration {
        if self.test.is_some() { TEST_POLL } else { STATUS_POLL }
    }

    fn test_point(&self, to_b: bool) -> i64 {
        let mm = if to_b { self.settings.test_b_mm } else { self.settings.test_a_mm };
        self.clamp((mm / self.settings.mm_per_step()).round() as i64)
    }

    fn start_test(&mut self) {
        let now = Instant::now();
        self.test = Some(TestRun {
            to_b: false,
            dwell_until: None,
            cycle: 0,
            started: now,
            cycle_started: now,
            last_cycle: None,
        });
        let s = &self.settings;
        let msg = format!(
            "Test: A {} mm <-> B {} mm, dwell {} ms, F{}",
            s.test_a_mm, s.test_b_mm, s.test_dwell_ms, s.test_feed
        );
        self.log(LogKind::Info, msg);
        self.move_to_steps(self.test_point(false), self.settings.test_feed);
    }

    fn update_test(&mut self) {
        let Some(t) = &self.test else { return };
        let to_b = t.to_b;
        match t.dwell_until {
            None => {
                // Arrived once the firmware is idle at the commanded point.
                let arrived = self
                    .status
                    .as_ref()
                    .is_some_and(|s| !s.running && s.pos * self.settings.sign() == self.target);
                if arrived {
                    let dwell = Duration::from_millis(self.settings.test_dwell_ms as u64);
                    self.test.as_mut().unwrap().dwell_until = Some(Instant::now() + dwell);
                }
            }
            Some(until) if Instant::now() >= until => {
                let limit = self.settings.test_cycles;
                let t = self.test.as_mut().unwrap();
                t.dwell_until = None;
                if to_b {
                    // Finished B's dwell: that's one full cycle.
                    t.cycle += 1;
                    t.last_cycle = Some(t.cycle_started.elapsed());
                    t.cycle_started = Instant::now();
                    if limit > 0 && t.cycle >= limit {
                        let msg = format!("Test done: {} cycles in {:.1} s", t.cycle, t.started.elapsed().as_secs_f64());
                        self.test = None;
                        self.log(LogKind::Info, msg);
                        return;
                    }
                }
                t.to_b = !to_b;
                self.move_to_steps(self.test_point(!to_b), self.settings.test_feed);
            }
            Some(_) => {}
        }
    }

    fn set_zero_here(&mut self) {
        self.send("G92 X0");
        self.homed = true;
        self.target = 0;
    }

    fn poll_device(&mut self) {
        let mut lines = Vec::new();
        let mut closed = None;
        if let Some(conn) = &self.conn {
            loop {
                match conn.rx.try_recv() {
                    Ok(FromDevice::Line(l)) => lines.push(l),
                    Ok(FromDevice::Closed(e)) => {
                        closed = Some(format!("Disconnected: {e}"));
                        break;
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        closed = Some("Disconnected".to_string());
                        break;
                    }
                }
            }
        }
        for line in lines {
            if let Some(s) = parse_status(&line) {
                self.status = Some(s);
                self.last_status = Instant::now();
                self.update_homing();
            } else if line.starts_with("error") {
                self.log(LogKind::Error, line);
            } else if line != "ok" || self.show_ok {
                self.log(LogKind::Received, line);
            }
        }
        if let Some(reason) = closed {
            self.disconnect(&reason);
        }
        self.update_test();

        if self.conn.is_some() && self.last_poll.elapsed() >= self.poll_interval() {
            self.realtime(b'?');
            self.last_poll = Instant::now();
        }
    }

    // --- UI ------------------------------------------------------------

    fn connection_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let connected = self.conn.is_some();
            ui.add_enabled_ui(!connected, |ui| {
                let selected = if self.settings.port.is_empty() {
                    "no ports".to_string()
                } else {
                    self.settings.port.clone()
                };
                egui::ComboBox::from_id_salt("port")
                    .selected_text(selected)
                    .show_ui(ui, |ui| {
                        for p in self.ports.clone() {
                            ui.selectable_value(&mut self.settings.port, p.clone(), p);
                        }
                    });
                if ui.button("⟳").on_hover_text("Refresh ports").clicked() {
                    self.refresh_ports();
                }
            });
            if connected {
                if ui.button("Disconnect").clicked() {
                    self.disconnect("Disconnected");
                }
            } else if ui
                .add_enabled(!self.settings.port.is_empty(), egui::Button::new("Connect"))
                .clicked()
            {
                self.connect(ui.ctx());
            }

            ui.separator();
            let (text, color) = match (&self.conn, &self.status) {
                (None, _) => ("OFFLINE", Color32::GRAY),
                (Some(_), _) if self.last_status.elapsed() > Duration::from_secs(1) => {
                    ("NO RESPONSE", Color32::from_rgb(230, 160, 40))
                }
                (Some(_), _) if self.homing.is_some() => ("HOMING", Color32::from_rgb(200, 130, 255)),
                (Some(_), Some(s)) if s.running => ("RUN", Color32::from_rgb(80, 170, 255)),
                (Some(_), _) => ("IDLE", Color32::from_rgb(90, 200, 110)),
            };
            ui.label(RichText::new(text).strong().color(color));
            if self.conn.is_some() && !self.homed {
                ui.label(RichText::new("NOT HOMED").strong().color(Color32::from_rgb(230, 160, 40)));
            }
        });
    }

    fn dro(&mut self, ui: &mut egui::Ui) {
        let s = self.status.clone().unwrap_or_default();
        let pos = self.pos();
        let mm = pos as f64 * self.settings.mm_per_step();
        egui::Frame::group(ui.style())
            .fill(Color32::from_rgb(16, 20, 24))
            .show(ui, |ui| {
                ui.set_min_width(ui.available_width());
                let big = |t: String| RichText::new(t).monospace().size(40.0);
                let small = |t: String| RichText::new(t).monospace().size(15.0);
                let color = if self.homed {
                    Color32::from_rgb(110, 230, 140)
                } else {
                    Color32::from_rgb(230, 160, 40)
                };
                let (main, secondary) = if self.settings.use_mm {
                    (format!("{mm:+08.3}"), format!("{pos:+} steps"))
                } else {
                    (format!("{pos:+06}"), format!("{mm:+.3} mm"))
                };
                ui.horizontal(|ui| {
                    ui.label(big("X".into()).color(Color32::LIGHT_GRAY));
                    ui.label(big(main).color(color));
                    ui.label(small(self.unit().into()).color(Color32::GRAY));
                });
                let range = if self.limits_active() {
                    let lo = self.from_steps(self.settings.limit_min);
                    let hi = self.from_steps(self.settings.limit_max);
                    format!("limits {lo} .. {hi} {}", self.unit())
                } else if self.settings.limits {
                    "limits off until homed".to_string()
                } else {
                    "limits disabled".to_string()
                };
                ui.label(small(format!("{secondary}   {range}")).color(Color32::LIGHT_GRAY));
                ui.label(
                    small(format!(
                        "F {} steps/s   raw {}   coils {}",
                        s.feed,
                        s.pos,
                        if s.hold { "HOLD" } else { "released" }
                    ))
                    .color(Color32::GRAY),
                );
            });
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        let enabled = self.conn.is_some();

        let estop = egui::Button::new(RichText::new("STOP").size(22.0).strong().color(Color32::WHITE))
            .fill(Color32::from_rgb(200, 30, 30))
            .min_size(egui::vec2(ui.available_width(), 44.0));
        if ui.add_enabled(enabled, estop).on_hover_text("Esc").clicked() {
            self.stop();
        }
        ui.add_space(8.0);

        self.test_panel(ui);
        ui.add_space(8.0);

        ui.add_enabled_ui(enabled && self.test.is_none(), |ui| {
            ui.horizontal(|ui| {
                ui.heading("Jog");
                ui.add_space(12.0);
                ui.selectable_value(&mut self.settings.use_mm, true, "mm");
                ui.selectable_value(&mut self.settings.use_mm, false, "steps");
            });
            ui.horizontal(|ui| {
                ui.label("Increment");
                if self.settings.use_mm {
                    for inc in [0.01, 0.05, 0.1, 0.5, 1.0] {
                        ui.selectable_value(&mut self.settings.increment_mm, inc, inc.to_string());
                    }
                } else {
                    for inc in [1, 10, 50, 100] {
                        ui.selectable_value(&mut self.settings.increment_steps, inc, inc.to_string());
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.label("Feed");
                ui.add(
                    egui::Slider::new(&mut self.settings.feed, 1..=1000)
                        .logarithmic(true)
                        .suffix(" steps/s"),
                );
            });
            ui.horizontal(|ui| {
                let size = egui::vec2(80.0, 36.0);
                if ui.add(egui::Button::new("◀ X−").min_size(size)).on_hover_text("← arrow").clicked() {
                    self.jog_increment(-1);
                }
                if ui.add(egui::Button::new("X+ ▶").min_size(size)).on_hover_text("→ arrow").clicked() {
                    self.jog_increment(1);
                }
                ui.separator();
                let minus = ui.add(egui::Button::new("◀◀ hold").min_size(size));
                let plus = ui.add(egui::Button::new("hold ▶▶").min_size(size));
                let held = if minus.is_pointer_button_down_on() {
                    Some(-1)
                } else if plus.is_pointer_button_down_on() {
                    Some(1)
                } else {
                    None
                };
                // Continuous jog: run toward the limit (or far away), stop on release.
                if held != self.jogging {
                    match held {
                        Some(dir) => {
                            self.start_continuous_jog(dir);
                            self.jogging = held;
                        }
                        None => self.stop(),
                    }
                }
            });

            ui.add_space(8.0);
            ui.heading("Position");
            ui.horizontal(|ui| {
                let speed = if self.settings.use_mm { 0.01 } else { 1.0 };
                let suffix = format!(" {}", self.unit());
                ui.add(
                    egui::DragValue::new(&mut self.goto_target)
                        .speed(speed)
                        .prefix("X ")
                        .suffix(suffix),
                );
                if ui.button("Go to").clicked() {
                    let steps = self.to_steps(self.goto_target);
                    self.move_to_steps(steps, self.settings.feed);
                }
                if ui.add_enabled(self.homed, egui::Button::new("X0")).clicked() {
                    self.move_to_steps(0, self.settings.feed);
                }
                if ui.add_enabled(self.homed, egui::Button::new("Far end")).clicked() {
                    self.move_to_steps(self.settings.limit_max, self.settings.feed);
                }
            });
            ui.horizontal(|ui| {
                let homing = self.homing.is_some();
                if ui
                    .add_enabled(!homing, egui::Button::new("⌂ Home to motor"))
                    .on_hover_text("Creep into the motor face, then set X0 there")
                    .clicked()
                {
                    self.start_homing();
                }
                if ui.button("Set X0 here").clicked() {
                    self.set_zero_here();
                }
            });
            ui.horizontal(|ui| {
                let hold = self.status.as_ref().is_some_and(|s| s.hold);
                if ui.selectable_label(hold, "Hold coils (M17)").clicked() {
                    self.send("M17");
                }
                if ui.selectable_label(!hold, "Release (M18)").clicked() {
                    self.send("M18");
                }
            });
        });

        ui.add_space(8.0);
        egui::CollapsingHeader::new("Axis setup").show(ui, |ui| self.axis_setup(ui));
    }

    fn test_panel(&mut self, ui: &mut egui::Ui) {
        let can_start = self.conn.is_some() && self.homed && self.homing.is_none();
        ui.heading("A ↔ B test");
        let running = self.test.is_some();
        ui.add_enabled_ui(!running, |ui| {
            let s = &mut self.settings;
            fn mm<'a>(v: &'a mut f64, label: &str) -> egui::DragValue<'a> {
                egui::DragValue::new(v)
                    .speed(0.01)
                    .range(-1000.0..=1000.0)
                    .max_decimals(3)
                    .prefix(format!("{label} "))
                    .suffix(" mm")
            }
            ui.horizontal(|ui| {
                ui.add(mm(&mut s.test_a_mm, "A"));
                ui.add(mm(&mut s.test_b_mm, "B"));
                ui.add(
                    egui::DragValue::new(&mut s.test_dwell_ms)
                        .range(0..=60_000)
                        .speed(10)
                        .prefix("dwell ")
                        .suffix(" ms"),
                );
            });
            ui.horizontal(|ui| {
                ui.add(
                    egui::DragValue::new(&mut s.test_feed)
                        .range(1..=1000)
                        .prefix("F ")
                        .suffix(" steps/s"),
                );
                ui.add(egui::DragValue::new(&mut s.test_cycles).range(0..=1_000_000).prefix("cycles "));
                ui.label(RichText::new(if s.test_cycles == 0 { "(until stopped)" } else { "" }).weak());
            });
        });

        ui.horizontal(|ui| {
            if running {
                if ui.button("■ Stop test").clicked() {
                    self.stop();
                }
            } else if ui
                .add_enabled(can_start, egui::Button::new("▶ Start test"))
                .on_disabled_hover_text("Connect and home first")
                .clicked()
            {
                self.start_test();
            }

            if let Some(t) = &self.test {
                let phase = match (t.to_b, t.dwell_until) {
                    (false, None) => "→ A".to_string(),
                    (true, None) => "→ B".to_string(),
                    (b, Some(until)) => {
                        let left = until.saturating_duration_since(Instant::now()).as_secs_f64();
                        format!("dwell at {} {left:.2}s", if b { "B" } else { "A" })
                    }
                };
                let total = match self.settings.test_cycles {
                    0 => String::new(),
                    n => format!(" / {n}"),
                };
                let last = t
                    .last_cycle
                    .map(|d| format!("   last cycle {:.2}s", d.as_secs_f64()))
                    .unwrap_or_default();
                ui.label(RichText::new(format!("cycle {}{total}   {phase}{last}", t.cycle + 1)).monospace());
            }
        });
    }

    fn axis_setup(&mut self, ui: &mut egui::Ui) {
        let s = &mut self.settings;
        ui.checkbox(&mut s.invert, "Invert direction (away from motor = +)");
        ui.horizontal(|ui| {
            ui.label("Travel");
            ui.add(egui::DragValue::new(&mut s.travel_steps).range(1..=100_000).suffix(" steps"));
            ui.label("=");
            ui.add(
                egui::DragValue::new(&mut s.travel_mm)
                    .range(0.001..=1000.0)
                    .speed(0.01)
                    .max_decimals(3)
                    .suffix(" mm"),
            );
        });
        ui.label(
            RichText::new(format!(
                "{:.5} mm/step, {:.1} steps/mm",
                s.mm_per_step(),
                1.0 / s.mm_per_step()
            ))
            .weak(),
        );
        ui.horizontal(|ui| {
            ui.checkbox(&mut s.limits, "Soft limits");
            ui.add(egui::DragValue::new(&mut s.limit_min).suffix(" steps"));
            ui.label("..");
            ui.add(egui::DragValue::new(&mut s.limit_max).suffix(" steps"));
        });
        ui.horizontal(|ui| {
            ui.label("Homing");
            ui.add(egui::DragValue::new(&mut s.home_feed).range(1..=1000).suffix(" steps/s"));
            ui.label("back off");
            ui.add(egui::DragValue::new(&mut s.home_backoff).range(0..=1000).suffix(" steps"));
        });
        if ui.small_button("Reset to defaults").clicked() {
            let port = std::mem::take(&mut s.port);
            *s = Settings { port, ..Default::default() };
        }
    }

    fn mdi_and_log(&mut self, ui: &mut egui::Ui) {
        ui.heading("MDI");
        ui.label(RichText::new("Raw firmware steps; soft limits and direction flip don't apply here.").weak());
        ui.horizontal(|ui| {
            let edit = ui.add_enabled(
                self.conn.is_some(),
                egui::TextEdit::singleline(&mut self.mdi)
                    .hint_text("e.g. G91 G0 X100 F300")
                    .desired_width(ui.available_width() - 60.0)
                    .font(egui::TextStyle::Monospace),
            );
            if edit.has_focus() {
                let (up, down) = ui.input(|i| (i.key_pressed(Key::ArrowUp), i.key_pressed(Key::ArrowDown)));
                if up && self.history_pos > 0 {
                    self.history_pos -= 1;
                    self.mdi = self.history[self.history_pos].clone();
                }
                if down && self.history_pos < self.history.len() {
                    self.history_pos += 1;
                    self.mdi = self.history.get(self.history_pos).cloned().unwrap_or_default();
                }
            }
            let enter = edit.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));
            if (ui.add_enabled(self.conn.is_some(), egui::Button::new("Send")).clicked() || enter)
                && !self.mdi.trim().is_empty()
            {
                let line = self.mdi.trim().to_string();
                self.send(&line);
                if self.history.last() != Some(&line) {
                    self.history.push(line);
                }
                self.history_pos = self.history.len();
                self.mdi.clear();
                edit.request_focus();
            }
        });

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Console");
            ui.checkbox(&mut self.show_ok, "show ok");
            if ui.small_button("Clear").clicked() {
                self.log.clear();
            }
        });
        egui::Frame::group(ui.style()).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink(false)
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for (kind, text) in &self.log {
                        let (prefix, color) = match kind {
                            LogKind::Sent => ("> ", Color32::from_rgb(120, 170, 255)),
                            LogKind::Received => ("< ", ui.visuals().text_color()),
                            LogKind::Error => ("! ", Color32::from_rgb(240, 90, 90)),
                            LogKind::Info => ("# ", Color32::GRAY),
                        };
                        ui.label(RichText::new(format!("{prefix}{text}")).monospace().color(color));
                    }
                });
        });
    }

    fn keyboard(&mut self, ctx: &egui::Context) {
        if self.conn.is_none() {
            return;
        }
        if ctx.input(|i| i.key_pressed(Key::Escape)) {
            self.stop();
        }
        // Arrow jogs only when no text field has focus and no test is running.
        if self.test.is_none() && ctx.memory(|m| m.focused().is_none()) {
            let (left, right) = ctx.input(|i| (i.key_pressed(Key::ArrowLeft), i.key_pressed(Key::ArrowRight)));
            if left {
                self.jog_increment(-1);
            }
            if right {
                self.jog_increment(1);
            }
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.poll_device();
        self.keyboard(&ctx);

        egui::Panel::top("connection").show(ui, |ui| {
            ui.add_space(4.0);
            self.connection_bar(ui);
            ui.add_space(4.0);
        });
        egui::Panel::left("controls")
            .resizable(false)
            .exact_size(440.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.add_space(6.0);
                    self.dro(ui);
                    ui.add_space(8.0);
                    self.controls(ui);
                });
            });
        egui::CentralPanel::default_margins().show(ui, |ui| {
            self.mdi_and_log(ui);
        });

        if self.conn.is_some() {
            ctx.request_repaint_after(self.poll_interval());
        }
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, &self.settings);
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Drv8833tinyNC")
            .with_inner_size([980.0, 680.0])
            .with_min_inner_size([780.0, 520.0]),
        ..Default::default()
    };
    eframe::run_native("Drv8833tinyNC", options, Box::new(|cc| Ok(Box::new(App::new(cc)))))
}
