// MicrometerLogger
// Author: J Taylor
// Supplied without warranty.
// Use at your own risk.


// src/main.rs
//
// Serial data logger + plotter UI for an optical micrometer (generic).
//
// Cargo.toml deps used here:
// serialport, chrono, plotters, plotters-iced, iced (canvas,tokio,advanced), rfd,
// dirs, serde (derive), serde_json

use serialport::SerialPort;
use std::fs::{self, OpenOptions};
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::thread;
use std::time::{Duration, SystemTime};

use chrono::Local;
use iced::time;
use iced::widget::{button, column, pick_list, row, text, text_input};
use iced::{Alignment, Color, Element, Length, Size, Subscription, Task};

use plotters::prelude::*; // includes FontTransform in plotters 0.3.6+
use plotters::style::ShapeStyle;
use plotters_iced::{Chart, ChartWidget, DrawingBackend};
use rfd::FileDialog;

use serde::{Deserialize, Serialize};

// ---------------- UI Messages ----------------

#[derive(Debug, Clone)]
enum Message {
    Connect,
    Disconnect,
    StartLogging, // "start new roll"
    PauseLogging, // toggle pause/resume
    EndRoll,      // finish roll without quitting
    Quit,
    SetUcl(String),
    SetLcl(String),
    SavePlot,
    ExportCsv,
    RefreshPorts,
    PortSelected(PortEntry),
    BaudRateSelected(u32),
    LabelPrefixChanged(String),
    DataBitsSelected(DataBitsOpt),
    ParitySelected(ParityOpt),
    StopBitsSelected(StopBitsOpt),
    Tick,
}

// ---------------- Serial thread messages/commands ----------------

#[derive(Debug, Clone)]
enum SerialMessage {
    Measurement(f32, String),
    Error(String),
    Invalid(String, String), // response, timestamp (e.g. "--" out of range)
}

#[derive(Debug, Clone)]
enum SerialCmd {
    Pause(bool),
    Stop,
}

// ---------------- Serial config dropdowns ----------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum DataBitsOpt {
    Seven,
    Eight,
}

impl std::fmt::Display for DataBitsOpt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DataBitsOpt::Seven => write!(f, "7"),
            DataBitsOpt::Eight => write!(f, "8"),
        }
    }
}

impl From<DataBitsOpt> for serialport::DataBits {
    fn from(v: DataBitsOpt) -> Self {
        match v {
            DataBitsOpt::Seven => serialport::DataBits::Seven,
            DataBitsOpt::Eight => serialport::DataBits::Eight,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum ParityOpt {
    None,
    Even,
    Odd,
}

impl std::fmt::Display for ParityOpt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParityOpt::None => write!(f, "None"),
            ParityOpt::Even => write!(f, "Even"),
            ParityOpt::Odd => write!(f, "Odd"),
        }
    }
}

impl From<ParityOpt> for serialport::Parity {
    fn from(v: ParityOpt) -> Self {
        match v {
            ParityOpt::None => serialport::Parity::None,
            ParityOpt::Even => serialport::Parity::Even,
            ParityOpt::Odd => serialport::Parity::Odd,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum StopBitsOpt {
    One,
    Two,
}

impl std::fmt::Display for StopBitsOpt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StopBitsOpt::One => write!(f, "1"),
            StopBitsOpt::Two => write!(f, "2"),
        }
    }
}

impl From<StopBitsOpt> for serialport::StopBits {
    fn from(v: StopBitsOpt) -> Self {
        match v {
            StopBitsOpt::One => serialport::StopBits::One,
            StopBitsOpt::Two => serialport::StopBits::Two,
        }
    }
}

// ---------------- Port dropdown item ----------------

#[derive(Debug, Clone, PartialEq, Eq)]
struct PortEntry {
    port_name: String, // e.g. "COM7"
    display: String,   // e.g. "COM7: USB Serial ..."
}

impl std::fmt::Display for PortEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.display)
    }
}

// ---------------- Settings file ----------------

#[derive(Debug, Clone, Serialize, Deserialize)]
struct AppSettings {
    port_name: String,
    baud_rate: u32,
    data_bits: DataBitsOpt,
    parity: ParityOpt,
    stop_bits: StopBitsOpt,
    ucl: f32,
    lcl: f32,
    label_prefix: String,
}

impl AppSettings {
    fn from_app(app: &MicrometerLoggerApp) -> Self {
        Self {
            port_name: app.port_name.clone(),
            baud_rate: app.baud_rate,
            data_bits: app.data_bits,
            parity: app.parity,
            stop_bits: app.stop_bits,
            ucl: app.ucl,
            lcl: app.lcl,
            label_prefix: app.label_prefix.clone(),
        }
    }
}

// ---------------- App ----------------

struct MicrometerLoggerApp {
    // serial
    port: Option<Box<dyn SerialPort>>,
    serial_cmd_tx: Option<Sender<SerialCmd>>,
    serial_rx: Option<Receiver<SerialMessage>>,
    serial_paused: bool,

    // available COM ports
    available_ports: Vec<PortEntry>,
    selected_port: Option<PortEntry>,

    // file
    file: Option<std::fs::File>,

    // data
    measurements: Vec<(f32, f32)>,        // rolling points for plot (elapsed_s, value)
    all_measurements: Vec<(String, f32)>, // full session for export (timestamp, value)
    start_time: SystemTime,
    logging: bool,

    // pause accounting (prevents "line back to start")
    paused_at: Option<SystemTime>,
    paused_total: Duration,

    // limits
    ucl: f32,
    lcl: f32,
    ucl_input: String,
    lcl_input: String,

    // config
    port_name: String,
    baud_rate: u32,
    data_bits: DataBitsOpt,
    parity: ParityOpt,
    stop_bits: StopBitsOpt,
    max_display_points: usize,
    label_prefix: String,

    // stats / ui
    current_measurement: Option<f32>,
    std_dev: Option<f64>,

    // validity/staleness
    last_measurement_time: Option<SystemTime>,
    last_invalid_time: Option<SystemTime>,
    no_data_reason: Option<String>,

    // last error (user visible)
    error: Option<String>,
}

impl Default for MicrometerLoggerApp {
    fn default() -> Self {
        let mut app = Self {
            port: None,
            serial_cmd_tx: None,
            serial_rx: None,
            serial_paused: true,

            available_ports: Vec::new(),
            selected_port: None,

            file: None,

            measurements: Vec::new(),
            all_measurements: Vec::new(),
            start_time: SystemTime::now(),
            logging: false,

            paused_at: None,
            paused_total: Duration::from_secs(0),

            ucl: 6.5,
            lcl: 6.0,
            ucl_input: "6.5".to_string(),
            lcl_input: "6.0".to_string(),

            port_name: "COM6".to_string(),
            baud_rate: 9600,
            data_bits: DataBitsOpt::Eight,
            parity: ParityOpt::None,
            stop_bits: StopBitsOpt::One,

            max_display_points: 300,
            label_prefix: String::new(),

            current_measurement: None,
            std_dev: None,

            last_measurement_time: None,
            last_invalid_time: None,
            no_data_reason: Some("Disconnected".to_string()),

            error: None,
        };

        // Load settings first
        if let Err(e) = app.load_settings_from_disk() {
            eprintln!("Settings load: {e}");
        }

        // Scan ports and update selected entry
        app.available_ports = Self::scan_ports();
        app.selected_port = app
            .available_ports
            .iter()
            .find(|p| p.port_name.eq_ignore_ascii_case(&app.port_name))
            .cloned();

        app
    }
}

impl MicrometerLoggerApp {
    fn title(&self) -> String {
        "Optical Micrometer Logger".to_string()
    }

    /// Universal log folder: Documents\OpticalMicrometer_logs
    /// Falls back to Desktop, then current directory.
    fn log_dir() -> PathBuf {
        let base = dirs::document_dir()
            .or_else(dirs::desktop_dir)
            .unwrap_or_else(|| std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")));
        base.join("OpticalMicrometer_logs")
    }

    fn settings_path() -> PathBuf {
        Self::log_dir().join("settings.json")
    }

    fn ensure_log_dir() -> std::io::Result<()> {
        fs::create_dir_all(Self::log_dir())
    }

    fn load_settings_from_disk(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let path = Self::settings_path();
        if !path.exists() {
            return Ok(());
        }

        let mut s = String::new();
        std::fs::File::open(&path)?.read_to_string(&mut s)?;
        let cfg: AppSettings = serde_json::from_str(&s)?;

        self.port_name = cfg.port_name;
        self.baud_rate = cfg.baud_rate;
        self.data_bits = cfg.data_bits;
        self.parity = cfg.parity;
        self.stop_bits = cfg.stop_bits;

        self.ucl = cfg.ucl;
        self.lcl = cfg.lcl;
        self.ucl_input = format!("{}", self.ucl);
        self.lcl_input = format!("{}", self.lcl);

        self.label_prefix = cfg.label_prefix;

        Ok(())
    }

    fn save_settings_to_disk(&self) -> Result<(), Box<dyn std::error::Error>> {
        Self::ensure_log_dir()?;
        let path = Self::settings_path();
        let tmp = path.with_extension("json.tmp");

        let cfg = AppSettings::from_app(self);
        let json = serde_json::to_string_pretty(&cfg)?;

        std::fs::write(&tmp, json.as_bytes())?;
        if path.exists() {
            let _ = std::fs::remove_file(&path);
        }
        std::fs::rename(&tmp, &path)?;
        Ok(())
    }

    fn make_prefix(&self) -> String {
        if self.label_prefix.trim().is_empty() {
            "".to_string()
        } else {
            format!("{}_", self.label_prefix.trim().replace(' ', "_"))
        }
    }

    fn scan_ports() -> Vec<PortEntry> {
        let mut out = Vec::new();

        if let Ok(ports) = serialport::available_ports() {
            for p in ports {
                let mut display = p.port_name.clone();

                match p.port_type {
                    serialport::SerialPortType::UsbPort(info) => {
                        let mut parts = Vec::new();
                        if let Some(m) = info.manufacturer {
                            parts.push(m);
                        }
                        if let Some(prod) = info.product {
                            parts.push(prod);
                        }

                        if !parts.is_empty() {
                            display = format!("{}: {}", p.port_name, parts.join(" "));
                        } else {
                            display = format!("{}: USB Serial", p.port_name);
                        }
                    }
                    serialport::SerialPortType::BluetoothPort => {
                        display = format!("{}: Bluetooth", p.port_name)
                    }
                    serialport::SerialPortType::PciPort => display = format!("{}: PCI", p.port_name),
                    serialport::SerialPortType::Unknown => {}
                }

                out.push(PortEntry {
                    port_name: p.port_name,
                    display,
                });
            }
        }

        out.sort_by(|a, b| a.display.cmp(&b.display));
        out
    }

    fn open_port(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let port = serialport::new(&self.port_name, self.baud_rate)
            .timeout(Duration::from_millis(500))
            .data_bits(self.data_bits.into())
            .parity(self.parity.into())
            .stop_bits(self.stop_bits.into())
            .open()?;

        self.port = Some(port);
        Ok(())
    }

    fn close_port(&mut self) {
        self.stop_serial_thread();
        self.port = None;
    }

    fn open_file(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        fs::create_dir_all(Self::log_dir())?;

        let timestamp = Local::now().format("%Y%m%d_%H%M%S").to_string();
        let prefix = self.make_prefix();
        let log_path = Self::log_dir().join(format!("{}measurements_{}.csv", prefix, timestamp));

        let mut file = OpenOptions::new().write(true).create(true).open(&log_path)?;
        writeln!(file, "Timestamp,Measurement")?;
        self.file = Some(file);
        Ok(())
    }

    fn start_serial_thread_if_needed(&mut self) {
        if self.port.is_none() {
            return;
        }
        if self.serial_cmd_tx.is_some() && self.serial_rx.is_some() {
            return;
        }

        let (cmd_tx, cmd_rx) = channel::<SerialCmd>();
        let (data_tx, data_rx) = channel::<SerialMessage>();

        self.serial_cmd_tx = Some(cmd_tx);
        self.serial_rx = Some(data_rx);

        let port = match self.port.as_ref().unwrap().try_clone() {
            Ok(p) => p,
            Err(e) => {
                self.error = Some(format!("Failed to clone serial port: {e}"));
                self.serial_cmd_tx = None;
                self.serial_rx = None;
                return;
            }
        };

        thread::spawn(move || {
            let mut paused = true;
            let mut port = port;

            loop {
                match cmd_rx.try_recv() {
                    Ok(SerialCmd::Pause(p)) => paused = p,
                    Ok(SerialCmd::Stop) => break,
                    Err(TryRecvError::Empty) => {}
                    Err(TryRecvError::Disconnected) => break,
                }

                if paused {
                    thread::sleep(Duration::from_millis(100));
                    continue;
                }

                let timestamp = Local::now().format("%Y-%m-%d %H:%M:%S").to_string();

                // NOTE: This request string is device/protocol-specific.
                // Keep/adjust it to match your micrometer's serial protocol.
                let command = "M0,0\r";
                if let Err(e) = port.write_all(command.as_bytes()) {
                    let _ = data_tx.send(SerialMessage::Error(format!(
                        "{}: Failed to write command: {}",
                        timestamp, e
                    )));
                    thread::sleep(Duration::from_millis(150));
                    continue;
                }
                let _ = port.flush();

                thread::sleep(Duration::from_millis(75));

                let mut buffer = [0u8; 1024];
                let bytes_read = match port.read(&mut buffer) {
                    Ok(0) => {
                        thread::sleep(Duration::from_millis(50));
                        continue;
                    }
                    Ok(n) => n,
                    Err(e) => {
                        let _ = data_tx.send(SerialMessage::Error(format!(
                            "{}: Read error: {}",
                            timestamp, e
                        )));
                        thread::sleep(Duration::from_millis(150));
                        continue;
                    }
                };

                let response = String::from_utf8_lossy(&buffer[..bytes_read]).trim().to_string();

                if response.starts_with("M0,") {
                    let parts: Vec<&str> = response.split(',').collect();
                    if parts.len() == 3 {
                        let measurement1 = parts[1];
                        let measurement2 = parts[2];

                        if measurement1.contains("--") || measurement2.contains("--") {
                            let _ = data_tx.send(SerialMessage::Invalid(response, timestamp));
                            continue;
                        }

                        let m1: f32 = match measurement1.parse() {
                            Ok(v) => v,
                            Err(e) => {
                                let _ = data_tx.send(SerialMessage::Error(format!(
                                    "{}: Failed to parse measurement1 '{}': {}",
                                    timestamp, measurement1, e
                                )));
                                continue;
                            }
                        };

                        // Validate m2 numeric but ignore it (duplicated in your case)
                        if let Err(e) = measurement2.parse::<f32>() {
                            let _ = data_tx.send(SerialMessage::Error(format!(
                                "{}: Failed to parse measurement2 '{}': {}",
                                timestamp, measurement2, e
                            )));
                            continue;
                        }

                        let _ = data_tx.send(SerialMessage::Measurement(m1, timestamp));
                    } else {
                        let _ = data_tx.send(SerialMessage::Invalid(response, timestamp));
                    }
                } else {
                    let _ = data_tx.send(SerialMessage::Error(format!(
                        "{}: Unexpected response: {}",
                        timestamp, response
                    )));
                }

                thread::sleep(Duration::from_millis(50));
            }
        });
    }

    fn set_serial_paused(&mut self, paused: bool) {
        self.serial_paused = paused;
        if let Some(tx) = &self.serial_cmd_tx {
            let _ = tx.send(SerialCmd::Pause(paused));
        }
    }

    fn stop_serial_thread(&mut self) {
        if let Some(tx) = self.serial_cmd_tx.take() {
            let _ = tx.send(SerialCmd::Stop);
        }
        self.serial_rx = None;
        self.serial_paused = true;
    }

    fn save_plot(&self) -> Result<(), Box<dyn std::error::Error>> {
        fs::create_dir_all(Self::log_dir())?;

        let timestamp = Local::now().format("%Y%m%d_%H%M%S").to_string();
        let prefix = self.make_prefix();
        let plot_path = Self::log_dir().join(format!("{}plot_{}.png", prefix, timestamp));

        let plot_path_str = plot_path.to_string_lossy();
        let root = BitMapBackend::new(plot_path_str.as_ref(), (800, 400)).into_drawing_area();
        root.fill(&WHITE)?;

        let (base_t, max_time) = if self.measurements.len() >= 2 {
            let base = self.measurements.first().unwrap().0;
            let span = self.measurements.last().unwrap().0 - base;
            (base, span.max(1.0))
        } else {
            (0.0f32, 60.0f32)
        };

        let mut chart = ChartBuilder::on(&root)
            .caption(
                "Optical Micrometer Measurements (Last 300)",
                ("sans-serif", 20).into_font(),
            )
            .margin(10)
            .x_label_area_size(40)
            .y_label_area_size(70)
            .build_cartesian_2d(0f64..max_time as f64, 2f64..10f64)?;

        chart
            .configure_mesh()
            .x_desc("Time (s)")
            .y_desc("")
            .axis_desc_style(("sans-serif", 15))
            .draw()?;

        // Saved plot: vertical label in margin
        let y_style: TextStyle = TextStyle::from(("sans-serif", 15).into_font())
            .transform(FontTransform::Rotate90);
        root.draw_text("Measurement (mm)", &y_style, (18, 200))?;

        if !self.measurements.is_empty() {
            chart.draw_series(LineSeries::new(
                self.measurements
                    .iter()
                    .map(|&(t, v)| ((t - base_t) as f64, v as f64)),
                &BLUE,
            ))?;
        }

        let thin_red = ShapeStyle::from(&RED).stroke_width(1);
        chart.draw_series(LineSeries::new(
            vec![(0f64, self.ucl as f64), (max_time as f64, self.ucl as f64)],
            thin_red,
        ))?;
        chart.draw_series(LineSeries::new(
            vec![(0f64, self.lcl as f64), (max_time as f64, self.lcl as f64)],
            thin_red,
        ))?;

        root.present()?;
        println!("Plot saved to {}", plot_path.display());
        Ok(())
    }

    fn export_csv(&self) -> Result<(), Box<dyn std::error::Error>> {
        let dir = Self::log_dir();
        fs::create_dir_all(&dir)?;

        if let Some(path) = FileDialog::new()
            .add_filter("CSV", &["csv"])
            .set_directory(dir.to_string_lossy().as_ref())
            .save_file()
        {
            let mut out = OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open(&path)?;

            writeln!(out, "Timestamp,Measurement")?;
            for (ts, m) in &self.all_measurements {
                writeln!(out, "{},{}", ts, m)?;
            }
            out.flush()?;
            println!("CSV exported to: {}", path.display());
        }
        Ok(())
    }

    fn update_std_dev(&mut self) {
        let values: Vec<f64> = self.measurements.iter().map(|&(_, v)| v as f64).collect();
        if values.len() > 1 {
            let mean = values.iter().sum::<f64>() / values.len() as f64;
            let variance = values
                .iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f64>()
                / (values.len() - 1) as f64;
            self.std_dev = Some(variance.sqrt());
        } else {
            self.std_dev = None;
        }
    }

    fn process_serial_message(&mut self, serial_msg: SerialMessage) {
        match serial_msg {
            SerialMessage::Measurement(m, timestamp) => {
                self.no_data_reason = None;
                self.error = None;

                let now = SystemTime::now();
                self.last_measurement_time = Some(now);
                self.current_measurement = Some(m);

                if let Some(file) = self.file.as_mut() {
                    if let Err(e) = writeln!(file, "{},{}", timestamp, m) {
                        eprintln!("{}: Failed to write to CSV: {}", timestamp, e);
                    }
                    let _ = file.flush();
                }

                // elapsed = (now - start_time) - paused_total
                let elapsed = now
                    .duration_since(self.start_time)
                    .unwrap_or(Duration::from_secs(0));
                let elapsed = elapsed
                    .checked_sub(self.paused_total)
                    .unwrap_or(Duration::from_secs(0));
                let elapsed_secs = elapsed.as_secs_f32();

                self.all_measurements.push((timestamp, m));

                if self.measurements.len() >= self.max_display_points {
                    self.measurements.remove(0);
                }
                self.measurements.push((elapsed_secs, m));

                self.update_std_dev();
            }
            SerialMessage::Invalid(response, timestamp) => {
                self.last_invalid_time = Some(SystemTime::now());
                self.current_measurement = None;
                self.std_dev = None;

                self.no_data_reason =
                    Some("NO TARGET / OUT OF RANGE (device returned “--”)".to_string());

                self.error = Some(format!(
                    "{}: Invalid measurement response: {}",
                    timestamp, response
                ));
            }
            SerialMessage::Error(msg) => {
                self.current_measurement = None;
                self.std_dev = None;
                self.no_data_reason = Some("No valid measurement (serial error)".to_string());
                self.error = Some(msg);
            }
        }
    }

    fn controls_locked(&self) -> bool {
        // Lock serial settings while connected OR logging
        self.port.is_some() || self.logging
    }

    fn make_status_banner(&self) -> (String, Color) {
        let gray = Color::from_rgb8(120, 120, 120);
        let green = Color::from_rgb8(0, 150, 0);
        let red = Color::from_rgb8(200, 0, 0);
        let amber = Color::from_rgb8(200, 120, 0);

        if self.port.is_none() {
            return ("DISCONNECTED".to_string(), gray);
        }

        if self.logging {
            if let Some(v) = self.current_measurement {
                if v >= self.lcl && v <= self.ucl {
                    ("IN SPEC".to_string(), green)
                } else {
                    ("OUT OF SPEC".to_string(), red)
                }
            } else if let Some(reason) = &self.no_data_reason {
                (reason.clone(), amber)
            } else {
                ("NO DATA".to_string(), amber)
            }
        } else {
            ("PAUSED".to_string(), gray)
        }
    }

    fn start_new_roll_session(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if self.port.is_none() {
            return Err("Not connected. Click Connect first.".into());
        }

        // Close previous file if any
        self.file = None;

        // Reset session data
        self.measurements.clear();
        self.all_measurements.clear();
        self.current_measurement = None;
        self.std_dev = None;

        // Reset time base
        self.start_time = SystemTime::now();
        self.paused_at = None;
        self.paused_total = Duration::from_secs(0);
        self.last_measurement_time = None;
        self.last_invalid_time = None;
        self.no_data_reason = None;

        // Open fresh file
        self.open_file()?;

        // Start acquisition
        self.logging = true;
        self.start_serial_thread_if_needed();
        self.set_serial_paused(false);

        Ok(())
    }

    fn toggle_pause_resume(&mut self) {
        if self.port.is_none() {
            self.error = Some("Not connected.".to_string());
            return;
        }

        let now = SystemTime::now();

        if self.logging {
            // PAUSE
            self.logging = false;
            self.paused_at = Some(now);
            self.set_serial_paused(true);
            self.no_data_reason = Some("PAUSED".to_string());
        } else {
            // RESUME
            if let Some(t0) = self.paused_at.take() {
                if let Ok(dt) = now.duration_since(t0) {
                    self.paused_total += dt;
                }
            }
            self.logging = true;
            self.no_data_reason = None;
            self.start_serial_thread_if_needed();
            self.set_serial_paused(false);
        }
    }

    fn end_roll(&mut self) {
        // Pause acquisition if running
        if self.port.is_some() {
            self.set_serial_paused(true);
        }
        self.logging = false;

        // Close file cleanly
        self.file = None;

        // Save snapshot plot (optional but useful)
        if let Err(e) = self.save_plot() {
            self.error = Some(format!("Save plot failed: {e}"));
        }

        // Reset session buffers ready for next roll
        self.measurements.clear();
        self.all_measurements.clear();
        self.current_measurement = None;
        self.std_dev = None;

        self.start_time = SystemTime::now();
        self.paused_at = None;
        self.paused_total = Duration::from_secs(0);

        self.no_data_reason = Some("Roll ended. Ready.".to_string());
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::RefreshPorts => {
                self.available_ports = Self::scan_ports();
                self.selected_port = self
                    .available_ports
                    .iter()
                    .find(|p| p.port_name.eq_ignore_ascii_case(&self.port_name))
                    .cloned();
            }

            Message::PortSelected(p) => {
                if self.controls_locked() {
                    return Task::none();
                }
                self.port_name = p.port_name.clone();
                self.selected_port = Some(p);

                let _ = self.save_settings_to_disk();
            }

            Message::Connect => {
                self.error = None;

                if self.port.is_some() {
                    self.error = Some("Already connected. Use Disconnect.".to_string());
                    return Task::none();
                }

                match self.open_port() {
                    Ok(_) => {
                        self.start_serial_thread_if_needed();
                        self.set_serial_paused(true);
                        self.no_data_reason = Some("PAUSED".to_string());
                        let _ = self.save_settings_to_disk();
                    }
                    Err(e) => self.error = Some(format!("Open port failed: {e}")),
                }
            }

            Message::Disconnect => {
                self.logging = false;
                self.file = None;
                self.close_port();

                self.current_measurement = None;
                self.std_dev = None;
                self.no_data_reason = Some("Disconnected".to_string());

                self.paused_at = None;
                self.paused_total = Duration::from_secs(0);

                let _ = self.save_settings_to_disk();
            }

            Message::StartLogging => {
                self.error = None;

                // Start a NEW roll/session
                match self.start_new_roll_session() {
                    Ok(_) => {}
                    Err(e) => self.error = Some(format!("{e}")),
                }

                let _ = self.save_settings_to_disk();
            }

            Message::PauseLogging => {
                self.error = None;
                self.toggle_pause_resume();
                let _ = self.save_settings_to_disk();
            }

            Message::EndRoll => {
                self.error = None;
                self.end_roll();
            }

            Message::Quit => {
                let _ = self.save_settings_to_disk();
                self.stop_serial_thread();
                self.file = None;
                let _ = self.save_plot();
                return iced::exit::<Message>();
            }

            Message::SetUcl(input) => {
                self.ucl_input = input.clone();
                if let Ok(v) = input.parse::<f32>() {
                    self.ucl = v;
                    let _ = self.save_settings_to_disk();
                }
            }

            Message::SetLcl(input) => {
                self.lcl_input = input.clone();
                if let Ok(v) = input.parse::<f32>() {
                    self.lcl = v;
                    let _ = self.save_settings_to_disk();
                }
            }

            Message::SavePlot => {
                if let Err(e) = self.save_plot() {
                    self.error = Some(format!("Save plot failed: {e}"));
                }
            }

            Message::ExportCsv => {
                if let Err(e) = self.export_csv() {
                    self.error = Some(format!("CSV export failed: {e}"));
                }
            }

            Message::BaudRateSelected(baud) => {
                if self.controls_locked() {
                    return Task::none();
                }
                self.baud_rate = baud;
                let _ = self.save_settings_to_disk();
            }

            Message::LabelPrefixChanged(prefix) => {
                self.label_prefix = prefix;
                let _ = self.save_settings_to_disk();
            }

            Message::DataBitsSelected(v) => {
                if self.controls_locked() {
                    return Task::none();
                }
                self.data_bits = v;
                let _ = self.save_settings_to_disk();
            }

            Message::ParitySelected(v) => {
                if self.controls_locked() {
                    return Task::none();
                }
                self.parity = v;
                let _ = self.save_settings_to_disk();
            }

            Message::StopBitsSelected(v) => {
                if self.controls_locked() {
                    return Task::none();
                }
                self.stop_bits = v;
                let _ = self.save_settings_to_disk();
            }

            Message::Tick => {
                // Drain serial messages
                let mut pending = Vec::new();
                if let Some(rx) = self.serial_rx.as_ref() {
                    while let Ok(m) = rx.try_recv() {
                        pending.push(m);
                    }
                }
                for m in pending {
                    self.process_serial_message(m);
                }

                // Staleness watchdog (only while logging)
                if self.logging {
                    let stale_after = Duration::from_millis(900);
                    let now = SystemTime::now();

                    let is_stale = self
                        .last_measurement_time
                        .and_then(|t| now.duration_since(t).ok())
                        .map(|age| age > stale_after)
                        .unwrap_or(true);

                    if is_stale {
                        self.current_measurement = None;
                        self.std_dev = None;

                        let recent_invalid = self
                            .last_invalid_time
                            .and_then(|t| now.duration_since(t).ok())
                            .map(|age| age < Duration::from_secs(2))
                            .unwrap_or(false);

                        if recent_invalid {
                            self.no_data_reason
                                .get_or_insert_with(|| "NO TARGET / OUT OF RANGE".to_string());
                        } else {
                            self.no_data_reason
                                .get_or_insert_with(|| "NO DATA (timeout)".to_string());
                        }
                    }
                }
            }
        }

        Task::none()
    }

    fn view(&self) -> Element<'_, Message> {
        // Avoid returning Elements that borrow temporaries
        let log_dir_text: String = Self::log_dir().to_string_lossy().to_string();

        fn action_button<'a>(
            label: &'a str,
            enabled: bool,
            msg: Message,
        ) -> iced::widget::Button<'a, Message> {
            let mut b = button(label);
            if enabled {
                b = b.on_press(msg);
            }
            b
        }

        let green = Color::from_rgb8(0, 200, 0);
        let red = Color::from_rgb8(200, 0, 0);

        let logging_dot = text("●").size(22.0).color(if self.logging { green } else { red });
        let serial_dot = text("●").size(22.0).color(if self.port.is_some() { green } else { red });

        let (banner_text, banner_color) = self.make_status_banner();
        let banner = row![
            text("■").size(34.0).color(banner_color),
            text(banner_text).size(30.0).color(banner_color),
        ]
            .spacing(10)
            .align_y(Alignment::Center);

        let pause_label = if self.logging { "Pause" } else { "Resume" };

        let current_text = match self.current_measurement {
            Some(v) => format!("{:.3} mm", v),
            None => "---".to_string(),
        };
        let std_dev_text = match self.std_dev {
            Some(v) => format!("{:.4} mm", v),
            None => "---".to_string(),
        };

        let in_limits = self
            .current_measurement
            .map(|v| v >= self.lcl && v <= self.ucl)
            .unwrap_or(false);

        let current_color: Color = match self.current_measurement {
            None => Color::from_rgb8(153, 153, 153),
            Some(_) if in_limits => Color::from_rgb8(0, 153, 0),
            Some(_) => Color::from_rgb8(217, 0, 0),
        };

        let baud_options: Vec<u32> = vec![1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200];
        let data_bits_options = vec![DataBitsOpt::Seven, DataBitsOpt::Eight];
        let parity_options = vec![ParityOpt::None, ParityOpt::Even, ParityOpt::Odd];
        let stop_bits_options = vec![StopBitsOpt::One, StopBitsOpt::Two];

        let locked = self.controls_locked();

        let port_widget: Element<'_, Message> = if locked {
            text(
                self.selected_port
                    .as_ref()
                    .map(|p| p.display.as_str())
                    .unwrap_or(&self.port_name),
            )
                .into()
        } else {
            pick_list(
                self.available_ports.clone(),
                self.selected_port.clone(),
                Message::PortSelected,
            )
                .width(Length::Fixed(420.0))
                .into()
        };

        let baud_widget: Element<'_, Message> = if locked {
            text(format!("{}", self.baud_rate)).into()
        } else {
            pick_list(baud_options, Some(self.baud_rate), Message::BaudRateSelected)
                .width(Length::Fixed(140.0))
                .into()
        };

        let databits_widget: Element<'_, Message> = if locked {
            text(format!("{}", self.data_bits)).into()
        } else {
            pick_list(data_bits_options, Some(self.data_bits), Message::DataBitsSelected)
                .width(Length::Fixed(110.0))
                .into()
        };

        let parity_widget: Element<'_, Message> = if locked {
            text(format!("{}", self.parity)).into()
        } else {
            pick_list(parity_options, Some(self.parity), Message::ParitySelected)
                .width(Length::Fixed(120.0))
                .into()
        };

        let stopbits_widget: Element<'_, Message> = if locked {
            text(format!("{}", self.stop_bits)).into()
        } else {
            pick_list(stop_bits_options, Some(self.stop_bits), Message::StopBitsSelected)
                .width(Length::Fixed(110.0))
                .into()
        };

        // Enable logic:
        let can_connect = self.port.is_none();
        let can_disconnect = self.port.is_some();
        let can_start_new_roll = self.port.is_some(); // always allowed when connected
        let can_pause_resume = self.port.is_some();
        let can_end_roll = self.port.is_some(); // and/or self.file.is_some()

        let controls = column![
            row![text("Logging:"), logging_dot, text("   Serial:"), serial_dot]
                .spacing(8)
                .align_y(Alignment::Center),
            banner,
            row![
                text("Filename Prefix:"),
                text_input("Roll / Order / Customer", &self.label_prefix)
                    .on_input(Message::LabelPrefixChanged)
                    .width(Length::Fixed(560.0)),
            ]
            .spacing(10)
            .align_y(Alignment::Center),
            row![text("Port:"), port_widget, text("Baud:"), baud_widget]
                .spacing(10)
                .align_y(Alignment::Center),
            row![
                text("Data bits:"),
                databits_widget,
                text("Parity:"),
                parity_widget,
                text("Stop bits:"),
                stopbits_widget,
            ]
            .spacing(10)
            .align_y(Alignment::Center),
            row![
                action_button("Connect", can_connect, Message::Connect),
                action_button("Disconnect", can_disconnect, Message::Disconnect),
                action_button("Start New Roll", can_start_new_roll, Message::StartLogging),
                action_button(pause_label, can_pause_resume, Message::PauseLogging),
                action_button("End Roll", can_end_roll, Message::EndRoll),
                button("Save Plot").on_press(Message::SavePlot),
                button("Export CSV").on_press(Message::ExportCsv),
                button("Quit").on_press(Message::Quit),
            ]
            .spacing(10),
            row![
                text("UCL (mm):"),
                text_input("6.5", &self.ucl_input)
                    .on_input(Message::SetUcl)
                    .width(Length::Fixed(90.0)),
                text("LCL (mm):"),
                text_input("6.0", &self.lcl_input)
                    .on_input(Message::SetLcl)
                    .width(Length::Fixed(90.0)),
                text("Log folder:"),
                text(log_dir_text).size(14.0),
            ]
            .spacing(10)
            .align_y(Alignment::Center),
            row![
                text("Current Measurement:").size(24.0),
                text(current_text).size(40.0).color(current_color),
                text("Std Dev:").size(18.0),
                text(std_dev_text).size(18.0),
            ]
            .spacing(14)
            .align_y(Alignment::Center),
        ]
            .align_x(Alignment::Start)
            .spacing(12)
            .padding(10);

        let mut content = column![controls].align_x(Alignment::Center).spacing(10);

        let chart = MyChart {
            measurements: &self.measurements,
            ucl: self.ucl,
            lcl: self.lcl,
        };

        let plot: Element<Message> = ChartWidget::new(chart)
            .width(Length::Fixed(800.0))
            .height(Length::Fixed(400.0))
            .into();

        content = content.push(plot);

        if let Some(err) = &self.error {
            content = content.push(text(err).color(Color::from_rgb8(255, 0, 0)));
        }

        content.into()
    }

    fn subscription(&self) -> Subscription<Message> {
        let serial_sub = if self.serial_rx.is_some() {
            time::every(Duration::from_millis(100)).map(|_| Message::Tick)
        } else {
            Subscription::none()
        };

        let ports_sub = time::every(Duration::from_secs(2)).map(|_| Message::RefreshPorts);

        Subscription::batch(vec![serial_sub, ports_sub])
    }
}

// ---------------- Chart ----------------

struct MyChart<'a> {
    measurements: &'a Vec<(f32, f32)>,
    ucl: f32,
    lcl: f32,
}

impl<'a> Chart<Message> for MyChart<'a> {
    type State = ();

    fn build_chart<DB: DrawingBackend>(
        &self,
        _state: &Self::State,
        mut builder: plotters_iced::ChartBuilder<DB>,
    ) {
        let (base_t, max_time) = if self.measurements.len() >= 2 {
            let base = self.measurements.first().unwrap().0;
            let span = self.measurements.last().unwrap().0 - base;
            (base, span.max(1.0))
        } else {
            (0.0f32, 60.0f32)
        };

        let mut chart = builder
            .caption("Optical Micrometer Measurements (Last 300)", ("sans-serif", 20))
            .margin(10)
            .set_label_area_size(LabelAreaPosition::Left, 70)
            .set_label_area_size(LabelAreaPosition::Bottom, 40)
            .build_cartesian_2d(0f64..max_time as f64, 2f64..10f64)
            .expect("Failed to build chart");

        chart
            .configure_mesh()
            .x_desc("Time (s)")
            .y_desc("")
            .axis_desc_style(("sans-serif", 15))
            .draw()
            .expect("Failed to draw mesh");

        if !self.measurements.is_empty() {
            chart
                .draw_series(LineSeries::new(
                    self.measurements
                        .iter()
                        .map(|&(t, v)| ((t - base_t) as f64, v as f64)),
                    &BLUE,
                ))
                .expect("Failed to draw series");
        }

        let thin_red = ShapeStyle::from(&RED).stroke_width(1);

        chart
            .draw_series(LineSeries::new(
                vec![(0f64, self.ucl as f64), (max_time as f64, self.ucl as f64)],
                thin_red,
            ))
            .expect("Failed to draw UCL");

        chart
            .draw_series(LineSeries::new(
                vec![(0f64, self.lcl as f64), (max_time as f64, self.lcl as f64)],
                thin_red,
            ))
            .expect("Failed to draw LCL");

        // Vertical label inside plot area (plotters_iced doesn't expose margin area)
        let y_style: TextStyle = TextStyle::from(("sans-serif", 13).into_font())
            .transform(FontTransform::Rotate90);
        let label_pos: (f64, f64) = (1.2, 9.0);

        let _ = chart.draw_series(std::iter::once(Text::new(
            "Measurement (mm)",
            label_pos,
            y_style,
        )));
    }
}

fn main() -> iced::Result {
    iced::application(
        MicrometerLoggerApp::title,
        MicrometerLoggerApp::update,
        MicrometerLoggerApp::view,
    )
        .subscription(MicrometerLoggerApp::subscription)
        .window(iced::window::Settings {
            size: Size::new(1020.0, 790.0),
            resizable: false,
            ..Default::default()
        })
        .antialiasing(true)
        .run()
}