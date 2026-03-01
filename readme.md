# MicrometerLogger

MicrometerLogger was written to fill a gap in available tooling for the **Keyence LS-7001 optical micrometer** — specifically the need to continuously capture, log, and statistically analyse measurements during production runs. The Keyence LS-7001 outputs measurements over RS232, but no off-the-shelf software provided real-time SPC charting and per-roll CSV logging in a lightweight desktop application. This tool was built to address that.

While conceived around the LS-7001, it should work with any RS232 optical micrometer that uses the `M0,0\r` polling protocol.

![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20macOS-blue)
![Language](https://img.shields.io/badge/language-Rust-orange)

---

## Features

### Data Acquisition
- Connects to the Keyence LS-7001 (or compatible device) over any RS232 or USB-to-RS232 serial port
- Samples at ~10 measurements per second (100ms interval) using accurate `Instant`-based timing
- Configurable serial parameters: baud rate, data bits (7/8), parity (None/Even/Odd), stop bits (1/2)
- Detects and displays out-of-range readings (`--` from the device)
- Auto-scans for available serial ports; rescans every 2 seconds

### Real-Time Charting
- Live scrolling chart showing the last 30 seconds of data (300 points)
- SPC overlay lines displayed on the chart:
  - **UCL / LCL** — user-defined Upper and Lower Control Limits (red)
  - **Mean** (green)
  - **±1σ** (cyan), **±2σ** (magenta), **±3σ** (orange) — dashed
- Live **Cp** and **Cpk** capability indices shown as text on the chart
- Dynamic Y-axis: scales to fit UCL/LCL ±2mm padding, expanded to show all data

### Data Logging
- Logs every measurement to a timestamped CSV file (`measurements_<timestamp>.csv`)
- Timestamps include millisecond precision (`HH:MM:SS.mmm`)
- Supports Pause/Resume during a roll — elapsed time accounts for paused periods
- "End Roll" finalises a roll and saves a summary plot without closing the app
- Optional filename label prefix for organising output files by job or batch

### End-of-Roll Summary Plot
- PNG summary chart saved automatically at the end of each roll
- Includes all SPC overlay lines (mean, ±1/2/3σ, UCL, LCL)
- Statistics annotation: sample count (n), mean, standard deviation, min, max
- Decimates to ~1000 points when data exceeds 2000 samples for a clean image

### Settings Persistence
- Serial port, baud rate, serial parameters, UCL/LCL, and label prefix are saved automatically
- Settings stored as JSON in the log directory; loaded on next launch
- Atomic save (write to temp file then rename) prevents corruption

### File Outputs
All output files are saved to `{Documents}/OpticalMicrometer_logs/` (falls back to Desktop, then current directory):

| File | Description |
|------|-------------|
| `{prefix}measurements_{timestamp}.csv` | Timestamped measurement log (one row per sample) |
| `{prefix}summary_{timestamp}.png` | End-of-roll summary plot with SPC overlay |
| `settings.json` | Persisted application configuration |

---

## Requirements

- A Keyence LS-7001 optical micrometer (or compatible RS232 device using the `M0,0\r` protocol)
- RS232 interface cable or USB-to-RS232 adapter
- Rust toolchain (stable) — [install from rustup.rs](https://rustup.rs)

---

## Building

```bash
# Debug build
cargo build

# Release build (recommended for use)
cargo build --release

# Run directly
cargo run
```

### macOS App Bundle

To create a `.app` bundle with the correct icon for Finder/Dock:

```bash
./scripts/bundle-macos.sh
# Output: target/MicrometerLogger.app
# Copy to /Applications to install
```

### Windows

The release build suppresses the console window automatically. The application icon is embedded into the `.exe` via `build.rs`.

---

## Usage

1. **Select a serial port** from the dropdown (the list refreshes automatically)
2. **Configure serial parameters** to match your device (baud rate, data bits, parity, stop bits)
3. **(Optional)** Set a **label prefix** to tag output files for the current job or batch
4. **(Optional)** Set **UCL** and **LCL** values to define your control limits
5. Press **Connect** to open the serial port
6. Press **Start New Roll** to begin logging — a CSV file is created immediately
7. Monitor the live chart; use **Pause / Resume** as needed
8. Press **End Roll** to finalise the roll — the summary PNG is saved and logging stops
9. Press **Start New Roll** again for the next roll, or **Disconnect** when done

---

## Serial Protocol

The application polls the device by sending `M0,0\r` every 100ms and parses the response:

- **Valid measurement:** `M0,<value1>,<value2>` — the first value is recorded
- **Out of range:** `--` — logged as an invalid reading; does not affect SPC statistics
- **Timeout:** 500ms per operation

---

## SPC Reference

| Metric | Formula |
|--------|---------|
| Cp | `(UCL − LCL) / (6σ)` |
| Cpk | `min((UCL − mean) / 3σ, (mean − LCL) / 3σ)` |

A Cp/Cpk ≥ 1.33 is typically considered capable for production processes.

---

## License

MIT License — see `LICENCE` for details. Supplied without warranty; use at your own risk.
