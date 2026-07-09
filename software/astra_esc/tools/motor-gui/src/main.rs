//! Really-really-simple live GUI for the ihm08m1-foc telemetry stream.
//!
//! The firmware prints one `defmt` telemetry line at 10 Hz over the ST-Link
//! (see `src/main.rs` in the firmware). This tool reads those lines, parses
//! each one, and scroll-plots the motor stats.
//!
//! Two ways to feed it, picked automatically:
//!   * Launched in a terminal  -> it spawns `probe-rs run` for you (one command).
//!   * stdin is a pipe/file     -> it reads that instead, e.g.
//!         cargo run --release | motor-gui --stdin
//!         motor-gui --print < captured.log      # headless: parse + dump, no window
//!
//! Flags: --stdin  --attach  --print  --elf <PATH>  --chip <NAME>

mod parse;

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, IsTerminal};
use std::process::{Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use eframe::egui;
use egui::Color32;
use egui_plot::{Legend, Line, Plot, PlotPoints};

use parse::{parse_line, Mode, Sample};

/// How many samples to keep (10 Hz -> ~90 s of scroll-back).
const WINDOW: usize = 900;

// --- series colours ---
const C_RPM: Color32 = Color32::from_rgb(90, 175, 255);
const C_WOBS: Color32 = Color32::from_rgb(0, 205, 205);
const C_WOL: Color32 = Color32::from_rgb(150, 150, 150);
const C_ID: Color32 = Color32::from_rgb(100, 150, 255);
const C_IQ: Color32 = Color32::from_rgb(255, 150, 60);
const C_IQREF: Color32 = Color32::from_rgb(255, 90, 90);
const C_IA: Color32 = Color32::from_rgb(90, 205, 130);
const C_IB: Color32 = Color32::from_rgb(200, 130, 220);
const C_IC: Color32 = Color32::from_rgb(235, 160, 70); // reconstructed -ia-ib
const C_VBUS: Color32 = Color32::from_rgb(240, 205, 70);
const C_ISR: Color32 = Color32::from_rgb(220, 110, 220);
const C_IDREF: Color32 = Color32::from_rgb(170, 200, 255); // id setpoint (pairs with C_ID)
const C_VD: Color32 = Color32::from_rgb(120, 220, 160);
const C_VQ: Color32 = Color32::from_rgb(240, 170, 90);

/// (probe-rs chip, release-ELF path) presets, one per firmware MCU backend.
/// These mirror the `mcu-g4` / `mcu-h5` cargo features in the firmware.
const G4: (&str, &str) = (
    "STM32G474RETx",
    "../../target/thumbv7em-none-eabihf/release/ihm08m1-foc",
);
const H5: (&str, &str) = (
    "STM32H533RETx",
    "../../target/thumbv8m.main-none-eabihf/release/ihm08m1-foc",
);

struct Config {
    use_stdin: bool,
    attach: bool,
    print: bool,
    elf: String,
    chip: String,
}

impl Config {
    fn from_args() -> Config {
        let mut cfg = Config {
            use_stdin: false,
            attach: false,
            print: false,
            // default: the G4 release full-FOC firmware built one level up.
            // Use --h5 (or --chip/--elf) to point at the H533 build instead.
            elf: G4.1.to_string(),
            chip: G4.0.to_string(),
        };
        let mut args = std::env::args().skip(1);
        while let Some(a) = args.next() {
            match a.as_str() {
                "--stdin" => cfg.use_stdin = true,
                "--attach" => cfg.attach = true,
                "--print" => cfg.print = true,
                // MCU presets (set chip + elf together). Explicit --chip/--elf
                // after these still override the individual field.
                "--g4" => (cfg.chip, cfg.elf) = (G4.0.into(), G4.1.into()),
                "--h5" => (cfg.chip, cfg.elf) = (H5.0.into(), H5.1.into()),
                "--elf" => cfg.elf = args.next().unwrap_or_default(),
                "--chip" => cfg.chip = args.next().unwrap_or_default(),
                "-h" | "--help" => {
                    println!(
                        "motor-gui — live plot of ihm08m1-foc telemetry\n\n\
                         USAGE:\n  \
                         cargo run --release [-- FLAGS]\n\n\
                         FLAGS:\n  \
                         --g4           target the NUCLEO-G474RE build (default)\n  \
                         --h5           target the NUCLEO-H533RE build (sets chip + ELF)\n  \
                         --stdin        read telemetry from stdin instead of spawning probe-rs\n  \
                         --attach       use `probe-rs attach` (don't reflash) instead of `run`\n  \
                         --print        headless: parse stdin and print samples, no window\n  \
                         --elf <PATH>   firmware ELF for probe-rs (default: {})\n  \
                         --chip <NAME>  target chip (default: {})",
                        cfg.elf, cfg.chip
                    );
                    std::process::exit(0);
                }
                other => eprintln!("motor-gui: ignoring unknown arg {other:?}"),
            }
        }
        cfg
    }
}

/// Messages from the reader thread(s) to the UI.
enum Msg {
    Sample(Sample),
    /// Any non-telemetry line (banner, "-> RUN", probe-rs status/errors).
    Info(String),
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cfg = Config::from_args();

    // Headless mode: no window, just prove the parse pipeline on a pipe/file.
    if cfg.print {
        let stdin = std::io::stdin();
        let mut n = 0usize;
        for line in stdin.lock().lines() {
            let line = line?;
            if let Some(s) = parse_line(&line) {
                n += 1;
                println!("{s:?}");
            }
        }
        eprintln!("motor-gui: parsed {n} sample(s)");
        return Ok(());
    }

    let (rx, source) = spawn_reader(&cfg);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 860.0])
            .with_title("ihm08m1-foc — motor telemetry"),
        ..Default::default()
    };
    eframe::run_native(
        "motor-gui",
        options,
        Box::new(move |_cc| Ok(Box::new(App::new(rx, source)))),
    )?;
    Ok(())
}

/// Start the background reader(s) and return the channel + a human description
/// of where data is coming from.
fn spawn_reader(cfg: &Config) -> (Receiver<Msg>, String) {
    let (tx, rx) = mpsc::channel();

    // Read stdin if asked, or if stdin is redirected (a pipe/file).
    let piped = !std::io::stdin().is_terminal();
    if cfg.use_stdin || piped {
        let tx = tx.clone();
        thread::spawn(move || read_into(std::io::stdin().lock(), tx));
        return (rx, "stdin".to_string());
    }

    // Otherwise spawn probe-rs and read its output.
    let sub = if cfg.attach { "attach" } else { "run" };
    let desc = format!("probe-rs {} --chip {} {}", sub, cfg.chip, cfg.elf);
    let spawn = Command::new("probe-rs")
        .args([sub, "--chip", &cfg.chip, &cfg.elf])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn();

    match spawn {
        Ok(mut child) => {
            // defmt output has historically gone to stdout, but read both so
            // we're robust to that and surface probe-rs errors from stderr.
            if let Some(out) = child.stdout.take() {
                let tx = tx.clone();
                thread::spawn(move || read_into(BufReader::new(out), tx));
            }
            if let Some(err) = child.stderr.take() {
                let tx = tx.clone();
                thread::spawn(move || read_into(BufReader::new(err), tx));
            }
            thread::spawn(move || {
                let status = child.wait();
                let _ = tx.send(Msg::Info(format!("probe-rs exited: {status:?}")));
            });
        }
        Err(e) => {
            let _ = tx.send(Msg::Info(format!(
                "could not start probe-rs ({e}). Install probe-rs-tools, or pipe logs in: \
                 `cargo run --release | motor-gui --stdin`."
            )));
        }
    }
    (rx, desc)
}

/// Read a stream line-by-line, forwarding parsed samples and everything else.
fn read_into<R: BufRead>(reader: R, tx: mpsc::Sender<Msg>) {
    for line in reader.lines() {
        let Ok(line) = line else { break };
        let msg = match parse_line(&line) {
            Some(s) => Msg::Sample(s),
            None if line.trim().is_empty() => continue,
            None => Msg::Info(line),
        };
        if tx.send(msg).is_err() {
            break; // UI gone
        }
    }
}

/// One buffered point: receive-time (seconds since start) + the sample.
struct Point {
    t: f64,
    s: Sample,
}

struct App {
    rx: Receiver<Msg>,
    source: String,
    points: VecDeque<Point>,
    latest: Option<Sample>,
    last_log: String,
    start: Instant,
    last_rx: Option<Instant>,
}

impl App {
    fn new(rx: Receiver<Msg>, source: String) -> Self {
        App {
            rx,
            source,
            points: VecDeque::with_capacity(WINDOW),
            latest: None,
            last_log: String::new(),
            start: Instant::now(),
            last_rx: None,
        }
    }

    fn drain(&mut self) {
        while let Ok(msg) = self.rx.try_recv() {
            match msg {
                Msg::Sample(s) => {
                    let t = self.start.elapsed().as_secs_f64();
                    if self.points.len() >= WINDOW {
                        self.points.pop_front();
                    }
                    self.points.push_back(Point { t, s });
                    self.latest = Some(s);
                    self.last_rx = Some(Instant::now());
                }
                Msg::Info(line) => self.last_log = line,
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.drain();

        egui::TopBottomPanel::top("status").show(ctx, |ui| {
            ui.add_space(4.0);
            header(ui, self);
            ui.add_space(4.0);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            if self.points.is_empty() {
                ui.centered_and_justified(|ui| {
                    ui.label("waiting for telemetry…");
                });
                return;
            }
            let pts = &self.points;
            egui::ScrollArea::vertical().show(ui, |ui| {
                Plot::new("rpm")
                    .height(150.0)
                    .legend(Legend::default())
                    .show(ui, |pu| {
                        pu.line(series(pts, C_RPM, "mech rpm", |s| s.mech_rpm));
                        pu.set_auto_bounds(true.into());
                    });
                ui.label("mechanical speed (rpm)");
                ui.separator();

                Plot::new("dq")
                    .height(160.0)
                    .legend(Legend::default())
                    .show(ui, |pu| {
                        pu.line(series(pts, C_ID, "id", |s| s.id));
                        pu.line(series(pts, C_IDREF, "id*", |s| s.id_ref));
                        pu.line(series(pts, C_IQ, "iq", |s| s.iq));
                        pu.line(series(pts, C_IQREF, "iq*", |s| s.iq_ref));
                        pu.set_auto_bounds(true.into());
                    });
                ui.label("d/q currents (A) — id* / iq* are the commanded setpoints (id* is the align current)");
                ui.separator();

                Plot::new("vdq")
                    .height(150.0)
                    .legend(Legend::default())
                    .show(ui, |pu| {
                        pu.line(series(pts, C_VD, "vd", |s| s.vd));
                        pu.line(series(pts, C_VQ, "vq", |s| s.vq));
                        pu.set_auto_bounds(true.into());
                    });
                ui.label("commanded d/q voltage (V) — current-loop PI output after the circle limit");
                ui.separator();

                Plot::new("phase")
                    .height(150.0)
                    .legend(Legend::default())
                    .show(ui, |pu| {
                        pu.line(series(pts, C_IA, "ia", |s| s.ia));
                        pu.line(series(pts, C_IB, "ib", |s| s.ib));
                        pu.line(series(pts, C_IC, "ic", |s| s.ic));
                        pu.set_auto_bounds(true.into());
                    });
                ui.label("phase currents (A) — ic measured directly (falls back to −ia−ib on older logs)");
                ui.separator();

                Plot::new("omega")
                    .height(150.0)
                    .legend(Legend::default())
                    .show(ui, |pu| {
                        pu.line(series(pts, C_WOBS, "w_obs", |s| s.w_obs));
                        pu.line(series(pts, C_WOL, "w_ol", |s| s.w_ol));
                        pu.set_auto_bounds(true.into());
                    });
                ui.label("observer vs open-loop electrical speed (rad/s) — watch them converge at handoff");
                ui.separator();

                ui.columns(2, |cols| {
                    Plot::new("vbus")
                        .height(140.0)
                        .legend(Legend::default())
                        .show(&mut cols[0], |pu| {
                            pu.line(series(pts, C_VBUS, "vbus", |s| s.vbus));
                            pu.set_auto_bounds(true.into());
                        });
                    cols[0].label("bus voltage (V)");

                    Plot::new("isr")
                        .height(140.0)
                        .legend(Legend::default())
                        .show(&mut cols[1], |pu| {
                            pu.line(series(pts, C_ISR, "isr_max", |s| s.isr_us));
                            pu.set_auto_bounds(true.into());
                        });
                    cols[1].label("worst-case ISR time (µs) — control budget is 40 µs");
                });
            });
        });

        // Poll the channel a few times a second even without OS events.
        ctx.request_repaint_after(Duration::from_millis(50));
    }
}

/// Build a plot line for one field across the whole buffer.
fn series(pts: &VecDeque<Point>, color: Color32, name: &str, get: impl Fn(&Sample) -> f32) -> Line {
    let data: PlotPoints = pts.iter().map(|p| [p.t, get(&p.s) as f64]).collect();
    Line::new(data).name(name).color(color)
}

fn mode_color(m: Mode) -> Color32 {
    match m {
        Mode::Idle => Color32::GRAY,
        Mode::Align => Color32::from_rgb(230, 210, 60),
        Mode::OpenLoop => Color32::from_rgb(240, 160, 40),
        Mode::Run => Color32::from_rgb(70, 210, 90),
        Mode::Fault => Color32::from_rgb(230, 70, 70),
        Mode::Other => Color32::LIGHT_GRAY,
    }
}

fn header(ui: &mut egui::Ui, app: &App) {
    ui.horizontal(|ui| {
        ui.strong("source:");
        ui.monospace(&app.source);
        ui.separator();
        ui.label(format!("{} samples", app.points.len()));
        ui.separator();
        match app.last_rx {
            Some(t) => {
                let age = t.elapsed().as_secs_f32();
                let (txt, col) = if age < 1.0 {
                    ("● live".to_string(), Color32::from_rgb(70, 210, 90))
                } else {
                    (format!("○ stale {age:.0}s"), Color32::from_rgb(230, 160, 60))
                };
                ui.colored_label(col, txt);
            }
            None => {
                ui.colored_label(Color32::GRAY, "○ no data yet");
            }
        }
    });

    if let Some(s) = app.latest {
        ui.horizontal_wrapped(|ui| {
            ui.strong("mode:");
            ui.colored_label(mode_color(s.mode), s.mode.label());
            ui.separator();
            stat(ui, "rpm", format!("{:.0}", s.mech_rpm));
            stat(ui, "vbus", format!("{:.1} V", s.vbus));
            stat(ui, "pot", format!("{:.0}%", (s.pot / 4095.0 * 100.0).clamp(0.0, 100.0)));
            stat(ui, "id", format!("{:.2} A", s.id));
            stat(ui, "id*", format!("{:.2} A", s.id_ref));
            stat(ui, "iq", format!("{:.2} A", s.iq));
            stat(ui, "iq*", format!("{:.2} A", s.iq_ref));
            stat(ui, "vd", format!("{:.2} V", s.vd));
            stat(ui, "vq", format!("{:.2} V", s.vq));
            stat(ui, "θ", format!("{:.2} rad", s.theta));
            stat(ui, "isr", format!("{:.1} µs", s.isr_us));
        });
    }

    if !app.last_log.is_empty() {
        ui.horizontal(|ui| {
            ui.weak("log:");
            ui.weak(&app.last_log);
        });
    }
}

fn stat(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(format!("{label} "));
    ui.strong(value);
    ui.separator();
}
