//! Turn one line of the firmware's defmt telemetry into numbers.
//!
//! The firmware prints exactly one line per 10 Hz telemetry tick, built from
//! this format string in `src/main.rs`:
//!
//! ```text
//! {} | ia {} ib {} ic {} A | id {} iq {} (iq* {}) | th {} | \
//! w_obs {} w_ol {} (~{} rpm) | vbus {} V pot {} | isr_max {} us | \
//! id* {} vd {} vq {} V
//! ```
//!
//! `ic` (directly measured phase C) and the trailing `id* .. vd .. vq .. V`
//! block are later additions; both are parsed optionally so logs/firmware
//! predating them still decode (missing `ic` -> reconstructed as -ia-ib; the
//! missing vd block reads 0).
//!
//! which renders (with the `probe-rs`/defmt timestamp + level prefix) as e.g.
//!
//! ```text
//! 0.100000 INFO  Run | ia 0.512 ib -0.301 A | id 0.102 iq 2.305 (iq* 2.5) | \
//! th 1.5708 | w_obs 450.1 w_ol 0.0 (~614.2 rpm) | vbus 11.1 V pot 2048.0 | isr_max 12.5 us
//! ```
//!
//! We match the telemetry body anywhere in the line, so the exact log prefix
//! (timestamp, level, colour codes) does not matter and non-telemetry log
//! lines (boot banner, "-> RUN", faults) simply do not match.

use std::sync::OnceLock;

use regex::Regex;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Idle,
    Align,
    OpenLoop,
    Run,
    Fault,
    Other,
}

impl Mode {
    fn parse(s: &str) -> Mode {
        match s {
            "Idle" => Mode::Idle,
            "Align" => Mode::Align,
            "OpenLoop" => Mode::OpenLoop,
            "Run" => Mode::Run,
            "Fault" => Mode::Fault,
            _ => Mode::Other,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Mode::Idle => "IDLE",
            Mode::Align => "ALIGN",
            Mode::OpenLoop => "OPEN LOOP",
            Mode::Run => "RUN",
            Mode::Fault => "FAULT",
            Mode::Other => "?",
        }
    }
}

/// One decoded telemetry sample. Field names match the firmware.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub mode: Mode,
    pub ia: f32,
    pub ib: f32,
    /// Phase-C current, reconstructed as `-ia - ib` (not present in the
    /// telemetry line — the firmware measures two phases and reconstructs the
    /// third from ia+ib+ic=0, so we do the same here).
    pub ic: f32,
    pub id: f32,
    pub iq: f32,
    pub iq_ref: f32,
    pub theta: f32,
    pub w_obs: f32,
    pub w_ol: f32,
    pub mech_rpm: f32,
    pub vbus: f32,
    pub pot: f32,
    pub isr_us: f32,
    /// Align d-axis current setpoint (0 on firmware without the extra block).
    pub id_ref: f32,
    /// Current-loop d-axis voltage command, after the circle limit (V).
    pub vd: f32,
    /// Current-loop q-axis voltage command, after the circle limit (V).
    pub vq: f32,
}

fn re() -> &'static Regex {
    static RE: OnceLock<Regex> = OnceLock::new();
    RE.get_or_init(|| {
        // `\S+` for every number so scientific notation / inf / nan all pass
        // through to f32::from_str, which handles them. The literal anchors
        // ("(iq*", "(~ ... rpm)", "isr_max ... us") make a false match on any
        // other log line effectively impossible.
        Regex::new(concat!(
            // `ic` is optional: newer firmware measures phase C directly and
            // prints it here; older logs omit it and we reconstruct -ia-ib.
            r"(\w+)\s*\|\s*ia\s+(\S+)\s+ib\s+(\S+)(?:\s+ic\s+(\S+))?\s+A",
            r"\s*\|\s*id\s+(\S+)\s+iq\s+(\S+)\s+\(iq\*\s+(\S+)\)",
            r"\s*\|\s*th\s+(\S+)",
            r"\s*\|\s*w_obs\s+(\S+)\s+w_ol\s+(\S+)\s+\(~\s*(\S+)\s+rpm\)",
            r"\s*\|\s*vbus\s+(\S+)\s+V\s+pot\s+(\S+)",
            r"\s*\|\s*isr_max\s+(\S+)\s+us",
            // Optional trailing block; older firmware/logs omit it entirely.
            r"(?:\s*\|\s*id\*\s+(\S+)\s+vd\s+(\S+)\s+vq\s+(\S+)\s+V)?",
        ))
        .expect("telemetry regex is valid")
    })
}

/// Parse a single log line. Returns `None` for any line that is not a
/// telemetry sample (banners, state transitions, faults, cargo output, ...).
pub fn parse_line(line: &str) -> Option<Sample> {
    let c = re().captures(line)?;
    let num = |i: usize| -> Option<f32> { c.get(i)?.as_str().parse::<f32>().ok() };
    // Optional groups: missing or unparseable -> 0.0 (keeps old lines parsing).
    let num_opt = |i: usize| -> f32 {
        c.get(i).and_then(|m| m.as_str().parse::<f32>().ok()).unwrap_or(0.0)
    };
    let ia = num(2)?;
    let ib = num(3)?;
    // ic: directly measured (group 4) when the firmware sends it, otherwise
    // reconstructed from ia+ib+ic=0 so older logs still decode.
    let ic = c.get(4).and_then(|m| m.as_str().parse::<f32>().ok()).unwrap_or(-ia - ib);
    Some(Sample {
        mode: Mode::parse(c.get(1)?.as_str()),
        ia,
        ib,
        ic,
        id: num(5)?,
        iq: num(6)?,
        iq_ref: num(7)?,
        theta: num(8)?,
        w_obs: num(9)?,
        w_ol: num(10)?,
        mech_rpm: num(11)?,
        vbus: num(12)?,
        pot: num(13)?,
        isr_us: num(14)?,
        id_ref: num_opt(15),
        vd: num_opt(16),
        vq: num_opt(17),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const LINE: &str = "0.100000 INFO  Run | ia 0.512 ib -0.301 A | id 0.102 iq 2.305 (iq* 2.5) | th 1.5708 | w_obs 450.1 w_ol 0.0 (~614.2 rpm) | vbus 11.1 V pot 2048.0 | isr_max 12.5 us | id* 3.0 vd 1.5 vq 4.2 V";

    /// The old format, before the id*/vd/vq block was appended.
    const LINE_LEGACY: &str = "0.100000 INFO  Run | ia 0.512 ib -0.301 A | id 0.102 iq 2.305 (iq* 2.5) | th 1.5708 | w_obs 450.1 w_ol 0.0 (~614.2 rpm) | vbus 11.1 V pot 2048.0 | isr_max 12.5 us";

    /// Newer format: a directly-measured `ic` in the phase block (0.777, which is
    /// deliberately NOT -ia-ib = -0.211) plus a trailing rawC.
    const LINE_IC: &str = "0.100000 INFO  Run | ia 0.512 ib -0.301 ic 0.777 A | id 0.102 iq 2.305 (iq* 2.5) | th 1.5708 | w_obs 450.1 w_ol 0.0 (~614.2 rpm) | vbus 11.1 V pot 2048.0 | isr_max 12.5 us | id* 3.0 vd 1.5 vq 4.2 V | rawA 1587 rawB 2626 rawC 2050";

    #[test]
    fn uses_measured_ic_when_present() {
        let s = parse_line(LINE_IC).expect("should parse");
        assert_eq!(s.ia, 0.512);
        assert_eq!(s.ib, -0.301);
        assert_eq!(s.ic, 0.777); // measured value, NOT the -0.211 reconstruction
        assert_eq!(s.id, 0.102);
        assert_eq!(s.iq, 2.305);
        assert_eq!(s.vq, 4.2);
    }

    #[test]
    fn parses_a_full_telemetry_line() {
        let s = parse_line(LINE).expect("should parse");
        assert_eq!(s.mode, Mode::Run);
        assert_eq!(s.ia, 0.512);
        assert_eq!(s.ib, -0.301);
        assert!((s.ic - (-0.211)).abs() < 1e-5, "ic = -ia-ib, got {}", s.ic);
        assert_eq!(s.id, 0.102);
        assert_eq!(s.iq, 2.305);
        assert_eq!(s.iq_ref, 2.5);
        assert_eq!(s.theta, 1.5708);
        assert_eq!(s.w_obs, 450.1);
        assert_eq!(s.w_ol, 0.0);
        assert_eq!(s.mech_rpm, 614.2);
        assert_eq!(s.vbus, 11.1);
        assert_eq!(s.pot, 2048.0);
        assert_eq!(s.isr_us, 12.5);
        assert_eq!(s.id_ref, 3.0);
        assert_eq!(s.vd, 1.5);
        assert_eq!(s.vq, 4.2);
    }

    #[test]
    fn parses_legacy_line_without_vd_block() {
        // Firmware/logs predating the id*/vd/vq block still decode; the new
        // fields default to 0.0 rather than failing the whole parse.
        let s = parse_line(LINE_LEGACY).expect("legacy line should still parse");
        assert_eq!(s.isr_us, 12.5);
        assert_eq!(s.id_ref, 0.0);
        assert_eq!(s.vd, 0.0);
        assert_eq!(s.vq, 0.0);
    }

    #[test]
    fn parses_without_a_log_prefix() {
        // bare message body, no timestamp/level
        let body = &LINE[LINE.find("Run").unwrap()..];
        assert!(parse_line(body).is_some());
    }

    #[test]
    fn parses_each_mode() {
        for (word, mode) in [
            ("Idle", Mode::Idle),
            ("Align", Mode::Align),
            ("OpenLoop", Mode::OpenLoop),
            ("Fault", Mode::Fault),
        ] {
            let line = LINE.replacen("Run", word, 1);
            assert_eq!(parse_line(&line).unwrap().mode, mode);
        }
    }

    #[test]
    fn ignores_non_telemetry_lines() {
        for line in [
            "--- ihm08m1-foc bring-up build ---",
            "0.050000 INFO  -> RUN (observer handoff, omega=450.0 rad/s)",
            // the overcurrent line uses `ia=..` (no space) so must NOT match
            "0.001 ERROR OVERCURRENT trip: ia=25.0 ib=-30.0 ic=5.0 A",
            "   Compiling ihm08m1-foc v0.2.0",
            "",
        ] {
            assert_eq!(parse_line(line), None, "should not parse: {line:?}");
        }
    }

    #[test]
    fn tolerates_inf_and_nan() {
        let line = LINE.replacen("450.1", "inf", 1).replacen("0.102", "NaN", 1);
        let s = parse_line(&line).unwrap();
        assert!(s.w_obs.is_infinite());
        assert!(s.id.is_nan());
    }
}
