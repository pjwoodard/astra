//! Sensorless FOC firmware with staged bring-up
#![no_std]
#![no_main]

mod foc;
mod mcu;
mod observer;

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::pac::interrupt;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use panic_probe as _;

use foc::{clampf, clarke, inv_park, limit_voltage, park, sin_cos, svpwm, wrap_angle, Pi};
use observer::FluxObserver;

// The blue button B1 is on PC13, whose EXTI line is serviced by the shared
// EXTI15_10 vector. ExtiInput needs that IRQ bound to embassy's handler.
bind_interrupts!(struct Irqs {
    EXTI15_10 => exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI15_10>;
});

const PWM_FREQ_HZ: f32 = 25_000.0;
const DT: f32 = 1.0 / PWM_FREQ_HZ;

// --- Motor: Turnigy Multistar 2216-800Kv, 14 pole (7 pole pairs) ---
const POLE_PAIRS: f32 = 7.0;
const R_PHASE: f32 = 0.09; // ESTIMATE — measure line-to-line / 2
const L_PHASE: f32 = 25.0e-6; // ESTIMATE — measure line-to-line / 2
const KV: f32 = 800.0;
const FLUX_LINKAGE: f32 = 60.0 / (1.732_050_8 * 6.283_185_3 * KV * POLE_PAIRS);

// --- Current limits ---
const I_MAX: f32 = 8.0;
const I_TRIP: f32 = 10.0;

// --- Startup sequence ---
const I_ALIGN: f32 = 3.0;
const ALIGN_RAMP_S: f32 = 0.3;
const ALIGN_HOLD_S: f32 = 0.4;
const I_STARTUP: f32 = 3.5;
const OL_ACCEL: f32 = 900.0; // elec rad/s^2
const OMEGA_HANDOFF: f32 = 700.0 / 60.0 * 6.283_185_3 * POLE_PAIRS; // 700 mech rpm

// --- Speed command range (mechanical rpm from the pot) ---
const RPM_MIN: f32 = 800.0;
const RPM_MAX: f32 = 5000.0;
const RPM_TO_OMEGA_E: f32 = 6.283_185_3 / 60.0 * POLE_PAIRS;

// --- Loop gains ---
const CURR_KP: f32 = L_PHASE * 5000.0; // wc ~ 2*pi*800 Hz
const CURR_KI: f32 = R_PHASE * 5000.0;
const SPEED_KP: f32 = 0.004;
const SPEED_KI: f32 = 0.05;
const OBS_GAMMA: f32 = 1.0e8;
const PLL_KP: f32 = 450.0;
const PLL_KI: f32 = 62_500.0;

// --- Board scaling (X-NUCLEO-IHM08M1) ---
const CURR_AMP_PER_LSB: f32 = 3.3 / 4096.0 / (5.7 * 0.010); // TSV994 sense gain 5.7, 10 mOhm shunt
/// Flip to -1.0 if ALIGN faults instantly / id runs away negative in telemetry.
const CURR_SIGN: f32 = -1.0;
const VBUS_V_PER_LSB: f32 = 3.3 / 4096.0 * (169.0 + 9.31) / 9.31;
const VBUS_DEFAULT: f32 = 11.1;
/// Plausible bus-sense window. A reading outside this is treated as a sensor
/// fault (e.g. a bad VBUS divider) and ignored in favour of VBUS_DEFAULT, so a
/// bogus value can never corrupt vmax or the SVPWM denominator. Raise the top
/// if you actually run a high bus.
const VBUS_SANE_MIN: f32 = 6.0;
const VBUS_SANE_MAX: f32 = 45.0;

/// Telemetry every N slow ticks (slow tick = 1 kHz) -> 10 Hz
const TELEM_DIV: u32 = 100;

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

static ENABLE: AtomicBool = AtomicBool::new(false);
static FAULT: AtomicBool = AtomicBool::new(false);
static MODE_DISPLAY: AtomicU8 = AtomicU8::new(0); // 0 idle, 1 active, 2 fault

#[derive(Clone, Copy, PartialEq, defmt::Format)]
enum Mode {
    Idle,
    Align,
    OpenLoop,
    Run,
    Fault,
}

struct Foc {
    mode: Mode,
    t_state: f32,
    theta: f32,
    omega_ol: f32,
    id_ref: f32,
    iq_ref: f32,
    omega_ref: f32,
    pot_filt: f32,
    vbus: f32,
    offset_a: f32,
    offset_b: f32,
    offset_c: f32,
    pid_id: Pi,
    pid_iq: Pi,
    pid_speed: Pi,
    obs: FluxObserver,
    slow_cnt: u32,
    telem_cnt: u32,
    adc_pending: bool,
    // Debug/telemetry mirrors
    dbg_ia: f32,
    dbg_ib: f32,
    dbg_ic: f32, // directly measured phase C (not the -ia-ib reconstruction)
    dbg_id: f32,
    dbg_iq: f32,
    dbg_vd: f32,
    dbg_vq: f32,
    // Raw injected ADC counts before offset/scale — for probing the inputs
    // directly (ADC1 injected, ADC2 injected).
    dbg_raw_a: u16,
    dbg_raw_b: u16,
    dbg_raw_c: u16, // ADC1 2nd injected: phase C (PC0)
    isr_max_cycles: u32,
}

static mut FOC: Foc = Foc {
    mode: Mode::Idle,
    t_state: 0.0,
    theta: 0.0,
    omega_ol: 0.0,
    id_ref: 0.0,
    iq_ref: 0.0,
    omega_ref: 0.0,
    pot_filt: 0.0,
    vbus: VBUS_DEFAULT,
    offset_a: 2048.0,
    offset_b: 2048.0,
    offset_c: 2048.0,
    pid_id: Pi::new(CURR_KP, CURR_KI, 30.0),
    pid_iq: Pi::new(CURR_KP, CURR_KI, 30.0),
    pid_speed: Pi::new(SPEED_KP, SPEED_KI, I_MAX),
    obs: FluxObserver::new(R_PHASE, L_PHASE, FLUX_LINKAGE, OBS_GAMMA, PLL_KP, PLL_KI),
    slow_cnt: 0,
    telem_cnt: 0,
    adc_pending: false,
    dbg_ia: 0.0,
    dbg_ib: 0.0,
    dbg_ic: 0.0,
    dbg_id: 0.0,
    dbg_iq: 0.0,
    dbg_vd: 0.0,
    dbg_vq: 0.0,
    dbg_raw_a: 0,
    dbg_raw_b: 0,
    dbg_raw_c: 0,
    isr_max_cycles: 0,
};

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Embassy owns the vector table, the 170 MHz clock tree, and the async time
    // driver (on TIM2). The hard-real-time peripherals (TIM1 complementary PWM,
    // the injected ADC, and the control ISR) are still brought up register-exact
    // through the mcu backend on the stm32g4 PAC — embassy's HAL can't express
    // them.
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        // HSI16 / 4 * 85 / 2 = 170 MHz — identical tree to the former
        // hand-rolled setup_clocks(). boost is required above 150 MHz.
        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1; // TIM1 kernel clock = 170 MHz
        config.rcc.boost = true;
    }
    let p = embassy_stm32::init(config);

    // DWT cycle counter for ISR duration measurement (present on M4 and M33).
    // Core peripherals — steal because embassy_stm32::init consumed the device
    // peripherals but leaves the Cortex-M core block to us.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    mcu::setup_gpio();
    mcu::setup_tim1();
    mcu::setup_adc();
    mcu::log_clock_check();

    defmt::info!("--- STM32G4 FOC ---");

    // TIM1 free-runs with MOE=0 -> ADC triggers fire, no gate drive, no current:
    // perfect for measuring the zero-current amplifier offsets.
    let (offset_a, offset_b, offset_c) = mcu::calibrate_current_offsets();
    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(FOC);
        s.offset_a = offset_a;
        s.offset_b = offset_b;
        s.offset_c = offset_c;
    }
    defmt::info!(
        "current offsets: A = {=f32} B = {=f32} C = {=f32} counts (expect ~2048 = 1.65 V)",
        offset_a,
        offset_b,
        offset_c
    );

    // Housekeeping peripherals, now owned by embassy and driven off the hot
    // path. PB2 = expansion-board status LED; PC13 = blue button B1. B1 on this
    // board is ACTIVE-HIGH — idle is held low by the internal pull-down and a
    // press drives PC13 high, so a press is a RISING edge.
    let status_led = Output::new(p.PB2, Level::Low, Speed::Low);
    let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down, Irqs);

    mcu::enable_adc_interrupt();

    defmt::info!("Running, hit the blue button to start/stop the motor");

    // embassy-executor 0.10: the task fn does the pool allocation and returns
    // Result<SpawnToken, SpawnError>; Spawner::spawn then consumes the token.
    spawner.spawn(button_task(button).unwrap());
    spawner.spawn(telemetry_task().unwrap());

    // LED heartbeat (this task). 100 ms half-period (200 ms -> 5 Hz) reproduces
    // the fault blink the old 1 ms `blink/100 % 2` counter produced.
    let mut status_led = status_led;
    let mut phase = false;
    loop {
        Timer::after_millis(100).await;
        phase = !phase;
        let on = match MODE_DISPLAY.load(Ordering::Relaxed) {
            0 => false, // idle: off
            2 => phase, // fault: blink
            _ => true,  // active: on
        };
        status_led.set_level(if on { Level::High } else { Level::Low });
    }
}

/// Blue button (B1/PC13, ACTIVE-HIGH with internal pull-down): each press
/// toggles the motor enable, or clears a latched fault. Interrupt-driven via
/// EXTI — the task parks until the rising edge (press) wakes it, then debounces.
#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static, Async>) {
    loop {
        button.wait_for_falling_edge().await; // press pulls PC13 high
        if FAULT.load(Ordering::Relaxed) {
            ENABLE.store(false, Ordering::Relaxed);
            FAULT.store(false, Ordering::Relaxed);
            defmt::info!("fault cleared; system disarmed");
        } else {
            let en = !ENABLE.load(Ordering::Relaxed);
            ENABLE.store(en, Ordering::Relaxed);
            defmt::info!("button: enable = {=bool}", en);
        }
        Timer::after_millis(30).await; // debounce
    }
}

/// Telemetry printer. The control ISR hands off a snapshot via TELEM_SIGNAL at
/// 10 Hz; the (blocking, RTT-backed) defmt formatting happens here in a task,
/// never inside the ISR — that is what keeps it off the 40 us control budget.
#[embassy_executor::task]
async fn telemetry_task() {
    loop {
        let t = TELEM_SIGNAL.wait().await;
        defmt::info!(
            "{} | ia {=f32} ib {=f32} ic {=f32} A | id {=f32} iq {=f32} (iq* {=f32}) | th {=f32} | w_obs {=f32} w_ol {=f32} (~{=f32} rpm) | vbus {=f32} V pot {=f32} | isr_max {=f32} us | id* {=f32} vd {=f32} vq {=f32} V | rawA {=u16} rawB {=u16} rawC {=u16}",
            t.mode, t.ia, t.ib, t.ic, t.id, t.iq, t.iq_ref,
            t.theta, t.w_obs, t.w_ol, t.mech_rpm, t.vbus, t.pot, t.isr_us,
            t.id_ref, t.vd, t.vq, t.raw_a, t.raw_b, t.raw_c
        );
    }
}

// ---------------------------------------------------------------------------
// The 25 kHz control interrupt
// ---------------------------------------------------------------------------
//
// The injected end-of-sequence IRQ fires once per PWM period and runs the whole
// loop. Its vector name is the only device-specific bit, so each backend gets a
// thin shim that calls the shared `control_isr()`.

#[cfg(feature = "mcu-g4")]
#[interrupt]
fn ADC1_2() {
    control_isr();
}

#[cfg(feature = "mcu-h5")]
#[interrupt]
fn ADC1() {
    control_isr();
}

#[inline(always)]
fn control_isr() {
    let s: &mut Foc = unsafe { &mut *core::ptr::addr_of_mut!(FOC) };

    // Timing pin up + cycle stamp
    mcu::timing_pin_high();
    let t0 = cortex_m::peripheral::DWT::cycle_count();

    // --- Phase currents ---
    // Two shunts are measured (raw0 = phase A on ADC1, raw1 = phase B on ADC2)
    // and phase C is reconstructed from ia+ib+ic=0.
    let (raw0, raw1) = mcu::ack_and_read_currents();
    s.dbg_raw_a = raw0; // ADC1 injected raw count (PA0)
    s.dbg_raw_b = raw1; // ADC2 injected raw count (PC1)
    let ib = (raw1 as f32 - s.offset_b) * CURR_AMP_PER_LSB * CURR_SIGN;
    let ia = (raw0 as f32 - s.offset_a) * CURR_AMP_PER_LSB * CURR_SIGN;
    let ic = -ia - ib; // reconstructed
    s.dbg_ia = ia;
    s.dbg_ib = ib;

    // Directly-measured phase C (ADC1's 2nd injected conversion, PC0). This is
    // an INDEPENDENT measurement, unlike the reconstructed `ic = -ia-ib` the
    // control loop uses below — so it can reveal a phase that isn't actually
    // conducting (where the reconstruction is forced toward 0). Telemetry only;
    // the control math is unchanged.
    let raw_c = mcu::read_curr_c_raw();
    s.dbg_raw_c = raw_c;
    s.dbg_ic = (raw_c as f32 - s.offset_c) * CURR_AMP_PER_LSB * CURR_SIGN;

    // --- Over-current (armed whenever the gates can be driving) ---
    let oc_armed = s.mode != Mode::Idle;
    if oc_armed
        && (ia > I_TRIP
            || ia < -I_TRIP
            || ib > I_TRIP
            || ib < -I_TRIP
            || ic > I_TRIP
            || ic < -I_TRIP)
    {
        trip_fault(s);
        defmt::error!(
            "OVERCURRENT trip: ia={=f32} ib={=f32} ic={=f32} A",
            ia,
            ib,
            ic
        );
        isr_exit(s, t0);
        return;
    }

    let (i_alpha, i_beta) = clarke(ia, ib);
    s.t_state += DT;
    let enabled = ENABLE.load(Ordering::Relaxed);

    match s.mode {
        Mode::Idle => {
            mcu::set_duties(0.5, 0.5, 0.5);
            if enabled {
                enter(s, Mode::Align);
                s.pid_id.reset();
                s.pid_iq.reset();
                s.pid_speed.reset();
                s.theta = 0.0;
                s.omega_ol = 0.0;
                mcu::enable_gates();
                defmt::info!("-> ALIGN");
            }
            MODE_DISPLAY.store(0, Ordering::Relaxed);
            slow_tick(s);
            isr_exit(s, t0);
            return;
        }
        Mode::Align => {
            s.theta = 0.0;
            let ramp = clampf(s.t_state / ALIGN_RAMP_S, 0.0, 1.0);
            s.id_ref = I_ALIGN * ramp;
            s.iq_ref = 0.0;
            if s.t_state > ALIGN_RAMP_S + ALIGN_HOLD_S {
                enter(s, Mode::OpenLoop);
                s.omega_ol = 0.0;
                defmt::info!("-> OPEN LOOP");
            }
        }
        Mode::OpenLoop => {
            s.omega_ol += OL_ACCEL * DT;
            s.theta = wrap_angle(s.theta + s.omega_ol * DT);
            s.id_ref = I_STARTUP;
            s.iq_ref = 0.0;

            if s.omega_ol >= OMEGA_HANDOFF {
                let werr = s.obs.omega - s.omega_ol;
                if werr < 0.25 * s.omega_ol && werr > -0.25 * s.omega_ol {
                    enter(s, Mode::Run);
                    s.id_ref = 0.0;
                    s.pid_speed.preload(I_STARTUP * 0.7);
                    s.omega_ref = OMEGA_HANDOFF;
                    defmt::info!("-> RUN (observer handoff, omega={=f32} rad/s)", s.obs.omega);
                } else if s.omega_ol > OMEGA_HANDOFF * 1.6 {
                    trip_fault(s);
                    defmt::error!(
                        "handoff FAILED: obs.omega={=f32} vs omega_ol={=f32} — tune R/L/OBS_GAMMA",
                        s.obs.omega,
                        s.omega_ol
                    );
                    isr_exit(s, t0);
                    return;
                }
            }
        }
        Mode::Run => {
            s.theta = s.obs.theta;
            s.id_ref = 0.0;
        }
        Mode::Fault => {
            // Latched until the button clears the FAULT flag (main loop).
            // Once it does, reconcile mode back to Idle so the system can
            // re-arm — otherwise Fault is a terminal state until reset.
            if FAULT.load(Ordering::Relaxed) {
                mcu::set_duties(0.5, 0.5, 0.5);
                MODE_DISPLAY.store(2, Ordering::Relaxed);
            } else {
                stop(s);
                defmt::info!("-> IDLE (fault cleared)");
            }
            slow_tick(s);
            isr_exit(s, t0);
            return;
        }
    }

    if !enabled && s.mode != Mode::Idle {
        stop(s);
        defmt::info!("-> IDLE (stopped)");
        isr_exit(s, t0);
        return;
    }
    MODE_DISPLAY.store(1, Ordering::Relaxed);

    // --- Current loops ---
    let (sin_t, cos_t) = sin_cos(s.theta);
    let (id, iq) = park(i_alpha, i_beta, sin_t, cos_t);
    s.dbg_id = id;
    s.dbg_iq = iq;

    let vd = s.pid_id.update(s.id_ref - id, DT);
    let vq = s.pid_iq.update(s.iq_ref - iq, DT);
    let vmax = 0.55 * s.vbus;
    let (vd, vq) = limit_voltage(vd, vq, vmax);
    s.dbg_vd = vd;
    s.dbg_vq = vq;
    let (v_alpha, v_beta) = inv_park(vd, vq, sin_t, cos_t);

    // --- Observer (runs in every powered state) ---
    s.obs.update(v_alpha, v_beta, i_alpha, i_beta, DT);

    // --- PWM out ---
    let (da, db, dc) = svpwm(v_alpha, v_beta, s.vbus);
    mcu::set_duties(da, db, dc);

    slow_tick(s);
    isr_exit(s, t0);
}

/// 1 kHz housekeeping: pot + VBUS acquisition, speed loop, 10 Hz telemetry.
fn slow_tick(s: &mut Foc) {
    s.slow_cnt += 1;
    if s.slow_cnt < 25 {
        return;
    }
    s.slow_cnt = 0;

    // Alternate: collect previous regular conversions / start new ones
    if s.adc_pending {
        if let Some(raw) = mcu::try_read_vbus_raw() {
            let v_meas = raw as f32 * VBUS_V_PER_LSB;
            if v_meas > VBUS_SANE_MIN && v_meas < VBUS_SANE_MAX {
                s.vbus = 0.9 * s.vbus + 0.1 * v_meas;
            } else {
                // Implausible reading (bad divider / connection). Don't let it
                // drive vmax or the SVPWM denominator — fall back to a safe
                // assumed bus. A pinned VBUS_DEFAULT in telemetry flags the fault.
                s.vbus = VBUS_DEFAULT;
            }
        }
        if let Some(raw) = mcu::try_read_pot_raw() {
            s.pot_filt = 0.95 * s.pot_filt + 0.05 * (raw as f32);
        }
        s.adc_pending = false;
    } else {
        mcu::start_vbus_pot_conv();
        s.adc_pending = true;
    }

    // Speed loop (full build only reaches Run)
    let frac = clampf(s.pot_filt / 4095.0, 0.0, 1.0);
    let rpm = RPM_MIN + (RPM_MAX - RPM_MIN) * frac;
    let target = rpm * RPM_TO_OMEGA_E;
    if s.mode == Mode::Run {
        let step = 5.0; // elec rad/s per ms
        if target > s.omega_ref + step {
            s.omega_ref += step;
        } else if target < s.omega_ref - step {
            s.omega_ref -= step;
        } else {
            s.omega_ref = target;
        }
        s.iq_ref = s.pid_speed.update(s.omega_ref - s.obs.omega, 0.001);
    }

    s.telem_cnt += 1;
    if s.telem_cnt >= TELEM_DIV {
        s.telem_cnt = 0;
        let isr_us = s.isr_max_cycles as f32 / mcu::CYCLES_PER_US;
        s.isr_max_cycles = 0;
        TELEM_SIGNAL.signal(Telemetry {
            mode: s.mode,
            ia: s.dbg_ia,
            ib: s.dbg_ib,
            ic: s.dbg_ic,
            id: s.dbg_id,
            iq: s.dbg_iq,
            iq_ref: s.iq_ref,
            id_ref: s.id_ref,
            vd: s.dbg_vd,
            vq: s.dbg_vq,
            theta: s.theta,
            w_obs: s.obs.omega,
            w_ol: s.omega_ol,
            mech_rpm: s.obs.omega / RPM_TO_OMEGA_E,
            vbus: s.vbus,
            pot: s.pot_filt,
            isr_us,
            raw_a: s.dbg_raw_a,
            raw_b: s.dbg_raw_b,
            raw_c: s.dbg_raw_c,
        });
    }
}

/// Snapshot handed out of the ISR at 10 Hz; formatted/printed by telemetry_task.
#[derive(Clone, Copy)]
struct Telemetry {
    mode: Mode,
    ia: f32,
    ib: f32,
    ic: f32,
    id: f32,
    iq: f32,
    iq_ref: f32,
    id_ref: f32,
    vd: f32,
    vq: f32,
    theta: f32,
    w_obs: f32,
    w_ol: f32,
    mech_rpm: f32,
    vbus: f32,
    pot: f32,
    isr_us: f32,
    raw_a: u16,
    raw_b: u16,
    raw_c: u16,
}

/// ISR -> telemetry_task hand-off. Replaces the old `static mut TELEM` +
/// `TELEM_READY` poll: the ISR `signal()`s the latest snapshot and the task
/// `wait()`s on it, with no unsafe and no busy polling.
static TELEM_SIGNAL: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();

/// Record worst-case ISR duration and drop the timing pin.
#[inline(always)]
fn isr_exit(s: &mut Foc, t0: u32) {
    let dt = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t0);
    if dt > s.isr_max_cycles {
        s.isr_max_cycles = dt;
    }
    mcu::timing_pin_low();
}

// ---------------------------------------------------------------------------
// State helpers
// ---------------------------------------------------------------------------

fn enter(s: &mut Foc, m: Mode) {
    if m == Mode::OpenLoop {
        s.obs.seed(s.theta, 0.0);
    }
    s.mode = m;
    s.t_state = 0.0;
}

fn stop(s: &mut Foc) {
    mcu::disable_gates();
    mcu::set_duties(0.5, 0.5, 0.5);
    s.mode = Mode::Idle;
    s.t_state = 0.0;
    s.id_ref = 0.0;
    s.iq_ref = 0.0;
    MODE_DISPLAY.store(0, Ordering::Relaxed);
}

fn trip_fault(s: &mut Foc) {
    mcu::disable_gates();
    mcu::set_duties(0.5, 0.5, 0.5);
    s.mode = Mode::Fault;
    ENABLE.store(false, Ordering::Relaxed);
    FAULT.store(true, Ordering::Relaxed);
    MODE_DISPLAY.store(2, Ordering::Relaxed);
}
