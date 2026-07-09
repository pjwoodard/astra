//! STM32H533 (NUCLEO-H533RE) backend. Cortex-M33 @ 250 MHz.
//!
//! Same X-NUCLEO-IHM08M1 shield, same physical pins as the G474 board (the
//! Nucleo-64 Arduino/morpho pinout is preserved across the two boards), but a
//! different silicon generation: the `stm32h5` PAC uses METHOD-STYLE register
//! access (`dp.RCC.cr()` not `dp.RCC.cr`), the clock tree is PWR-VOS + PLL1
//! (not G4 boost + PLLCFGR), and the ADC is the newer H5/U5-generation IP.
//!
//! !!! HARDWARE-UNVERIFIED !!! This backend compiles against the PAC but has
//! not been run on a board. Two numbers below come from the STM32H533 datasheet
//! (DS14539) and MUST be confirmed before driving gates: the TIM1 alternate
//! function (`AF_TIM1`) and the ADC channel numbers.

pub use stm32h5::stm32h533 as pac;
pub use pac::interrupt;

// --- Clock / timing facts this device presents to the control code ---
pub const SYSCLK_HZ: u32 = 250_000_000;
pub const CYCLES_PER_US: f32 = 250.0;
/// 250 MHz / (2 * 25 kHz), center-aligned.
pub const TIM_ARR: u16 = 5000;

/// BDTR.DTG for ~704 ns dead time at a 250 MHz timer clock (tDTS = 4 ns).
/// Encoding 10xxxxxx: DT = (64 + DTG[5:0]) * 2 * tDTS -> (64+24)*8 ns = 704 ns.
/// Matches the G474 build's ~706 ns.
const DEAD_TIME_DTG: u8 = 0x98;

// ---------------------------------------------------------------------------
// Board-specific magic numbers — VERIFY against STM32H533 datasheet DS14539
// ---------------------------------------------------------------------------

/// TIM1 alternate function on PA7/8/9/10, PB0/1. AF1 on STM32H5 (was AF6 on G4).
const AF_TIM1: u8 = 1;

/// ADC channel numbers for the shield's analog pins on the H533 (DS14539 /
/// ST CubeMX DB). On H5 every one of these pins is reachable by BOTH ADC1 and
/// ADC2, so the split below is a free choice. (On the G474 the numbers were
/// PA0=IN1, PC1=IN7, PA1=IN2, PA4=IN17 — all different.)
///
/// POT PIN CHANGED FOR THE H533RE: the shield's speed-pot sits on the morpho
/// "A2" slot (CN7-32). On the classic Nucleo-64 (and the G474) that pin is PA4,
/// but the NUCLEO-H533RE puts its ST-LINK virtual-COM-port on USART3 (PA3/PA4),
/// so PA4 is consumed by the VCP and CN7-32 is wired to PA2 instead (UM3121
/// Table 17). So the pot is read on PA2 = ADC ch14 here, not PA4/ch18.
/// (If the pot still doesn't track, continuity-check CN7-32: ST says PA2, but
/// Zephyr's board file claims PB1 — and PB1 = ch5 would collide with the WL
/// gate, so it could not be used for the pot without rework.)
const CH_CURR_A: u8 = 0; //  PA0 curr A  (ADC1 injected)  -> SMPR1
const CH_CURR_C: u8 = 10; // PC0 curr C  (ADC1 injected, shunt-bc only) -> SMPR2
const CH_CURR_B: u8 = 11; // PC1 curr B  (ADC2 injected)  -> SMPR2
const CH_VBUS: u8 = 1; //    PA1 VBUS    (ADC1 regular)   -> SMPR1
const CH_POT: u8 = 14; //    PA2 pot     (ADC2 regular)   -> SMPR2  (H533RE: A2=PA2, not PA4)

// ---------------------------------------------------------------------------
// One-time bring-up
// ---------------------------------------------------------------------------

pub fn setup_clocks() {
    let dp = unsafe { pac::Peripherals::steal() };
    let rcc = dp.RCC;
    let pwr = dp.PWR;
    let flash = dp.FLASH;

    // 1) Voltage scaling VOS0 for the 250 MHz range (PWR is always clocked on
    //    H5 — no RCC PWREN bit, unlike G4).
    pwr.voscr().modify(|_, w| unsafe { w.vos().bits(0b11) }); // VOS0
    while pwr.vossr().read().vosrdy().bit_is_clear() {}

    // 2) Flash latency before raising the clock: 5 WS + WRHIGHFREQ=2 at VOS0/250 MHz.
    flash.acr().modify(|_, w| unsafe { w.latency().bits(5).wrhighfreq().bits(0b10) });
    flash.acr().modify(|_, w| w.prften().set_bit());
    while flash.acr().read().latency().bits() != 5 {}

    // 3) HSI at full 64 MHz (reset default is /2 = 32 MHz).
    rcc.cr().modify(|_, w| w.hsion().set_bit());
    while rcc.cr().read().hsirdy().bit_is_clear() {}
    rcc.cr().modify(|_, w| unsafe { w.hsidiv().bits(0b00) }); // HSIDIV = /1 -> 64 MHz

    // 4) PLL1: HSI 64 / M=16 = 4 MHz ref; *N=125 = 500 MHz VCO; /P=2 = 250 MHz.
    //    N is written as N-1, P as P-1, M raw.
    rcc.pll1cfgr().modify(|_, w| unsafe {
        w.pll1src().bits(0b01) //   HSI
            .pll1m().bits(16) //     ref = 4 MHz
            .pll1rge().bits(0b11) // input range 4..8 MHz
            .pll1vcosel().clear_bit() // wide VCO (192..836 MHz)
            .pll1fracen().clear_bit() // integer mode
            .pll1pen().set_bit() //   enable P output -> SYSCLK
    });
    rcc.pll1divr().modify(|_, w| unsafe {
        w.pll1n().bits(124).pll1p().bits(1) // N-1 = 125, P-1 = 2
    });
    rcc.cr().modify(|_, w| w.pll1on().set_bit());
    while rcc.cr().read().pll1rdy().bit_is_clear() {}

    // 5) All buses /1 (every bus is in spec at 250 MHz on H5).
    rcc.cfgr2().modify(|_, w| unsafe {
        w.hpre().bits(0).ppre1().bits(0).ppre2().bits(0).ppre3().bits(0)
    });

    // 6) Switch SYSCLK to PLL1.
    rcc.cfgr1().modify(|_, w| unsafe { w.sw().bits(0b11) });
    while rcc.cfgr1().read().sws().bits() != 0b11 {}
}

/// SWS should read 0b11 (PLL1 selected). If it does not, every "us" figure in
/// the telemetry (which assumes 250 MHz) is mis-scaled.
pub fn log_clock_check() {
    let dp = unsafe { pac::Peripherals::steal() };
    defmt::info!(
        "clock check: SWS={=u8} (want 3=PLL1) PLL1RDY={=bool}",
        dp.RCC.cfgr1().read().sws().bits(),
        dp.RCC.cr().read().pll1rdy().bit_is_set()
    );
}

pub fn setup_gpio() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.RCC.ahb2enr().modify(|_, w| {
        w.gpioaen().set_bit().gpioben().set_bit().gpiocen().set_bit()
    });

    let gpioa = dp.GPIOA;
    let gpiob = dp.GPIOB;
    let gpioc = dp.GPIOC;

    // PA0/PA1 analog (curr A, VBUS); PA2 analog (pot — H533RE routes the shield's
    // A2/pot slot to PA2, not PA4; PA4 is the ST-LINK VCP here). PA7..PA10 AF
    // (TIM1); PA5 output (LD2, stage 0 only). PA4 left at reset (VCP owns it).
    gpioa.moder().modify(|_, w| unsafe {
        w.mode0().bits(0b11)
            .mode1().bits(0b11)
            .mode2().bits(0b11)
            .mode7().bits(0b10)
            .mode8().bits(0b10)
            .mode9().bits(0b10)
            .mode10().bits(0b10)
    });
    #[cfg(feature = "stage0-pwm")]
    gpioa.moder().modify(|_, w| unsafe { w.mode5().bits(0b01) });
    gpioa.ospeedr().modify(|_, w| unsafe {
        w.ospeed7().bits(0b11).ospeed8().bits(0b11).ospeed9().bits(0b11).ospeed10().bits(0b11)
    });
    gpioa.afrl().modify(|_, w| unsafe { w.afrel7().bits(AF_TIM1) });
    gpioa.afrh().modify(|_, w| unsafe {
        w.afrel8().bits(AF_TIM1).afrel9().bits(AF_TIM1).afrel10().bits(AF_TIM1)
    });

    // PB0/PB1 AF (CH2N/CH3N); PB2 LED output
    gpiob.moder().modify(|_, w| unsafe {
        w.mode0().bits(0b10).mode1().bits(0b10).mode2().bits(0b01)
    });
    gpiob.ospeedr().modify(|_, w| unsafe { w.ospeed0().bits(0b11).ospeed1().bits(0b11) });
    gpiob.afrl().modify(|_, w| unsafe { w.afrel0().bits(AF_TIM1).afrel1().bits(AF_TIM1) });

    // PC0/PC1 analog; PC10 ISR timing output; PC13 button input
    gpioc.moder().modify(|_, w| unsafe {
        w.mode0().bits(0b11).mode1().bits(0b11).mode10().bits(0b01).mode13().bits(0b00)
    });
    gpioc.ospeedr().modify(|_, w| unsafe { w.ospeed10().bits(0b11) });
}

pub fn setup_tim1() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.RCC.apb2enr().modify(|_, w| w.tim1en().set_bit());
    let tim = dp.TIM1;

    tim.psc().write(|w| unsafe { w.bits(0) });
    tim.arr().write(|w| unsafe { w.bits(TIM_ARR as u32) });
    tim.rcr().write(|w| unsafe { w.bits(1) }); // update on valleys only

    tim.ccmr1_output().modify(|_, w| unsafe {
        w.oc1m().bits(0b110).oc1pe().set_bit().oc2m().bits(0b110).oc2pe().set_bit()
    });
    // CH3 = phase-W PWM. CH4 is an internal-only compare used to trigger the ADC.
    tim.ccmr2_output().modify(|_, w| unsafe {
        w.oc3m().bits(0b110).oc3pe().set_bit().oc4m().bits(0b110).oc4pe().set_bit()
    });

    tim.ccer().modify(|_, w| {
        w.cc1e().set_bit().cc1ne().set_bit()
            .cc2e().set_bit().cc2ne().set_bit()
            .cc3e().set_bit().cc3ne().set_bit()
    });

    // TRGO = OC4REF (MMS = 0b111). With CH4 in PWM mode 1 and CCR4 just below
    // ARR, OC4REF has one rising edge per period on the downslope past the peak,
    // where all three low-side FETs conduct — the valid shunt-sampling instant.
    tim.cr2().modify(|_, w| unsafe { w.mms().bits(0b111) });

    tim.bdtr().modify(|_, w| unsafe {
        w.dtg().bits(DEAD_TIME_DTG).ossr().set_bit().ossi().set_bit().moe().clear_bit()
    });

    set_duties(0.5, 0.5, 0.5);
    tim.ccr4().write(|w| unsafe { w.bits((TIM_ARR - 100) as u32) });
    tim.cr1().modify(|_, w| unsafe { w.cms().bits(0b01).arpe().set_bit() });

    tim.egr().write(|w| w.ug().set_bit());
    tim.cr1().modify(|_, w| w.cen().set_bit());
}

pub fn setup_adc() {
    let dp = unsafe { pac::Peripherals::steal() };

    dp.RCC.ahb2enr().modify(|_, w| w.adcen().set_bit());
    // ADC kernel clock = HCLK; common prescaler /4 -> 62.5 MHz (async CKMODE=0).
    dp.RCC.ccipr5().modify(|_, w| unsafe { w.adcdacsel().bits(0b000) }); // HCLK
    dp.ADCC.ccr().modify(|_, w| unsafe { w.presc().bits(0b0011) }); // /6 -> 41.7 MHz (was 0b0010 = /4, 62.5 MHz)

    // ADC1 injected reads phase A (PA0) normally, or phase C (PC0) under shunt-bc
    // (phase-A input unavailable). ADC2 injected always reads phase B (PC1).
    #[cfg(not(feature = "shunt-bc"))]
    let inj1 = CH_CURR_A;
    #[cfg(feature = "shunt-bc")]
    let inj1 = CH_CURR_C;
    setup_one_adc(&dp.ADC1, inj1, CH_VBUS);
    setup_one_adc(&dp.ADC2, CH_CURR_B, CH_POT);

    dp.ADC1.cr().modify(|_, w| w.jadstart().set_bit());
    dp.ADC2.cr().modify(|_, w| w.jadstart().set_bit());
}

/// Bring one ADC out of deep-power-down, calibrate, enable, and program one
/// injected channel (`inj`, TIM1-triggered) plus one regular channel (`reg`).
fn setup_one_adc(adc: &pac::adc1::RegisterBlock, inj: u8, reg: u8) {
    adc.cr().modify(|_, w| w.deeppwd().clear_bit());
    adc.cr().modify(|_, w| w.advregen().set_bit());
    cortex_m::asm::delay(250 * 30); // > tADCVREG_STUP
    adc.cr().modify(|_, w| w.adcaldif().clear_bit()); // single-ended calibration
    adc.cr().modify(|_, w| w.adcal().set_bit());
    while adc.cr().read().adcal().bit_is_set() {}
    cortex_m::asm::delay(250);
    adc.isr().write(|w| w.adrdy().clear_bit_by_one());
    adc.cr().modify(|_, w| w.aden().set_bit());
    while adc.isr().read().adrdy().bit_is_clear() {}

    // Injected: 1 conversion of `inj`, hardware-triggered on TIM1_TRGO rising
    // (TRGO = OC4REF, set in setup_tim1). JEXTSEL=Tim1Trgo(0), JEXTEN=rising.
    adc.jsqr().write(|w| unsafe {
        w.jl().bits(0).jextsel().tim1_trgo().jexten().rising_edge().jsq1().bits(inj)
    });
    // Regular: 1 conversion of `reg`, software-started.
    adc.sqr1().write(|w| unsafe { w.l().bits(0).sq1().bits(reg) });

    set_sample_time(adc, inj);
    set_sample_time(adc, reg);
}

/// Long sample time (0b110) for one channel, picking SMPR1 (ch 0-9) or SMPR2.
fn set_sample_time(adc: &pac::adc1::RegisterBlock, ch: u8) {
    if ch <= 9 {
        adc.smpr1().modify(|r, w| unsafe { w.bits(r.bits() | (0b110 << (ch * 3))) });
    } else {
        adc.smpr2().modify(|r, w| unsafe { w.bits(r.bits() | (0b110 << ((ch - 10) * 3))) });
    }
}

pub fn calibrate_current_offsets() -> (f32, f32) {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut sum_a: u32 = 0;
    let mut sum_b: u32 = 0;
    const N: u32 = 512;

    for _ in 0..N {
        while dp.ADC1.isr().read().jeos().bit_is_clear() {}
        dp.ADC1.isr().write(|w| w.jeos().clear_bit_by_one());
        dp.ADC2.isr().write(|w| w.jeos().clear_bit_by_one());
        sum_a += dp.ADC1.jdr1().read().bits() & 0xFFF;
        sum_b += dp.ADC2.jdr1().read().bits() & 0xFFF;
    }
    ((sum_a / N) as f32, (sum_b / N) as f32)
}

pub fn enable_adc_interrupt() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.ADC1.ier().modify(|_, w| w.jeosie().set_bit());
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ADC1) };
}

// ---------------------------------------------------------------------------
// Hot path (control ISR)
// ---------------------------------------------------------------------------

#[inline(always)]
pub fn timing_pin_high() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.GPIOC.bsrr().write(|w| w.bs10().set_bit());
}

#[inline(always)]
pub fn timing_pin_low() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.GPIOC.bsrr().write(|w| w.br10().set_bit());
}

#[inline(always)]
pub fn ack_and_read_currents() -> (u16, u16) {
    let dp = unsafe { pac::Peripherals::steal() };
    // ADC1 drives the IRQ, so clear its injected flag. Both ADCs are triggered
    // by the same OC4REF edge and finish together, so ADC2's result is ready.
    dp.ADC1.isr().write(|w| w.jeos().clear_bit_by_one());
    let raw_a = (dp.ADC1.jdr1().read().bits() & 0xFFF) as u16; // PA0 (phase A)
    let raw_b = (dp.ADC2.jdr1().read().bits() & 0xFFF) as u16; // PC1 (phase B)
    (raw_a, raw_b)
}

#[inline(always)]
pub fn set_duties(da: f32, db: f32, dc: f32) {
    let dp = unsafe { pac::Peripherals::steal() };
    let arr = TIM_ARR as f32;
    dp.TIM1.ccr1().write(|w| unsafe { w.bits((da * arr) as u32) });
    dp.TIM1.ccr2().write(|w| unsafe { w.bits((db * arr) as u32) });
    dp.TIM1.ccr3().write(|w| unsafe { w.bits((dc * arr) as u32) });
}

#[inline(always)]
pub fn enable_gates() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.TIM1.bdtr().modify(|_, w| w.moe().set_bit());
}

#[inline(always)]
pub fn disable_gates() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.TIM1.bdtr().modify(|_, w| w.moe().clear_bit());
}

// ---------------------------------------------------------------------------
// Slow path (1 kHz): VBUS on ADC1 regular, pot on ADC2 regular
// ---------------------------------------------------------------------------

#[inline(always)]
pub fn try_read_vbus_raw() -> Option<u16> {
    let dp = unsafe { pac::Peripherals::steal() };
    if dp.ADC1.isr().read().eoc().bit_is_set() {
        Some((dp.ADC1.dr().read().bits() & 0xFFF) as u16)
    } else {
        None
    }
}

#[inline(always)]
pub fn try_read_pot_raw() -> Option<u16> {
    let dp = unsafe { pac::Peripherals::steal() };
    if dp.ADC2.isr().read().eoc().bit_is_set() {
        Some((dp.ADC2.dr().read().bits() & 0xFFF) as u16)
    } else {
        None
    }
}

#[inline(always)]
pub fn start_vbus_pot_conv() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.ADC1.cr().modify(|_, w| w.adstart().set_bit());
    dp.ADC2.cr().modify(|_, w| w.adstart().set_bit());
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------

/// B1 on PC13 is active-low; returns true while it is held down.
#[inline(always)]
pub fn button_pressed() -> bool {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.GPIOC.idr().read().id13().bit_is_clear()
}

#[inline(always)]
pub fn set_status_led(on: bool) {
    let dp = unsafe { pac::Peripherals::steal() };
    if on {
        dp.GPIOB.bsrr().write(|w| w.bs2().set_bit());
    } else {
        dp.GPIOB.bsrr().write(|w| w.br2().set_bit());
    }
}

#[inline(always)]
pub fn set_nucleo_led(on: bool) {
    let dp = unsafe { pac::Peripherals::steal() };
    if on {
        dp.GPIOA.bsrr().write(|w| w.bs5().set_bit());
    } else {
        dp.GPIOA.bsrr().write(|w| w.br5().set_bit());
    }
}
