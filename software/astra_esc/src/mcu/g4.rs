pub use stm32g4::stm32g474 as pac;
// NOTE: the interrupt vector table and the `#[interrupt]` binding now come from
// embassy-stm32's PAC (this crate is built without `rt`). See main.rs.

pub const CYCLES_PER_US: f32 = 170.0;
pub const TIM_ARR: u16 = 3400;
const DEAD_TIME_TICKS: u8 = 120; 

// ---------------------------------------------------------------------------
// One-time bring-up
// ---------------------------------------------------------------------------

// The clock tree (HSI16 / 4 * 85 / 2 = 170 MHz, boost mode, flash latency) is
// now configured by embassy_stm32::init() in main.rs — see the rcc::Config
// there, which mirrors the former hand-rolled sequence exactly.

/// Ground-truth clock check. SWS should read 0b11 (PLL) and HPRE 0 (AHB /1).
/// If either is wrong, every "us" figure in the telemetry is mis-scaled.
pub fn log_clock_check() {
    let dp = unsafe { pac::Peripherals::steal() };
    let cfgr = dp.RCC.cfgr.read();
    defmt::info!(
        "clock check: SWS={=u8} (want 3=PLL) HPRE={=u8} (want 0=/1) PLLRDY={=bool}",
        cfgr.sws().bits(),
        cfgr.hpre().bits(),
        dp.RCC.cr.read().pllrdy().bit_is_set()
    );
}

pub fn setup_gpio() {
    let dp = unsafe { pac::Peripherals::steal() };
    let rcc = &dp.RCC;
    rcc.ahb2enr
        .modify(|_, w| w.gpioaen().set_bit().gpioben().set_bit().gpiocen().set_bit());

    let gpioa = &dp.GPIOA;
    let gpiob = &dp.GPIOB;
    let gpioc = &dp.GPIOC;

    // PA0/PA1/PA4 analog; PA7..PA10 AF (TIM1). (PA5/LD2 and the LEDs/button are
    // owned by embassy now — see main.rs — so they're not configured here.)
    gpioa.moder.modify(|_, w| {
        w.moder0().bits(0b11)
            .moder1().bits(0b11)
            .moder4().bits(0b11)
            .moder7().bits(0b10)
            .moder8().bits(0b10)
            .moder9().bits(0b10)
            .moder10().bits(0b10)
    });
    gpioa.ospeedr.modify(|_, w| {
        w.ospeedr7().bits(0b11).ospeedr8().bits(0b11).ospeedr9().bits(0b11).ospeedr10().bits(0b11)
    });
    gpioa.afrl.modify(|_, w| w.afrl7().bits(6));
    gpioa.afrh.modify(|_, w| w.afrh8().bits(6).afrh9().bits(6).afrh10().bits(6));

    // PB0/PB1 AF6 (CH2N/CH3N). PB2 = status LED is an embassy Output.
    gpiob.moder.modify(|_, w| w.moder0().bits(0b10).moder1().bits(0b10));
    gpiob.ospeedr.modify(|_, w| w.ospeedr0().bits(0b11).ospeedr1().bits(0b11));
    gpiob.afrl.modify(|_, w| w.afrl0().bits(6).afrl1().bits(6));

    // PC0/PC1 analog; PC10 ISR timing output. PC13 = button is an embassy ExtiInput.
    gpioc.moder.modify(|_, w| {
        w.moder0().bits(0b11).moder1().bits(0b11).moder10().bits(0b01).moder13().bits(0b00)
    });
    gpioc.ospeedr.modify(|_, w| w.ospeedr10().bits(0b11));
}

pub fn setup_tim1() {
    let dp = unsafe { pac::Peripherals::steal() };
    let rcc = &dp.RCC;
    let tim = &dp.TIM1;

    rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());

    tim.psc.write(|w| unsafe { w.bits(0) });
    tim.arr.write(|w| unsafe { w.bits(TIM_ARR as u32) });
    // Update event on valleys only — reloads the CCR/ARR preloads at the start
    // of each period. NOT the ADC trigger (that's OC4REF at the peak, below).
    tim.rcr.write(|w| unsafe { w.bits(1) });

    tim.ccmr1_output().modify(|_, w| {
        w.oc1m().bits(0b110).oc1pe().set_bit().oc2m().bits(0b110).oc2pe().set_bit()
    });
    // CH3 = phase-W PWM. CH4 is an internal-only compare (no pin) that triggers
    // the ADC at the right instant — see the CCR4 / MMS setup below.
    tim.ccmr2_output().modify(|_, w| {
        w.oc3m().bits(0b110).oc3pe().set_bit().oc4m().bits(0b110).oc4pe().set_bit()
    });

    // CHx = high-side (HIN, active-high); CHxN = low-side (LIN). The L6398's LIN
    // is ACTIVE-LOW and wants the SAME logic level as HIN, not the raw complement,
    // so invert the complementary outputs (CCxNP=1). Without this the low side is
    // driven backwards (shoot-through command during the high-side-on window).
    tim.ccer.modify(|_, w| {
        w.cc1e().set_bit().cc1ne().set_bit().cc1np().set_bit()
            .cc2e().set_bit().cc2ne().set_bit().cc2np().set_bit()
            .cc3e().set_bit().cc3ne().set_bit().cc3np().set_bit()
    });

    // TRGO = OC4REF. With CH4 in PWM mode 1 and CCR4 just below ARR, OC4REF has
    // exactly one rising edge per period — on the downslope just past the peak,
    // inside the window where all three low-side FETs conduct. That is the only
    // instant the low-side shunts carry the true phase currents.
    tim.cr2.modify(|_, w| unsafe { w.mms().bits(0b111) }); // TRGO = OC4REF

    tim.bdtr.modify(|_, w| unsafe {
        w.dtg().bits(DEAD_TIME_TICKS).ossr().set_bit().ossi().set_bit().moe().clear_bit()
    });

    set_duties(0.5, 0.5, 0.5);
    // ADC trigger point, kept below the max phase duty (0.95*ARR) so the OC4REF
    // edge always lands while every low-side FET is on.
    tim.ccr4().write(|w| unsafe { w.bits((TIM_ARR - 100) as u32) });
    tim.cr1.modify(|_, w| unsafe { w.cms().bits(0b01).arpe().set_bit() });

    tim.egr.write(|w| w.ug().set_bit());
    tim.cr1.modify(|_, w| w.cen().set_bit());
}

pub fn setup_adc() {
    let dp = unsafe { pac::Peripherals::steal() };
    let rcc = &dp.RCC;

    rcc.ahb2enr.modify(|_, w| w.adc12en().set_bit());
    rcc.ccipr.modify(|_, w| unsafe { w.adc12sel().bits(0b10) }); // SYSCLK
    dp.ADC12_COMMON.ccr.modify(|_, w| unsafe { w.presc().bits(0b0010) }); // /4 = 42.5 MHz

    // --- ADC1: injected ch1 (PA0, curr A), regular ch2 (PA1, VBUS) ---
    let adc1 = &dp.ADC1;
    adc1.cr.modify(|_, w| w.deeppwd().clear_bit());
    adc1.cr.modify(|_, w| w.advregen().set_bit());
    cortex_m::asm::delay(170 * 30);
    adc1.cr.modify(|_, w| w.adcal().set_bit());
    while adc1.cr.read().adcal().bit_is_set() {}
    cortex_m::asm::delay(170);
    adc1.isr.write(|w| w.adrdy().set_bit());
    adc1.cr.modify(|_, w| w.aden().set_bit());
    while adc1.isr.read().adrdy().bit_is_clear() {}

    // Injected: 2 conversions on TIM1_TRGO — phase A (IN1/PA0) then phase C
    // (IN6/PC0), so all three phase currents are sampled on one trigger
    // (JDR1 = A, JDR2 = C; phase B is on ADC2). PC0 = ADC12_IN6 on the G474.
    adc1.jsqr.write(|w| unsafe {
        w.jl().bits(1).jextsel().bits(0).jexten().bits(0b01).jsq1().bits(1).jsq2().bits(6)
    });
    adc1.smpr1.modify(|_, w| {
        w.smp1().bits(0b001).smp2().bits(0b110).smp6().bits(0b110)
    });
    adc1.sqr1.write(|w| unsafe { w.l().bits(0).sq1().bits(2) });

    // --- ADC2: injected ch7 (PC1, curr B), regular ch17 (PA4, pot) ---
    let adc2 = &dp.ADC2;
    adc2.cr.modify(|_, w| w.deeppwd().clear_bit());
    adc2.cr.modify(|_, w| w.advregen().set_bit());
    cortex_m::asm::delay(170 * 30);
    adc2.cr.modify(|_, w| w.adcal().set_bit());
    while adc2.cr.read().adcal().bit_is_set() {}
    cortex_m::asm::delay(170);
    adc2.isr.write(|w| w.adrdy().set_bit());
    adc2.cr.modify(|_, w| w.aden().set_bit());
    while adc2.isr.read().adrdy().bit_is_clear() {}

    adc2.jsqr.write(|w| unsafe {
        w.jl().bits(0).jextsel().bits(0).jexten().bits(0b01).jsq1().bits(7)
    });
    adc2.smpr1.modify(|_, w| w.smp7().bits(0b001));
    adc2.smpr2.modify(|_, w| w.smp17().bits(0b110));
    adc2.sqr1.write(|w| unsafe { w.l().bits(0).sq1().bits(17) });

    adc1.cr.modify(|_, w| w.jadstart().set_bit());
    adc2.cr.modify(|_, w| w.jadstart().set_bit());
}

/// TIM1 free-runs with MOE=0 -> ADC triggers fire, no gate drive, no current:
/// average the zero-current amplifier offsets. Returns (offset_a, offset_b) counts.
pub fn calibrate_current_offsets() -> (f32, f32, f32) {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut sum_a: u32 = 0;
    let mut sum_b: u32 = 0;
    let mut sum_c: u32 = 0;
    const N: u32 = 512;

    for _ in 0..N {
        while dp.ADC1.isr.read().jeos().bit_is_clear() {}
        dp.ADC1.isr.write(|w| w.jeos().set_bit());
        dp.ADC2.isr.write(|w| w.jeos().set_bit());
        sum_a += dp.ADC1.jdr1.read().bits() & 0xFFF;
        sum_b += dp.ADC2.jdr1.read().bits() & 0xFFF;
        sum_c += dp.ADC1.jdr2.read().bits() & 0xFFF; // phase C (2nd injected)
    }
    ((sum_a / N) as f32, (sum_b / N) as f32, (sum_c / N) as f32)
}

pub fn enable_adc_interrupt() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.ADC1.ier.modify(|_, w| w.jeosie().set_bit());

    // Interrupt priorities. The 25 kHz control ISR is hard real-time (40 us
    // budget); it must never wait on embassy's housekeeping. embassy enables its
    // TIM2 time-driver IRQ without setting a priority, so it sits at the reset
    // default (level 0 = highest) — equal to ADC1_2, meaning a TIM2 handler in
    // flight could delay control-ISR *entry*. Give ADC1_2 the top level and push
    // TIM2 one step down so ADC1_2 strictly preempts it. Lower number = higher
    // priority; the G4 implements 4 priority bits, so the level sits in the top
    // nibble (N << 4).
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    unsafe {
        cp.NVIC.set_priority(pac::Interrupt::ADC1_2, 0 << 4);
        cp.NVIC.set_priority(pac::Interrupt::TIM2, 1 << 4);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ADC1_2);
    }
}

// ---------------------------------------------------------------------------
// Hot path (control ISR)
// ---------------------------------------------------------------------------

#[inline(always)]
pub fn timing_pin_high() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.GPIOC.bsrr.write(|w| w.bs10().set_bit());
}

#[inline(always)]
pub fn timing_pin_low() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.GPIOC.bsrr.write(|w| w.br10().set_bit());
}

/// Clear the injected end-of-sequence flag and return the two raw 12-bit
/// phase-current samples (A on ADC1, B on ADC2).
#[inline(always)]
pub fn ack_and_read_currents() -> (u16, u16) {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.ADC1.isr.write(|w| w.jeos().set_bit());
    let raw_a = (dp.ADC1.jdr1.read().bits() & 0xFFF) as u16;
    let raw_b = (dp.ADC2.jdr1.read().bits() & 0xFFF) as u16;
    (raw_a, raw_b)
}

/// Directly-measured third phase current: ADC1's 2nd injected conversion, phase
/// C (PC0 / IN6). Sampled on the same OC4REF trigger as A and B, so it's
/// coherent with ack_and_read_currents(). Unlike the `-ia-ib` reconstruction,
/// this reflects whether phase C actually carries current.
#[inline(always)]
pub fn read_curr_c_raw() -> u16 {
    let dp = unsafe { pac::Peripherals::steal() };
    (dp.ADC1.jdr2.read().bits() & 0xFFF) as u16
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
    dp.TIM1.bdtr.modify(|_, w| w.moe().set_bit());
}

#[inline(always)]
pub fn disable_gates() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.TIM1.bdtr.modify(|_, w| w.moe().clear_bit());
}

// ---------------------------------------------------------------------------
// Slow path (1 kHz housekeeping): VBUS on ADC1 regular, pot on ADC2 regular
// ---------------------------------------------------------------------------

#[inline(always)]
pub fn try_read_vbus_raw() -> Option<u16> {
    let dp = unsafe { pac::Peripherals::steal() };
    if dp.ADC1.isr.read().eoc().bit_is_set() {
        Some((dp.ADC1.dr.read().bits() & 0xFFF) as u16)
    } else {
        None
    }
}

#[inline(always)]
pub fn try_read_pot_raw() -> Option<u16> {
    let dp = unsafe { pac::Peripherals::steal() };
    if dp.ADC2.isr.read().eoc().bit_is_set() {
        Some((dp.ADC2.dr.read().bits() & 0xFFF) as u16)
    } else {
        None
    }
}

#[inline(always)]
pub fn start_vbus_pot_conv() {
    let dp = unsafe { pac::Peripherals::steal() };
    dp.ADC1.cr.modify(|_, w| w.adstart().set_bit());
    dp.ADC2.cr.modify(|_, w| w.adstart().set_bit());
}
