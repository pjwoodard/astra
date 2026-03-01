//! RP2350 hardware definitions.

use embassy_rp::gpio::{Level, Output};

/// GPIO output pin — implements [`embedded_hal::digital::OutputPin`].
pub type Led = Output<'static>;

/// RP2350 PWM slice — implements [`embedded_hal::pwm::SetDutyCycle`].
pub type PwmChannel = embassy_rp::pwm::Pwm<'static>;

/// Ready-to-use peripherals returned by [`init()`].
pub struct Peripherals {
    /// On-board LED (GPIO 25 on Pico 2).
    pub led: Led,
}

/// Initialize the RP2350 and return configured [`Peripherals`].
pub fn init() -> Peripherals {
    let p = embassy_rp::init(Default::default());

    let led = Output::new(p.PIN_25, Level::Low);

    Peripherals { led }
}
