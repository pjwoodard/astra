//! STM32H7 hardware definitions.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};

/// GPIO output pin — implements [`embedded_hal::digital::OutputPin`].
pub type Led = Output<'static>;

/// STM32 PWM channel — implements [`embedded_hal::pwm::SetDutyCycle`].
pub type PwmChannel = embassy_stm32::timer::simple_pwm::SimplePwmChannel<
    'static,
    embassy_stm32::peripherals::TIM2,
>;

/// Ready-to-use peripherals returned by [`init()`].
pub struct Peripherals {
    /// LD1 (green) on NUCLEO-H753ZI (PB0).
    pub led: Led,
    /// TIM2 CH1 PWM on PA0.
    pub motor: PwmChannel,
}

/// Initialize the STM32H7 and return configured [`Peripherals`].
pub fn init() -> Peripherals {
    let p = embassy_stm32::init(Default::default());

    // LD1 (green) on NUCLEO-H753ZI is PB0
    let led = Output::new(p.PB0, Level::Low, Speed::Low);

    // TIM2 CH1 PWM on PA0
    let ch1_pin = PwmPin::new(p.PA0, embassy_stm32::gpio::OutputType::PushPull);
    let pwm = SimplePwm::new(
        p.TIM2,
        Some(ch1_pin),
        None,
        None,
        None,
        khz(10),
        CountingMode::EdgeAlignedUp,
    );
    let channels = pwm.split();

    Peripherals {
        led,
        motor: channels.ch1,
    }
}
