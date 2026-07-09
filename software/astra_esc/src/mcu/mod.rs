//! Device abstraction layer.
//!
//! The control code in `main.rs` (state machine, FOC math, telemetry) is
//! MCU-independent. Everything that touches a peripheral register lives in a
//! per-device backend and is reached only through the small function/constant
//! surface re-exported here. Select the backend with a feature:
//!
//!   --features mcu-g4   NUCLEO-G474RE  (Cortex-M4F, 170 MHz)   [default]
//!   --features mcu-h5   NUCLEO-H533RE  (Cortex-M33, 250 MHz)
//!
//! Both backends expose the identical API (checked by the compiler because
//! `main.rs` compiles against whichever one is active):
//!
//!   Constants
//!     CYCLES_PER_US: f32    SYSCLK / 1e6, for DWT cycle count -> microseconds
//!     TIM_ARR: u16          TIM1 auto-reload for 25 kHz center-aligned PWM
//!
//!   One-time bring-up (call in this order from main). The clock tree itself is
//!   set up earlier by embassy_stm32::init() in main.rs, not here.
//!     log_clock_check()         defmt sanity dump of the clock config
//!     setup_gpio()              motor / sensor pins (LEDs & button are embassy)
//!     setup_tim1()              25 kHz center-aligned complementary PWM + ADC trigger
//!     setup_adc()               injected (phase currents) + regular (VBUS, pot)
//!     calibrate_current_offsets() -> (offset_a, offset_b, offset_c)  zero-current counts
//!     enable_adc_interrupt()    set priorities + unmask the injected-EOC IRQ
//!
//!   Hot path (the control ISR)
//!     timing_pin_high() / timing_pin_low()
//!     ack_and_read_currents() -> (raw_a, raw_b)   clears the injected flag
//!     read_curr_c_raw() -> u16                     ADC1's 2nd injected (phase C)
//!     set_duties(da, db, dc)                       per-phase duty in [0,1]
//!     enable_gates() / disable_gates()             MOE on/off
//!
//!   Slow path (1 kHz housekeeping)
//!     try_read_vbus_raw() -> Option<u16>
//!     try_read_pot_raw()  -> Option<u16>
//!     start_vbus_pot_conv()
//!
//! The LEDs and button are no longer part of this API — they are embassy HAL
//! types (Output / ExtiInput) owned directly by main.rs.

#[cfg(all(feature = "mcu-g4", feature = "mcu-h5"))]
compile_error!("Enable exactly one MCU backend: mcu-g4 OR mcu-h5, not both.");

#[cfg(not(any(feature = "mcu-g4", feature = "mcu-h5")))]
compile_error!("Enable one MCU backend: --features mcu-g4 (default) or --no-default-features --features mcu-h5.");

#[cfg(feature = "mcu-g4")]
mod g4;
#[cfg(feature = "mcu-g4")]
pub use g4::*;

#[cfg(feature = "mcu-h5")]
mod h5;
#[cfg(feature = "mcu-h5")]
pub use h5::*;