//! Platform hardware abstraction.
//!
//! Each platform module defines:
//! - Type aliases for peripherals (all satisfying embedded-hal traits)
//! - A [`Peripherals`] struct bundling initialized hardware
//! - An [`init()`] function that configures clocks/pins and returns [`Peripherals`]

#[cfg(not(any(feature = "rp2350", feature = "stm32h7")))]
compile_error!("Select a platform feature: rp2350 or stm32h7");

#[cfg(feature = "rp2350")]
mod rp2350;
#[cfg(feature = "rp2350")]
pub use rp2350::*;

#[cfg(feature = "stm32h7")]
mod stm32h7;
#[cfg(feature = "stm32h7")]
pub use stm32h7::*;
