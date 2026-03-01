#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use {defmt_rtt as _, panic_probe as _};
use astra::heartbeat_stm;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // LD1 (green) on NUCLEO-H753ZI is PB0
    let led = Output::new(p.PB0, Level::Low, Speed::Low);

    spawner.spawn(heartbeat_stm(led).unwrap());

    loop {
        embassy_time::Timer::after_secs(1).await;
    }
}
