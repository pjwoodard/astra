#![no_std]
#![no_main]

use astra::heartbeat;
use astra::motor_pwm;
use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let hw = astra::hw::init();

    spawner.spawn(motor_pwm(hw.motor).unwrap());
    spawner.spawn(heartbeat(hw.led).unwrap());

    loop {
        embassy_time::Timer::after_secs(1).await;
    }
}
