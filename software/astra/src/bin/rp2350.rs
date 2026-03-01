#![no_std]
#![no_main]

use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};
use astra::heartbeat;

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Astra"),
    embassy_rp::binary_info::rp_program_description!(c"Astra RP2350 firmware"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let hw = astra::hw::init();

    spawner.spawn(heartbeat(hw.led).unwrap());

    loop {
        embassy_time::Timer::after_secs(1).await;
    }
}
