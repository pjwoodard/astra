use embedded_hal_1::digital::OutputPin;
use core::option_env;
use core::module_path;
use defmt::info;

pub async fn heartbeat_task_logic<P: OutputPin>(mut led: P)
{
// 1Hz frequency = 1 full cycle (ON + OFF) per second
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(1));

    loop {
        // --- LED ON ---
        info!("led on!");
        led.set_high().unwrap();
        
        // Keep it on for 500ms
        embassy_time::Timer::after_millis(500).await;

        // --- LED OFF ---
        info!("led off!");
        led.set_low().unwrap();

        // Wait for the remainder of the 1-second period.
        // This effectively creates the 500ms "OFF" time.
        ticker.next().await;
    }
}