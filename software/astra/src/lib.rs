#![no_std]

use crate::heartbeat_task_logic::heartbeat_task_logic;
use crate::motor_task_logic::motor_task_logic;

pub mod heartbeat_task_logic;
pub mod hw;
pub mod motor_task_logic;

// Tasks are written once — the type aliases in hw resolve per-platform.
#[embassy_executor::task]
pub async fn heartbeat(led: hw::Led) {
    heartbeat_task_logic(led).await;
}

#[embassy_executor::task]
pub async fn motor_pwm(mut pwm: hw::PwmChannel) {
    motor_task_logic(&mut pwm).await;
}