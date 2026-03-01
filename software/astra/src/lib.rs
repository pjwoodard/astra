#![no_std]

use crate::heartbeat_task_logic::heartbeat_task_logic;

#[cfg(feature = "rp2350")]
use crate::motor_task_logic::motor_task_logic;

pub mod heartbeat_task_logic;
pub mod motor_task_logic;

#[cfg(feature = "rp2350")]
#[embassy_executor::task]
pub async fn heartbeat_rp(led: embassy_rp::gpio::Output<'static>)
{
    heartbeat_task_logic(led).await;
}

#[cfg(feature = "rp2350")]
#[embassy_executor::task]
pub async fn pwm_rp(mut pwm: embassy_rp::pwm::Pwm<'static>)
{
    motor_task_logic(&mut pwm).await;
}

#[cfg(feature = "stm32h7")]
#[embassy_executor::task]
pub async fn heartbeat_stm(led: embassy_stm32::gpio::Output<'static>)
{
    heartbeat_task_logic(led).await;
}