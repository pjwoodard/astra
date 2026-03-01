use embedded_hal_1::pwm::SetDutyCycle;

pub async fn motor_task_logic<P: SetDutyCycle>(pwm: &mut P)
{
    let max = pwm.max_duty_cycle();

    let _ = pwm.set_duty_cycle(max / 2);
}