//! 舵机驱动

use crate::mbus;
use crate::message::*;
use embedded_hal::PwmPin;

pub struct Servo<PWM> {
    pwm: PWM,
}

impl<PWM: PwmPin<Duty = u16>> Servo<PWM> {
    pub fn new(mut pwm: PWM) -> Self {
        pwm.enable();
        Self { pwm }
    }
}

impl<PWM: PwmPin<Duty = u16>> Servo<PWM> {
    /// 最小占空比
    pub fn lowest_duty(&mut self) {
        self.pwm.set_duty(0);
    }

    /// 半占空比
    pub fn half_duty(&mut self) {
        self.pwm.set_duty(self.pwm.get_max_duty() / 2);
    }

    /// 全占空比
    pub fn full_duty(&mut self) {
        self.pwm.set_duty(self.pwm.get_max_duty());
    }

    /// 给油，duty范围0.1-0.5
    pub fn set_duty(&mut self, duty: f32) {
        self.pwm
            .set_duty((self.pwm.get_max_duty() as f32 * duty) as u16);
        mbus::bus().call("/led/g/toggle", Message::None);
    }
}
