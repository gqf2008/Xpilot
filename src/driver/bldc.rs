//! 无刷电机驱动

use crate::mbus;
use crate::message::*;
use embedded_hal::PwmPin;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum State {
    Locked,
    Unlocked,
}
pub struct Motor<PWM> {
    pwm: PWM,
    state: State,
}

impl<PWM: PwmPin<Duty = u16>> Motor<PWM> {
    pub fn new(pwm: PWM) -> Self {
        Self {
            pwm,
            state: State::Locked,
        }
    }
}

impl<PWM: PwmPin<Duty = u16>> Motor<PWM> {
    /// 锁定马达，绿关，红开
    pub fn lock(&mut self) {
        self.state = State::Locked;
        self.pwm.disable();
        mbus::mbus().call("/led/green", Message::Control(Signal::Led(LedSignal::Off)));
        mbus::mbus().call("/led/red", Message::Control(Signal::Led(LedSignal::On)));
    }
    /// 解锁马达，红闪3下，绿开
    pub fn unlock(&mut self) {
        self.state = State::Unlocked;
        for _ in 0..6 {
            mbus::mbus().call("/led/red", Message::Control(Signal::Led(LedSignal::Toggle)));
            xtask::delay_us(1000 * 500);
        }
        mbus::mbus().call("/led/green", Message::Control(Signal::Led(LedSignal::On)));
        self.pwm.enable();
    }

    /// 最小油门
    pub fn lowest(&mut self) {
        if self.state == State::Unlocked {
            self.pwm.set_duty(self.pwm.get_max_duty() / 2);
        }
    }

    /// 半油门
    pub fn half(&mut self) {
        if self.state == State::Unlocked {
            self.throttle(0.75)
        }
    }

    /// 全油门
    pub fn full(&mut self) {
        if self.state == State::Unlocked {
            self.pwm.set_duty(self.pwm.get_max_duty());
        }
    }

    /// 给油，duty范围0.1-0.5
    pub fn throttle(&mut self, duty: f32) {
        if self.state == State::Unlocked {
            self.pwm
                .set_duty((self.pwm.get_max_duty() as f32 * (duty + 0.5)) as u16);
            mbus::mbus().call(
                "/led/blue",
                Message::Control(Signal::Led(LedSignal::Toggle)),
            );
        }
    }
}
