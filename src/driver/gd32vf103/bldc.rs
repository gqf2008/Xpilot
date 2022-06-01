//! 无刷电机驱动

use embedded_hal::Pwm;
use xtask::bsp::longan_nano::hal::gpio::gpioa::{PA6, PA7};
use xtask::bsp::longan_nano::hal::gpio::gpiob::{PB0, PB1};
use xtask::bsp::longan_nano::hal::pac::TIMER2;
use xtask::bsp::longan_nano::hal::pwm::Channel;
use xtask::bsp::longan_nano::hal::time::*;
use xtask::bsp::longan_nano::hal::{
    afio::Afio,
    gpio::Active,
    pwm::{NoRemap, PwmTimer},
    rcu::Rcu,
};
use xtask::bsp::longan_nano::led::Led;

static mut MOTOR: Option<Motor> = None;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum State {
    Locked,
    Unlocked,
}
pub struct Motor {
    pwm: PwmTimer<TIMER2, NoRemap>,
    max_duty: u16,
    state: State,
}

pub fn init<T>(
    timer: TIMER2,
    pins: (
        Option<PA6<T>>,
        Option<PA7<T>>,
        Option<PB0<T>>,
        Option<PB1<T>>,
    ),
    afio: &mut Afio,
    rcu: &mut Rcu,
) where
    T: Active,
{
    let pa6 = if let Some(pa6) = pins.0 {
        Some(pa6.into_alternate_push_pull())
    } else {
        None
    };
    let pa7 = if let Some(pa7) = pins.1 {
        Some(pa7.into_alternate_push_pull())
    } else {
        None
    };
    let pb0 = if let Some(pb0) = pins.2 {
        Some(pb0.into_alternate_push_pull())
    } else {
        None
    };
    let pb1 = if let Some(pb1) = pins.3 {
        Some(pb1.into_alternate_push_pull())
    } else {
        None
    };
    let mut pwm = PwmTimer::<TIMER2, NoRemap>::timer2(
        timer,
        (pa6.as_ref(), pa7.as_ref(), pb0.as_ref(), pb1.as_ref()),
        rcu,
        afio,
    );
    pwm.set_period(1000.hz());
    if pa6.is_some() {
        pwm.enable(Channel::CH0);
    }
    if pa7.is_some() {
        pwm.enable(Channel::CH1);
    }
    if pb0.is_some() {
        pwm.enable(Channel::CH2);
    }
    if pb1.is_some() {
        pwm.enable(Channel::CH3);
    }
    if let Some(red) = super::led::red() {
        red.on();
    }
    let max_duty = pwm.get_max_duty();
    unsafe {
        MOTOR.replace(Motor {
            pwm,
            max_duty,
            state: State::Locked,
        })
    };
}

pub(crate) fn motor() -> &'static mut Motor {
    unsafe { MOTOR.as_mut().unwrap() }
}

impl Motor {
    /// 锁定马达，绿关，红开
    pub fn lock(&mut self) {
        self.state = State::Locked;
        if let Some(green) = super::led::green() {
            green.off();
        }
        if let Some(red) = super::led::red() {
            red.on();
        }
    }
    /// 解锁马达，红闪3下，绿开
    pub fn unlock(&mut self) {
        self.state = State::Unlocked;
        if let Some(red) = super::led::red() {
            for _ in 0..6 {
                red.toggle();
                xtask::delay_us(1000 * 500);
            }
        }
        if let Some(green) = super::led::green() {
            green.on();
        }
    }

    /// 最小油门
    pub fn lowest_throttle(&mut self, ch: Channel) {
        if self.state == State::Unlocked {
            self.pwm.set_duty(ch, self.max_duty / 2);
        }
    }

    /// 半油门
    pub fn half_throttle(&mut self, ch: Channel) {
        if self.state == State::Unlocked {
            self.throttle(ch, 0.75);
        }
    }

    /// 全油门
    pub fn full_throttle(&mut self, ch: Channel) {
        if self.state == State::Unlocked {
            self.pwm.set_duty(ch, self.max_duty);
        }
    }

    /// 给油，duty范围0.1-0.5
    pub fn throttle(&mut self, ch: Channel, duty: f32) {
        if self.state == State::Unlocked {
            self.pwm
                .set_duty(ch, (self.max_duty as f32 * (duty + 0.5)) as u16);
            if let Some(green) = super::led::green() {
                green.toggle();
            }
        }
    }
}
