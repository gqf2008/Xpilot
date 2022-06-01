//! 舵机驱动

use embedded_hal::Pwm;
use xtask::bsp::longan_nano::hal::gpio::gpiob::{PB6, PB7, PB8, PB9};
use xtask::bsp::longan_nano::hal::pac::TIMER3;
use xtask::bsp::longan_nano::hal::pwm::Channel;
use xtask::bsp::longan_nano::hal::time::*;
use xtask::bsp::longan_nano::hal::{
    afio::Afio,
    gpio::Active,
    pwm::{NoRemap, PwmTimer},
    rcu::Rcu,
};

static mut SERVO: Option<Servo> = None;

pub struct Servo {
    pwm: PwmTimer<TIMER3, NoRemap>,
    max_duty: u16,
}

pub fn init<T>(
    timer: TIMER3,
    pins: (
        Option<PB6<T>>,
        Option<PB7<T>>,
        Option<PB8<T>>,
        Option<PB9<T>>,
    ),
    afio: &mut Afio,
    rcu: &mut Rcu,
) where
    T: Active,
{
    let pb6 = if let Some(pb6) = pins.0 {
        Some(pb6.into_alternate_push_pull())
    } else {
        None
    };
    let pb7 = if let Some(pb7) = pins.1 {
        Some(pb7.into_alternate_push_pull())
    } else {
        None
    };
    let pb8 = if let Some(pb8) = pins.2 {
        Some(pb8.into_alternate_push_pull())
    } else {
        None
    };
    let pb9 = if let Some(pb9) = pins.3 {
        Some(pb9.into_alternate_push_pull())
    } else {
        None
    };
    let mut pwm = PwmTimer::<TIMER3, NoRemap>::timer3(
        timer,
        (pb6.as_ref(), pb7.as_ref(), pb8.as_ref(), pb9.as_ref()),
        rcu,
        afio,
    );
    pwm.set_period(1000.hz());
    if pb6.is_some() {
        pwm.enable(Channel::CH0);
    }
    if pb7.is_some() {
        pwm.enable(Channel::CH1);
    }
    if pb8.is_some() {
        pwm.enable(Channel::CH2);
    }
    if pb9.is_some() {
        pwm.enable(Channel::CH3);
    }

    let max_duty = pwm.get_max_duty();
    unsafe { SERVO.replace(Servo { pwm, max_duty }) };
}

pub fn servo() -> &'static mut Servo {
    unsafe { SERVO.as_mut().unwrap() }
}

impl Servo {
    /// 设置占空比
    pub fn duty(&mut self, ch: Channel, duty: f32) {
        self.pwm.set_duty(ch, (self.max_duty as f32 * duty) as u16);
    }
}
