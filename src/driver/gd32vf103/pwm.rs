use xtask::bsp::longan_nano::hal::{
    afio::Afio,
    gpio::{
        gpioa::{PA1, PA2},
        Active,
    },
    pac::TIMER1,
    pwm::{NoRemap, PwmTimer},
    rcu::Rcu,
};

pub static mut PWM1: Option<PwmTimer<TIMER1, NoRemap>> = None;
pub fn init<X, Y>(timer: TIMER1, pins: (PA1<X>, PA2<Y>), afio: &mut Afio, rcu: &mut Rcu)
where
    X: Active,
    Y: Active,
{
    let pa1 = pins.0.into_alternate_push_pull();
    let pa2 = pins.1.into_alternate_push_pull();
    let pwm =
        PwmTimer::<TIMER1, NoRemap>::timer1(timer, (None, Some(&pa1), Some(&pa2), None), rcu, afio);
    unsafe { PWM1.replace(pwm) };
}

pub(crate) fn pwm1() -> &'static mut PwmTimer<TIMER1, NoRemap> {
    unsafe { PWM1.as_mut().unwrap() }
}
