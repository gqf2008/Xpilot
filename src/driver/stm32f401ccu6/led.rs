use xtask::bsp::greenpill::hal::gpio::Pin;
use xtask::bsp::greenpill::led::Led;

use crate::mbus;
static mut LED: Option<Led> = None;

pub unsafe fn init(pin: Pin<'C', 13>) {
    log::info!("init led");
    let led = Led::new(pin);
    LED.replace(led);
    log::info!("init led ok");
    mbus::mbus().register_serivce("/led/blue/on", |_, msg| {});
}

pub fn blue() -> Option<&'static mut Led> {
    unsafe { LED.as_mut() }
}
