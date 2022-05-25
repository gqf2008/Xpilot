use hal::pac::Peripherals;
use hal::prelude::*;
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::led::{rgb, BLUE, GREEN, RED};

static mut LED: Option<(RED, GREEN, BLUE)> = None;

pub(crate) unsafe fn init() {
    let dp = Peripherals::steal();
    let rcu = super::rcu::rcu();
    let gpioa = dp.GPIOA.split(rcu);
    let gpioc = dp.GPIOC.split(rcu);

    let (red, green, blue) = rgb(gpioc.pc13, gpioa.pa1, gpioa.pa2);
    LED.replace((red, green, blue));
}

pub fn led() -> (&'static mut RED, &'static mut GREEN, &'static mut BLUE) {
    unsafe {
        let (r, g, b) = LED.as_mut().unwrap();
        (r, g, b)
    }
}

pub fn red() -> &'static mut RED {
    unsafe {
        let (red, _, _) = LED.as_mut().unwrap();
        red
    }
}

pub fn green() -> &'static mut GREEN {
    unsafe {
        let (_, green, _) = LED.as_mut().unwrap();
        green
    }
}

pub fn blue() -> &'static mut BLUE {
    unsafe {
        let (_, _, blue) = LED.as_mut().unwrap();
        blue
    }
}
