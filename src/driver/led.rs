use super::PERIPHERAL;
use hal::prelude::*;
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::led::{rgb, BLUE, GREEN, RED};

pub static mut LED: Option<(RED, GREEN, BLUE)> = None;

pub(crate) unsafe fn init() {
    if let Some(dp) = &mut PERIPHERAL {
        let rcu = super::rcu::rcu();
        let gpioa = dp.GPIOA.split(rcu);
        let gpioc = dp.GPIOC.split(rcu);
        let (red, green, blue) = rgb(gpioc.pc13, gpioa.pa1, gpioa.pa2);
        LED.replace((red, green, blue));
    }
}

pub fn led() -> (&'static mut RED, &'static mut GREEN, &'static mut BLUE) {
    unsafe {
        let (r, g, b) = &mut LED.unwrap();
        (r, g, b)
    }
}

pub fn red() -> &'static mut RED {
    unsafe {
        let (red, _, _) = &mut LED.unwrap();
        red
    }
}

pub fn green() -> &'static mut GREEN {
    unsafe {
        let (_, green, _) = &mut LED.unwrap();
        green
    }
}

pub fn blue() -> &'static mut BLUE {
    unsafe {
        let (_, _, blue) = &mut LED.unwrap();
        blue
    }
}
