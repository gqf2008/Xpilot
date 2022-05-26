use hal::gpio::gpioa::{PA1, PA2};
use hal::gpio::gpioc::PC13;
use hal::gpio::Active;
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::led::{rgb, BLUE, GREEN, RED};

static mut LED: Option<(RED, GREEN, BLUE)> = None;

pub(crate) unsafe fn init<X, Y, Z>(red: PC13<X>, green: PA1<Y>, blue: PA2<Z>)
where
    X: Active,
    Y: Active,
    Z: Active,
{
    let (red, green, blue) = rgb(red, green, blue);
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
