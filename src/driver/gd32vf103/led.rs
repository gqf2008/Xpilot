use hal::gpio::gpioa::{PA1, PA2};
use hal::gpio::gpioc::PC13;
use hal::gpio::Active;
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::led::{rgb, BLUE, GREEN, RED};

static mut LED_RED: Option<RED> = None;
static mut LED_GREEN: Option<GREEN> = None;
static mut LED_BLUE: Option<BLUE> = None;

pub(crate) unsafe fn init<T>(red: Option<PC13<T>>, green: Option<PA1<T>>, blue: Option<PA2<T>>)
where
    T: Active,
{
    // let (red, green, blue) = rgb(red, green, blue);
    if let Some(port) = red {
        LED_RED.replace(RED::new(port));
    }
    if let Some(port) = green {
        LED_GREEN.replace(GREEN::new(port));
    }
    if let Some(port) = blue {
        LED_BLUE.replace(BLUE::new(port));
    }
}

pub fn red() -> Option<&'static mut RED> {
    unsafe { LED_RED.as_mut() }
}

pub fn green() -> Option<&'static mut GREEN> {
    unsafe { LED_GREEN.as_mut() }
}

pub fn blue() -> Option<&'static mut BLUE> {
    unsafe { LED_BLUE.as_mut() }
}
