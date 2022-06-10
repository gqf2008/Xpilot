use crate::mbus;
#[cfg(feature = "stm32f427vit6")]
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
#[cfg(feature = "stm32f427vit6")]
use xtask::bsp::greenpill::hal::gpio::Output;
use xtask::bsp::greenpill::hal::gpio::Pin;

#[cfg(feature = "stm32f401ccu6")]
use xtask::bsp::greenpill::led::Led;

#[cfg(feature = "stm32f401ccu6")]
static mut LED: Option<Led> = None;

#[cfg(feature = "stm32f427vit6")]
static mut RED: Option<Led427<Pin<'C', 6, Output>>> = None;

#[cfg(feature = "stm32f427vit6")]
static mut GREEN: Option<Led427<Pin<'C', 7, Output>>> = None;

#[cfg(feature = "stm32f427vit6")]
static mut BLUE: Option<Led427<Pin<'A', 8, Output>>> = None;

#[cfg(feature = "stm32f401ccu6")]
pub unsafe fn init_401(pin: Pin<'C', 13>) {
    log::info!("init led");
    let led = Led::new(pin);
    LED.replace(led);
    log::info!("init led ok");
    mbus::mbus()
        .register("/led/r/on", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.on();
            }
        })
        .register("/led/r/off", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.off();
            }
        })
        .register("/led/r/toggle", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.toggle();
            }
        })
        .register("/led/g/on", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.on();
            }
        })
        .register("/led/g/off", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.off();
            }
        })
        .register("/led/g/toggle", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.toggle();
            }
        })
        .register("/led/b/on", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.on();
            }
        })
        .register("/led/b/off", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.off();
            }
        })
        .register("/led/b/toggle", |_, _| {
            if let Some(led) = LED.as_mut() {
                led.toggle();
            }
        });
}

#[cfg(feature = "stm32f427vit6")]
pub unsafe fn init_427(red: Pin<'C', 6>, green: Pin<'C', 7>, blue: Pin<'A', 8>) {
    log::info!("init led");
    let led = Led427::new(red.into_push_pull_output());
    RED.replace(led);
    let led = Led427::new(green.into_push_pull_output());
    GREEN.replace(led);
    let led = Led427::new(blue.into_push_pull_output());
    BLUE.replace(led);
    mbus::mbus()
        .register("/led/r/on", |_, _| {
            if let Some(led) = RED.as_mut() {
                led.on();
            }
        })
        .register("/led/r/off", |_, _| {
            if let Some(led) = RED.as_mut() {
                led.off();
            }
        })
        .register("/led/r/toggle", |_, _| {
            if let Some(led) = RED.as_mut() {
                led.toggle();
            }
        })
        .register("/led/g/on", |_, _| {
            if let Some(led) = GREEN.as_mut() {
                led.on();
            }
        })
        .register("/led/g/off", |_, _| {
            if let Some(led) = GREEN.as_mut() {
                led.off();
            }
        })
        .register("/led/g/toggle", |_, _| {
            if let Some(led) = GREEN.as_mut() {
                led.toggle();
            }
        })
        .register("/led/b/on", |_, _| {
            if let Some(led) = BLUE.as_mut() {
                led.on();
            }
        })
        .register("/led/b/off", |_, _| {
            if let Some(led) = BLUE.as_mut() {
                led.off();
            }
        })
        .register("/led/b/toggle", |_, _| {
            if let Some(led) = BLUE.as_mut() {
                led.toggle();
            }
        });
    log::info!("init led ok");
}

#[cfg(feature = "stm32f427vit6")]
pub struct Led427<T> {
    port: T,
}
#[cfg(feature = "stm32f427vit6")]
impl<T: OutputPin + ToggleableOutputPin> Led427<T> {
    pub fn new(port: T) -> Self {
        Self { port }
    }
    pub fn off(&mut self) {
        self.port.set_high().ok();
    }

    pub fn on(&mut self) {
        self.port.set_low().ok();
    }

    pub fn toggle(&mut self) {
        self.port.toggle().ok();
    }
}
