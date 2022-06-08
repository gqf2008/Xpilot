use crate::mbus;
use crate::message::*;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use xtask::bsp::greenpill::hal::gpio::Output;
use xtask::bsp::greenpill::hal::gpio::Pin;
use xtask::bsp::greenpill::led::Led;

static mut LED: Option<Led> = None;

static mut RED: Option<Led427<Pin<'C', 6, Output>>> = None;
static mut GREEN: Option<Led427<Pin<'C', 7, Output>>> = None;
static mut BLUE: Option<Led427<Pin<'A', 8, Output>>> = None;

pub unsafe fn init_401(pin: Pin<'C', 13>) {
    log::info!("init led");
    let led = Led::new(pin);
    LED.replace(led);
    log::info!("init led ok");
    mbus::mbus().register_serivce("/led/blue", |_, msg| {});
}

pub unsafe fn init_427(red: Pin<'C', 6>, green: Pin<'C', 7>, blue: Pin<'A', 8>) {
    log::info!("init led");
    let led = Led427::new(red.into_push_pull_output());
    RED.replace(led);
    let led = Led427::new(green.into_push_pull_output());
    GREEN.replace(led);
    let led = Led427::new(blue.into_push_pull_output());
    BLUE.replace(led);
    mbus::mbus().register_serivce("/led/red", |_, msg| match msg {
        Message::Control(Signal::Led(LedSignal::On)) => {
            if let Some(red) = RED.as_mut() {
                red.on();
            }
        }
        Message::Control(Signal::Led(LedSignal::Off)) => {
            if let Some(red) = RED.as_mut() {
                red.off();
            }
        }
        Message::Control(Signal::Led(LedSignal::Toggle)) => {
            if let Some(red) = RED.as_mut() {
                red.toggle();
            }
        }
        _ => {}
    });
    mbus::mbus().register_serivce("/led/green", |_, msg| match msg {
        Message::Control(Signal::Led(LedSignal::On)) => {
            if let Some(red) = GREEN.as_mut() {
                red.on();
            }
        }
        Message::Control(Signal::Led(LedSignal::Off)) => {
            if let Some(red) = GREEN.as_mut() {
                red.off();
            }
        }
        Message::Control(Signal::Led(LedSignal::Toggle)) => {
            if let Some(red) = GREEN.as_mut() {
                red.toggle();
            }
        }
        _ => {}
    });
    mbus::mbus().register_serivce("/led/blue", |_, msg| match msg {
        Message::Control(Signal::Led(LedSignal::On)) => {
            if let Some(red) = BLUE.as_mut() {
                red.on();
            }
        }
        Message::Control(Signal::Led(LedSignal::Off)) => {
            if let Some(red) = BLUE.as_mut() {
                red.off();
            }
        }
        Message::Control(Signal::Led(LedSignal::Toggle)) => {
            if let Some(red) = BLUE.as_mut() {
                red.toggle();
            }
        }
        _ => {}
    });
    log::info!("init led ok");
}

pub struct Led427<T> {
    port: T,
}

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
