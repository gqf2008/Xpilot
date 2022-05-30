use embedded_hal::timer::CountDown;
use hal::gpio::gpioa::{PA1, PA2};
use hal::gpio::gpioc::PC13;
use hal::gpio::Active;
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::hal::eclic::{Level, Priority, TriggerType, *};
use xtask::bsp::longan_nano::hal::pac::TIMER0;
use xtask::bsp::longan_nano::hal::pac::{Interrupt, ECLIC};
use xtask::bsp::longan_nano::hal::rcu::Rcu;
use xtask::bsp::longan_nano::hal::time::*;
use xtask::bsp::longan_nano::hal::timer::{Event, Timer};
use xtask::bsp::longan_nano::led::Led;
use xtask::bsp::longan_nano::led::{BLUE, GREEN, RED};

static mut LED_RED: Option<RED> = None;
static mut LED_GREEN: Option<GREEN> = None;
static mut LED_BLUE: Option<BLUE> = None;
static mut TIMER: Option<Timer<TIMER0>> = None;

pub(crate) unsafe fn init<T>(
    timer: TIMER0,
    red: Option<PC13<T>>,
    green: Option<PA1<T>>,
    blue: Option<PA2<T>>,
    rcu: &mut Rcu,
) where
    T: Active,
{
    if let Some(port) = red {
        LED_RED.replace(RED::new(port));
    }
    if let Some(port) = green {
        LED_GREEN.replace(GREEN::new(port));
    }
    if let Some(port) = blue {
        LED_BLUE.replace(BLUE::new(port));
    }
    let timer = Timer::timer0(timer, 1.hz(), rcu);
    TIMER.replace(timer);
    start_interrupt(2);
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

unsafe fn start_interrupt(count_hz: u32) {
    if let Some(timer) = TIMER.as_mut() {
        timer.start(count_hz.hz());
        timer.listen(Event::Update);

        ECLIC::setup(
            Interrupt::TIMER0_UP,
            TriggerType::Level,
            Level::L3,
            Priority::P8,
        );
        ECLIC::unmask(Interrupt::TIMER0_UP);
    }
}
unsafe fn clear_update_interrupt_flag() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_update_interrupt_flag();
    }
}

#[export_name = "TIMER0_UP"]
unsafe fn on_timer0() {
    clear_update_interrupt_flag();
    if let Some(red) = red() {
        red.toggle();
    }
}
