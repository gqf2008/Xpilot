use hal::gpio::gpioa::{PA1, PA2};
use hal::gpio::gpioc::PC13;
use hal::gpio::Active;
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::led::{Led, BLUE, GREEN, RED};

static mut LED_RED: Option<RED> = None;
static mut LED_GREEN: Option<GREEN> = None;
static mut LED_BLUE: Option<BLUE> = None;

pub(crate) unsafe fn init<T>(red: Option<PC13<T>>, green: Option<PA1<T>>, blue: Option<PA2<T>>)
where
    T: Active,
{
    if let Some(port) = red {
        let mut red = RED::new(port);
        red.off();
        LED_RED.replace(red);
    }
    if let Some(port) = green {
        let mut green = GREEN::new(port);
        green.off();
        LED_GREEN.replace(green);
    }
    if let Some(port) = blue {
        let mut blue = BLUE::new(port);
        blue.off();
        LED_BLUE.replace(blue);
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

// unsafe fn start_interrupt(count_hz: u32) {
//     if let Some(timer) = TIMER.as_mut() {
//         timer.start(count_hz.hz());
//         timer.listen(Event::Update);

//         ECLIC::setup(
//             Interrupt::TIMER0_UP,
//             TriggerType::Level,
//             Level::L3,
//             Priority::P8,
//         );
//         ECLIC::unmask(Interrupt::TIMER0_UP);
//     }
// }
// unsafe fn clear_update_interrupt_flag() {
//     if let Some(timer) = TIMER.as_mut() {
//         timer.clear_update_interrupt_flag();
//     }
// }

// #[export_name = "TIMER0_UP"]
// unsafe fn on_timer0() {
//     clear_update_interrupt_flag();
//     if let Some(red) = red() {
//         red.toggle();
//     }
// }
