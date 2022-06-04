#[cfg(feature = "gd32vf103")]
mod gd32vf103;
#[cfg(feature = "stm32f4")]
mod stm32f401ccu6;
#[cfg(feature = "gd32vf103")]
pub use gd32vf103::{bldc, led, mpu6050, serial, servo};
#[cfg(feature = "stm32f4")]
pub use stm32f401ccu6::led;

pub mod bldc;
pub mod servo;

pub fn init() {
    #[cfg(feature = "gd32vf103")]
    unsafe {
        gd32vf103::init()
    }
    #[cfg(feature = "stm32f4")]
    unsafe {
        stm32f401ccu6::init()
    }
}
