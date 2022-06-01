mod gd32vf103;
mod stm32f411ccu6;
pub use gd32vf103::{bldc, led, mpu6050, serial, servo};

pub fn init() {
    unsafe { gd32vf103::init() }
}
