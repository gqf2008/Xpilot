mod gd32vf103;
mod stm32f411ccu6;
pub use gd32vf103::{led, mpu6050, pwm};

pub fn init() {
    unsafe { gd32vf103::init() }
}
