mod gd32vf103;
mod stm32f411ccu6;
pub use gd32vf103::{led, mpu6050, mpu6050_dmp};

pub fn init() {
    unsafe { gd32vf103::init() }
}
