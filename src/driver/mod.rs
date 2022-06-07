#[cfg(feature = "gd32vf103")]
mod gd32vf103;
#[cfg(feature = "stm32f4")]
mod stm32f401ccu6;
#[cfg(feature = "gd32vf103")]
pub use gd32vf103::{bldc, led, mpu6050, serial, servo};
#[cfg(feature = "stm32f4")]
pub use stm32f401ccu6::led;

pub mod bldc;
pub mod mpu6050;
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

#[derive(Copy, Clone, Debug, Default)]
pub struct Quaternion {
    w: f32,
    x: f32,
    y: f32,
    z: f32,
}

impl Quaternion {
    pub fn to_euler(self) -> EulerAngle {
        use libm::*;
        let q0 = self.w;
        let q1 = self.x;
        let q2 = self.y;
        let q3 = self.z;
        let yaw = atan2f(
            2.0 * (q1 * q2 + q0 * q3),
            q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
        ) * 57.3;

        let pitch = asinf(-2.0 * q1 * q3 + 2.0 * q0 * q2) * 57.3;
        let roll = atan2f(
            2.0 * q2 * q3 + 2.0 * q0 * q1,
            -2.0 * q1 * q1 - 2.0 * q2 * q2 + 1.0,
        ) * 57.3;
        EulerAngle { yaw, pitch, roll }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct EulerAngle {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

/// 重力加速度，单位g
#[derive(Copy, Clone, Debug, Default)]
pub struct Accel {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// 角速度，单位rad/s
#[derive(Copy, Clone, Debug, Default)]
pub struct Gyro {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// 罗盘
#[derive(Copy, Clone, Debug, Default)]
pub struct Compass {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

//气压计
#[derive(Copy, Clone, Debug, Default)]
pub struct Barometer {
    pub h: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Distance(f32);

#[derive(Copy, Clone, Debug, Default)]
pub struct Gps {
    pub longitude: f32,
    pub latitude: f32,
}
