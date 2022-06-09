#[cfg(feature = "gd32vf103")]
mod gd32vf103;
#[cfg(feature = "gd32vf103")]
pub use gd32vf103::{bldc, led, mpu6050, serial, servo};

#[cfg(any(feature = "stm32f401ccu6", feature = "stm32f427vit6"))]
mod stm32f4;

pub mod bldc;
pub mod icm20602;
pub mod mpu6050;
pub mod servo;

use libm::acosf;

pub fn init() {
    log::info!("Initialize driver");
    #[cfg(feature = "gd32vf103")]
    unsafe {
        gd32vf103::init()
    }
    #[cfg(any(feature = "stm32f401ccu6", feature = "stm32f427vit6"))]
    unsafe {
        stm32f4::init()
    }
    log::info!("Initialize driver ok");
}

#[derive(Copy, Clone, Debug, Default)]
pub struct ImuData {
    pub accel: Option<Accel>,
    pub temp: Option<f32>,
    pub gyro: Option<Gyro>,
    pub compass: Option<Compass>,
    pub quaternion: Option<Quaternion>,
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
        let w = self.w;
        let x = self.x;
        let y = self.y;
        let z = self.z;
        let yaw = atan2f(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z)) * 57.29577;
        let pitch = -asinf(2.0 * w * y - 2.0 * x * z) * 57.29577;
        let roll = atan2f(2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)) * 57.29577;
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

impl Accel {
    // 加速计计算角度
    pub fn to_degree(self) -> (f32, f32, f32) {
        (
            acosf(self.x) * 57.29577,
            acosf(self.y) * 57.29577,
            acosf(self.z) * 57.29577,
        )
    }
    // 加速计计算弧度
    pub fn to_radians(self) -> (f32, f32, f32) {
        (acosf(self.x), acosf(self.y), acosf(self.z))
    }
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

/// 气压计
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
