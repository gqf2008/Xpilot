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

use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

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
    pub quaternion: Option<UnitQuaternion<f32>>,
}

impl ImuData {
    pub fn quate(mut self, quat: UnitQuaternion<f32>) -> Self {
        self.quaternion = Some(quat);
        self
    }
    pub fn accel(mut self, accel: Accel) -> Self {
        self.accel = Some(accel);
        self
    }
    pub fn gyro(mut self, gyro: Gyro) -> Self {
        self.gyro = Some(gyro);
        self
    }
    pub fn compass(mut self, compass: Compass) -> Self {
        self.compass = Some(compass);
        self
    }
    pub fn temp(mut self, temp: f32) -> Self {
        self.temp = Some(temp);
        self
    }
}

/// 四元数
pub type Quaternion = UnitQuaternion<f32>;

/// 重力加速度，单位g
pub type Accel = Vector3<f32>;
/// 角速度，单位rad/s
pub type Gyro = Vector3<f32>;
/// 罗盘
pub type Compass = Vector3<f32>;

/// 气压计
#[derive(Copy, Clone, Debug, Default)]
pub struct Barometer {
    pub h: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Distance(pub f32);

#[derive(Copy, Clone, Debug, Default)]
pub struct Gps {
    pub longitude: f32,
    pub latitude: f32,
}
