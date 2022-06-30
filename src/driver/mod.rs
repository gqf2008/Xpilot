#[cfg(feature = "gd32vf103")]
mod gd32vf103;
#[cfg(feature = "gd32vf103")]
pub use gd32vf103::{bldc, led, mpu6050, serial, servo};

#[cfg(any(feature = "stm32f401ccu6", feature = "stm32f427vit6"))]
pub mod stm32f4;

pub mod bldc;
pub mod mpu6050;
pub mod ppm;
pub mod sbus;
pub mod servo;

use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

pub fn init() {
    #[cfg(feature = "gd32vf103")]
    unsafe {
        gd32vf103::init();
        log::info!("Initialize gd32vf103 driver ok");
    }
    #[cfg(feature = "stm32f401ccu6")]
    unsafe {
        stm32f4::init();
        log::info!("Initialize stm32f401ccu6 driver ok");
    }
    #[cfg(feature = "stm32f427vit6")]
    unsafe {
        stm32f4::init();
        log::info!("Initialize stm32f427vit6 driver ok");
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Euler {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl Euler {
    pub fn new(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self { roll, pitch, yaw }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct ImuData {
    pub accel: Option<Accel>,
    pub temp: Option<f32>,
    pub gyro: Option<Gyro>,
    pub compass: Option<Compass>,
    pub quaternion: Option<UnitQuaternion<f32>>,
    pub euler: Option<Euler>,
}

impl ImuData {
    pub fn quate(&mut self, quat: UnitQuaternion<f32>) {
        self.quaternion = Some(quat);
    }

    pub fn euler(mut self, euler: Euler) -> Self {
        self.euler = Some(euler);
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

//欧拉角转四元数
pub fn from_euler((roll, pitch, yaw): (f32, f32, f32)) -> (f32, f32, f32, f32) {
    use libm::*;
    let cr2 = cosf(roll * 0.5);
    let cp2 = cosf(pitch * 0.5);
    let cy2 = cosf(yaw * 0.5);
    let sr2 = sinf(roll * 0.5);
    let sp2 = sinf(pitch * 0.5);
    let sy2 = sinf(yaw * 0.5);

    let q1 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    let q2 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    let q3 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    let q4 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
    (q1, q2, q3, q4)
}

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
