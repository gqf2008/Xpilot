//! AHRS九轴姿态融合
//!

use super::Filter;

use ahrs::{Ahrs, Madgwick};
use nalgebra::{UnitQuaternion, Vector3};
pub struct AhrsFilter {
    madgwick: Madgwick<f32>,
}

impl AhrsFilter {
    pub fn new(sample_period: f32, gain: f32) -> Self {
        Self {
            madgwick: Madgwick::new(sample_period, gain),
        }
    }
}
type Gyro = Vector3<f32>;
type Accel = Vector3<f32>;
type Mag = Vector3<f32>;

impl Filter<(Gyro, Accel, Option<Mag>), UnitQuaternion<f32>> for AhrsFilter {
    fn do_filter(
        &mut self,
        (gyro, acc, mag): (Gyro, Accel, Option<Mag>),
        output: &mut UnitQuaternion<f32>,
    ) {
        if let Ok(quat) = if let Some(mag) = &mag {
            self.madgwick.update(&gyro, &acc, mag)
        } else {
            self.madgwick.update_imu(&gyro, &acc)
        } {
            *output = quat.clone();
        }
    }
}
