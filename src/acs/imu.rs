//! 惯性测量单元
//!
use crate::{driver::ImuData, mbus};
use ahrs::{Ahrs, Madgwick};
static mut IMU: Option<InertialMeasurementUnit> = None;

pub fn start() {}
pub struct InertialMeasurementUnit {
    ahrs: Madgwick<f32>,
}

impl InertialMeasurementUnit {
    pub fn update(&self, data: ImuData) {}
}
