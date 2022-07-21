use eskf;
use nalgebra::{Point3, Vector3};

pub struct ErrorStateKalmanFilter {
    filter: eskf::ESKF,
}

impl ErrorStateKalmanFilter {
    pub fn new() -> Self {
        let filter = eskf::Builder::new().build();
        Self { filter }
    }
}
