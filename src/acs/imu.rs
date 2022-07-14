//! 惯性测量单元，接收陀螺仪、加速度计、磁力计数据，融合计算输出欧拉角
//!
use crate::acs::filter::first_order::FirstOrderFilter3;
use crate::acs::filter::jitter_filter::JitterFilter3;
use crate::acs::filter::Filter;
use crate::driver::Euler;
use crate::{driver::ImuData, mbus, message::Message};
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;

use xtask::{Queue, TaskBuilder};
static mut IMU_FILTER: Option<ImuFilter> = None;

static mut Q: Option<Queue<ImuData>> = None;
pub fn start() {
    unsafe {
        let q = Queue::with_capacity(100);
        Q.replace(q);
        IMU_FILTER.replace(ImuFilter::new());
        mbus::bus().subscribe("/imu/raw", |_, msg| match msg {
            Message::ImuData(data) => {
                if let Some(q) = Q.as_mut() {
                    q.push_back_isr(data).ok();
                }
            }
            _ => {}
        });
    }
    TaskBuilder::new()
        .name("imu_raw_filter")
        .priority(1)
        .stack_size(1024)
        .spawn(|| unsafe {
            let mut filter = FirstOrderFilter3::new(0.3).chain(JitterFilter3::new(0.01));
            loop {
                if let Some(q) = Q.as_mut() {
                    if let Some(mut data) = q.pop_front() {
                        if let Some(imu) = IMU_FILTER.as_mut() {
                            imu.update(&mut data);
                            if let Some(quat) = data.quaternion {
                                let (roll, pitch, yaw) = quat.euler_angles();
                                let mut output = Vector3::<f32>::default();
                                filter.do_filter(
                                    Vector3::<f32>::from_column_slice(&[roll, pitch, yaw]),
                                    &mut output,
                                );
                                mbus::bus().publish(
                                    "/imu",
                                    Message::ImuData(
                                        data.euler(Euler::new(output[0], output[1], output[2])),
                                    ),
                                );
                            }
                        }
                    }
                }
            }
        });
}
pub struct ImuFilter {
    ahrs: Madgwick<f32>,
}

impl ImuFilter {
    fn new() -> Self {
        Self {
            ahrs: Madgwick::new(1.0 / 100.0, 0.1),
        }
    }
}

impl ImuFilter {
    pub fn update(&mut self, data: &mut ImuData) {
        if let Some(acc) = data.accel {
            if let Some(gyro) = data.gyro {
                if let Some(mag) = data.compass {
                    if let Ok(quat) = self.ahrs.update(&gyro, &acc, &mag) {
                        data.quate(*quat)
                    }
                } else {
                    if let Ok(quat) = self.ahrs.update_imu(&gyro, &acc) {
                        data.quate(*quat);
                    }
                }
            }
        }
    }
}
