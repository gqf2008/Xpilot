//! 惯性测量单元，接收陀螺仪、加速度计、磁力计数据，融合计算输出欧拉角
//!
use crate::acs::filter::first_order::FirstOrderFilter3;
use crate::acs::filter::limiting::LimitingFilter3;
use crate::acs::filter::moving_average::MovingAverageFilter3;
use crate::acs::filter::{Chain, Filter};
use crate::driver::Euler;
use crate::{driver::ImuData, mbus, message::Message};
use ahrs::{Ahrs, Madgwick};

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
            loop {
                if let Some(q) = Q.as_mut() {
                    if let Some(mut data) = q.pop_front() {
                        if let Some(imu) = IMU_FILTER.as_mut() {
                            imu.update(&mut data);
                            if let Some(quat) = data.quaternion {
                                let (roll, pitch, yaw) = quat.euler_angles();
                                mbus::bus().publish(
                                    "/imu",
                                    Message::ImuData(data.euler(Euler::new(roll, pitch, yaw))),
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
    accel: Chain<FirstOrderFilter3, MovingAverageFilter3<50>>,
    gyro: Chain<FirstOrderFilter3, MovingAverageFilter3<50>>,
    compass: Chain<Chain<LimitingFilter3, FirstOrderFilter3>, MovingAverageFilter3<60>>,
}

impl ImuFilter {
    fn new() -> Self {
        Self {
            ahrs: Madgwick::new(1.0 / 100.0, 0.1),
            accel: FirstOrderFilter3::new(0.01).chain(MovingAverageFilter3::<50>::new()),
            gyro: FirstOrderFilter3::new(0.01).chain(MovingAverageFilter3::<50>::new()),
            compass: LimitingFilter3::new(3.0)
                .chain(FirstOrderFilter3::new(0.01))
                .chain(MovingAverageFilter3::<60>::new()),
        }
    }
}

impl ImuFilter {
    pub fn update(&mut self, data: &mut ImuData) {
        if let Some(mut acc) = data.accel {
            self.accel.do_filter(acc, &mut acc);

            if let Some(mut gyro) = data.gyro {
                self.gyro.do_filter(gyro, &mut gyro);
                if let Some(mut mag) = data.compass {
                    self.compass.do_filter(mag, &mut mag);
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
