//! 惯性测量单元，接收陀螺仪、加速度计、磁力计数据，融合计算输出欧拉角
//!
use crate::acs::filter::first_order::FirstOrderFilter;
use crate::acs::filter::limiting::LimitingFilter;
use crate::acs::filter::moving_average::MovingAverageFilter;
use crate::acs::filter::Filter;
use crate::driver::Euler;
use crate::{driver::ImuData, mbus, message::Message};
use ahrs::{Ahrs, Madgwick};
use xtask::{Queue, TaskBuilder};
static mut IMU: Option<InertialMeasurementUnit> = None;

static mut Q: Option<Queue<ImuData>> = None;
pub fn start() {
    unsafe {
        let q = Queue::new();
        Q.replace(q);

        IMU.replace(InertialMeasurementUnit {
            ahrs: Madgwick::new(1.0 / 100.0, 0.1),
        });
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
        .name("imu")
        .priority(1)
        .stack_size(1024)
        .spawn(|| unsafe {
            let mut dither_roll =
                FirstOrderFilter::new(0.01).chain(MovingAverageFilter::<50>::new());
            let mut dither_pitch =
                FirstOrderFilter::new(0.01).chain(MovingAverageFilter::<50>::new());
            let mut dither_yaw = LimitingFilter::new(3.0)
                .chain(FirstOrderFilter::new(0.01))
                .chain(MovingAverageFilter::<60>::new());
            loop {
                if let Some(q) = Q.as_mut() {
                    if let Some(mut data) = q.pop_front() {
                        if let Some(imu) = IMU.as_mut() {
                            imu.update(&mut data);
                            if let Some(quat) = data.quaternion {
                                let (roll, pitch, yaw) = quat.euler_angles();
                                let mut froll = 0.0;
                                let mut fpitch = 0.0;
                                let mut fyaw = 0.0;
                                dither_roll.do_filter(roll, &mut froll);
                                dither_pitch.do_filter(pitch, &mut fpitch);
                                dither_yaw.do_filter(yaw, &mut fyaw);
                                mbus::bus().publish(
                                    "/imu",
                                    Message::ImuData(data.euler(Euler::new(froll, fpitch, fyaw))),
                                );
                            }
                        }
                    }
                }
            }
        });
}
pub struct InertialMeasurementUnit {
    ahrs: Madgwick<f32>,
}

impl InertialMeasurementUnit {
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
