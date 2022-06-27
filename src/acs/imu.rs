//! 惯性测量单元，接收陀螺仪、加速度计、磁力计数据，融合计算输出欧拉角
//!
use crate::{
    driver::{Accel, Compass, Gyro, ImuData},
    mbus,
    message::Message,
};
use ahrs::{Ahrs, Madgwick};
use xtask::{Queue, TaskBuilder};

static mut IMU: Option<InertialMeasurementUnit> = None;

static mut Q: Option<Queue<()>> = None;
pub fn start() {
    unsafe {
        let q = Queue::new();
        Q.replace(q);

        IMU.replace(InertialMeasurementUnit {
            ahrs: Madgwick::new(1.0 / 100.0, 0.1),
        });
        mbus::bus().subscribe("/imu/raw", |_, _| {
            if let Some(q) = Q.as_mut() {
                q.push_back_isr(()).ok();
            }
        });
    }
    TaskBuilder::new()
        .name("imu")
        .priority(1)
        .stack_size(1024)
        .spawn(|| unsafe {
            loop {
                if let Some(q) = Q.as_mut() {
                    if let Some(_) = q.pop_front() {
                        if let Some(imu) = IMU.as_mut() {
                            if let Some(mpu) = crate::driver::stm32f4::mpu9250::mpu() {
                                if let Ok(all) = mpu.all::<[f32; 3]>() {
                                    let acc = Accel::new(all.accel[0], all.accel[1], all.accel[2]);
                                    let gyro = Gyro::new(all.gyro[0], all.gyro[1], all.gyro[2]);
                                    let mag = Compass::new(all.mag[0], all.mag[1], all.mag[2]);
                                    let mut data =
                                        ImuData::default().accel(acc).gyro(gyro).compass(mag);
                                    imu.update(&mut data);
                                    mbus::bus().publish("/imu", Message::ImuData(data));
                                }
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
                        data.quate(*quat);
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
