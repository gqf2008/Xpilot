use mpu6050_dmp::{quaternion::Quaternion, yaw_pitch_roll::YawPitchRoll};
use xtask::TaskBuilder;

use crate::driver;

pub fn start() {
    TaskBuilder::new()
        .name("icm")
        .priority(1)
        .stack_size(1024)
        .spawn(|| {
            if let Some(mpu) = driver::mpu6050_dmp::mpu() {
                loop {
                    match mpu.accel() {
                        Ok(acc) => {
                            log::info!("{:?}", acc);
                        }
                        Err(err) => {
                            log::error!("accel error {:?}", err);
                            // xtask::sleep_ms(50);
                            continue;
                        }
                    }
                    match mpu.gyro() {
                        Ok(gyro) => {
                            log::info!("{:?}", gyro);
                        }
                        Err(err) => {
                            log::error!("gyro error {:?}", err);
                            //  xtask::sleep_ms(50);
                            continue;
                        }
                    }
                    xtask::sleep_ms(100);
                }
            }
        });
}
