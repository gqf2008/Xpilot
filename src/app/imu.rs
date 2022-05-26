use crate::driver;
use mpu6050::{device::MOT_DETECT_STATUS, *};
use xtask::{Delay, TaskBuilder};

pub fn start() {
    // motion_detecte();
    sampling();
}

fn sampling() {
    let mpu = driver::mpu6050::mpu();
    TaskBuilder::new()
        .name("icm")
        .priority(1)
        .stack_size(1024)
        .spawn(move || loop {
            // get roll and pitch estimate
            if let Some(rp) = mpu.get_acc_angles().ok() {
                log::info!("r/p: {:?}", rp);
            }
            // get temp
            if let Some(temp) = mpu.get_temp().ok() {
                log::info!("temp: {:?}c", temp);
            }

            // get gyro data, scaled with sensitivity
            if let Some(gyro) = mpu.get_gyro().ok() {
                log::info!("gyro: {:?}", gyro);
            }

            // get accelerometer data, scaled with sensitivity
            if let Some(acc) = mpu.get_acc().ok() {
                log::info!("accel: {:?}", acc);
            }
            xtask::sleep_ms(100);
        });
}

fn motion_detecte() {
    let mpu = driver::mpu6050::mpu();
    mpu.setup_motion_detection().ok();
    TaskBuilder::new().name("icm").priority(1).spawn(move || {
        let mut count: u8 = 0;
        let mut delay = Delay::new();
        loop {
            if let Ok(_) = mpu.get_motion_detected() {
                log::info!(
                    "YEAH BUDDY. Motion by axes: {:b}",
                    mpu.read_byte(MOT_DETECT_STATUS::ADDR).unwrap()
                );
                count += 1;
            }

            xtask::sleep_ms(10);

            if count > 5 {
                mpu.reset_device(&mut delay).ok();
                count = 0;
            }
        }
    });
}
