mod imu;
mod led;
mod mpu_dmp;

pub fn start() {
    led::start();
    //mpu_dmp::start();
    imu::start();
}
