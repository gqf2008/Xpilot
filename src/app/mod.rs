use crate::driver;

mod imu;

pub fn start() {
    //driver::bldc::motor().unlock();
    imu::start();
}
