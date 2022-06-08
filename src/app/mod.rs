mod imu;

pub fn start() {
    //driver::bldc::motor().unlock();
    imu::start();
}
