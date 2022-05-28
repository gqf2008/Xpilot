mod imu;
mod led;

pub fn start() {
    led::start();
    imu::start();
}
