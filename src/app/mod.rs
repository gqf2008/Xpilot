use crate::driver;

mod imu;
mod led;
// mod pwm_led;

pub fn start() {
    //led::start();
    // pwm_led::start();
    imu::start();
    driver::bldc::motor().unlock();
}
