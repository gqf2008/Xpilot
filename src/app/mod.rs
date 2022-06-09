mod controller;
mod station_anotc;
pub fn start() {
    //driver::bldc::motor().unlock();
    controller::start();
    station_anotc::start();
}
