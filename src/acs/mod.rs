//! 姿态控制系统 Attitude Control System
mod adrc;
pub mod filter;
mod imu;

pub fn start() {
    imu::start();
    log::info!("Start xpilot acs ok");
}
