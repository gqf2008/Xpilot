//! 姿态控制系统 Attitude Control System
mod adrc;
pub mod filter;
mod imu;

pub fn start() {
    log::info!("Start xpilot attitude control system");
    imu::start();
    log::info!("Start xpilot attitude control system ok");
}
