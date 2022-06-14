//! 姿态控制系统 Attitude Control System
mod adrc;
#[cfg(feature = "heli")]
mod heli;
mod imu;

pub fn start() {
    log::info!("Start xpilot attitude control system");
    #[cfg(feature = "heli")]
    heli::start();
    log::info!("Start xpilot attitude control system ok");
}
