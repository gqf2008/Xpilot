#[cfg(feature = "anotc")]
mod anotc;
mod imu;
#[cfg(feature = "mavlink")]
mod mavlink;
#[cfg(feature = "msp")]
mod msp;

pub fn start() {
    imu::start();
    #[cfg(feature = "anotc")]
    anotc::start();
    #[cfg(feature = "mavlink")]
    mavlink::start();
    #[cfg(feature = "msp")]
    msp::start();
    log::info!("Start xpilot application ok");
}
