#[cfg(feature = "anotc")]
mod anotc;
#[cfg(feature = "mavlink")]
mod mavlink;
#[cfg(feature = "msp")]
mod msp;

pub fn start() {
    #[cfg(feature = "anotc")]
    anotc::start();
    #[cfg(feature = "mavlink")]
    mavlink::start();
    #[cfg(feature = "msp")]
    msp::start();
    log::info!("Start xpilot application ok");
}
