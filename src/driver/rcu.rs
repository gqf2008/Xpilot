use super::PERIPHERAL;
use hal::{prelude::*, rcu::Rcu, rcu::RcuExt};
use xtask::bsp::longan_nano::hal;

pub static mut RCU: Option<Rcu> = None;

pub(crate) unsafe fn init() {
    if let Some(dp) = &mut PERIPHERAL {
        let rcu = dp
            .RCU
            .configure()
            .ext_hf_clock(8.mhz())
            .sysclk(108.mhz())
            .freeze();
        RCU.replace(rcu);
    }
}

pub fn rcu() -> &'static mut Rcu {
    unsafe { &mut RCU.unwrap() }
}
