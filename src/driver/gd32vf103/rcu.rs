use hal::{prelude::*, rcu::Rcu, rcu::RcuExt};
use xtask::bsp::longan_nano::hal::{self, pac::RCU};

static mut RCU: Option<Rcu> = None;

pub(crate) unsafe fn init(rcu: RCU) {
    let rcu = rcu
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();
    RCU.replace(rcu);
}

pub fn rcu() -> &'static mut Rcu {
    unsafe { RCU.as_mut().unwrap() }
}
