use hal::{pac::Peripherals, prelude::*, rcu::Rcu, rcu::RcuExt};
use xtask::bsp::longan_nano::hal;

static mut RCU: Option<Rcu> = None;

pub(crate) unsafe fn init() {
    let dp = Peripherals::steal();
    let rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();
    RCU.replace(rcu);
}

pub fn rcu() -> &'static mut Rcu {
    unsafe { RCU.as_mut().unwrap() }
}
