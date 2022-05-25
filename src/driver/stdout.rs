use super::PERIPHERAL;
use hal::{gpio::GpioExt, pac, prelude::*, rcu::RcuExt, signature};
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::stdout;

/// 初始化USART0
pub(crate) unsafe fn init() {
    if let Some(dp) = &mut PERIPHERAL {
        let rcu = super::rcu::rcu();
        let gpioa = dp.GPIOA.split(rcu);
        let mut afio = dp.AFIO.constrain(&mut rcu);
        stdout::configure(
            dp.USART0,
            gpioa.pa9,
            gpioa.pa10,
            57600.bps(),
            &mut afio,
            &mut rcu,
        );
    }
}
