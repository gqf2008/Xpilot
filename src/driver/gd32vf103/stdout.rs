use hal::{gpio::GpioExt, pac::Peripherals, prelude::*};
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::stdout;

/// 初始化USART0
pub(crate) unsafe fn init() {
    let dp = Peripherals::steal();
    let rcu = super::rcu::rcu();
    let gpioa = dp.GPIOA.split(rcu);
    let mut afio = dp.AFIO.constrain(rcu);
    stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        57600.bps(),
        &mut afio,
        rcu,
    );
}
