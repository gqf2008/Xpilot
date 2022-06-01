pub mod bldc;
pub mod led;
pub mod mpu6050;
pub mod serial;
pub mod servo;

mod rcu;

use hal::gpio::GpioExt;
use hal::pac::Peripherals;
use hal::prelude::*;
use hal::signature;

use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::hal::exti::Exti;
use xtask::bsp::longan_nano::hal::exti::ExtiLine;
use xtask::bsp::longan_nano::hal::exti::TriggerEdge;
use xtask::bsp::longan_nano::stdout;

pub unsafe fn init() {
    if let Some(dp) = Peripherals::take() {
        rcu::init(dp.RCU);
        let rcu = rcu::rcu();
        let pa = dp.GPIOA.split(rcu);
        let pb = dp.GPIOB.split(rcu);
        let pc = dp.GPIOC.split(rcu);

        // let pd = dp.GPIOD.split(rcu);
        // let pe = dp.GPIOE.split(rcu);
        let mut afio = dp.AFIO.constrain(rcu);
        stdout::configure(dp.USART0, pa.pa9, pa.pa10, 57600.bps(), &mut afio, rcu);
        log::info!(
            "Starting [debug_id={:#08X}, flash_size: {}KB, sram_size={}KB]",
            dp.DBG.id.read().bits(),
            signature::flash_size_kb(),
            signature::sram_size_kb(),
        );

        led::init(Some(pc.pc13), Some(pa.pa1), Some(pa.pa2));
        bldc::init(
            dp.TIMER2,
            (Some(pa.pa6), Some(pa.pa7), Some(pb.pb0), Some(pb.pb1)),
            &mut afio,
            rcu,
        );
        mpu6050::init(dp.TIMER0, (pb.pb10, pb.pb11), rcu, dp.I2C1);
        //mpu6050_dmp::init((pb.pb10, pb.pb11), rcu, dp.I2C1);
    }
}
