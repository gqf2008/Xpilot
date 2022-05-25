pub mod led;
pub mod mpu6050;
mod rcu;
mod stdout;

use hal::pac::Peripherals;
use hal::signature;
use xtask::bsp::longan_nano::hal;

pub unsafe fn init() {
    rcu::init();
    led::init();
    stdout::init();
    mpu6050::init();
    log::info!(
        "Starting [debug_id={:#08X}, flash_size: {}KB, sram_size={}KB]",
        Peripherals::steal().DBG.id.read().bits(),
        signature::flash_size_kb(),
        signature::sram_size_kb(),
    );
}
