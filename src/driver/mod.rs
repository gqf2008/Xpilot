mod led;
mod mpu6050;
mod rcu;
mod stdout;

use hal::pac::Peripherals;
use hal::signature;
use xtask::bsp::longan_nano::hal;

pub static mut PERIPHERAL: Option<Peripherals> = None;

pub fn init() {
    unsafe {
        if let Some(dp) = Peripherals::take() {
            PERIPHERAL.replace(dp);
        }
        rcu::init();
        stdout::init();
        mpu6050::init();
    }
    log::info!(
        "Starting [debug_id={:#08X}, flash_size: {}KB, sram_size={}KB]",
        &mut PERIPHERAL.unwrap().DBG.id.read().bits(),
        signature::flash_size_kb(),
        signature::sram_size_kb(),
    );
}
