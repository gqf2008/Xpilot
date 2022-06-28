#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(const_option)]
#![feature(associated_type_bounds)]
#![feature(const_fn_floating_point_arithmetic)]

extern crate alloc;

mod acs;
mod app;
mod driver;
mod drone;
mod fsm;
mod mbus;
mod message;

#[cfg(any(feature = "stm32f401ccu6", feature = "stm32f427vit6"))]
use xtask::arch::cortex_m::rt;
#[cfg(feature = "gd32vf103")]
use xtask::arch::riscv::rt;

unsafe fn init_heap() {
    #[cfg(feature = "gd32vf103")]
    {
        extern "C" {
            static _sheap: u8;
        }
        let start_addr = &_sheap as *const u8 as usize;
        let size = 32 * 1024;
        xtask::init(start_addr, size);
        log::info!(
            "Initialize heap, start_addr:0x{:02X} size: {}",
            start_addr,
            size
        );
    }
    #[cfg(feature = "stm32f401ccu6")]
    {
        let start_addr = rt::heap_start() as usize;
        let size = 60 * 1024;
        //4k留给主栈
        xtask::init(start_addr, size);
        log::info!(
            "Initialize heap, start_addr:0x{:02X} size: {}",
            start_addr,
            size
        );
    }

    #[cfg(feature = "stm32f427vit6")]
    {
        let start_addr = rt::heap_start() as usize;
        let size = 186 * 1024;
        xtask::init_heap(start_addr, size);
        log::info!(
            "Initialize heap, start_addr:0x{:02X} size: {}",
            start_addr,
            size
        );
    }
}

#[rt::entry]
fn main() -> ! {
    xtask::init_logger();
    unsafe {
        init_heap();
    }
    driver::init();
    // 启动应用
    app::start();
    // 启动姿态控制
    acs::start();
    //启动调度器
    xtask::start()
}
