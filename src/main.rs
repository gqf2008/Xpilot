#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(const_option)]

mod app;
mod driver;
mod mbus;
mod message;

extern crate alloc;

#[cfg(any(feature = "stm32f401ccu6", feature = "stm32f427vit6"))]
use xtask::arch::cortex_m::rt;
#[cfg(feature = "gd32vf103")]
use xtask::arch::riscv::rt;

/// 初始化外设驱动
unsafe fn init() {
    #[cfg(feature = "gd32vf103")]
    {
        extern "C" {
            static _sheap: u8;
        }
        let start_addr = &_sheap as *const u8 as usize;
        let size = 32 * 1024;
        log::info!(
            "Initialize heap, start_addr:0x{:02X} size: {}",
            start_addr,
            size
        );
        xtask::init(start_addr, size);
    }
    #[cfg(feature = "stm32f401ccu6")]
    {
        let start_addr = rt::heap_start() as usize;
        let size = 60 * 1024;
        log::info!(
            "Initialize heap, start_addr:0x{:02X} size: {}",
            start_addr,
            size
        );
        //4k留给主栈
        xtask::init(start_addr, size);
    }

    #[cfg(feature = "stm32f427vit6")]
    {
        let start_addr = rt::heap_start() as usize;
        let size = 184 * 1024;
        log::info!(
            "Initialize heap, start_addr:0x{:02X} size: {}",
            start_addr,
            size
        );
        //8k留给主栈
        xtask::init(start_addr, size);
    }
    driver::init();
}

#[rt::entry]
fn main() -> ! {
    unsafe {
        init();
    }
    log::info!("Start xpilot application");
    // 启动应用
    app::start();
    //启动调度器
    xtask::start()
}
