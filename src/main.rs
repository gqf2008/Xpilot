#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(const_option)]

mod app;
mod driver;
mod mbus;
mod message;

extern crate alloc;

#[cfg(feature = "stm32f4")]
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
        xtask::init(start_addr, 32 * 1024);
    }
    #[cfg(feature = "stm32f4")]
    {
        let start_addr = rt::heap_start() as usize;
        //4k留给主栈
        xtask::init(start_addr, 60 * 1024);
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
    log::info!("Start xtask");
    //启动调度器
    xtask::start()
}
