#![no_std]
#![no_main]

mod app;
mod driver;
mod mbus;
mod message;

extern crate alloc;

use xtask::arch::riscv::rt;
use xtask::prelude::*;

/// 初始化外设驱动
#[rt::pre_init]
unsafe fn init() {
    extern "C" {
        static _sheap: u8;
    }
    let start_addr = &_sheap as *const u8 as usize;
    xtask::init(start_addr, 32 * 1024);
    driver::init();
}

#[rt::entry]
fn main() -> ! {
    // 启动应用
    app::start();
    mbus::mbus().subscribe("a.b", |_topic, _msg| {});
    //启动调度器
    xtask::start()
}
