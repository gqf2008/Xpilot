#![no_std]
#![no_main]

mod app;
mod driver;
mod mbus;
mod message;

extern crate alloc;
use hal::{gpio::GpioExt, pac, prelude::*, rcu::RcuExt, signature};
use xtask::arch::riscv::rt;
use xtask::bsp::longan_nano::hal;
use xtask::bsp::longan_nano::led::{rgb, Led, BLUE, GREEN, RED};
use xtask::prelude::*;

fn init() {
    extern "C" {
        static _sheap: u8;
    }
    let start_addr = unsafe { &_sheap as *const u8 as usize };
    xtask::init(start_addr, 32 * 1024);
}

#[rt::entry]
fn main() -> ! {
    //初始化外设&内存
    init();
    //bus
    example_bus();
    //启动调度器
    xtask::start()
}

fn example_bus() {
    static BUS: Bus<Event> = Bus::new();
    #[derive(Debug, Clone, Copy)]
    enum Event {
        YawPitchRoll { yaw: f32, pitch: f32, roll: f32 },
        Mouse(isize, isize),
    }
    let kqueue = Queue::new();
    let krecv = kqueue.clone();
    let mqueue = Queue::new();
    let mrecv = mqueue.clone();
    BUS.subscribe("ypr", move |topic, ev| match ev {
        Event::YawPitchRoll { yaw, pitch, roll } => {
            kqueue.push_back(ev);
        }
        _ => {}
    });
    BUS.subscribe("ev.mouse", move |topic, ev| match ev {
        Event::Mouse(x, y) => {
            mqueue.push_back(ev);
        }
        _ => {}
    });

    // 模拟两个中断服务程序
    TaskBuilder::new().name("isr1").spawn(move || loop {
        // BUS.publish("ev.key", Event::Key(8));
        // xtask::sleep_ms(1000);
    });
    TaskBuilder::new().name("isr2").spawn(move || loop {
        BUS.publish("ev.mouse", Event::Mouse(8378, 10036));
        xtask::sleep_ms(100);
    });

    // 两个从中断服务接收数据的服务
    TaskBuilder::new().name("key.service").spawn(move || loop {
        if let Some(msg) = krecv.pop_front() {
            log::info!("收到消息key {:?}", msg);
        }
    });
    TaskBuilder::new()
        .name("mouse.service")
        .spawn(move || loop {
            if let Some(msg) = mrecv.pop_front() {
                log::info!("收到消息mouse {:?}", msg);
            }
        });
}

fn example_led(mut red: RED, mut green: GREEN, mut blue: BLUE) {
    TaskBuilder::new()
        .name("green")
        .priority(1)
        .spawn(move || loop {
            green.toggle();
            xtask::sleep_ms(500);
        });

    TaskBuilder::new()
        .name("red")
        .priority(1)
        .spawn(move || loop {
            red.toggle();
            xtask::sleep_ms(500);
        });

    TaskBuilder::new()
        .name("blue")
        .priority(1)
        .spawn(move || loop {
            blue.toggle();
            xtask::sleep_ms(500);
        });
}
