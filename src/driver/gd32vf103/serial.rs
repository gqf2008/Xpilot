//! Stdout based on the UART hooked up to the debug connector

use core::fmt::{self, Write};
use nb::block;
use xtask::arch::riscv::interrupt;

use xtask::bsp::longan_nano::hal::{
    afio::Afio,
    gpio::{
        gpioa::{PA2, PA3},
        Active,
    },
    pac::USART1,
    prelude::*,
    rcu::Rcu,
    serial::{Config, Parity, Rx, Serial, StopBits, Tx},
    time::Bps,
};

static mut SERIAL: Option<SerialWrapper> = None;

pub fn init<X, Y>(
    uart: USART1,
    tx: PA2<X>,
    rx: PA3<Y>,
    baud_rate: Bps,
    afio: &mut Afio,
    rcu: &mut Rcu,
) where
    X: Active,
    Y: Active,
{
    let tx = tx.into_alternate_push_pull();
    let rx = rx.into_floating_input();
    let config = Config {
        baudrate: baud_rate,
        parity: Parity::ParityNone,
        stopbits: StopBits::STOP1,
    };
    let serial = Serial::new(uart, (tx, rx), config, afio, rcu);
    let (tx, rx) = serial.split();
    interrupt::free(|_| unsafe {
        SERIAL.replace(SerialWrapper(tx, rx));
    })
}

#[inline]
pub fn write_str(s: &str) {
    unsafe {
        if let Some(stdout) = SERIAL.as_mut() {
            let _ = stdout.write_str(s);
        }
    }
}

#[inline]
pub fn write_fmt(args: fmt::Arguments) {
    unsafe {
        if let Some(stdout) = SERIAL.as_mut() {
            let _ = stdout.write_fmt(args);
        }
    }
}

struct SerialWrapper(Tx<USART1>, Rx<USART1>);

impl fmt::Write for SerialWrapper {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.as_bytes() {
            if *byte == '\n' as u8 {
                let res = block!(self.0.write('\r' as u8));

                if res.is_err() {
                    return Err(::core::fmt::Error);
                }
            }

            let res = block!(self.0.write(*byte));

            if res.is_err() {
                return Err(::core::fmt::Error);
            }
        }
        Ok(())
    }
}
