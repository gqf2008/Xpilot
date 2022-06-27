use crate::mbus;
use crate::message::{Message, Telem};

use super::nvic::NVICExt;
use alloc::vec::Vec;
use embedded_hal::serial::Read;
use embedded_hal::serial::Write;
use xtask::arch::cortex_m;
use xtask::bsp::greenpill::hal::{
    pac,
    pac::USART1,
    serial::{Rx, Tx},
};

static mut RX: Option<Rx<USART1, u8>> = None;
static mut TX: Option<Tx<USART1, u8>> = None;

pub unsafe fn init(mut rx: Rx<USART1, u8>, tx: Tx<USART1, u8>) {
    rx.listen();
    rx.listen_idle();
    RX.replace(rx);
    TX.replace(tx);
    cortex_m::peripheral::NVIC::priority(pac::Interrupt::USART1, 0x01);
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);

    mbus::bus().register("/telem/tx", |_, msg| match msg {
        Message::Telem(Telem::Multiwii(b)) | Message::Telem(Telem::Mavlink(b)) => {
            if let Some(tx) = TX.as_mut() {
                b.iter().try_for_each(|c| nb::block!(tx.write(*c))).ok();
            }
        }
        _ => {}
    });
}

#[export_name = "USART1"]
unsafe fn usart1_isr() {
    const BUFFER_LEN: usize = 1024;
    static mut BUFFER: &mut [u8; BUFFER_LEN] = &mut [0; BUFFER_LEN];
    static mut WIDX: usize = 0;
    xtask::sync::free(|_| {
        if let Some(rx) = RX.as_mut() {
            if rx.is_rx_not_empty() {
                match nb::block!(rx.read()) {
                    Ok(w) => {
                        BUFFER[WIDX] = w;
                        WIDX += 1;
                        if WIDX >= BUFFER_LEN - 1 {
                            WIDX = 0;
                        }
                    }
                    Err(err) => {
                        log::info!("{:?}", err);
                    }
                }
            } else if rx.is_idle() {
                rx.clear_idle_interrupt();
                let mut buf = Vec::new();
                buf.extend_from_slice(&BUFFER[0..WIDX]);
                mbus::bus().publish_isr("/telem/rx", Message::Telem(Telem::Multiwii(buf)));
                WIDX = 0;
            }
        }
    });
}
