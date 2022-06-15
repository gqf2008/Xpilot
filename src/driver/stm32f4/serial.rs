use super::nvic::NVICExt;
use embedded_hal::serial::Read;
use xtask::arch::cortex_m;
use xtask::bsp::greenpill::hal::{pac, pac::USART1, serial::Rx};

static mut RX: Option<Rx<USART1, u8>> = None;

pub unsafe fn init(mut rx: Rx<USART1, u8>) {
    rx.listen();
    rx.listen_idle();
    RX.replace(rx);
    cortex_m::peripheral::NVIC::priority(pac::Interrupt::USART1, 0x01);
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);
}

#[export_name = "USART1"]
unsafe fn usart1_isr() {
    const BUFFER_LEN: usize = 1024;
    static mut BUFFER: &mut [u8; BUFFER_LEN] = &mut [0; BUFFER_LEN];
    static mut WIDX: usize = 0;
    cortex_m::interrupt::free(|_| {
        if let Some(rx) = RX.as_mut() {
            if rx.is_rx_not_empty() {
                match nb::block!(rx.read()) {
                    Ok(w) => {
                        BUFFER[WIDX] = w;
                        WIDX += 1;
                        if WIDX >= BUFFER_LEN - 1 {
                            log::info!("{:?}", &BUFFER[..]);
                            WIDX = 0;
                        }
                    }
                    Err(err) => {
                        log::info!("{:?}", err);
                    }
                }
            } else if rx.is_idle() {
                rx.clear_idle_interrupt();
                log::info!("{:?}", &BUFFER[0..WIDX]);
                WIDX = 0;
            }
        }
    });
}
