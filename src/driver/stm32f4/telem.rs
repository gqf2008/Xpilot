use crate::mbus;
use crate::message::{Message, Telem};

use super::nvic::NVICExt;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use embedded_hal::serial::Write;
use xtask::arch::cortex_m;
use xtask::bsp::greenpill::hal::dma::traits::{Stream, StreamISR};
use xtask::bsp::greenpill::hal::dma::{
    config::{DmaConfig, FifoThreshold, Priority},
    PeripheralToMemory, Stream5, StreamsTuple, Transfer,
};
use xtask::bsp::greenpill::hal::pac::DMA2;
use xtask::bsp::greenpill::hal::{
    pac,
    pac::USART1,
    serial::{Rx, Tx},
};
const DMA_BUFFER_SIZE: usize = 1024;
static mut BUFFER: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];
static mut TX: Option<Tx<USART1, u8>> = None;

static mut DMA: Mutex<RefCell<Option<RxDma>>> = Mutex::new(RefCell::new(None));

type RxDma =
    Transfer<Stream5<DMA2>, 4, Rx<USART1>, PeripheralToMemory, &'static mut [u8; DMA_BUFFER_SIZE]>;

trait USART1Ext {
    fn clear_idle_interrupt();
}

impl USART1Ext for USART1 {
    fn clear_idle_interrupt() {
        unsafe {
            let _ = (*Self::ptr()).sr.read();
            let _ = (*Self::ptr()).dr.read();
        }
    }
}
pub unsafe fn init(mut rx: Rx<USART1, u8>, tx: Tx<USART1, u8>, dma: DMA2) {
    rx.listen_idle();
    let stream5 = StreamsTuple::new(dma).5;
    let buf = &mut BUFFER;
    let mut dma = Transfer::init_peripheral_to_memory(
        stream5,
        rx,
        buf,
        None,
        DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .fifo_threshold(FifoThreshold::Full)
            .priority(Priority::VeryHigh),
    );
    dma.start(|_rx| {});
    TX.replace(tx);
    cortex_m::interrupt::free(|cs| *DMA.borrow(cs).borrow_mut() = Some(dma));
    cortex_m::peripheral::NVIC::priority(pac::Interrupt::USART1, 0xff);
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);
    mbus::bus().register("/telem/tx", |_, msg| match msg {
        Message::Telem(Telem::Multiwii(b)) | Message::Telem(Telem::Mavlink(b)) => {
            if let Some(tx) = TX.as_mut() {
                b.iter().try_for_each(|c| nb::block!(tx.write(*c))).ok();
            }
        }
        _ => {}
    });
    log::info!("Initialize telem ok")
}

#[export_name = "USART1"]
unsafe fn usart1_idle_isr() {
    USART1::clear_idle_interrupt();

    log::info!("usart1_idle_isr");

    read_dma();
}

unsafe fn read_dma() {
    static mut TRANSFER: Option<RxDma> = None;
    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| DMA.borrow(cs).replace(None).unwrap())
    });
    static mut BUF: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];
    log::info!("read_dma next_transfer");
    // transfer.get_stream().set_channel()
    match transfer.next_transfer(&mut BUF) {
        Ok((buf, _)) => {
            log::info!("read_dma {:?}", buf);
        }
        Err(err) => {
            log::error!("read_dma {:?}", err);
        }
    }
}
