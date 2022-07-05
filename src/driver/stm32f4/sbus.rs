use crate::mbus;
use crate::message::{Message, Telem};

use super::nvic::NVICExt;
use alloc::vec::Vec;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use xtask::arch::cortex_m;
use xtask::bsp::greenpill::hal::pac::{DMA1, USART3};
use xtask::bsp::greenpill::hal::{pac, pac::interrupt, serial::Rx};
use xtask::{
    arch::cortex_m::singleton,
    bsp::greenpill::hal::dma::{
        config::DmaConfig, PeripheralToMemory, Stream1, StreamsTuple, Transfer,
    },
};

const DMA_BUFFER_SIZE: usize = 256;

static mut DMA: Mutex<RefCell<Option<RxDma>>> = Mutex::new(RefCell::new(None));

type RxDma =
    Transfer<Stream1<DMA1>, 4, Rx<USART3>, PeripheralToMemory, &'static mut [u8; DMA_BUFFER_SIZE]>;

trait USART3Ext {
    fn clear_idle_interrupt();
}

impl USART3Ext for USART3 {
    fn clear_idle_interrupt() {
        unsafe {
            let _ = (*Self::ptr()).sr.read();
            let _ = (*Self::ptr()).dr.read();
        }
    }
}

pub unsafe fn init(mut rx: Rx<USART3, u8>, dma: DMA1) {
    rx.listen_idle();
    let stream1 = StreamsTuple::new(dma).1;
    let buf = singleton!(: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE]).unwrap();
    for (i, b) in buf.iter_mut().enumerate() {
        *b = i as u8;
    }
    let mut dma = Transfer::init_peripheral_to_memory(
        stream1,
        rx,
        buf,
        None,
        DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true),
    );
    dma.start(|_rx| {});

    cortex_m::interrupt::free(|cs| *DMA.borrow(cs).borrow_mut() = Some(dma));
    cortex_m::peripheral::NVIC::priority(pac::Interrupt::USART3, 0x01);
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART3);
    log::info!("Initialize sbus ok")
}

#[interrupt]
unsafe fn USART3() {
    USART3::clear_idle_interrupt();

    //log::info!("usart3_idle_isr");

    read_dma();
}

unsafe fn read_dma() {
    // log::info!("usart3_dma1_isr");
    static mut TRANSFER: Option<RxDma> = None;
    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| DMA.borrow(cs).replace(None).unwrap())
    });
    static mut BUF: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];

    match transfer.next_transfer(&mut BUF) {
        Ok((buf, _)) => {
            let mut data = Vec::new();
            data.extend_from_slice(buf);
            //TODO SBUS decode
            xtask::sync::free(|_| {
                mbus::bus().publish_isr("/telem/sbus", Message::Telem(Telem::Raw(data)));
            })
        }
        Err(err) => {
            log::error!("read_dma1 {:?}", err);
        }
    }
}
