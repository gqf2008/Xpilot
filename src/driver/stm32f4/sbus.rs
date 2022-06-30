use crate::mbus;
use crate::message::{Message, Telem};

use super::nvic::NVICExt;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use xtask::arch::cortex_m;
use xtask::bsp::greenpill::hal::pac::{DMA1, USART3};
use xtask::bsp::greenpill::hal::{pac, serial::Rx};
use xtask::{
    arch::cortex_m::singleton,
    bsp::greenpill::hal::dma::{
        config::DmaConfig, traits::StreamISR, PeripheralToMemory, Stream1, StreamsTuple, Transfer,
    },
};

const DMA_BUFFER_SIZE: usize = 256;

static mut DMA: Mutex<RefCell<Option<RxDma>>> = Mutex::new(RefCell::new(None));

type RxDma =
    Transfer<Stream1<DMA1>, 4, Rx<USART3>, PeripheralToMemory, &'static mut [u8; DMA_BUFFER_SIZE]>;

pub unsafe fn init(rx: Rx<USART3, u8>, dma: DMA1) {
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
            .fifo_enable(true)
            .fifo_error_interrupt(true)
            .transfer_complete_interrupt(true),
    );
    dma.start(|_rx| {});

    cortex_m::interrupt::free(|cs| *DMA.borrow(cs).borrow_mut() = Some(dma));
    cortex_m::peripheral::NVIC::priority(pac::Interrupt::DMA1_STREAM1, 0x01);
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM1);
    log::info!("Initialize sbus ok")
}

#[export_name = "DMA1_STREAM1"]
unsafe fn usart1_dma_isr() {
    log::info!("usart3_dma_isr");
    static mut TRANSFER: Option<RxDma> = None;

    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| DMA.borrow(cs).replace(None).unwrap())
    });
    if Stream1::<DMA1>::get_fifo_error_flag() {
        transfer.clear_fifo_error_interrupt();
    }
    if Stream1::<DMA1>::get_transfer_complete_flag() {
        transfer.clear_transfer_complete_interrupt();

        static mut BUFFER: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];
        for (i, b) in BUFFER.iter_mut().enumerate() {
            *b = (i + 1) as u8;
        }
        let (buf, _) = transfer.next_transfer(&mut BUFFER).unwrap();
        log::info!("usart3_dma_isr {:?}", buf);
    }
}
