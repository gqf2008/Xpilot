use crate::mbus;
use crate::message::{Message, Telem};

use super::nvic::NVICExt;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use embedded_hal::serial::Write;
use xtask::arch::cortex_m;
use xtask::bsp::greenpill::hal::pac::DMA2;
use xtask::bsp::greenpill::hal::{
    pac,
    pac::USART1,
    serial::{Rx, Tx},
};
use xtask::{
    arch::cortex_m::singleton,
    bsp::greenpill::hal::dma::{
        config::DmaConfig, traits::StreamISR, PeripheralToMemory, Stream5, StreamsTuple, Transfer,
    },
};
const DMA_BUFFER_SIZE: usize = 256;

static mut TX: Option<Tx<USART1, u8>> = None;
static mut DMA: Mutex<RefCell<Option<RxDma>>> = Mutex::new(RefCell::new(None));

type RxDma =
    Transfer<Stream5<DMA2>, 4, Rx<USART1>, PeripheralToMemory, &'static mut [u8; DMA_BUFFER_SIZE]>;

pub unsafe fn init(rx: Rx<USART1, u8>, tx: Tx<USART1, u8>, dma: DMA2) {
    let stream5 = StreamsTuple::new(dma).5;
    let buf = singleton!(: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE]).unwrap();
    for (i, b) in buf.iter_mut().enumerate() {
        *b = i as u8;
    }
    let mut dma = Transfer::init_peripheral_to_memory(
        stream5,
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
    TX.replace(tx);
    cortex_m::interrupt::free(|cs| *DMA.borrow(cs).borrow_mut() = Some(dma));
    cortex_m::peripheral::NVIC::priority(pac::Interrupt::DMA2_STREAM5, 0x01);
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA2_STREAM5);

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

#[export_name = "DMA2_STREAM5"]
unsafe fn usart1_dma_isr() {
    log::info!("usart1_dma_isr");
    static mut TRANSFER: Option<RxDma> = None;

    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| DMA.borrow(cs).replace(None).unwrap())
    });
    if Stream5::<DMA2>::get_fifo_error_flag() {
        log::info!("usart1_dma_isr error");
        transfer.clear_fifo_error_interrupt();
    } else if Stream5::<DMA2>::get_transfer_complete_flag() {
        log::info!("usart1_dma_isr complete");
        transfer.clear_transfer_complete_interrupt();

        static mut BUFFER: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];
        for (i, b) in BUFFER.iter_mut().enumerate() {
            *b = (i + 1) as u8;
        }
        let (buf, _) = transfer.next_transfer(&mut BUFFER).unwrap();
        log::info!("usart1_dma_isr {:?}", buf);
    }
}
