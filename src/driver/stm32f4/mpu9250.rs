use super::nvic::NVICExt;
use crate::mbus;
use mpu9250::*;
use shared_bus::{NullMutex, SpiProxy};
#[cfg(any(feature = "stm32f427vit6", feature = "stm32f401ccu6"))]
use xtask::bsp::greenpill::hal::pac::SPI1;
use xtask::bsp::greenpill::hal::timer::CounterHz;
use xtask::bsp::greenpill::hal::{
    gpio::{Output, PushPull},
    spi::{Master, TransferModeNormal},
};
use xtask::Delay;
use xtask::{
    arch::cortex_m::peripheral::NVIC,
    bsp::greenpill::hal::{
        gpio::{Alternate, Pin},
        pac::{Interrupt, TIM1},
        prelude::*,
        rcc::Clocks,
        spi::Spi,
        timer::{Event, Timer1},
    },
};

pub type MPU = Mpu9250<
    SpiDevice<
        SpiProxy<
            'static,
            NullMutex<
                Spi<
                    SPI1,
                    (
                        Pin<'A', 5, Alternate<5, PushPull>>,
                        Pin<'A', 6, Alternate<5, PushPull>>,
                        Pin<'A', 7, Alternate<5, PushPull>>,
                    ),
                    TransferModeNormal,
                    Master,
                >,
            >,
        >,
        Pin<'A', 4, Output<PushPull>>,
    >,
    Marg,
>;
#[cfg(any(feature = "stm32f427vit6", feature = "stm32f401ccu6"))]
static mut MPU: Option<MPU> = None;
static mut TIMER: Option<CounterHz<TIM1>> = None;

#[cfg(any(feature = "stm32f427vit6", feature = "stm32f401ccu6"))]
pub(crate) unsafe fn init(
    tim: TIM1,
    spi: SpiProxy<
        'static,
        NullMutex<
            Spi<
                SPI1,
                (
                    Pin<'A', 5, Alternate<5, PushPull>>,
                    Pin<'A', 6, Alternate<5, PushPull>>,
                    Pin<'A', 7, Alternate<5, PushPull>>,
                ),
                TransferModeNormal,
                Master,
            >,
        >,
    >,
    ncs: Pin<'A', 4, Output<PushPull>>,
    clocks: &Clocks,
) {
    log::info!("Initialize mpu9250");
    let sample_rate = 100;
    let mut delay = Delay::new();
    match Mpu9250::marg_default(spi, ncs, &mut delay) {
        Ok(mut mpu) => {
            //校准
            if let Err(err) = mpu.calibrate_at_rest::<_, [f32; 3]>(&mut delay) {
                log::error!("calibrate_at_rest {:?}", err);
            }
            MPU.replace(mpu);
            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer.start((sample_rate as u32).Hz()).ok();
            timer.listen(Event::Update);
            TIMER.replace(timer);
            NVIC::priority(Interrupt::TIM1_UP_TIM10, 8);
            NVIC::unmask(Interrupt::TIM1_UP_TIM10);
            log::info!("Initialize mpu9250 ok");
        }
        Err(err) => {
            panic!("{:?}", err);
        }
    }
}

pub fn mpu() -> Option<&'static mut MPU> {
    unsafe { MPU.as_mut() }
}
#[export_name = "TIM1_UP_TIM10"]
unsafe fn timer_isr() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_interrupt(Event::Update);
    }
    xtask::sync::free(|_| {
        mbus::bus().publish_isr("/imu/raw", crate::message::Message::DataReady);
    })
}
