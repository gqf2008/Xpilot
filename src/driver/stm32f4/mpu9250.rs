use super::nvic::NVICExt;
use crate::driver::{Accel, Compass, Gyro, ImuData};
use crate::mbus;
use crate::message::Message;
use core::cell::RefCell;

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
    arch::cortex_m::interrupt,
    arch::cortex_m::interrupt::Mutex,
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
static mut MPU: Mutex<RefCell<Option<MPU>>> = Mutex::new(RefCell::new(None));
static mut TIMER: Mutex<RefCell<Option<CounterHz<TIM1>>>> = Mutex::new(RefCell::new(None));

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

            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer.start((sample_rate as u32).Hz()).ok();
            timer.listen(Event::Update);

            interrupt::free(|cs| *MPU.borrow(cs).borrow_mut() = Some(mpu));
            interrupt::free(|cs| *TIMER.borrow(cs).borrow_mut() = Some(timer));
            NVIC::priority(Interrupt::TIM1_UP_TIM10, 0x01);
            NVIC::unmask(Interrupt::TIM1_UP_TIM10);
            log::info!("Initialize mpu9250 ok");
        }
        Err(err) => {
            panic!("{:?}", err);
        }
    }
}

// pub fn mpu() -> Option<&'static mut MPU> {
//     unsafe { MPU.as_mut() }
// }
#[export_name = "TIM1_UP_TIM10"]
unsafe fn timer_isr() {
    static mut TIM: Option<CounterHz<TIM1>> = None;
    static mut MPU9250: Option<MPU> = None;
    let timer =
        TIM.get_or_insert_with(|| interrupt::free(|cs| TIMER.borrow(cs).replace(None).unwrap()));

    timer.clear_interrupt(Event::Update);
    let mpu =
        MPU9250.get_or_insert_with(|| interrupt::free(|cs| MPU.borrow(cs).replace(None).unwrap()));
    if let Ok(all) = mpu.all::<[f32; 3]>() {
        let acc = Accel::new(all.accel[0], all.accel[1], all.accel[2]);
        let gyro = Gyro::new(all.gyro[0], all.gyro[1], all.gyro[2]);
        let mag = Compass::new(all.mag[0], all.mag[1], all.mag[2]);
        let data = ImuData::default().accel(acc).gyro(gyro).compass(mag);
        xtask::sync::free(|_| {
            mbus::bus().publish_isr("/imu/raw", Message::ImuData(data));
        })
    }
}
