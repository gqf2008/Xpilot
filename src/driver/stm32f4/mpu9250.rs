use crate::driver::{Accel, Compass, Gyro, ImuData};
use crate::mbus;
use ahrs::{Ahrs, Madgwick};
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

#[cfg(any(feature = "stm32f427vit6", feature = "stm32f401ccu6"))]
static mut MPU: Option<
    Mpu9250<
        SpiDevice<
            SpiProxy<
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
    >,
> = None;
static mut TIMER: Option<CounterHz<TIM1>> = None;
static mut MADGWICK: Option<Madgwick<f32>> = None;

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
    MADGWICK.replace(Madgwick::new(1.0 / sample_rate as f32, 0.1));
    let mut delay = Delay::new();

    match Mpu9250::marg_default(spi, ncs, &mut delay) {
        Ok(mut mpu) => {
            let who_am_i = mpu.who_am_i().expect("could not read WHO_AM_I");
            let mag_who_am_i = mpu
                .ak8963_who_am_i()
                .expect("could not read magnetometer's WHO_AM_I");
            log::info!("WHO_AM_I: 0x{:x}", who_am_i);
            log::info!("AK8963 WHO_AM_I: 0x{:x}", mag_who_am_i);

            if let Err(err) = mpu.calibrate_at_rest::<_, [f32; 3]>(&mut delay) {
                log::info!("calibrate_at_rest {:?}", err);
            }
            MPU.replace(mpu);
            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer.start((sample_rate as u32).Hz()).ok();
            timer.listen(Event::Update);
            TIMER.replace(timer);
            NVIC::unmask(Interrupt::TIM1_UP_TIM10);
            log::info!("Initialize mpu9250 ok");
        }
        Err(err) => {
            panic!("{:?}", err);
        }
    }
}

#[export_name = "TIM1_UP_TIM10"]
unsafe fn timer_isr() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_interrupt(Event::Update);
    }
    let ahrs = MADGWICK.as_mut().unwrap();
    xtask::sync::free(|_| {
        if let Some(mpu) = MPU.as_mut() {
            if let Ok(all) = mpu.all::<[f32; 3]>() {
                let acc = Accel::new(all.accel[0], all.accel[1], all.accel[2]);
                let gyro = Gyro::new(all.gyro[0], all.gyro[1], all.gyro[2]);
                let mag = Compass::new(all.mag[0], all.mag[1], all.mag[2]);
                let mut data = ImuData::default().accel(acc).gyro(gyro).compass(mag);
                if let Ok(quat) = ahrs.update(&gyro, &acc, &mag) {
                    data = data.quate(*quat)
                }
                mbus::mbus().publish_isr("/imu", crate::message::Message::ImuData(data));
            }
        }
    })
}
