use crate::driver::ImuData;
use crate::filter::dither::DitherFilter;
use crate::mbus;
use ahrs::{Ahrs, Madgwick};
use icm20689::{Builder, SpiInterface, ICM20689};
use nalgebra::Vector3;
use shared_bus::{NullMutex, SpiProxy};
#[cfg(feature = "stm32f401ccu6")]
use xtask::bsp::greenpill::hal::pac::I2C1;
#[cfg(feature = "stm32f427vit6")]
use xtask::bsp::greenpill::hal::pac::SPI1;

use xtask::bsp::greenpill::hal::spi::Spi;
use xtask::bsp::greenpill::hal::timer::CounterHz;
use xtask::bsp::greenpill::hal::{
    gpio::{Output, PushPull},
    spi::{Master, TransferModeNormal},
};

use xtask::{
    arch::cortex_m::peripheral::NVIC,
    bsp::greenpill::hal::{
        gpio::{Alternate, Pin},
        pac::{Interrupt, TIM1},
        prelude::*,
        rcc::Clocks,
        timer::{Event, Timer1},
    },
};

#[cfg(feature = "stm32f401ccu6")]
pub type MPU = Icm20602<
    I2cProxy<
        'static,
        NullMutex<
            I2c<
                I2C1,
                (
                    Pin<'B', 8, Alternate<4, OpenDrain>>,
                    Pin<'B', 9, Alternate<4, OpenDrain>>,
                ),
            >,
        >,
    >,
>;

#[cfg(feature = "stm32f427vit6")]
pub type MPU = ICM20689<
    SpiInterface<
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
>;

static mut MPU: Option<MPU> = None;
static mut TIMER: Option<CounterHz<TIM1>> = None;

#[cfg(feature = "stm32f427vit6")]
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
    use xtask::Delay;

    log::info!("Initialize Icm20602");
    let sample_rate = 100;
    MADGWICK.replace(Madgwick::new(1.0 / sample_rate as f32, 0.1));
    let mut mpu = Builder::new_spi(spi, ncs);
    let mut delay = Delay::new();
    if let Err(err) = mpu.setup(&mut delay) {
        log::info!("Initialize Icm20602 {:?}", err);
    } else {
        MPU.replace(mpu);
        let mut timer = Timer1::new(tim, clocks).counter_hz();
        timer.start((sample_rate as u32).Hz()).ok();
        timer.listen(Event::Update);
        TIMER.replace(timer);
        NVIC::unmask(Interrupt::TIM1_UP_TIM10);
        log::info!("Initialize Icm20602 ok");
    }
}

#[export_name = "TIM1_UP_TIM10"]
unsafe fn timer_isr() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_interrupt(Event::Update);
    }
    xtask::sync::free(|_| {
        if let Some(mpu) = MPU.as_mut() {
            match mpu.get_scaled_accel() {
                Ok(acc) => match mpu.get_scaled_gyro() {
                    Ok(gyro) => {
                        let acc = Vector3::new(acc[0], acc[1], acc[2]);
                        let gyro = Vector3::new(gyro[0], gyro[1], gyro[2]);
                        let data = ImuData::default().gyro(gyro).accel(acc);
                        mbus::mbus().publish_isr("/imu", crate::message::Message::ImuData(data));
                    }
                    Err(err) => {
                        log::error!("Icm20602 error {:?}", err);
                    }
                },
                Err(err) => {
                    log::error!("Icm20602 error {:?}", err);
                }
            }
        }
    })
}
