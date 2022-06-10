use crate::mbus;
use embedded_hal::blocking::i2c::{self, *};
use mpu9250::*;

use shared_bus::{I2cProxy, NullMutex};
#[cfg(feature = "stm32f401ccu6")]
use xtask::bsp::greenpill::hal::pac::I2C1;
#[cfg(feature = "stm32f427vit6")]
use xtask::bsp::greenpill::hal::pac::I2C2;

use xtask::bsp::greenpill::hal::timer::CounterHz;
use xtask::{
    arch::cortex_m::peripheral::NVIC,
    bsp::greenpill::hal::{
        gpio::{Alternate, OpenDrain, Pin},
        i2c::I2c,
        pac::{interrupt, Interrupt, TIM1},
        prelude::*,
        rcc::Clocks,
        timer::{Event, Timer1},
    },
};

#[cfg(feature = "stm32f427vit6")]
static mut MPU: Option<
    Mpu9250<
        I2cDevice<
            I2cProxy<
                'static,
                NullMutex<
                    I2c<
                        I2C2,
                        (
                            Pin<'B', 10, Alternate<4, OpenDrain>>,
                            Pin<'B', 11, Alternate<4, OpenDrain>>,
                        ),
                    >,
                >,
            >,
        >,
        Marg,
    >,
> = None;
static mut TIMER: Option<CounterHz<TIM1>> = None;

#[cfg(feature = "stm32f427vit6")]
pub(crate) unsafe fn init_427(
    tim: TIM1,
    i2c: I2cProxy<
        'static,
        NullMutex<
            I2c<
                I2C2,
                (
                    Pin<'B', 10, Alternate<4, OpenDrain>>,
                    Pin<'B', 11, Alternate<4, OpenDrain>>,
                ),
            >,
        >,
    >,
    clocks: &Clocks,
) {
    use xtask::Delay;

    log::info!("Initialize mpu9250");
    let mut delay = Delay::new();
    match Mpu9250::marg_default(i2c, &mut delay) {
        Ok(mut mpu) => {
            let who_am_i = mpu.who_am_i().expect("could not read WHO_AM_I");
            let mag_who_am_i = mpu
                .ak8963_who_am_i()
                .expect("could not read magnetometer's WHO_AM_I");
            log::info!("WHO_AM_I: 0x{:x}", who_am_i);
            log::info!("AK8963 WHO_AM_I: 0x{:x}", mag_who_am_i);
            mpu.calibrate_at_rest::<_, [f32; 3]>(&mut delay).ok();

            MPU.replace(mpu);
            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer.start(1000.Hz()).ok();
            timer.listen(Event::Update);
            TIMER.replace(timer);
            NVIC::unmask(Interrupt::TIM1_UP_TIM10);
            log::info!("Initialize mpu9250 ok");
        }
        Err(err) => {
            log::error!("Initialize mpu9250 error {:?}", err);
        }
    }
}

// #[interrupt]
unsafe fn TIM1_UP_TIM10() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_interrupt(Event::Update);
    }
    xtask::sync::free(|_| {
        if let Some(mpu) = MPU.as_mut() {
            if let Ok(all) = mpu.all::<[f32; 3]>() {
                log::info!("accel {:?}", all.accel);
                log::info!("gyro {:?}", all.gyro);
                // log::info!("mag {:?}", all.mag);
                log::info!("temp {:?}", all.temp);
                // all.accel
            }
            // match mpu.accel_gyro() {
            //     Ok(mut data) => {
            //         mpu.update_quaternion(&mut data);
            //         mbus::mbus().publish_isr("/imu", crate::message::Message::ImuData(data));
            //     }

            //     Err(err) => {
            //         log::error!("mpu6050 error {:?}", err);
            //     }
            // }
        }
    })
}
