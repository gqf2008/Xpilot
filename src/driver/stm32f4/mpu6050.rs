use crate::driver::mpu6050::*;
use crate::mbus;
use ahrs::{Ahrs, Madgwick};
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

#[cfg(feature = "stm32f401ccu6")]
pub type MPU = Mpu6050<
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
pub type MPU = Mpu6050<
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
>;

static mut MPU: Option<MPU> = None;
static mut TIMER: Option<CounterHz<TIM1>> = None;
static mut MADGWICK: Option<Madgwick<f32>> = None;

#[cfg(feature = "stm32f401ccu6")]
pub(crate) unsafe fn init_401(
    tim: TIM1,
    i2c: I2cProxy<
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
    clocks: &Clocks,
) {
    log::info!("Initialize mpu6050");
    match Mpu6050::new(i2c).build() {
        Ok(mut mpu) => {
            mpu.cal_gyro_offset().ok();
            MPU.replace(mpu);
            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer
                .start((crate::driver::mpu6050::SAMPLE_RATE as u32).Hz())
                .ok();
            timer.listen(Event::Update);
            TIMER.replace(timer);
            NVIC::unmask(Interrupt::TIM1_UP_TIM10);
            log::info!("Initialize mpu6050 ok");
        }
        Err(err) => {
            log::error!("Initialize mpu6050 error {:?}", err);
        }
    }
}

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
    log::info!("Initialize mpu6050");
    let sample_rate = 1000;
    MADGWICK.replace(Madgwick::new(1.0 / sample_rate as f32, 0.1));
    match Mpu6050::new(i2c).with_sample_rate(sample_rate).build() {
        Ok(mpu) => {
            MPU.replace(mpu);
            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer.start((sample_rate as u32).Hz()).ok();
            timer.listen(Event::Update);
            TIMER.replace(timer);
            NVIC::unmask(Interrupt::TIM1_UP_TIM10);
            log::info!("Initialize mpu6050 ok");
        }
        Err(err) => {
            log::error!("Initialize mpu6050 error {:?}", err);
        }
    }
}

#[interrupt]
unsafe fn TIM1_UP_TIM10() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_interrupt(Event::Update);
    }
    let ahrs = MADGWICK.as_mut().unwrap();
    xtask::sync::free(|_| {
        if let Some(mpu) = MPU.as_mut() {
            match mpu.accel_gyro() {
                Ok(data) => {
                    let gyro = data.gyro.unwrap();
                    let acc = data.accel.unwrap();
                    if let Ok(quat) = ahrs.update_imu(&gyro, &acc) {
                        let data = data.quate(*quat);
                        mbus::mbus().publish_isr("/imu", crate::message::Message::ImuData(data));
                    }
                }

                Err(err) => {
                    log::error!("mpu6050 error {:?}", err);
                }
            }
        }
    })
}
