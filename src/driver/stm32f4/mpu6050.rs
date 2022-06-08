use crate::driver::{mpu6050::*, Accel, Gyro};
use crate::mbus;

use shared_bus::{I2cProxy, NullMutex};
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

pub(crate) unsafe fn init(
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
    match Mpu6050::new(i2c).with_sample_rate(1000).build() {
        Ok(mut mpu) => {
            mpu.cal_gyro_offset().ok();
            MPU.replace(mpu);
            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer.start(100.Hz()).ok();
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
    if let Some(mpu) = MPU.as_mut() {
        match mpu.accel_gyro() {
            Ok(data) => {
                mbus::mbus().publish_isr("/imu", crate::message::Message::Accel(data.accel));
                mbus::mbus().publish_isr("/imu", crate::message::Message::Gyro(data.gyro));
                mbus::mbus().publish_isr("/imu", crate::message::Message::ImuData(data));
                // let quate = data.to_quat();
                // mbus::mbus().publish_isr("/imu", crate::message::Message::Quaternion(quate));
                // mbus::mbus().publish_isr(
                //     "/imu",
                //     crate::message::Message::YawPitchRoll(quate.to_euler()),
                // );
            }

            Err(err) => {
                log::error!("mpu6050 error {:?}", err);
            }
        }
    }
}
