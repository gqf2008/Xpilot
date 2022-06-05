use crate::driver::mpu6050::*;
use crate::mbus;

use shared_bus::{I2cProxy, NullMutex};
use xtask::bsp::greenpill::hal::timer::CounterHz;
use xtask::{
    arch::cortex_m::peripheral::NVIC,
    bsp::greenpill::hal::{
        gpio::{Alternate, OpenDrain, Pin},
        i2c::I2c,
        pac::{interrupt, Interrupt, I2C1, TIM1},
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
                I2C1,
                (
                    Pin<'B', 8, Alternate<4, OpenDrain>>,
                    Pin<'B', 9, Alternate<4, OpenDrain>>,
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
    log::info!("init mpu6050");
    match Mpu6050::new(i2c).build() {
        Ok(mut mpu) => {
            mpu.cal_gyro_offset().ok();
            MPU.replace(mpu);
            let mut timer = Timer1::new(tim, clocks).counter_hz();
            timer.start(100.Hz()).ok();
            timer.listen(Event::Update);
            TIMER.replace(timer);
            NVIC::unmask(Interrupt::TIM1_UP_TIM10);
            log::info!("init mpu6050 ok");
        }
        Err(err) => {
            log::error!("mpu6050 init error {:?}", err);
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
                mbus::mbus().publish_isr(
                    "/imu",
                    crate::message::Message::Acc {
                        x: data.accel.x,
                        y: data.accel.y,
                        z: data.accel.z,
                    },
                );
                mbus::mbus().publish_isr(
                    "/imu",
                    crate::message::Message::Gyro {
                        x: data.gyro.x,
                        y: data.gyro.y,
                        z: data.gyro.z,
                    },
                );
                let (yaw, pitch, roll) = crate::driver::yaw_pitch_roll(
                    data.gyro.x,
                    data.gyro.y,
                    data.gyro.z,
                    data.accel.x,
                    data.accel.y,
                    data.accel.z,
                );
                mbus::mbus().publish_isr(
                    "/imu",
                    crate::message::Message::YawPitchRoll { yaw, pitch, roll },
                );
            }

            Err(err) => {
                log::error!("mpu6050 error {:?}", err);
            }
        }
    }
}
