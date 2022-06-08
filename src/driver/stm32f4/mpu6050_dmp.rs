use crate::mbus;
use mpu6050_dmp::{
    address::Address, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};
use shared_bus::{I2cProxy, NullMutex};
#[cfg(feature = "stm32f401ccu6")]
use xtask::bsp::greenpill::hal::pac::I2C1;
#[cfg(feature = "stm32f427vit6")]
use xtask::bsp::greenpill::hal::pac::I2C2;
use xtask::bsp::greenpill::hal::timer::CounterHz;
use xtask::time::Delay;
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

#[cfg(feature = "stm32f27vit6")]
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
    log::info!("init mpu6050_dmp");

    let mut delay = Delay::new();
    match Mpu6050::new(i2c, Address::default()) {
        Ok(mut mpu) => match mpu.initialize_dmp(&mut delay) {
            Ok(_) => {
                mpu.calibrate_accel(150, &mut delay).ok();
                mpu.calibrate_gyro(150, &mut delay).ok();
                MPU.replace(mpu);
                let mut timer = Timer1::new(tim, clocks).counter_hz();
                timer.start(100.Hz()).ok();
                timer.listen(Event::Update);
                TIMER.replace(timer);
                NVIC::unmask(Interrupt::TIM1_UP_TIM10);
                log::info!("init mpu6050_dmp ok");
            }
            Err(err) => {
                log::error!("mpu6050 dmp init dmp error {:?}", err);
            }
        },
        Err(err) => {
            log::error!("mpu6050 dmp init error {:?}", err);
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
    log::info!("init mpu6050_dmp");

    let mut delay = Delay::new();
    match Mpu6050::new(i2c, Address::default()) {
        Ok(mut mpu) => match mpu.initialize_dmp(&mut delay) {
            Ok(_) => {
                mpu.calibrate_accel(150, &mut delay).ok();
                mpu.calibrate_gyro(150, &mut delay).ok();
                MPU.replace(mpu);
                let mut timer = Timer1::new(tim, clocks).counter_hz();
                timer.start(100.Hz()).ok();
                timer.listen(Event::Update);
                TIMER.replace(timer);
                NVIC::unmask(Interrupt::TIM1_UP_TIM10);
                log::info!("init mpu6050_dmp ok");
            }
            Err(err) => {
                log::error!("mpu6050 dmp init dmp error {:?}", err);
            }
        },
        Err(err) => {
            log::error!("mpu6050 dmp init error {:?}", err);
        }
    }
}

// pub(crate) fn mpu() -> Option<&'static mut MPU> {
//     unsafe { MPU.as_mut() }
// }

#[interrupt]
unsafe fn TIM1_UP_TIM10() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_interrupt(Event::Update);
    }
    if let Some(mpu) = MPU.as_mut() {
        match mpu.accel() {
            Ok(acc) => {
                mbus::mbus().publish_isr(
                    "/imu",
                    crate::message::Message::Accel(crate::driver::Accel {
                        x: acc.x() as f32 / 16384.0,
                        y: acc.y() as f32 / 16384.0,
                        z: acc.z() as f32 / 16384.0,
                    }),
                );
            }
            Err(err) => {
                log::error!("mpu6050 error {:?}", err);
            }
        }
        match mpu.gyro() {
            Ok(gyro) => {
                const PI_180: f32 = core::f32::consts::PI / 180.0;
                mbus::mbus().publish_isr(
                    "/imu",
                    crate::message::Message::Gyro(crate::driver::Gyro {
                        x: gyro.x() as f32 * PI_180 / 16.4,
                        y: gyro.y() as f32 * PI_180 / 16.4,
                        z: gyro.z() as f32 * PI_180 / 16.4,
                    }),
                );
            }
            Err(err) => {
                log::error!("mpu6050 error {:?}", err);
            }
        }
        match mpu.get_fifo_count() {
            Ok(len) => {
                if len >= 28 {
                    let mut buf = [0; 28];
                    match mpu.read_fifo(&mut buf) {
                        Ok(buf) => {
                            if let Some(quat) = Quaternion::from_bytes(&buf[..16]) {
                                let quat = quat.normalize();
                                let ypr = YawPitchRoll::from(quat);
                                mbus::mbus().publish_isr(
                                    "/imu",
                                    crate::message::Message::Quaternion(
                                        crate::driver::Quaternion {
                                            w: quat.w,
                                            x: quat.x,
                                            y: quat.y,
                                            z: quat.z,
                                        },
                                    ),
                                );
                                mbus::mbus().publish_isr(
                                    "/imu",
                                    crate::message::Message::YawPitchRoll(
                                        crate::driver::EulerAngle {
                                            yaw: ypr.yaw * 57.29577,
                                            pitch: ypr.pitch * 57.29577,
                                            roll: ypr.roll * 57.29577,
                                        },
                                    ),
                                );
                            }
                        }
                        Err(err) => {
                            log::error!("mpu6050 error {:?}", err);
                        }
                    }
                }
            }
            Err(err) => {
                log::error!("mpu6050 error {:?}", err);
            }
        }
    }
}
