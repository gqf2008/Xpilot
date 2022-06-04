use crate::mbus;
use mpu6050_dmp::{
    address::Address, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};
use shared_bus::{BusManager, BusManagerSimple, I2cProxy, NullMutex};
use xtask::bsp::greenpill::hal::timer::CounterHz;
use xtask::time::Delay;
use xtask::{
    arch::cortex_m::peripheral::NVIC,
    bsp::greenpill::hal::{
        gpio::{Alternate, OpenDrain, Pin, PB8, PB9},
        i2c::I2c,
        pac::{interrupt, Interrupt, I2C1, TIM1},
        prelude::*,
        rcc::Clocks,
        timer::{Event, Timer1},
    },
};

pub type I2CBUS = BusManager<
    NullMutex<
        I2c<
            I2C1,
            (
                Pin<'B', 8, Alternate<4, OpenDrain>>,
                Pin<'B', 9, Alternate<4, OpenDrain>>,
            ),
        >,
    >,
>;

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

static mut I2C: Option<I2CBUS> = None;
static mut MPU: Option<MPU> = None;
static mut TIMER: Option<CounterHz<TIM1>> = None;

pub(crate) unsafe fn init(tim: TIM1, i2c: I2C1, pins: (PB8, PB9), clocks: &Clocks) {
    log::info!("init mpu6050_dmp");
    let scl = pins
        .0
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();
    let sda = pins
        .1
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();
    I2C.replace(BusManagerSimple::new(i2c.i2c(
        (scl, sda),
        400.kHz(),
        clocks,
    )));
    let mut delay = Delay::new();
    match Mpu6050::new(I2C.as_ref().unwrap().acquire_i2c(), Address::default()) {
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
                    crate::message::Message::Acc {
                        x: acc.x() as f32 / 16384.0,
                        y: acc.y() as f32 / 16384.0,
                        z: acc.z() as f32 / 16384.0,
                    },
                );
            }
            Err(err) => {
                log::error!("mpu6050 error {:?}", err);
            }
        }
        match mpu.gyro() {
            Ok(gyro) => {
                mbus::mbus().publish_isr(
                    "/imu",
                    crate::message::Message::Gyro {
                        x: gyro.x() as f32 / 16.4 * 57.3,
                        y: gyro.y() as f32 / 16.4 * 57.3,
                        z: gyro.z() as f32 / 16.4 * 57.3,
                    },
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
                            let quat = Quaternion::from_bytes(&buf[..16]).unwrap();
                            let ypr = YawPitchRoll::from(quat);
                            mbus::mbus().publish_isr(
                                "/imu",
                                crate::message::Message::Quaternion {
                                    w: quat.w,
                                    x: quat.x,
                                    y: quat.y,
                                    z: quat.z,
                                },
                            );
                            mbus::mbus().publish_isr(
                                "/imu",
                                crate::message::Message::YawPitchRoll {
                                    yaw: ypr.yaw,
                                    pitch: ypr.pitch,
                                    roll: ypr.roll,
                                },
                            );
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
