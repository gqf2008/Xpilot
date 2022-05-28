use mpu6050_dmp::{address::Address, sensor::Mpu6050};
use xtask::bsp::longan_nano::hal::{
    gpio::{
        gpiob::{PB10, PB11},
        Alternate, Floating, Input, OpenDrain,
    },
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::I2C1,
    rcu::Rcu,
    time::*,
};
use xtask::time::Delay;

pub type MPU = Mpu6050<BlockingI2c<I2C1, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>>;

static mut MPU: Option<MPU> = None;

pub(crate) unsafe fn init(
    pins: (PB10<Input<Floating>>, PB11<Input<Floating>>),
    rcu: &mut Rcu,
    i2c1: I2C1,
) {
    let scl = pins.0.into_alternate_open_drain();
    let sda = pins.1.into_alternate_open_drain();

    let i2c = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        rcu,
        1000,
        10,
        1000,
        1000,
    );
    // let mut mpu = Mpu6050::new(i2c, Address::default()).unwrap();
    // mpu.initialize_dmp(&mut delay).ok();
    match Mpu6050::new(i2c, Address::default()) {
        Ok(mut mpu) => match mpu.initialize_dmp(&mut Delay::new()) {
            Ok(_) => {
                MPU.replace(mpu);

                log::info!("mpu6050 init dmp ok");
            }
            Err(err) => {
                log::error!("mpu6050 init dmp error {:?}", err);
            }
        },
        Err(err) => {
            log::error!("mpu6050 init error {:?}", err);
        }
    }
}

pub(crate) fn mpu() -> Option<&'static mut MPU> {
    unsafe { MPU.as_mut() }
}
