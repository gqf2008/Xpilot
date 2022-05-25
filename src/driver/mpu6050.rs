use super::PERIPHERAL;
use hal::{gpio::GpioExt, pac, prelude::*, rcu::RcuExt, signature};
use mpu6050_dmp::{
    address::Address, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};
use xtask::bsp::longan_nano::hal::{
    self,
    gpio::{
        gpiob::{PB6, PB7},
        Alternate, OpenDrain,
    },
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::I2C0,
};
pub type MPU = Mpu6050<BlockingI2c<I2C0, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>>;

static mut MPU: Option<MPU> = None;

pub(crate) unsafe fn init() {
    if let Some(dp) = &mut PERIPHERAL {
        let rcu = super::rcu::rcu();
        let gpiob = dp.GPIOB.split(rcu);
        let mut afio = dp.AFIO.constrain(&mut rcu);
        let scl = gpiob.pb6.into_alternate_open_drain();
        let sda = gpiob.pb7.into_alternate_open_drain();

        let i2c = BlockingI2c::i2c0(
            dp.I2C0,
            (scl, sda),
            &mut afio,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            &mut rcu,
            1000,
            10,
            1000,
            1000,
        );
        let mpu = Mpu6050::new(i2c, Address::default()).unwrap();
        MPU.replace(mpu);
    }
}

pub(crate) fn mpu() -> &'static mut MPU {
    unsafe { &mut MPU.unwrap() }
}
