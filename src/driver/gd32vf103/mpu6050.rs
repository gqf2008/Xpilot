use hal::{gpio::GpioExt, pac::Peripherals, prelude::*};
use mpu6050_dmp::{address::Address, sensor::Mpu6050};
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
    let dp = Peripherals::steal();
    let rcu = super::rcu::rcu();
    let gpiob = dp.GPIOB.split(rcu);
    let mut afio = dp.AFIO.constrain(rcu);
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
        rcu,
        1000,
        10,
        1000,
        1000,
    );
    let mpu = Mpu6050::new(i2c, Address::default()).unwrap();
    MPU.replace(mpu);
}

pub(crate) fn mpu() -> &'static mut MPU {
    unsafe { MPU.as_mut().unwrap() }
}
