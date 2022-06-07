pub mod led;
pub mod mpu6050;
// pub mod mpu6050_dmp;

use shared_bus::{BusManager, BusManagerSimple, NullMutex};
use xtask::bsp::greenpill::hal::{
    gpio::{Alternate, OpenDrain, Pin},
    i2c::I2c,
    pac,
    pac::I2C1,
    prelude::*,
};
use xtask::bsp::greenpill::stdout;

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

static mut I2C1: Option<I2CBUS> = None;

pub unsafe fn init() {
    if let Some(dp) = pac::Peripherals::take() {
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        match dp
            .USART1
            .tx(gpioa.pa9.into_alternate(), 115200.bps(), &clocks)
        {
            Ok(tx) => {
                stdout::use_tx1(tx);
            }
            Err(err) => {
                panic!("{:?}", err);
            }
        }

        led::init(gpioc.pc13);
        let channels = (
            gpioa.pa0.into_alternate(),
            gpioa.pa1.into_alternate(),
            gpioa.pa2.into_alternate(),
            gpioa.pa3.into_alternate(),
        );
        let (ch1, ch2, ch3, ch4) = dp.TIM2.pwm_hz(channels, 128.kHz(), &clocks).split();

        //主旋翼
        let motor = super::bldc::Motor::new(ch1);
        //斜盘舵机
        let servo1 = super::servo::Servo::new(ch2);
        let servo2 = super::servo::Servo::new(ch3);
        let servo3 = super::servo::Servo::new(ch4);
        //todo 锁尾舵/尾旋翼

        let scl = gpiob
            .pb8
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();
        let sda = gpiob
            .pb9
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();
        I2C1.replace(BusManagerSimple::new(dp.I2C1.i2c(
            (scl, sda),
            400.kHz(),
            &clocks,
        )));
        if let Some(bus) = I2C1.as_ref() {
            let i2c1 = bus.acquire_i2c();
            //陀螺仪
            mpu6050::init(dp.TIM1, i2c1, &clocks);
            //mpu6050_dmp::init(dp.TIM1, i2c1, &clocks);
        }
    }
}
