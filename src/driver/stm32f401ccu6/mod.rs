pub mod led;
pub mod mpu6050_dmp;

use hal::{pac, prelude::*};
use xtask::bsp::greenpill::{hal, stdout};

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
        //十字盘舵机
        let servo1 = super::servo::Servo::new(ch2);
        let servo2 = super::servo::Servo::new(ch3);
        let servo3 = super::servo::Servo::new(ch4);
        //todo 尾舵/尾马达

        //陀螺仪
        mpu6050_dmp::init(dp.TIM1, dp.I2C1, (gpiob.pb8, gpiob.pb9), &clocks);
    }
}
