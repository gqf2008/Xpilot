pub mod led;
#[cfg(not(feature = "dmp"))]
pub mod mpu6050;
#[cfg(feature = "dmp")]
pub mod mpu6050_dmp;

use shared_bus::{BusManager, BusManagerSimple, NullMutex};
use xtask::bsp::greenpill::hal::{
    gpio::{Alternate, OpenDrain, Pin},
    i2c::I2c,
    pac,
    prelude::*,
};

#[cfg(feature = "stm32f401ccu6")]
use xtask::bsp::greenpill::hal::pac::I2C1;
#[cfg(feature = "stm32f427vit6")]
use xtask::bsp::greenpill::hal::pac::I2C2;

use xtask::bsp::greenpill::stdout;

#[cfg(feature = "stm32f427vit6")]
pub type I2CBUS = BusManager<
    NullMutex<
        I2c<
            I2C2,
            (
                Pin<'B', 10, Alternate<4, OpenDrain>>,
                Pin<'B', 11, Alternate<4, OpenDrain>>,
            ),
        >,
    >,
>;

#[cfg(feature = "stm32f401ccu6")]
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
static mut I2C: Option<I2CBUS> = None;

pub unsafe fn init() {
    use xtask::chip::CPU_CLOCK_HZ;
    use xtask::chip::SYSTICK_CLOCK_HZ;
    use xtask::chip::TICK_CLOCK_HZ;
    log::info!("CPU_CLOCK {}Hz", CPU_CLOCK_HZ);
    log::info!("SYSTICK_CLOCK {}Hz", SYSTICK_CLOCK_HZ);
    log::info!("TICK_CLOCK {}Hz", TICK_CLOCK_HZ);
    if let Some(dp) = pac::Peripherals::take() {
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .hclk((CPU_CLOCK_HZ as u32).Hz())
            .sysclk((SYSTICK_CLOCK_HZ as u32).Hz())
            .freeze();
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        match dp
            .USART1
            .tx(gpioa.pa9.into_alternate(), 460800.bps(), &clocks)
        {
            Ok(tx) => {
                stdout::use_tx1(tx);
            }
            Err(err) => {
                panic!("{:?}", err);
            }
        }
        #[cfg(feature = "stm32f401ccu6")]
        led::init_401(gpioc.pc13);
        #[cfg(feature = "stm32f427vit6")]
        led::init_427(gpioc.pc6, gpioc.pc7, gpioa.pa8);
        let channels = (
            gpioa.pa0.into_alternate(),
            gpioa.pa1.into_alternate(),
            gpioa.pa2.into_alternate(),
            gpioa.pa3.into_alternate(),
        );
        let (ch1, ch2, ch3, ch4) = dp.TIM2.pwm_hz(channels, 128.kHz(), &clocks).split();

        log::info!("Initialize motor");
        //主旋翼
        let motor = super::bldc::Motor::new(ch1);
        log::info!("Initialize servos");
        //斜盘舵机
        let servo1 = super::servo::Servo::new(ch2);
        let servo2 = super::servo::Servo::new(ch3);
        let servo3 = super::servo::Servo::new(ch4);
        //todo 锁尾舵机/尾旋翼
        #[cfg(feature = "stm32f401ccu6")]
        {
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
            I2C.replace(BusManagerSimple::new(dp.I2C1.i2c(
                (scl, sda),
                400.kHz(),
                &clocks,
            )));
        }

        #[cfg(feature = "stm32f427vit6")]
        {
            let scl = gpiob
                .pb10
                .into_alternate()
                .internal_pull_up(true)
                .set_open_drain();
            let sda = gpiob
                .pb11
                .into_alternate()
                .internal_pull_up(true)
                .set_open_drain();
            I2C.replace(BusManagerSimple::new(dp.I2C2.i2c(
                (scl, sda),
                400.kHz(),
                &clocks,
            )));
        }
        if let Some(bus) = I2C.as_ref() {
            let i2c = bus.acquire_i2c();
            //陀螺仪
            #[cfg(not(feature = "dmp"))]
            {
                #[cfg(feature = "stm32f401ccu6")]
                mpu6050::init_401(dp.TIM1, i2c, &clocks);
                #[cfg(feature = "stm32f427vit6")]
                mpu6050::init_427(dp.TIM1, i2c, &clocks);
            }
            #[cfg(feature = "dmp")]
            {
                #[cfg(feature = "stm32f401ccu6")]
                mpu6050_dmp::init_401(dp.TIM1, i2c, &clocks);
                #[cfg(feature = "stm32f427vit6")]
                mpu6050_dmp::init_427(dp.TIM1, i2c, &clocks);
            }
        }
    }
}
