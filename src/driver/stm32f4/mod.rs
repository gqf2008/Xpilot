#[cfg(feature = "icm20602")]
pub mod icm20602;
pub mod led;
#[cfg(feature = "mpu6050")]
pub mod mpu6050;
#[cfg(feature = "mpu9250")]
pub mod mpu9250;
pub mod nvic;
pub mod telem;

use shared_bus::{BusManager, BusManagerSimple, NullMutex};
use xtask::bsp::greenpill::hal::{
    dma::StreamsTuple,
    gpio::{Alternate, OpenDrain, Pin},
    i2c::I2c,
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};
use xtask::bsp::greenpill::hal::{
    flash::FlashExt,
    gpio::PushPull,
    serial::config::{Config, DmaConfig},
    spi::{Master, TransferModeNormal},
};

#[cfg(feature = "stm32f401ccu6")]
use xtask::bsp::greenpill::hal::pac::I2C1;
#[cfg(feature = "stm32f427vit6")]
use xtask::bsp::greenpill::hal::pac::I2C2;
#[cfg(any(feature = "stm32f427vit6", feature = "stm32f401ccu6"))]
use xtask::bsp::greenpill::hal::pac::SPI1;

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

#[cfg(any(feature = "stm32f427vit6", feature = "stm32f401ccu6"))]
pub type SPIBUS = BusManager<
    NullMutex<
        Spi<
            SPI1,
            (
                Pin<'A', 5, Alternate<5, PushPull>>,
                Pin<'A', 6, Alternate<5, PushPull>>,
                Pin<'A', 7, Alternate<5, PushPull>>,
            ),
            TransferModeNormal,
            Master,
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
static mut SPI: Option<SPIBUS> = None;

pub unsafe fn init() {
    use xtask::chip::CPU_CLOCK_HZ;
    use xtask::chip::SYSTICK_CLOCK_HZ;
    use xtask::chip::TICK_CLOCK_HZ;
    log::info!("CPU_CLOCK {}Hz", CPU_CLOCK_HZ);
    log::info!("SYSTICK_CLOCK {}Hz", SYSTICK_CLOCK_HZ);
    log::info!("OSTICK_CLOCK {}Hz", TICK_CLOCK_HZ);

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

        match dp.USART1.serial(
            (gpioa.pa9.into_alternate(), gpioa.pa10.into_alternate()),
            Config::default().baudrate(115200.bps()),
            &clocks,
        ) {
            Ok(serial) => {
                let (tx, rx) = serial.split();
                telem::init(rx, tx);
            }
            Err(err) => {
                panic!("{:?}", err);
            }
        }

        log::info!(
            "Flash Address:0x{:x},Length={}, DualBank:{}",
            dp.FLASH.address(),
            dp.FLASH.len(),
            dp.FLASH.dual_bank()
        );
        #[cfg(feature = "stm32f401ccu6")]
        led::init(gpioc.pc13);
        #[cfg(feature = "stm32f427vit6")]
        led::init(gpioc.pc6, gpioc.pc7, gpioa.pa8);
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

        #[cfg(any(feature = "stm32f427vit6", feature = "stm32f401ccu6"))]
        {
            let sck = gpioa.pa5.into_alternate();
            let miso = gpioa.pa6.into_alternate();
            let mosi = gpioa.pa7.into_alternate().internal_pull_up(true);

            let mode = Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            };

            let spi = Spi::new(dp.SPI1, (sck, miso, mosi), mode, 4.MHz(), &clocks);
            let bus = BusManagerSimple::new(spi);
            SPI.replace(bus);
        }
        if let Some(bus) = SPI.as_ref() {
            let ncs = gpioa.pa4.into_push_pull_output();
            let spi = bus.acquire_spi();
            #[cfg(feature = "mpu9250")]
            {
                mpu9250::init(dp.TIM1, spi, ncs, &clocks);
            }
            #[cfg(feature = "icm20602")]
            {
                #[cfg(feature = "stm32f401ccu6")]
                icm20602::init(dp.TIM1, i2c, &clocks);
                #[cfg(feature = "stm32f427vit6")]
                icm20602::init(dp.TIM1, spi, ncs, &clocks)
            }
        }
        #[cfg(feature = "mpu6050")]
        if let Some(bus) = I2C.as_ref() {
            let i2c = bus.acquire_i2c();
            #[cfg(feature = "stm32f401ccu6")]
            mpu6050::init(dp.TIM1, i2c, &clocks);
            #[cfg(feature = "stm32f427vit6")]
            mpu6050::init(dp.TIM1, i2c, &clocks);
        }
    }
}
