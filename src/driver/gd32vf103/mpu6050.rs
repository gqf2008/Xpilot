use mpu6050::Mpu6050;
use xtask::bsp::longan_nano::hal::{
    eclic::*,
    exti::{Exti, ExtiLine, TriggerEdge},
    gpio::{
        gpiob::{PB0, PB10, PB11},
        Alternate, Floating, Input, OpenDrain,
    },
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::{Interrupt, ECLIC, EXTI, I2C1},
    rcu::Rcu,
    time::*,
};
use xtask::time::Delay;

use crate::mbus;

pub type MPU = Mpu6050<BlockingI2c<I2C1, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>>;

static mut MPU: Option<MPU> = None;
static mut EXTILINE: Option<ExtiLine> = None;

pub(crate) unsafe fn init(
    exti: EXTI,
    pins: (
        PB0<Input<Floating>>,
        PB10<Input<Floating>>,
        PB11<Input<Floating>>,
    ),
    rcu: &mut Rcu,
    i2c1: I2C1,
) {
    let extiline = ExtiLine::from_gpio_line(pins.0.pin_number()).unwrap();
    let mut exti = Exti::new(exti);
    exti.listen(extiline, TriggerEdge::Falling);
    EXTILINE.replace(extiline);
    let scl = pins.1.into_alternate_open_drain();
    let sda = pins.2.into_alternate_open_drain();
    let i2c = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        rcu,
        10000,
        10,
        10000,
        10000,
    );
    let mut mpu = Mpu6050::new_with_sens(
        i2c,
        mpu6050::device::AccelRange::G2,
        mpu6050::device::GyroRange::D2000,
    );
    mpu.init(&mut Delay::new()).ok();
    mpu.set_accel_x_self_test(true).ok();
    mpu.set_accel_y_self_test(true).ok();
    mpu.set_accel_z_self_test(true).ok();
    MPU.replace(mpu);
    ECLIC::setup(
        Interrupt::EXTI_LINE0,
        TriggerType::FallingEdge,
        Level::L3,
        Priority::P15,
    );

    ECLIC::unmask(Interrupt::EXTI_LINE0);
}

#[export_name = "EXTI_LINE0"]
unsafe fn exti_line0() {
    if let Some(mpu) = MPU.as_mut() {
        if let Ok(gyro) = mpu.get_gyro() {
            if let Ok(acc) = mpu.get_acc() {
                let (gx, gy, gz) = (gyro.x, gyro.y, gyro.z);
                let (ax, ay, az) = (acc.x, acc.y, acc.z);
                mbus::mbus().publish_isr(
                    "/imu6050",
                    crate::message::Message::Imu6050 {
                        gx,
                        gy,
                        gz,
                        ax,
                        ay,
                        az,
                    },
                )
            }
        }
        let line = EXTILINE.unwrap();
        if Exti::is_pending(line) {
            Exti::clear(line);
        }
    }
}
