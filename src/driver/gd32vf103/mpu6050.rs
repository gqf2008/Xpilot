use crate::mbus;
use embedded_hal::timer::CountDown;
use libm::*;
use mpu6050::device::AccelRange;
use mpu6050::device::GyroRange;
use mpu6050::device::ACCEL_HPF;
use mpu6050::Mpu6050;
use xtask::bsp::longan_nano::hal::gpio::gpiob::PB5;
use xtask::bsp::longan_nano::hal::pac::TIMER0;
use xtask::bsp::longan_nano::hal::timer::{Event, Timer};
use xtask::bsp::longan_nano::hal::{
    eclic::*,
    exti::{Exti, ExtiLine, TriggerEdge},
    gpio::{
        gpiob::{PB10, PB11},
        Alternate, Floating, Input, OpenDrain,
    },
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::{Interrupt, ECLIC, EXTI, I2C1},
    rcu::Rcu,
    time::*,
};
use xtask::time::Delay;

pub type MPU = Mpu6050<BlockingI2c<I2C1, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>>;

static mut MPU: Option<MPU> = None;
static mut TIMER: Option<Timer<TIMER0>> = None;

pub(crate) unsafe fn init(
    timer: TIMER0,
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
    mpu.set_accel_range(AccelRange::G2).ok();
    mpu.set_gyro_range(GyroRange::D2000).ok();
    mpu.set_accel_hpf(ACCEL_HPF::_2P5).ok();
    mpu.set_accel_x_self_test(true).ok();
    mpu.set_accel_y_self_test(true).ok();
    mpu.set_accel_z_self_test(true).ok();
    MPU.replace(mpu);

    let mut timer = Timer::timer0(timer, 100.hz(), rcu);
    timer.start(100.hz());
    timer.listen(Event::Update);
    TIMER.replace(timer);
    ECLIC::setup(
        Interrupt::TIMER0_UP,
        TriggerType::Level,
        Level::L3,
        Priority::P8,
    );
    ECLIC::unmask(Interrupt::TIMER0_UP);
}

unsafe fn clear_update_interrupt_flag() {
    if let Some(timer) = TIMER.as_mut() {
        timer.clear_update_interrupt_flag();
    }
}

#[export_name = "TIMER0_UP"]
unsafe fn timer0_isr() {
    clear_update_interrupt_flag();
    if let Some(mpu) = MPU.as_mut() {
        if let Ok(gyro) = mpu.get_gyro() {
            if let Ok(acc) = mpu.get_acc() {
                let (yaw, mut pitch, mut roll) =
                    yaw_pitch_roll(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
                pitch += 19.3;
                roll += -16.3;
                mbus::mbus().publish_isr(
                    "/imu",
                    crate::message::Message::YawPitchRoll { yaw, pitch, roll },
                )
            }
        }
    }
}

unsafe fn yaw_pitch_roll(
    mut gx: f32,
    mut gy: f32,
    mut gz: f32,
    mut ax: f32,
    mut ay: f32,
    mut az: f32,
) -> (f32, f32, f32) {
    #[allow(non_upper_case_globals)]
    const Kp: f32 = 100.0; // 比例增益支配率收敛到加速度计/磁强计
    #[allow(non_upper_case_globals)]
    const Ki: f32 = 0.002; // 积分增益支配率的陀螺仪偏见的衔接

    const HALF_T: f32 = 0.001; // 采样周期的一半
    #[allow(non_upper_case_globals)]
    static mut q0: f32 = 1.0; // 四元数的元素，代表估计方向
    #[allow(non_upper_case_globals)]
    static mut q1: f32 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut q2: f32 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut q3: f32 = 0.0;

    // 按比例缩小积分误差
    #[allow(non_upper_case_globals)]
    static mut ex_int: f32 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut ey_int: f32 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut ez_int: f32 = 0.0;

    // 测量正常化
    let mut norm = sqrtf(ax * ax + ay * ay + az * az);
    ax = ax / norm; //单位化
    ay = ay / norm;
    az = az / norm;

    // 估计方向的重力
    let vx = 2.0 * (q1 * q3 - q0 * q2);
    let vy = 2.0 * (q0 * q1 + q2 * q3);
    let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
    let ex = ay * vz - az * vy;
    let ey = az * vx - ax * vz;
    let ez = ax * vy - ay * vx;

    // 积分误差比例积分增益
    ex_int = ex_int + ex * Ki;
    ey_int = ey_int + ey * Ki;
    ez_int = ez_int + ez * Ki;

    // 调整后的陀螺仪测量
    gx = gx + Kp * ex + ex_int;
    gy = gy + Kp * ey + ey_int;
    gz = gz + Kp * ez + ez_int;

    // 整合四元数率和正常化
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * HALF_T;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * HALF_T;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * HALF_T;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * HALF_T;

    // 正常化四元
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    let yaw = atan2f(
        2.0 * (q1 * q2 + q0 * q3),
        q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
    ) * 57.3; //y/yaw

    let pitch = asinf(-2.0 * q1 * q3 + 2.0 * q0 * q2) * 57.3; // x/pitch ,转换为度数
    let roll = atan2f(
        2.0 * q2 * q3 + 2.0 * q0 * q1,
        -2.0 * q1 * q1 - 2.0 * q2 * q2 + 1.0,
    ) * 57.3; //z/roll

    (yaw, pitch, roll)
}
