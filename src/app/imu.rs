use crate::{mbus::mbus, message::Message};
use libm::*;
use xtask::{isr_sprintln, Queue, TaskBuilder};

pub fn start() {
    let q = Queue::new();
    let sender = q.clone();
    TaskBuilder::new()
        .name("icm")
        .priority(1)
        .stack_size(1024)
        .spawn(move || sampling(q));
    mbus().subscribe("/imu6050", move |_, msg| {
        if let Err(err) = sender.push_back_isr(msg) {
            isr_sprintln!("error {:?}", err);
        }
    });
}

fn sampling(recv: Queue<Message>) {
    let mut count = 0u64;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::Imu6050 {
                    gx,
                    gy,
                    gz,
                    ax,
                    ay,
                    az,
                } => {
                    let (y, p, r) = unsafe { yaw_pitch_roll(gx, gy, gz, ax, ay, az) };
                    if count % 100 == 0 {
                        log::info!(" yaw:{}, pitch:{}, roll:{}", y, p, r,);
                    }
                    count += 1;
                }
                _ => {}
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
