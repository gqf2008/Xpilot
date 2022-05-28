use crate::driver;
use libm::*;
use xtask::TaskBuilder;

pub fn start() {
    sampling();
}

fn sampling() {
    if let Some(mpu) = driver::mpu6050::mpu() {
        TaskBuilder::new()
            .name("icm")
            .priority(1)
            .stack_size(1024)
            .spawn(move || {
                let mut count = 0;

                loop {
                    // get gyro data, scaled with sensitivity
                    if let Some(gyro) = mpu.get_gyro().ok() {
                        //log::info!("gyro: {:?}", gyro);
                        let (gx, gy, gz) = (gyro.x, gyro.y, gyro.z);
                        // get accelerometer data, scaled with sensitivity
                        if let Some(acc) = mpu.get_acc().ok() {
                            // log::info!("accel: {:?}", acc);
                            let (ax, ay, az) = (acc.x, acc.y, acc.z);
                            let (p, r, y) = unsafe {
                                ypr(
                                    gx as f64, gy as f64, gz as f64, ax as f64, ay as f64,
                                    az as f64,
                                )
                            };
                            count += 1;
                            if count % 100 == 0 {
                                log::info!("pitch:{}, roll:{} yaw:{}", p, r, y);
                                count = 0;
                            }
                        }
                    }
                    xtask::sleep_ms(10);
                }
            });
    }
}

unsafe fn ypr(
    mut gx: f64,
    mut gy: f64,
    mut gz: f64,
    mut ax: f64,
    mut ay: f64,
    mut az: f64,
) -> (f64, f64, f64) {
    #[allow(non_upper_case_globals)]
    const Kp: f64 = 100.0; // 比例增益支配率收敛到加速度计/磁强计
    #[allow(non_upper_case_globals)]
    const Ki: f64 = 0.002; // 积分增益支配率的陀螺仪偏见的衔接

    const HALF_T: f64 = 0.001; // 采样周期的一半
    #[allow(non_upper_case_globals)]
    static mut q0: f64 = 1.0; // 四元数的元素，代表估计方向
    #[allow(non_upper_case_globals)]
    static mut q1: f64 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut q2: f64 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut q3: f64 = 0.0;

    // 按比例缩小积分误差
    #[allow(non_upper_case_globals)]
    static mut ex_int: f64 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut ey_int: f64 = 0.0;
    #[allow(non_upper_case_globals)]
    static mut ez_int: f64 = 0.0;

    // 测量正常化
    let mut norm = sqrt(ax * ax + ay * ay + az * az);
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
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    // ANGLE.Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
    // ANGLE.Y= asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
    // ANGLE.X= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll

    let pitch = asin(-2.0 * q1 * q3 + 2.0 * q0 * q2) * 57.3; // y/pitch ,转换为度数
    let roll = atan2(
        2.0 * q2 * q3 + 2.0 * q0 * q1,
        -2.0 * q1 * q1 - 2.0 * q2 * q2 + 1.0,
    ) * 57.3; //x/roll
    let yaw = atan2(
        2.0 * (q1 * q2 + q0 * q3),
        q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
    ) * 57.3; //z/yaw

    // let yaw = atan2(
    //     2.0 * (q1 * q2 + q0 * q3),
    //     -2.0 * q2 * q2 - 2.0 * q3 * q3 + 1.0,
    // ) * 57.3;
    (pitch, roll, yaw)
}
