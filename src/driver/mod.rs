#[cfg(feature = "gd32vf103")]
mod gd32vf103;
#[cfg(feature = "stm32f4")]
mod stm32f401ccu6;
#[cfg(feature = "gd32vf103")]
pub use gd32vf103::{bldc, led, mpu6050, serial, servo};
#[cfg(feature = "stm32f4")]
pub use stm32f401ccu6::led;

pub mod bldc;
pub mod mpu6050;
pub mod servo;

pub fn init() {
    #[cfg(feature = "gd32vf103")]
    unsafe {
        gd32vf103::init()
    }
    #[cfg(feature = "stm32f4")]
    unsafe {
        stm32f401ccu6::init()
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct ImuData {
    pub accel: Accel,
    pub temp: f32,
    pub gyro: Gyro,
    pub compass: Compass,
}

impl ImuData {
    pub fn to_quat(self) -> Quaternion {
        unsafe {
            use libm::*;
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
            let mut ax = self.accel.x;
            let mut ay = self.accel.y;
            let mut az = self.accel.z;
            let mut gx = self.gyro.x;
            let mut gy = self.gyro.y;
            let mut gz = self.gyro.z;
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
            Quaternion {
                w: q0,
                x: q1,
                y: q2,
                z: q3,
            }
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Quaternion {
    w: f32,
    x: f32,
    y: f32,
    z: f32,
}

impl Quaternion {
    pub fn to_euler(self) -> EulerAngle {
        use libm::*;
        let q0 = self.w;
        let q1 = self.x;
        let q2 = self.y;
        let q3 = self.z;
        let yaw = atan2f(
            2.0 * (q1 * q2 + q0 * q3),
            q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
        ) * 57.3;

        let pitch = asinf(-2.0 * q1 * q3 + 2.0 * q0 * q2) * 57.3;
        let roll = atan2f(
            2.0 * q2 * q3 + 2.0 * q0 * q1,
            -2.0 * q1 * q1 - 2.0 * q2 * q2 + 1.0,
        ) * 57.3;
        EulerAngle { yaw, pitch, roll }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct EulerAngle {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

/// 重力加速度，单位g
#[derive(Copy, Clone, Debug, Default)]
pub struct Accel {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// 角速度，单位rad/s
#[derive(Copy, Clone, Debug, Default)]
pub struct Gyro {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// 罗盘
#[derive(Copy, Clone, Debug, Default)]
pub struct Compass {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

//气压计
#[derive(Copy, Clone, Debug, Default)]
pub struct Barometer {
    pub h: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Distance(f32);

#[derive(Copy, Clone, Debug, Default)]
pub struct Gps {
    pub longitude: f32,
    pub latitude: f32,
}
