#[derive(Debug, Clone, Copy)]
pub enum Message {
    // 欧拉角
    YawPitchRoll {
        yaw: f32,
        pitch: f32,
        roll: f32,
    },
    Imu6050 {
        gx: f32,
        gy: f32,
        gz: f32,
        ax: f32,
        ay: f32,
        az: f32,
    },
    // 角度
    Gyro {
        x: f32,
        y: f32,
        z: f32,
    },
    // 角加速度
    Acc {
        x: f32,
        y: f32,
        z: f32,
    },
    // 磁力计（罗盘）
    Compass {
        x: f32,
        y: f32,
        z: f32,
    },
    //气压计
    Barometer {
        h: f32,
    },

    Distance(f32),
    Gps {
        longitude: f32,
        latitude: f32,
    },
    Channel(u8),
    Cmd(Command),
}

#[derive(Debug, Clone, Copy)]
pub enum Command {}
