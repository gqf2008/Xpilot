#[derive(Debug, Clone, Copy)]
pub enum Message {
    YawPitchRoll { yaw: f32, pitch: f32, roll: f32 },
    Gyro { x: f32, y: f32, z: f32 },
    Acc { x: f32, y: f32, z: f32 },
    Barometer { h: f32 },
    Distance(f32),
    Gps { longitude: f32, latitude: f32 },
    Channel(u8),
}

pub enum Command {}
