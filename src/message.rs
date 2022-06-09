use crate::driver::{
    Accel, Barometer, Compass, Distance, EulerAngle, Gps, Gyro, ImuData, Quaternion,
};

#[derive(Debug, Clone, Copy)]
pub enum Message {
    ImuData(ImuData),
    // 角度
    Gyro(Gyro),
    // 角加速度
    Accel(Accel),
    // 罗盘
    Compass(Compass),
    //气压计
    Barometer(Barometer),

    Distance(Distance),
    Gps(Gps),
    //控制信号
    Control(Signal),
}

#[derive(Debug, Clone, Copy)]
pub enum Signal {
    Led(LedSignal),
    Motor {},
    Servo {},
}

#[derive(Debug, Clone, Copy)]
pub enum LedSignal {
    On,
    Off,
    Toggle,
}
