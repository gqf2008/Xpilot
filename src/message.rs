use crate::driver::{Accel, Barometer, Compass, Distance, Gps, Gyro, ImuData};

#[derive(Debug, Clone, Copy)]
pub enum Message {
    DataReady,
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

    None,
}

#[derive(Debug, Clone, Copy)]
pub enum Signal {
    Led,
    Motor {},
    Servo {},
}
