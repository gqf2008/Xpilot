use alloc::vec::Vec;

use crate::driver::{Accel, Barometer, Compass, Distance, Gps, Gyro, ImuData};

#[derive(Debug, Clone)]
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
    //遥测协议
    Telem(Telem),
    None,
}

//
#[derive(Debug, Clone)]
pub enum Telem {
    Multiwii(Vec<u8>),
    Mavlink(Vec<u8>),
}

#[derive(Debug, Clone)]
pub enum Signal {
    Led,
    Motor {},
    Servo {},
}
