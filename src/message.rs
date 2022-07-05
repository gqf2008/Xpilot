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

    //遥控信号
    RemoteControl(RC),
    //遥测数据
    Telem(Telem),
    None,
}

#[derive(Debug, Clone)]
pub enum Telem {
    Raw(Vec<u8>),
    Multiwii(multiwii_serial_protocol_v2::Packet),
}

#[derive(Debug, Clone)]
pub enum Signal {
    Led,
    Motor {},
    Servo {},
}
#[derive(Debug, Clone)]
pub enum RC {
    Hover,
    TurnLeft(f32),  //左转
    TrunRight(f32), //右转
    ReturnFlight,   //返航
    Move(f32, f32), //移动
}
