use crate::driver::bldc::Motor;
use crate::driver::servo::Servo;
use crate::fsm::Machine;
use crate::mbus;
use crate::message::*;
use xtask::{Queue, TaskBuilder};

pub fn start() {
    let q = Queue::new();
    let sender = q.clone();
    TaskBuilder::new()
        .name("heli")
        .priority(1)
        .stack_size(1024)
        .spawn(move || sampling(q));
    mbus::bus().subscribe("/rc", move |_, msg| {
        if let Err(err) = sender.push_back_isr(msg) {
            log::error!("error {:?}", err);
        }
    });
}

fn sampling(recv: Queue<Message>) {
    let mut imu_count = 0u64;
    #[cfg(feature = "mpu9250")]
    let m = 10;
    #[cfg(any(feature = "mpu6050", feature = "icm20602"))]
    let m = 100;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::ImuData(data) => {
                    if imu_count % m == 0 {
                        mbus::bus().call("/led/r/toggle", Message::None);
                    }
                    imu_count += 1;
                }

                _ => {}
            }
        }
    }
}

// 状态
#[derive(Default)]
pub enum State {
    #[default]
    Locked, //锁定
    Unlocked,  //解锁
    Roll,      //翻滚
    Stall,     //失速
    Ooc,       //失控 out of control
    Hover,     //悬停
    Manual,    //手动
    Auto,      //自动
    Following, //跟随
    Trick,     //特技
    Lost,      //失联
    Crash,     //坠机
}

// 直升机
pub struct Helix<PWM> {
    fsm: Machine<State, Message>,
    speed: u32,             //速度
    height: u32,            //高度
    roll: f32,              //翻滚角
    pitch: f32,             //俯仰角
    yaw: f32,               //偏航角
    position: (f32, f32),   //当前位置
    eu: ExecutionUnit<PWM>, //执行单元
}

struct ExecutionUnit<PWM> {
    motor: Motor<PWM>,  //主旋翼
    servo1: Servo<PWM>, //roll
    servo2: Servo<PWM>, //pitch
    servo3: Servo<PWM>, //
    servo4: Servo<PWM>, //yaw
}
