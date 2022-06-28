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
pub enum State {
    Locked,       //锁定
    Unlocked,     //解锁
    Roll,         //翻滚
    Stall,        //失速
    OutOfControl, //失控
    Hover,        //悬停
    TurnLeft,     //左转
    TrunRight,    //右转
    ReturnFlight, //返航
    MoveForward,  //前进
    MoveLeft,     //向左
    MoveRight,    //向右
    MoveBack,     //后退
    Autopilot,    //自动驾驶
    Following,    //跟随
    Trick,        //特技
}

pub enum Event {}

// 直升机
pub struct Helix {
    fsm: Machine<State, Message>,
    speed: u32,           //速度
    height: u32,          //高度
    course: u32,          //航向
    position: (f32, f32), //当前位置
}
