use crate::mbus;
use crate::message::*;
use crate::{
    driver::{EulerAngle, Gyro},
    mbus::mbus,
};
use xtask::{isr_sprintln, Queue, TaskBuilder};

pub fn start() {
    let q = Queue::new();
    let sender = q.clone();
    TaskBuilder::new()
        .name("pid")
        .priority(1)
        .stack_size(1024)
        .spawn(move || sampling(q));
    mbus().subscribe("/imu", move |_, msg| {
        if let Err(err) = sender.push_back_isr(msg) {
            isr_sprintln!("error {:?}", err);
        }
    });
}

fn sampling(recv: Queue<Message>) {
    let mut imu_count = 0u64;
    let mut ypr_count = 0u64;
    let mut count_acc = 0u64;
    let mut count_gpro = 0u64;
    let mut count_quat = 0u64;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::ImuData(data) => {
                    if imu_count % 500 == 0 {
                        mbus::mbus()
                            .call("/led/red", Message::Control(Signal::Led(LedSignal::Toggle)));
                        log::info!("{:?}", data);
                    }
                    imu_count += 1;
                }
                Message::Quaternion(quat) => {
                    if count_quat % 500 == 0 {
                        mbus::mbus()
                            .call("/led/red", Message::Control(Signal::Led(LedSignal::Toggle)));
                        log::info!("{:?}", quat);
                        log::info!("{:?}", quat.to_euler());
                    }
                    count_quat += 1;
                }
                Message::YawPitchRoll(EulerAngle { yaw, pitch, roll }) => {
                    if ypr_count % 500 == 0 {
                        mbus::mbus()
                            .call("/led/red", Message::Control(Signal::Led(LedSignal::Toggle)));
                        log::info!(" yaw:{:.5?}, pitch:{:.5?}, roll:{:.5?}", yaw, pitch, roll);
                    }
                    ypr_count += 1;
                }
                Message::Accel(accel) => {
                    if count_acc % 500 == 0 {
                        mbus::mbus().call(
                            "/led/green",
                            Message::Control(Signal::Led(LedSignal::Toggle)),
                        );
                        log::info!(
                            " acc_x:{:.5?}, acc_y:{:.5?}, acc_z:{:.5?}",
                            accel.x,
                            accel.y,
                            accel.z
                        );
                        log::info!("acc_angel {:?}", accel.to_degree());
                    }
                    count_acc += 1;
                }
                Message::Gyro(Gyro { x, y, z }) => {
                    if count_gpro % 500 == 0 {
                        mbus::mbus().call(
                            "/led/blue",
                            Message::Control(Signal::Led(LedSignal::Toggle)),
                        );
                        log::info!(" gyro_x:{:.5?}, gyro_y:{:.5?}, gyro_z:{:.5?}", x, y, z);
                    }
                    count_gpro += 1;
                }
                _ => {}
            }
        }
    }
}
