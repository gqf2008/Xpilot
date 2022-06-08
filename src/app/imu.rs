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
    let mut count_quat = 0u64;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::ImuData(data) => {
                    if imu_count % 100 == 0 {
                        mbus::mbus()
                            .call("/led/red", Message::Control(Signal::Led(LedSignal::Toggle)));
                        log::info!("{:?}", data);
                    }
                    imu_count += 1;
                }
                Message::Quaternion(quat) => {
                    if count_quat % 100 == 0 {
                        // mbus::mbus()
                        //     .call("/led/red", Message::Control(Signal::Led(LedSignal::Toggle)));
                        log::info!("{:?}", quat);
                        log::info!("{:?}", quat.to_euler());
                    }
                    count_quat += 1;
                }

                _ => {}
            }
        }
    }
}
