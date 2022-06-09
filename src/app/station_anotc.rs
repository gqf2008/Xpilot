use crate::mbus;
use crate::message::*;
use crate::{
    driver::{EulerAngle, Gyro},
    mbus::mbus,
};
use xtask::{Queue, TaskBuilder};

pub fn start() {
    let q = Queue::new();
    let sender = q.clone();
    TaskBuilder::new()
        .name("anotc")
        .stack_size(1024)
        .spawn(move || sync(q));
    mbus().subscribe("/imu", move |_, msg| match msg {
        Message::ImuData(_) | Message::Quaternion(_) => {
            if let Err(err) = sender.push_back_isr(msg) {
                log::error!("error {:?}", err);
            }
        }
        _ => {}
    });
}

fn sync(recv: Queue<Message>) {
    let mut imu_count = 0u64;
    let mut count_quat = 0u64;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::ImuData(data) => {
                    if imu_count % 1000 == 0 {
                        log::info!("{:?}", imu_count);
                    }
                    imu_count += 1;
                }
                Message::Quaternion(quat) => {
                    if count_quat % 1000 == 0 {
                        log::info!("{}", count_quat);
                    }
                    count_quat += 1;
                }

                _ => {}
            }
        }
    }
}
