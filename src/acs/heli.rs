use crate::mbus;
use crate::mbus::mbus;
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
    mbus().subscribe("/imu", move |_, msg| {
        if let Err(err) = sender.push_back_isr(msg) {
            log::error!("error {:?}", err);
        }
    });
}

fn sampling(recv: Queue<Message>) {
    let mut imu_count = 0u64;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::ImuData(data) => {
                    if imu_count % 1000 == 0 {
                        mbus::mbus().call("/led/r/toggle", Message::None);
                        //log::info!("{:?}", data);
                    }
                    imu_count += 1;
                }

                _ => {}
            }
        }
    }
}
