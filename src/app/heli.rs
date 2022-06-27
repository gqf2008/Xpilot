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
    mbus::bus().subscribe("/imu", move |_, msg| {
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
                        //log::info!("{:?}", data);
                    }
                    imu_count += 1;
                }

                _ => {}
            }
        }
    }
}
