use crate::{driver, mbus::mbus, message::Message};
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
    let mut count = 0u64;
    let mut count_acc = 0u64;
    let mut count_gpro = 0u64;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::YawPitchRoll { yaw, pitch, roll } => {
                    if count % 100 == 0 {
                        if let Some(led) = driver::led::blue() {
                            led.toggle();
                        }
                        log::info!(" yaw:{:.5?}, pitch:{:.5?}, roll:{:.5?}", yaw, pitch, roll);
                    }
                    count += 1;
                }
                Message::Acc { x, y, z } => {
                    if count_acc % 100 == 0 {
                        if let Some(led) = driver::led::blue() {
                            led.toggle();
                        }
                        log::info!(" acc_x:{:.5?}, acc_y:{:.5?}, acc_z:{:.5?}", x, y, z);
                    }
                    count_acc += 1;
                }
                Message::Gyro { x, y, z } => {
                    if count_gpro % 100 == 0 {
                        if let Some(led) = driver::led::blue() {
                            led.toggle();
                        }
                        log::info!(" gyro_x:{:.5?}, gyro_y:{:.5?}, gyro_z:{:.5?}", x, y, z);
                    }
                    count_gpro += 1;
                }
                _ => {}
            }
        }
    }
}
