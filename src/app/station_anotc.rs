use crate::driver::{EulerAngle, Quaternion};
use crate::mbus::{self, mbus};
use crate::message::*;
use alloc::vec;
use xtask::bsp::greenpill::stdout;
use xtask::{sync, Queue, TaskBuilder};

static mut Q: Option<Queue<Message>> = None;

pub fn start() {
    let q = Queue::new();
    unsafe {
        Q.replace(q);
    }
    TaskBuilder::new()
        .name("anotc")
        .stack_size(2024)
        .spawn(|| sync());
    mbus().subscribe("/imu", move |_, msg| match msg {
        Message::ImuData(_) => {
            let q: &'static Queue<Message> = unsafe { Q.as_ref().unwrap() };
            if let Err(err) = q.push_back_isr(msg) {
                log::error!("error {:?}", err);
            }
        }
        _ => {}
    });
}

fn sync() {
    let mut imu_count = 0u64;
    let recv: &'static Queue<Message> = unsafe { Q.as_ref().unwrap() };
    let mut buf = vec![0u8; 10];
    buf.push(0xAA);
    buf.push(0xAF);
    buf.push(0x06);
    buf.push(4);
    buf.push(1);
    buf.push(1);
    buf.push(1);
    buf.push(1);
    let (sum, check) = buf
        .iter()
        .fold((0u8, 0u8), |(sum, check), b| (sum + *b, check + sum + *b));
    buf.push(sum);
    buf.push(check);
    sync::free(|_| {
        stdout::write(&buf).ok();
    });
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::ImuData(data) => {
                    if imu_count % 10 == 0 {
                        if let Some(quat) = data.quaternion {
                            // send_quat(quat);
                            send_euler(quat.to_euler());
                        }
                    }
                    if imu_count % 100 == 0 {
                        mbus::mbus().call(
                            "/led/blue",
                            Message::Control(Signal::Led(LedSignal::Toggle)),
                        );
                    }
                    imu_count += 1;
                }

                _ => {}
            }
        }
    }
}

fn send_quat(quat: Quaternion) {
    let mut buf = vec![0u8; 22];
    buf.push(0xAA);
    buf.push(0xAF);
    buf.push(0x04);
    buf.push(16);
    quat.w
        .to_be_bytes()
        .iter()
        .chain(quat.x.to_be_bytes().iter())
        .chain(quat.y.to_be_bytes().iter())
        .chain(quat.z.to_be_bytes().iter())
        .into_iter()
        .for_each(|b| buf.push(*b));
    let (sum, check) = buf
        .iter()
        .fold((0u8, 0u8), |(sum, check), b| (sum + *b, check + sum + *b));
    buf.push(sum);
    buf.push(check);
    sync::free(|_| {
        stdout::write(&buf).ok();
    });
}

fn send_euler(euler: EulerAngle) {
    let mut buf = vec![0u8; 13];
    buf.push(0xAA);
    buf.push(0xAF);
    buf.push(0x03);
    buf.push(7);
    ((euler.roll * 100.0) as u16)
        .to_be_bytes()
        .iter()
        .chain(((euler.pitch * 100.0) as u16).to_be_bytes().iter())
        .chain(((euler.yaw * 100.0) as u16).to_be_bytes().iter())
        .into_iter()
        .for_each(|b| buf.push(*b));
    buf.push(0);
    let (sum, check) = buf
        .iter()
        .fold((0u8, 0u8), |(sum, check), b| (sum + *b, check + sum + *b));
    buf.push(sum);
    buf.push(check);
    sync::free(|_| {
        stdout::write(&buf).ok();
    });
}
