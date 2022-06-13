//! 匿名上位机通信协议

use crate::driver::{Accel, Gyro, Quaternion};
use crate::filter::dither::DitherFilter;
use crate::filter::first_order::FirstOrderFilter;
use crate::filter::Filter;
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
    let mut dither_roll = FirstOrderFilter::new(0.01).chain(DitherFilter::<10>::new());
    let mut dither_pitch = FirstOrderFilter::new(0.01).chain(DitherFilter::<10>::new());
    let mut dither_yaw = FirstOrderFilter::new(0.01).chain(DitherFilter::<100>::new());
    let recv: &'static Queue<Message> = unsafe { Q.as_ref().unwrap() };
    let mut buf = vec![0u8; 10];
    buf.push(0xAA);
    buf.push(0xAF);
    buf.push(0x06);
    buf.push(4);
    buf.push(1);
    buf.push(1);
    buf.push(1);
    buf.push(0);
    let (sum, check) = buf
        .iter()
        .fold((0u8, 0u8), |(sum, check), b| (sum + *b, check + sum + *b));
    buf.push(sum);
    buf.push(check);
    sync::free(|_| {
        stdout::write(&buf).ok();
    });
    #[cfg(feature = "mpu9250")]
    let m = 1;
    #[cfg(any(feature = "mpu6050", feature = "icm20602"))]
    let m = 10;
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::ImuData(data) => {
                    if let Some(quat) = data.quaternion {
                        let (mut roll, mut pitch, mut yaw) = quat.euler_angles();
                        dither_roll.do_filter(roll, &mut roll);
                        dither_pitch.do_filter(pitch, &mut pitch);
                        dither_yaw.do_filter(yaw, &mut yaw);
                        if imu_count % m == 0 {
                            send_quat(quat);
                            send_euler(quat.euler_angles());
                        }
                    }

                    if imu_count % (m * 10) == 0 {
                        mbus::mbus().call("/led/b/toggle", Message::None);
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
        .chain(quat.i.to_be_bytes().iter())
        .chain(quat.j.to_be_bytes().iter())
        .chain(quat.k.to_be_bytes().iter())
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

fn send_euler((roll, pitch, yaw): (f32, f32, f32)) {
    let mut buf = vec![0u8; 13];
    buf.push(0xAA);
    buf.push(0xAF);
    buf.push(0x03);
    buf.push(7);
    ((roll * 100.0) as i16)
        .to_be_bytes()
        .iter()
        .chain(((pitch * 100.0) as i16).to_be_bytes().iter())
        .chain(((yaw * 100.0) as i16).to_be_bytes().iter())
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

fn _send_accel_gyro(accel: Accel, gyro: Gyro) {
    let mut buf = vec![0u8; 13];
    buf.push(0xAA);
    buf.push(0xAF);
    buf.push(0x01);
    buf.push(7);
    buf.push(accel.x as u8);
    buf.push(accel.y as u8);
    buf.push(accel.z as u8);
    buf.push(gyro.x as u8);
    buf.push(gyro.y as u8);
    buf.push(gyro.z as u8);
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
