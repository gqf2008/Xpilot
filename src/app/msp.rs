/// MultiWii通信协议
///
///
///
use crate::driver::ImuData;

use crate::mbus;
use crate::message::*;
use alloc::vec;

use crossbeam::atomic::AtomicCell;
use multiwii_serial_protocol_v2::structs::*;
use multiwii_serial_protocol_v2::{Command, Packet};
use packed_struct::prelude::*;
use xtask::{Queue, TaskBuilder};

static IMU_DATA: AtomicCell<ImuData> = AtomicCell::new(ImuData {
    accel: None,
    temp: None,
    quaternion: None,
    gyro: None,
    compass: None,
    euler: None,
});

static mut Q: Option<Queue<Message>> = None;

pub fn start() {
    let q = Queue::new();
    unsafe {
        Q.replace(q);
    }
    TaskBuilder::new()
        .name("multiwii")
        .stack_size(1024)
        .spawn(process);

    mbus::bus().subscribe("/imu", move |_, msg| match msg {
        Message::ImuData(data) => {
            IMU_DATA.store(data);
        }
        _ => {}
    });

    mbus::bus().subscribe("/telem/msp", move |_, msg| match msg {
        Message::Telem(_) => {
            let q: &'static Queue<Message> = unsafe { Q.as_ref().unwrap() };
            if let Err(err) = q.push_back_isr(msg) {
                log::error!("error {:?}", err);
            }
        }
        _ => {}
    });
}

fn process() {
    let recv: &'static Queue<Message> = unsafe { Q.as_ref().unwrap() };
    loop {
        if let Some(msg) = recv.pop_front() {
            match msg {
                Message::Telem(Telem::Multiwii(msg)) => {
                    // log::info!("<<<< {:?}", msg);
                    match msg.cmd {
                        Command::MSP_API_VERSION => {
                            let api_version = MspApiVersion {
                                protocol_version: 2,
                                api_version_major: 2,
                                api_version_minor: 3,
                            };
                            if let Ok(b) = api_version.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_API_VERSION).with_data(b.to_vec()),
                                );
                            }
                        }

                        Command::MSP_ANALOG => {
                            let analog = MspAnalog {
                                legacy_battery_voltage: 255,
                                mah_drawn: 0xffff,
                                rssi: 1023 - 284,
                                amperage: 80,
                                battery_voltage: 4239,
                            };
                            if let Ok(b) = analog.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_ANALOG).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_ATTITUDE => {
                            let data = IMU_DATA.load();
                            if let Some(euler) = data.euler {
                                let atti = MspAttitude {
                                    roll: libm::floorf(euler.roll.to_degrees() * 10.0) as i16,
                                    pitch: libm::floorf(euler.pitch.to_degrees() * 10.0) as i16,
                                    yaw: libm::floorf(euler.yaw.to_degrees() * 10.0) as i16,
                                };
                                if let Ok(b) = atti.pack() {
                                    send_multiwii(
                                        Packet::new(Command::MSP_ATTITUDE).with_data(b.to_vec()),
                                    );
                                }
                            }
                        }
                        Command::MSP_MIXER_CONFIG => {
                            let conf = MspMixerConfig {
                                mixer_mode: MixerMode::QuadX,
                            };
                            if let Ok(b) = conf.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_MIXER_CONFIG).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_ACC_TRIM => {
                            let acc = MspAccTrim { pitch: 0, roll: 0 };
                            if let Ok(b) = acc.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_ACC_TRIM).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_STATUS => {
                            let status = MspStatus {
                                cycle_time: 100,
                                i2c_errors: 0,
                                sensors: MspAvailableSensors {
                                    gyro: true,
                                    sonar: true,
                                    gps: true,
                                    mag: true,
                                    baro: true,
                                    acc: true,
                                },
                                null1: 0,
                                flight_mode: 6,
                                profile: 2,
                                system_load: 0,
                            };
                            if let Ok(b) = status.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_STATUS).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_STATUS_EX => {
                            let status = MspStatusEx {
                                cycle_time: 100,
                                i2c_errors: 0,
                                sensors: MspAvailableSensors {
                                    gyro: true,
                                    sonar: true,
                                    gps: true,
                                    mag: true,
                                    baro: true,
                                    acc: true,
                                },
                                null1: 0,
                                flight_mode: 0,
                                current_pid_profile_index: 0,
                                average_system_load_percent: 0,
                                max_profile_count: 10,
                                current_control_rate_profile_index: 0,
                            };
                            if let Ok(b) = status.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_STATUS_EX).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_FC_VARIANT => {
                            let fc_variant = MspFlightControllerVariant {
                                identifier: "BTFL".as_bytes().try_into().expect(""),
                            };
                            if let Ok(b) = fc_variant.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_FC_VARIANT).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_FC_VERSION => {
                            let fc_version = MspFlightControllerVersion {
                                major: 4,
                                minor: 3,
                                patch: 0,
                            };
                            if let Ok(b) = fc_version.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_FC_VERSION).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_BUILD_INFO => {
                            let build_info = MspBuildInfo {
                                date_str: "2022-07-05 ".as_bytes().try_into().expect(""), //[0; 11],
                                time_str: "12:29:27".as_bytes().try_into().expect(""),
                                git_str: "1234567".as_bytes().try_into().expect(""),
                            };
                            if let Ok(b) = build_info.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_BUILD_INFO).with_data(b.to_vec()),
                                );
                            }
                        }
                        Command::MSP_BOARD_INFO => {
                            let board_info = MspBoardInfo {
                                board_id: "XCM4".as_bytes().try_into().expect(""),
                                hardware_revision: 11,
                                fc_type: 6,
                            };
                            if let Ok(b) = board_info.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_BOARD_INFO).with_data(b.to_vec()),
                                );
                            }
                        }

                        Command::MSP_UID => {
                            let uid = MspUniqueId {
                                uid: "XPILOT000000".as_bytes().try_into().expect(""),
                            };
                            if let Ok(b) = uid.pack() {
                                send_multiwii(Packet::new(Command::MSP_UID).with_data(b.to_vec()));
                            }
                        }
                        Command::MSP_NAME => {
                            let name: [u8; 17] =
                                "Xtask; Xpilot;   ".as_bytes().try_into().expect("");
                            send_multiwii(Packet::new(Command::MSP_NAME).with_data(name.to_vec()));
                        }
                        Command::MSP_SET_ARMING_DISABLED => {
                            send_multiwii(Packet::new(Command::MSP_SET_ARMING_DISABLED));
                        }
                        Command::MSP_SET_RTC => {
                            send_multiwii(Packet::new(Command::MSP_SET_RTC));
                        }
                        Command::MSP_FEATURE_CONFIG => {
                            send_multiwii(
                                Packet::new(Command::MSP_FEATURE_CONFIG)
                                    .with_data(u32::MAX.to_be_bytes().to_vec()),
                            );
                        }
                        Command::MSP_BATTERY_CONFIG => {
                            let battery = MspBatteryConfig {
                                vbat_min_cell_voltage: 100,
                                vbat_max_cell_voltage: 100,
                                vbat_warning_cell_voltage: 100,
                                battery_capacity: 5000,
                                voltage_meter_source: 100,
                                current_meter_source: 100,
                            };
                            if let Ok(b) = battery.pack() {
                                send_multiwii(
                                    Packet::new(Command::MSP_BATTERY_CONFIG).with_data(b.to_vec()),
                                );
                            }
                        }
                        _ => {
                            send_multiwii(Packet::new_code(msg.code));
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

fn send_multiwii(msg: Packet) {
    let mut buf = vec![0u8; msg.packet_size_bytes_v2()];
    if let Ok(_) = msg.serialize_v2(&mut buf) {
        mbus::bus().call("/telem/tx", Message::Telem(Telem::Raw(buf)));
        mbus::bus().call("/led/b/toggle", Message::None);
    }
}
