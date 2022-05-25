use xtask::bus::Bus;

use crate::message::Message;

/// 消息总线
static MBUS: Bus<Message> = Bus::new();

pub fn init() {}

pub fn publish() {}
pub fn subscribe(topic: &str) {}
