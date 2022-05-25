use crate::message::Message;
use xtask::bus::*;

/// 消息总线
static MBUS: Bus<Message> = Bus::new();

pub fn mbus<'a>() -> &'static Bus<'a, Message> {
    &MBUS
}
