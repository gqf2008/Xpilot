use crate::driver;
use embedded_hal::Pwm;
use xtask::bsp::longan_nano::hal::pwm::Channel;
use xtask::bsp::longan_nano::hal::time::*;
use xtask::TaskBuilder;

pub fn start() {
    let pwm1 = driver::pwm::pwm1();
    pwm1.set_period(1000.hz());
    pwm1.enable(Channel::CH1);
    pwm1.enable(Channel::CH2);
    TaskBuilder::new().name("r").priority(1).spawn(|| {
        let pwm = driver::pwm::pwm1();
        let max = pwm.get_max_duty();
        let duty = &[
            0.0f32 * max as f32,
            0.1 * max as f32,
            0.2 * max as f32,
            0.3 * max as f32,
            0.4 * max as f32,
            0.5 * max as f32,
            0.6 * max as f32,
            0.7 * max as f32,
            0.8 * max as f32,
            0.9 * max as f32,
            1. * max as f32,
        ];
        loop {
            for i in duty {
                pwm.set_duty(Channel::CH1, *i as u16);
                //log::info!("duty {} max {}", pwm.get_duty(Channel::CH1), max);
                xtask::sleep_ms(100);
            }
            xtask::sleep_ms(1000);
            for i in duty {
                pwm.set_duty(Channel::CH1, *i as u16);
                xtask::sleep_ms(100);
            }
            xtask::sleep_ms(1000);
        }
    });

    TaskBuilder::new().name("g").priority(1).spawn(|| {
        let pwm = driver::pwm::pwm1();
        let max = pwm.get_max_duty();
        let duty = &[
            0.0f32,
            0.1 * max as f32,
            0.2 * max as f32,
            0.3 * max as f32,
            0.4 * max as f32,
            0.5 * max as f32,
            0.6 * max as f32,
            0.7 * max as f32,
            0.8 * max as f32,
            0.9 * max as f32,
            1. * max as f32,
        ];
        loop {
            for i in duty {
                pwm.set_duty(Channel::CH2, *i as u16);
                xtask::sleep_ms(100);
            }
            xtask::sleep_ms(1000);
            for i in duty {
                pwm.set_duty(Channel::CH2, *i as u16);
                xtask::sleep_ms(100);
            }
            xtask::sleep_ms(1000);
        }
    });

    // TaskBuilder::new()
    //     .name("b")
    //     .priority(1)
    //     .spawn(move || loop {
    //         blue.toggle();
    //         xtask::sleep_ms(500);
    //     });
}
