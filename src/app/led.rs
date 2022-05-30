use crate::driver;
use xtask::bsp::longan_nano::led::Led;
use xtask::TaskBuilder;

pub fn start() {
    TaskBuilder::new().name("g").priority(1).spawn(|| {
        if let Some(green) = driver::led::green() {
            loop {
                green.toggle();
                xtask::sleep_ms(500);
            }
        }
    });

    TaskBuilder::new().name("b").priority(1).spawn(|| {
        if let Some(blue) = driver::led::blue() {
            loop {
                blue.toggle();
                xtask::sleep_ms(500);
            }
        }
    });
}
