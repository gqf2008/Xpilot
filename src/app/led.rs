use crate::driver;
use xtask::bsp::longan_nano::led::Led;
use xtask::TaskBuilder;

pub fn start() {
    let (red, green, blue) = driver::led::led();
    TaskBuilder::new()
        .name("green")
        .priority(1)
        .spawn(move || loop {
            green.toggle();
            xtask::sleep_ms(500);
        });

    TaskBuilder::new()
        .name("red")
        .priority(1)
        .spawn(move || loop {
            red.toggle();
            xtask::sleep_ms(500);
        });

    TaskBuilder::new()
        .name("blue")
        .priority(1)
        .spawn(move || loop {
            blue.toggle();
            xtask::sleep_ms(500);
        });
}
