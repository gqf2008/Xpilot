use std::path::PathBuf;
use std::{env, fs};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out_dir.display());

    #[cfg(feature = "gd32vf103")]
    fs::copy("src/chip/gd32vf103/memory.x", out_dir.join("memory.x")).unwrap();

    #[cfg(feature = "stm32f401ccu6")]
    fs::copy("src/driver/stm32f4/memory-401.x", out_dir.join("memory.x")).unwrap();
    #[cfg(feature = "stm32f427vit6")]
    fs::copy("src/driver/stm32f4/memory-427.x", out_dir.join("memory.x")).unwrap();

    println!("cargo:rerun-if-changed=memory.x");
}
