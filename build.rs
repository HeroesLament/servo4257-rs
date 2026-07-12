//! Selects the flash/RAM layout per build via mutually-exclusive features:
//!   layout-boot -> memory/boot.x   (bootloader, 16K @ 0x08000000)
//!   layout-app  -> memory/app.x    (application, @ 0x08004000)
//!   (default)   -> memory/full.x   (standalone, full 128K)
//! The chosen file is copied to OUT_DIR/memory.x, which cortex-m-rt's link.x
//! includes. OUT_DIR is on the linker search path (rustc-link-search below).
use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    let src = if env::var_os("CARGO_FEATURE_LAYOUT_BOOT").is_some() {
        "memory/boot.x"
    } else if env::var_os("CARGO_FEATURE_LAYOUT_APP").is_some() {
        "memory/app.x"
    } else {
        "memory/full.x"
    };

    fs::copy(src, out.join("memory.x")).expect("copy memory.x");
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory/boot.x");
    println!("cargo:rerun-if-changed=memory/app.x");
    println!("cargo:rerun-if-changed=memory/full.x");
    println!("cargo:rerun-if-changed=build.rs");
}
