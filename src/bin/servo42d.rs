#![no_std]
#![no_main]
//! SERVO42D binary. Build: cargo build --bin servo42d --features board-42d
//! --target thumbv7em-none-eabihf
use servo4257_rs as fw;
#[cortex_m_rt::entry]
fn main() -> ! { fw::run() }
