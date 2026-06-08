#![no_std]
#![no_main]
//! SERVO57D binary. Build: cargo build --bin servo57d --features board-57d
//! --target thumbv7em-none-eabihf
use servo4257_rs as fw;
#[cortex_m_rt::entry]
fn main() -> ! { fw::run() }
