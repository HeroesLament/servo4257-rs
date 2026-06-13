#![no_std]
#![no_main]
//! SERVO57D / board-57d binary.
//! Build: cargo build --bin servo57d --features board-57d --target thumbv7em-none-eabihf

// Panic handler (halt) -- Stage 1 placeholder; swap for panic-probe when
// defmt logging is wired.
use panic_halt as _;
use n32l4 as _; // retains the PAC interrupt vector table (cortex-m-rt)
use servo4257_rs as fw;

#[cortex_m_rt::entry]
fn main() -> ! {
    fw::run()
}
