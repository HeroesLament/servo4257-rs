//! Combined async diagnostic. Keeper task yields (never sleeps) so SWD stays
//! live; alarm task uses Timer::after to exercise the CC1 alarm path.
//! SWD markers @0x2000_4000:
//!  [0] magic 0xE10B_0003
//!  [1] now()-ms          (should climb ~1000/sec  -> [1] 4MHz fix works)
//!  [2] alarm B_COUNT     (climbs ~10/sec ONLY if Timer wakes -> alarm works)
//!  [3] DBG_CTRL readback (bit0 set -> DBG_SLEEP write sticks -> PAC ok)
//!  [4] ISR_COUNT         (TIM2 ISR entries: overflow ~15/sec)
//!  [5] CC1_COUNT         (CC1 alarm fires -> the alarm path itself)
//!  [6] RCC_CFG           ([1] reset clock config cross-check)
//!  [7] keeper A_COUNT    (executor cycling)
#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;
use cortex_m_rt::entry;
use embassy_executor::Executor;
use embassy_futures::yield_now;
use embassy_time::{Instant, Timer};
use n32l4 as _;
use panic_halt as _;
use servo4257_rs::rt::time_driver::{CC1_COUNT, ISR_COUNT};
use static_cell::StaticCell;

const M: *mut u32 = 0x2000_4000 as *mut u32;

#[inline]
fn w(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(M.add(i), v) };
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[allow(dead_code)]
async fn keeper_unused() {
    let mut a: u32 = 0;
    loop {
        a = a.wrapping_add(1);
        w(1, Instant::now().as_millis() as u32);
        w(4, ISR_COUNT.load(Relaxed));
        w(5, CC1_COUNT.load(Relaxed));
        w(7, a);
        yield_now().await;
    }
}

#[embassy_executor::task]
async fn alarm() {
    let mut b: u32 = 0;
    loop {
        b = b.wrapping_add(1);
        w(2, b);
        w(1, Instant::now().as_millis() as u32);
        w(4, ISR_COUNT.load(Relaxed));
        w(5, CC1_COUNT.load(Relaxed));
        Timer::after_millis(100).await;
    }
}

#[entry]
fn main() -> ! {
    w(0, 0xE10B_0004);
    servo4257_rs::rt::init_time_driver(4_000_000); // [1] fix: MSI 4 MHz reset clock
    unsafe {
        (*n32l4::n32l406::Dbg::ptr()).ctrl().modify(|_, w| w.sleep().set_bit());
        w(3, core::ptr::read_volatile(0xE004_2004 as *const u32));
        w(6, core::ptr::read_volatile(0x4002_1004 as *const u32));
    }
    let ex = EXECUTOR.init(Executor::new());
    ex.run(|s| {
        s.spawn(alarm().unwrap());
    });
}
