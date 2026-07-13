//! Embassy time-base smoke test. Busy-polls embassy_time::now() (no await), so
//! the core never sleeps and SWD stays alive. Read over SWD:
//!   magic  0xE10B_0002 @ 0x2000_4000
//!   now()-milliseconds @ 0x2000_4004  (should climb ~1000/sec)
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_executor::Executor;
use embassy_time::Instant;
use n32l4 as _;
use panic_halt as _;
use static_cell::StaticCell;

const MAGIC: *mut u32 = 0x2000_4000 as *mut u32;
const COUNT: *mut u32 = 0x2000_4004 as *mut u32;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn heartbeat() {
    unsafe { core::ptr::write_volatile(MAGIC, 0xE10B_0002) };
    loop {
        let ms = Instant::now().as_millis() as u32;
        unsafe { core::ptr::write_volatile(COUNT, ms) };
    }
}

#[entry]
fn main() -> ! {
    servo4257_rs::rt::init_time_driver(16_000_000);
    let ex = EXECUTOR.init(Executor::new());
    ex.run(|spawner| {
        spawner.spawn(heartbeat().unwrap());
    });
}
