//! Async CAN smoke test — proves the `rt::can` brick on silicon under the
//! executor: an RX task awaits frames and echoes them, while a heartbeat task
//! transmits on a timer. Exercises RX-interrupt wake, async TX (mailbox-wait),
//! the time driver's self-calibrated init, and their coexistence.
//!
//! Clocked like the bootloader/appstub (HSE 8 MHz, PCLK1 = 4 MHz) so
//! `BTR_500K_PCLK4` is correct. CAN on PA11 (RX) / PA12 (TX), node 1.
//!
//! SWD markers @0x2000_4000:
//!   [0] magic 0xCA5C_0001
//!   [1] now()-ms (climbs ~1000/sec)
//!   [5] last received COB-ID
//!   [6] 0xCA00_0000 | heartbeat count (TX path alive)
//!   [7] received-frame count (RX path alive)
//!
//! Wire test from the Elixir master: send a frame to 0x601 and watch the echo
//! come back on 0x581; [7] climbs per received frame, [6] per second.
#![no_std]
#![no_main]

use canopen_proto::transport::Frame;
use cortex_m_rt::entry;
use embassy_executor::Executor;
use embassy_time::{Instant, Timer};
use n32l4 as _;
use n32l4xx_hal::{can::Can, gpio::GpioExt, pac, prelude::*};
use panic_halt as _;
use servo4257_rs::rt::{self, can};
use static_cell::StaticCell;

const M: *mut u32 = 0x2000_4000 as *mut u32;

#[inline]
fn w(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(M.add(i), v) };
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

/// Echo every received frame back on the node's TX COB-ID (0x581 for node 1),
/// and publish RX liveness markers.
#[embassy_executor::task]
async fn comms() {
    let mut rx: u32 = 0;
    loop {
        let f = can::recv().await;
        rx = rx.wrapping_add(1);
        w(7, rx);
        w(5, f.id as u32);
        w(1, Instant::now().as_millis() as u32);
        if let Some(echo) = Frame::new(0x581, f.payload()) {
            let _ = can::send(&echo).await;
        }
    }
}

/// Emit a CANopen-style heartbeat (0x701, one status byte) once a second to
/// prove the async TX path independent of RX.
#[embassy_executor::task]
async fn beat() {
    let mut hb: u32 = 0;
    loop {
        Timer::after_millis(1000).await;
        if let Some(f) = Frame::new(0x701, &[0x05]) {
            let _ = can::send(&f).await;
        }
        hb = hb.wrapping_add(1);
        w(6, 0xCA00_0000 | (hb & 0xFFFF));
        w(1, Instant::now().as_millis() as u32);
    }
}

#[entry]
fn main() -> ! {
    w(0, 0xCA5C_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let clocks = dp
        .rcc
        .constrain()
        .cfgr
        .use_hse(8_000_000.Hz())
        .sysclk(8_000_000.Hz())
        .hclk(4_000_000.Hz())
        .pclk1(4_000_000.Hz())
        .freeze();

    // Time base, self-calibrated from the frozen clocks (exercises init_from_clocks).
    rt::init_time_driver_from_clocks(&clocks);

    // CAN pins + peripheral, then install the async driver.
    let gpioa = dp.gpioa.split();
    let hal_can = Can::new(dp.can);
    hal_can.assign_pins((gpioa.pa11, gpioa.pa12));
    can::init(hal_can, can::BTR_500K_PCLK4);

    let ex = EXECUTOR.init(Executor::new());
    ex.run(|s| {
        s.spawn(comms().unwrap());
        s.spawn(beat().unwrap());
    });
}
