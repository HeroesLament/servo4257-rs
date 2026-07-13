//! Minimal app that proves the purely-over-CAN dev loop. Brings up CAN, then on
//! any SDO access to index 0x1F51 (CiA-302 program control) from the master,
//! sets the shared STAY flag and resets into the bootloader — no SWD needed.
//! This is the seed of the real app's reflash handoff.
//!
//! SWD markers @0x2000_4000: [0] 0x0A99_0001 running / 0xB007_0000 handing off;
//! [6] 0xCA00_0000 CAN up; [7] rx frame count.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::can::Can;
use n32l4xx_hal::gpio::GpioExt;
use n32l4xx_hal::pac;
use n32l4xx_hal::prelude::*;
use panic_halt as _;
use servo4257_rs::canopen::transport::{BxCanTransport, CanTransport};

const STATUS: *mut u32 = 0x2000_4000 as *mut u32;
const BOOT_FLAG: *mut u32 = 0x2000_5FF8 as *mut u32;
const FLAG_STAY_IN_BOOT: u32 = 0xB007_57A4;

fn set(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(STATUS.add(i), v) };
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let _clocks = dp
        .rcc
        .constrain()
        .cfgr
        .use_hse(8_000_000.Hz())
        .sysclk(8_000_000.Hz())
        .hclk(4_000_000.Hz())
        .pclk1(4_000_000.Hz())
        .freeze();

    let gpioa = dp.gpioa.split();
    let hal_can = Can::new(dp.can);
    hal_can.assign_pins((gpioa.pa11, gpioa.pa12));
    let mut can = BxCanTransport::new(hal_can);

    set(0, 0x0A99_0001); // app stub running
    set(6, 0xCA00_0000); // CAN up

    let mut rx: u32 = 0;
    loop {
        if let Ok(Some(f)) = can.try_recv() {
            rx = rx.wrapping_add(1);
            set(7, rx);
            // CiA-302 "enter update mode": any SDO access (0x601) to 0x1F51.
            if f.id == 0x601 && f.len >= 3 && f.data[1] == 0x51 && f.data[2] == 0x1F {
                set(0, 0xB007_0000); // handing off to bootloader
                unsafe { core::ptr::write_volatile(BOOT_FLAG, FLAG_STAY_IN_BOOT) };
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }
}
