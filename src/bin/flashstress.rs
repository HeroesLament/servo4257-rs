#![no_std]
#![no_main]
//! Flash reliability stress at the bootloader's clock (HSE/2 = 4 MHz, icache/prefetch off).
//! Loops erase+program+verify on the META scratch page N times, then parks.
//! Status @ 0x2000_1000 over SWD:
//!   [0] 0xF1A5_0002 magic
//!   [1] completed iteration count
//!   [2] stage: 0xE1 erasing, 0xE2 erased, 0xF3 programming, 0xF4 programmed
//!   [3] error: 0=ok, 0xEE erase-err, 0xFF write-err, 0x0BAD verify-mismatch
//!   [7] 0xF1A5_D02E sentinel (only if all N done)
//! SWD alive + [7] set = flash reliable at 4 MHz. SWD dead mid-run = it hung.

use panic_halt as _;
use n32l4 as _;
use n32l4xx_hal::fmc::{FMCExt, Flash};
use n32l4xx_hal::pac;
use n32l4xx_hal::prelude::*;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

const STATUS: *mut u32 = 0x2000_1000 as *mut u32;
const TEST_OFFSET: u32 = 0x0001_F800;
const ITERS: u32 = 300;
const PATTERN: [u32; 4] = [0xDEAD_BEEF, 0xCAFE_BABE, 0x1234_5678, 0xA5A5_5A5A];

fn set(idx: usize, val: u32) {
    unsafe { core::ptr::write_volatile(STATUS.add(idx), val) }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    set(0, 0xF1A5_0002);
    set(1, 0);
    set(2, 0);
    set(3, 0);
    set(7, 0);

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
    unsafe {
        (*pac::Flash::ptr())
            .ac()
            .modify(|_, w| w.icahen().clear_bit().prftbfe().clear_bit());
    }

    let mut flash: Flash = dp.flash.constrain();

    let mut bytes = [0u8; 16];
    for (i, w) in PATTERN.iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }

    let mut n = 0u32;
    while n < ITERS {
        set(2, 0xE1);
        if flash.erase(TEST_OFFSET, TEST_OFFSET + Flash::ERASE_SIZE as u32).is_err() {
            set(3, 0xEE);
            break;
        }
        set(2, 0xE2);
        set(2, 0xF3);
        if flash.write(TEST_OFFSET, &bytes).is_err() {
            set(3, 0xFF);
            break;
        }
        set(2, 0xF4);
        let mut rb = [0u8; 16];
        let _ = flash.read(TEST_OFFSET, &mut rb);
        if rb != bytes {
            set(3, 0x0BAD);
            break;
        }
        n += 1;
        set(1, n);
    }
    set(7, 0xF1A5_D02E);
    loop {
        cortex_m::asm::nop();
    }
}
