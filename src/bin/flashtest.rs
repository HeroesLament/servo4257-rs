#![no_std]
#![no_main]
//! Flash-driver silicon verification harness.
//! Build: cargo build --release --bin flashtest --features board-57d
//! Flash via UART ROM bootloader, power-cycle, read status over SWD at
//! 0x2000_0000 and the written page at 0x0801_F800.
//!
//! Status block (8 x u32 @ 0x2000_0000), little-endian:
//!   [0] 0xF1A5_0001 magic
//!   [1] erase result (0=Ok 1=WriteProtected 2=ProgramError 3=OOB 4=NotAligned)
//!   [2] write result (same codes)
//!   [3..7] readback words (expect DEADBEEF CAFEBABE 12345678 A5A55A5A)
//!   [7] 0xF1A5_D02E completion sentinel

use panic_halt as _;
use n32l4 as _;
use n32l4xx_hal::fmc::{FMCExt, Flash, FlashError};
use n32l4xx_hal::pac;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

const STATUS: *mut u32 = 0x2000_0000 as *mut u32;
const TEST_OFFSET: u32 = 0x0001_F800;
const PATTERN: [u32; 4] = [0xDEAD_BEEF, 0xCAFE_BABE, 0x1234_5678, 0xA5A5_5A5A];

fn err_code(e: &FlashError) -> u32 {
    match e {
        FlashError::WriteProtected => 1,
        FlashError::ProgramError => 2,
        FlashError::OutOfBounds => 3,
        FlashError::NotAligned => 4,
    }
}

fn set(idx: usize, val: u32) {
    unsafe { core::ptr::write_volatile(STATUS.add(idx), val) }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    set(0, 0xF1A5_0001);
    set(1, 0xFFFF_FFFF);
    set(2, 0xFFFF_FFFF);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut flash: Flash = dp.flash.constrain();

    match flash.erase(TEST_OFFSET, TEST_OFFSET + Flash::ERASE_SIZE as u32) {
        Ok(()) => set(1, 0),
        Err(e) => set(1, err_code(&e)),
    }

    let mut bytes = [0u8; 16];
    for (i, w) in PATTERN.iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    match flash.write(TEST_OFFSET, &bytes) {
        Ok(()) => set(2, 0),
        Err(e) => set(2, err_code(&e)),
    }

    let mut rb = [0u8; 16];
    let _ = flash.read(TEST_OFFSET, &mut rb);
    for i in 0..4 {
        let w = u32::from_le_bytes([rb[i*4], rb[i*4+1], rb[i*4+2], rb[i*4+3]]);
        set(3 + i, w);
    }

    set(7, 0xF1A5_D02E);
    loop { cortex_m::asm::nop(); }
}
