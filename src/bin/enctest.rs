#![no_std]
#![no_main]
//! Encoder read test — reads the MT6816 (SPI1) and telemeters the raw rotor
//! angle so we can confirm it tracks the shaft. Output stage stays OFF, so the
//! motor is free to turn by hand. Prerequisite for closed-loop commutation.
//!
//! OTA app (layout-app): telemetry 0x181 phase 0xE0, [angle:u16 at bytes 2..4];
//! also [2] = angle in SWD markers. poll_reflash for over-CAN updates.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;

const TELEM_ID: u16 = 0x181;
const SYSCLK_HZ: u32 = 64_000_000;
const M: *mut u32 = 0x2000_4000 as *mut u32;

#[inline]
fn w(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(M.add(i), v) };
}

#[inline]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay((SYSCLK_HZ / 1000) * ms);
}

#[entry]
fn main() -> ! {
    w(0, 0xE000_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(false); // motor free to turn by hand

    loop {
        board.poll_reflash();
        let ang = board.rotor_angle();
        w(2, ang as u32);
        let a = ang.to_le_bytes();
        board.telemetry(TELEM_ID, &[0xE0, 0, a[0], a[1], 0, 0, 0, 0]);
        delay_ms(20);
    }
}
