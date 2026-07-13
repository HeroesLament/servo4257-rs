#![no_std]
#![no_main]
//! Half-bridge isolation test — drives each of the four half-bridges ALONE for
//! 3 s (a+, then a−, then b+, then b−), so you can feel which ones produce
//! holding torque. If a+ / b+ hold but a− / b− go limp, the "minus" bridges
//! aren't driving — a hardware (gate-driver) fault, not firmware. If all four
//! hold, the drive is fine and the earlier no-reverse result was elsewhere.
//!
//! Each step energizes one half-bridge against its partner held at GND, so the
//! coil sees a DC drive in one direction. OTA app: telemetry 0x181 (phase byte
//! 0xD0|step) + poll_reflash. Amplitude modest; supply current limit is the cap.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;

/// Drive duty as a percent of full scale.
const DRIVE_PCT: u32 = 35;
const HOLD_MS: u32 = 3000;

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
    w(0, 0x5D00_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    let d = (board.max_duty() as u32 * DRIVE_PCT / 100) as u16;

    board.set_raw_duties(0, 0, 0, 0);
    board.set_output_enable(true);

    // (a+, a−, b+, b−) — exactly one half-bridge driven each step.
    let steps: [(u16, u16, u16, u16); 4] =
        [(d, 0, 0, 0), (0, d, 0, 0), (0, 0, d, 0), (0, 0, 0, d)];

    loop {
        for (i, &(ap, am, bp, bm)) in steps.iter().enumerate() {
            board.set_raw_duties(ap, am, bp, bm);
            w(1, 0xD0 | i as u32);
            for _ in 0..(HOLD_MS / 20) {
                board.poll_reflash();
                delay_ms(20);
                let (ia, ib) = board.read_coil_currents();
                w(3, ia as u32 & 0xFFFF);
                w(4, ib as u32 & 0xFFFF);
                let ca = (ia as u16).to_le_bytes();
                let cb = (ib as u16).to_le_bytes();
                board.telemetry(
                    TELEM_ID,
                    &[0xD0 | i as u8, 0, i as u8, 0, ca[0], ca[1], cb[0], cb[1]],
                );
            }
        }
    }
}
