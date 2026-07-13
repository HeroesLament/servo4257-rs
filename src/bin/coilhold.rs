#![no_std]
#![no_main]
//! Single-coil static hold — isolates ONE coil to test its holding torque by
//! feel. Applies a fixed voltage vector at `HOLD_ANGLE` and holds it forever
//! (θ=0 → coil A only; θ=90° = 16384 → coil B only). Grab the shaft: a working
//! coil holds it stiff at a detent; a dead/open coil leaves it free.
//!
//! OTA app (layout-app): telemetry on 0x181 + poll_reflash. Amplitude high;
//! bench-supply current limit is the cap.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

/// Hold angle: 0 = +A, 16384 = +B, 32768 = −A, 49152 = −B.
const HOLD_ANGLE: u16 = 32768;
const AMP: i16 = 8000;

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
    w(0, 0x5C00_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);

    let (s, c) = sin_cos(HOLD_ANGLE);
    let va = (AMP as f32 * c) as i16;
    let vb = (AMP as f32 * s) as i16;

    board.apply_coil_voltages(va, vb);
    board.set_output_enable(true);
    w(1, HOLD_ANGLE as u32);

    loop {
        board.poll_reflash();
        delay_ms(20);
        let (ia, ib) = board.read_coil_currents();
        w(3, ia as u32 & 0xFFFF);
        w(4, ib as u32 & 0xFFFF);
        let a = HOLD_ANGLE.to_le_bytes();
        let ca = (ia as u16).to_le_bytes();
        let cb = (ib as u16).to_le_bytes();
        board.telemetry(TELEM_ID, &[0xC0, 0, a[0], a[1], ca[0], ca[1], cb[0], cb[1]]);
    }
}
