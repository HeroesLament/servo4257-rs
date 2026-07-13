#![no_std]
#![no_main]
//! Phase diagnostic — 4-step full-step commutation to test whether BOTH coils
//! produce torque. Drives +A, +B, −A, −B in turn (electrical 0/90/180/270),
//! each held ~1 s. If both phases drive, the rotor snaps through four positions
//! and walks around (continuous 1.8°/step rotation). If a coil is dead, the
//! rotor only reacts to the other coil's two steps and just stutters.
//!
//! OTA app (layout-app): telemetry on 0x181 with phase byte 0xB0|step, and
//! `poll_reflash` so it can be reflashed over CAN. Amplitude is high on the
//! assumption the bench supply's current limit is the real cap.
//!
//! SAFETY: relies on a current-limited supply. Coil B current is unmonitored.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

/// Drive amplitude — high; the supply current limit is the real cap.
const AMP: i16 = 8000;
/// Hold time per step (ms). Short → continuous wave-drive rotation.
const HOLD_MS: u32 = 60;
/// Electrical angles for +A, +B, −A, −B.
const STEPS: [u16; 4] = [0, 16384, 32768, 49152];

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

#[inline]
fn vector(a: u16, amp: i16) -> (i16, i16) {
    let (s, c) = sin_cos(a);
    let amp = amp as f32;
    ((amp * c) as i16, (amp * s) as i16)
}

#[inline]
fn telem(board: &mut ActiveBoard, phase: u8, angle: u16, ia: i16, ib: i16) {
    let a = angle.to_le_bytes();
    let ca = (ia as u16).to_le_bytes();
    let cb = (ib as u16).to_le_bytes();
    board.telemetry(TELEM_ID, &[phase, 0, a[0], a[1], ca[0], ca[1], cb[0], cb[1]]);
}

#[entry]
fn main() -> ! {
    w(0, 0x5B00_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);

    board.apply_coil_voltages(0, 0);
    board.set_output_enable(true);
    delay_ms(500);
    let (ba, bb) = board.read_coil_currents();
    w(6, ba as u32 & 0xFFFF);
    w(7, bb as u32 & 0xFFFF);

    loop {
        for (i, &ang) in STEPS.iter().enumerate() {
            let (va, vb) = vector(ang, AMP);
            board.apply_coil_voltages(va, vb);
            w(1, 0xB0 | i as u32);
            w(2, ang as u32);
            for _ in 0..(HOLD_MS / 20) {
                board.poll_reflash();
                delay_ms(20);
                let (ia, ib) = board.read_coil_currents();
                w(3, ia as u32 & 0xFFFF);
                w(4, ib as u32 & 0xFFFF);
                telem(&mut board, 0xB0 | i as u8, ang, ia, ib);
            }
        }
    }
}
