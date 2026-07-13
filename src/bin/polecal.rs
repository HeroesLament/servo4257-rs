#![no_std]
#![no_main]
//! Pole-pair calibration sweep. Align, then step the applied field through
//! several full electrical cycles in discrete 45° jumps (which the rotor
//! follows), logging the encoder at each. The encoder travel over N electrical
//! cycles gives the true pole-pair count: |Δenc| = N·65536 / POLE_PAIRS, so
//! POLE_PAIRS = N·65536 / |Δenc|.
//!
//! OTA app: telemetry 0x181 — during sweep 0xCA [step:u8, enc:u16]; when done
//! 0xCB [enc_start:u16, enc_end:u16, cycles:u8]. poll_reflash throughout.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

const AMP: i16 = 6000;
const STEP: u16 = 8192; // 45° electrical
const NSTEPS: u32 = 40; // 40 × 45° = 5 electrical cycles
const HOLD_MS: u32 = 250;

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

#[entry]
fn main() -> ! {
    w(0, 0x0CA1_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);

    // Align to field 0.
    let (va, vb) = vector(0, AMP);
    board.apply_coil_voltages(va, vb);
    board.set_output_enable(true);
    delay_ms(800);
    let enc_start = board.rotor_angle();
    w(6, enc_start as u32);

    // Sweep the field forward in 45° jumps.
    let mut field: u32 = 0;
    for step in 0..NSTEPS {
        let a = (field & 0xFFFF) as u16;
        let (va, vb) = vector(a, AMP);
        board.apply_coil_voltages(va, vb);
        for _ in 0..(HOLD_MS / 20) {
            board.poll_reflash();
            delay_ms(20);
        }
        let enc = board.rotor_angle();
        w(2, enc as u32);
        let e = enc.to_le_bytes();
        board.telemetry(TELEM_ID, &[0xCA, step as u8, e[0], e[1], 0, 0, 0, 0]);
        field += STEP as u32;
    }

    let enc_end = board.rotor_angle();
    w(7, enc_end as u32);
    let cycles = (NSTEPS / 8) as u8; // 8 × 45° = one cycle

    // Report result forever.
    loop {
        board.poll_reflash();
        let s = enc_start.to_le_bytes();
        let e = enc_end.to_le_bytes();
        board.telemetry(TELEM_ID, &[0xCB, s[0], s[1], e[0], e[1], cycles, 0, 0]);
        delay_ms(100);
    }
}
