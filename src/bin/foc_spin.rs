#![no_std]
#![no_main]
//! Sensored voltage-mode commutation — closed-loop, encoder-based spin.
//!
//! 1) ALIGN: drive the field at electrical zero, let the rotor settle, read the
//!    encoder there → the electrical offset.
//! 2) COMMUTATE: each tick, read the encoder, compute the rotor electrical angle,
//!    and apply the voltage vector 90° ahead of it. The field tracks the rotor,
//!    so it self-commutates — no stall, no open-loop start problem. It spins up
//!    to a terminal velocity set by AMP vs. friction: smooth, constant rotation.
//!
//! No current sensing needed (it's unreliable here). OTA app: telemetry on 0x181
//! (phase 0xF0: [enc:u16, theta_e:u16, vel:u16]) + poll_reflash.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::encoder::{offset_from_alignment, POLE_PAIRS};
use servo4257_rs::motion::trig::sin_cos;

/// Applied vector amplitude. Terminal speed scales with this; supply limit caps it.
/// Raised from 3000: with the split_duty scaling (mag = |v|·max_duty/i16::MAX),
/// 3000 is only ~9% duty — not enough torque to break static friction/detent, so
/// the rotor just holds the align point. 12000 (~37% duty) develops real torque.
const AMP: i16 = 12000;
/// Field lead angle. Empirically, LEAD=16384 (90°) landed the field ON the
/// rotor → a stable servo hold (field−rotor=0), so the alignment reference sits
/// a quarter-cycle off. Shift another 90° → LEAD=32768 gives true quadrature
/// (field−rotor=+90°) = max constant torque → continuous rotation.
const LEAD: u16 = 32768;

/// Rotor electrical angle with the encoder direction REVERSED — the MT6816
/// counts opposite to this motor's electrical rotation, so we negate the
/// encoder term (`offset − enc·pole_pairs`). Without this, `field − rotor`
/// varies as sin(lead − 2θ) and the rotor parks instead of spinning.
#[inline]
fn theta_e_rev(enc: u16, offset: u16) -> u16 {
    offset.wrapping_sub(enc.wrapping_mul(POLE_PAIRS))
}

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
    w(0, 0xF0C0_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);

    // ---- Align: field at electrical zero, let the rotor settle, sample enc ----
    let (va, vb) = vector(0, AMP);
    board.apply_coil_voltages(va, vb);
    board.set_output_enable(true);
    delay_ms(800);
    let enc0 = board.rotor_angle();
    let offset = offset_from_alignment(enc0);
    w(1, offset as u32);

    // ---- Commutate: hold the field LEAD ahead of the rotor ----
    let mut last_enc = board.rotor_angle();
    let mut tick: u32 = 0;
    loop {
        board.poll_reflash();

        let enc = board.rotor_angle();
        let theta_e = theta_e_rev(enc, offset);
        let (va, vb) = vector(theta_e.wrapping_add(LEAD), AMP);
        board.apply_coil_voltages(va, vb);

        w(2, enc as u32);
        w(3, theta_e as u32);

        if tick % 200 == 0 {
            let vel = enc.wrapping_sub(last_enc); // enc delta per ~200 ticks
            last_enc = enc;
            let e = enc.to_le_bytes();
            let t = theta_e.to_le_bytes();
            let v = vel.to_le_bytes();
            board.telemetry(TELEM_ID, &[0xF0, 0, e[0], e[1], t[0], t[1], v[0], v[1]]);
        }
        tick = tick.wrapping_add(1);

        cortex_m::asm::delay(3200); // ~50 µs → ~20 kHz commutation
    }
}
