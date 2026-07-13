#![no_std]
#![no_main]
//! Open-loop microstep spin — the FIRST time the motor energizes.
//!
//! Drives a rotating voltage vector with NO current/position feedback: for a
//! 2-phase motor, `(v_a, v_b) = amp·(cos θ, sin θ)` and we advance θ slowly.
//! This is exactly microstepping, and it validates the 64 MHz clock, the TIM3
//! bridge PWM, the H-bridge power stage, and the coil phase order — everything
//! the closed current loop will sit on top of.
//!
//! ## SAFETY — read before flashing
//! Flashing this WILL energize the motor a few seconds after reset. Before you
//! run it:
//!   * Power the drive from a **current-limited bench supply** set LOW (~0.5 A).
//!     That supply's fold-back is the real safety net, not the firmware.
//!   * Start with the shaft free / lightly loaded.
//!   * `AMP` starts deliberately tiny (~3.7% of bridge voltage). Raise it only
//!     while watching the current markers and the supply.
//! The firmware also captures the zero-current ADC baseline and trips the output
//! stage OFF if either coil current deviates past `TRIP_COUNTS` — an approximate
//! backstop (the raw ADC scale isn't calibrated yet).
//!
//! Staging: A) enable stage, zero vector (baseline) → B) static hold at θ=0 →
//! C) slow rotation. Watch it over SWD.
//!
//! SWD markers @0x2000_4000:
//!   [0] 0x5900_0001 magic
//!   [1] phase: 0x0A zero · 0x0B hold · 0x0C spin · 0xFA17 TRIP/fault
//!   [2] electrical angle (u16)
//!   [3] coil-A current (raw ADC counts)
//!   [4] coil-B current (raw ADC counts)
//!   [6] baseline (zero-current) coil-A counts
//!   [7] baseline (zero-current) coil-B counts

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

/// Applied voltage-vector amplitude, in `i16` CoilCommand units.
/// 4000 ≈ 12% of full bridge voltage (~0.3-0.4 A on a NEMA23 at 12 V). Raised
/// from 1200 to get past detent torque / test weak-vs-coil-B-dead. Raise further
/// only while watching current; coil B's current is unmonitored (dead sense).
const AMP: i16 = 4000;
/// Rotation speed for phase C, in electrical revolutions per second (slow).
const SPIN_ELEC_HZ: f32 = 2.0;
/// Over-current trip: |raw ADC − baseline| in counts. Approximate backstop.
const TRIP_COUNTS: i32 = 450;

/// Telemetry COB-ID (TPDO1-style for node 1). Frame layout (8 bytes):
///   [0] phase  [1] pad  [2..4] angle u16 LE  [4..6] iA u16 LE  [6..8] iB u16 LE
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

/// Voltage vector at electrical angle `a`, amplitude `amp`: (amp·cos, amp·sin).
#[inline]
fn vector(a: u16, amp: i16) -> (i16, i16) {
    let (s, c) = sin_cos(a);
    let amp = amp as f32;
    ((amp * c) as i16, (amp * s) as i16)
}

#[inline]
fn tripped(cur: i16, base: i32) -> bool {
    ((cur as i32) - base).abs() > TRIP_COUNTS
}

/// Pack and transmit one telemetry frame (best-effort over CAN).
#[inline]
fn telem(board: &mut ActiveBoard, phase: u8, angle: u16, ia: i16, ib: i16) {
    let a = angle.to_le_bytes();
    let ca = (ia as u16).to_le_bytes();
    let cb = (ib as u16).to_le_bytes();
    board.telemetry(TELEM_ID, &[phase, 0, a[0], a[1], ca[0], ca[1], cb[0], cb[1]]);
}

/// Kill the output stage and park forever.
fn fault(board: &mut ActiveBoard) -> ! {
    board.apply_coil_voltages(0, 0);
    board.set_output_enable(false);
    w(1, 0xFA17);
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    w(0, 0x5900_0001);

    let dp = pac::Peripherals::take().unwrap();
    // init() sets the 64 MHz clock tree and comes up with the stage DISABLED.
    let mut board = ActiveBoard::init(dp);

    // ---- Phase A: enable stage with a ZERO vector (both coils mid-duty, no
    // differential → no net current). Capture the zero-current ADC baseline. ----
    board.apply_coil_voltages(0, 0);
    board.set_output_enable(true);
    w(1, 0x0A);
    delay_ms(500); // let the injected ADC settle

    let (ba, bb) = board.read_coil_currents();
    let base_a = ba as i32;
    let base_b = bb as i32;
    w(6, ba as u32 & 0xFFFF);
    w(7, bb as u32 & 0xFFFF);

    // Hold zero for ~2.5 s, watching that nothing is flowing unexpectedly.
    for _ in 0..25 {
        delay_ms(100);
        let (ia, ib) = board.read_coil_currents();
        w(3, ia as u32 & 0xFFFF);
        w(4, ib as u32 & 0xFFFF);
        telem(&mut board, 0x0A, 0, ia, ib);
        if tripped(ia, base_a) || tripped(ib, base_b) {
            fault(&mut board);
        }
    }

    // ---- Phase B: static vector at θ=0, amplitude AMP. The rotor should snap
    // to and hold a detent; coil A carries a DC current, coil B ~zero. ----
    w(1, 0x0B);
    w(2, 0);
    let (va, vb) = vector(0, AMP);
    board.apply_coil_voltages(va, vb);
    for _ in 0..30 {
        delay_ms(100);
        let (ia, ib) = board.read_coil_currents();
        w(3, ia as u32 & 0xFFFF);
        w(4, ib as u32 & 0xFFFF);
        telem(&mut board, 0x0B, 0, ia, ib);
        if tripped(ia, base_a) || tripped(ib, base_b) {
            fault(&mut board);
        }
    }

    // ---- Phase C: slow open-loop rotation. ----
    w(1, 0x0C);
    // Per-1 ms angle step: 65536 · SPIN_ELEC_HZ / 1000 (u16 units per ms).
    let dstep = (65536.0 * SPIN_ELEC_HZ / 1000.0) as u32;
    let mut angle: u32 = 0;
    let mut tick: u32 = 0;
    loop {
        let a16 = (angle & 0xFFFF) as u16;
        let (va, vb) = vector(a16, AMP);
        board.apply_coil_voltages(va, vb);
        w(2, a16 as u32);

        let (ia, ib) = board.read_coil_currents();
        w(3, ia as u32 & 0xFFFF);
        w(4, ib as u32 & 0xFFFF);
        if tripped(ia, base_a) || tripped(ib, base_b) {
            fault(&mut board);
        }
        // Telemetry every ~50 ms (the loop is 1 ms).
        if tick % 50 == 0 {
            telem(&mut board, 0x0C, a16, ia, ib);
        }
        tick = tick.wrapping_add(1);

        angle = angle.wrapping_add(dstep);
        delay_ms(1);
    }
}
