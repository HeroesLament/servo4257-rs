#![no_std]
#![no_main]
//! Open-loop microstep spin, **as a CiA-302 app** — the OTA-updatable motor
//! test. Same staged drive + CAN telemetry as `spin_openloop`, but built for
//! `layout-app` (loads at 0x08004000, under the bootloader) and it calls
//! `board.poll_reflash()` every loop: an SDO write to 0x1F51 over CAN safes the
//! bridge and reboots into the bootloader, so every future iteration flashes
//! over CAN with no SWD.
//!
//! Drives `(v_a, v_b) = amp·(cos θ, sin θ)` (2-phase microstepping), advancing θ
//! slowly. Comes up with the output stage disabled; energizes staged.
//!
//! SWD/CAN markers @0x2000_4000 (telemetry on COB-ID 0x181,
//! [phase, pad, angle:u16, iA:u16, iB:u16] LE):
//!   [0] 0x5A00_0001 magic   [1] phase 0x0A/0x0B/0x0C, 0xFA17 trip
//!   [2] angle   [3] iA raw   [4] iB raw   [6]/[7] baseline iA/iB

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

/// Voltage-vector amplitude (i16 CoilCommand). Kept low so both coils together
/// stay within the supply budget (no diagonal fold-back). Open-loop voltage
/// drive is R/Vbus-sensitive — the current loop will fix this properly later.
const AMP: i16 = 3000;
/// Rotation speed for phase C (electrical rev/s). Slow — open-loop must start
/// gently or the rotor stalls and dithers instead of tracking the field.
const SPIN_ELEC_HZ: f32 = 0.5;
/// Over-current trip: |raw ADC − baseline| counts (coil A; coil B sense is dead).
const TRIP_COUNTS: i32 = 450;
/// Telemetry COB-ID; frame = [phase, pad, angle:u16, iA:u16, iB:u16] LE.
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
fn tripped(cur: i16, base: i32) -> bool {
    ((cur as i32) - base).abs() > TRIP_COUNTS
}

#[inline]
fn telem(board: &mut ActiveBoard, phase: u8, angle: u16, ia: i16, ib: i16) {
    let a = angle.to_le_bytes();
    let ca = (ia as u16).to_le_bytes();
    let cb = (ib as u16).to_le_bytes();
    board.telemetry(TELEM_ID, &[phase, 0, a[0], a[1], ca[0], ca[1], cb[0], cb[1]]);
}

fn fault(board: &mut ActiveBoard) -> ! {
    board.apply_coil_voltages(0, 0);
    board.set_output_enable(false);
    w(1, 0xFA17);
    // Keep answering enter-update so we can still reflash out of a fault.
    loop {
        board.poll_reflash();
        cortex_m::asm::delay(64_000);
    }
}

#[entry]
fn main() -> ! {
    w(0, 0x5A00_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp); // 64 MHz clock; stage DISABLED

    // ---- Phase A: enable stage, ZERO vector, capture zero-current baseline ----
    board.apply_coil_voltages(0, 0);
    board.set_output_enable(true);
    w(1, 0x0A);
    delay_ms(500);

    let (ba, bb) = board.read_coil_currents();
    let base_a = ba as i32;
    let base_b = bb as i32;
    w(6, ba as u32 & 0xFFFF);
    w(7, bb as u32 & 0xFFFF);

    for _ in 0..25 {
        board.poll_reflash();
        delay_ms(100);
        let (ia, ib) = board.read_coil_currents();
        w(3, ia as u32 & 0xFFFF);
        w(4, ib as u32 & 0xFFFF);
        telem(&mut board, 0x0A, 0, ia, ib);
        if tripped(ia, base_a) || tripped(ib, base_b) {
            fault(&mut board);
        }
    }

    // ---- Phase B: static hold at θ=0 ----
    w(1, 0x0B);
    w(2, 0);
    let (va, vb) = vector(0, AMP);
    board.apply_coil_voltages(va, vb);
    for _ in 0..30 {
        board.poll_reflash();
        delay_ms(100);
        let (ia, ib) = board.read_coil_currents();
        w(3, ia as u32 & 0xFFFF);
        w(4, ib as u32 & 0xFFFF);
        telem(&mut board, 0x0B, 0, ia, ib);
        if tripped(ia, base_a) || tripped(ib, base_b) {
            fault(&mut board);
        }
    }

    // ---- Phase C: slow open-loop rotation ----
    w(1, 0x0C);
    let dstep = (65536.0 * SPIN_ELEC_HZ / 1000.0) as u32;
    let mut angle: u32 = 0;
    let mut tick: u32 = 0;
    loop {
        board.poll_reflash();

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
        if tick % 50 == 0 {
            telem(&mut board, 0x0C, a16, ia, ib);
        }
        tick = tick.wrapping_add(1);

        angle = angle.wrapping_add(dstep);
        delay_ms(1);
    }
}
