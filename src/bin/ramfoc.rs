#![no_std]
#![no_main]
//! ramfoc — motor commutation driven over SWD via a RAM mailbox (no CAN needed).
//!
//! The host sets drive parameters with `probe-rs write` and reads telemetry with
//! `probe-rs read`, over the rock-solid ST-Link — the same control loop we had
//! over CAN, but on the debug link. Standalone layout (runs from flash base, no
//! bootloader), so `probe-rs download` + `reset` flashes and runs it directly.
//!
//! COMMAND mailbox @ 0x2000_4200 (host writes; app reads every tick):
//!   [0] enable  u32  0/1  — 0 = output off (also the reset-safe default)
//!   [1] mode    u32       — 0 = idle, 1 = sensored commutate, 2 = open-loop vector
//!   [2] amp     i32       — vector amplitude
//!   [3] lead    u32       — field lead angle (electrical, u16 range)
//!   [4] dir     u32  0/1  — encoder direction sign
//!   [5] angle   u32       — open-loop commanded field angle (mode 2)
//!   [6] offset  u32       — electrical offset (sensored)
//!
//! TELEMETRY @ 0x2000_4000 (host reads):
//!   [0] tick u32 (increments — proves the loop is alive)
//!   [1] enc u32   [2] theta_e u32   [3] vel (i16 in low bits)
//!   [4] iA u32    [5] iB u32

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::encoder::POLE_PAIRS;
use servo4257_rs::motion::trig::sin_cos;

const TELEM: *mut u32 = 0x2000_4000 as *mut u32;
const CMD: *mut u32 = 0x2000_4200 as *mut u32;

#[inline]
fn tw(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(TELEM.add(i), v) }
}
#[inline]
fn cr(i: usize) -> u32 {
    unsafe { core::ptr::read_volatile(CMD.add(i)) }
}
#[inline]
fn cw(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(CMD.add(i), v) }
}

#[inline]
fn vector(a: u16, amp: i16) -> (i16, i16) {
    let (s, c) = sin_cos(a);
    let amp = amp as f32;
    ((amp * c) as i16, (amp * s) as i16)
}

#[inline]
fn theta_e(enc: u16, offset: u16, dir: u32) -> u16 {
    let e = enc.wrapping_mul(POLE_PAIRS);
    if dir == 0 {
        e.wrapping_sub(offset)
    } else {
        offset.wrapping_sub(e)
    }
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(false);

    // Reset-safe command defaults so we never drive on uninitialized RAM.
    cw(0, 0); // enable off
    cw(1, 1); // mode = sensored
    cw(2, 4000); // amp
    cw(3, 16384); // lead 90 deg
    cw(4, 1); // dir
    cw(5, 0); // open-loop angle
    cw(6, 0); // offset

    tw(0, 0);
    let mut last_enc: u16 = 0;
    let mut tick: u32 = 0;

    loop {
        let enable = cr(0) != 0;
        let mode = cr(1);
        let amp = cr(2) as i32 as i16;
        let lead = cr(3) as u16;
        let dir = cr(4);
        let ol_angle = cr(5) as u16;
        let offset = cr(6) as u16;

        board.set_output_enable(enable);

        let enc = board.rotor_angle();
        let th = theta_e(enc, offset, dir);

        if enable {
            match mode {
                1 => {
                    let (va, vb) = vector(th.wrapping_add(lead), amp);
                    board.apply_coil_voltages(va, vb);
                }
                2 => {
                    let (va, vb) = vector(ol_angle, amp);
                    board.apply_coil_voltages(va, vb);
                }
                _ => board.apply_coil_voltages(0, 0),
            }
        } else {
            board.apply_coil_voltages(0, 0);
        }

        if tick % 400 == 0 {
            let vel = enc.wrapping_sub(last_enc);
            last_enc = enc;
            let (ia, ib) = board.read_coil_currents();
            tw(1, enc as u32);
            tw(2, th as u32);
            tw(3, vel as u32);
            tw(4, (ia as u16) as u32);
            tw(5, (ib as u16) as u32);
            tw(0, tick); // write tick last: a changing tick means fresh telemetry
        }
        tick = tick.wrapping_add(1);

        cortex_m::asm::delay(2000); // pace loop ~15-20 kHz
    }
}
