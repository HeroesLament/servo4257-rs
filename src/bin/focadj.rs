#![no_std]
#![no_main]
//! focadj — CAN-tunable sensored commutation, for *finding* the spin.
//!
//! Runs a tight on-MCU commutation loop (tens of kHz) exactly like `foc_spin`,
//! but with AMP, LEAD, and encoder DIRECTION adjustable LIVE over CAN. The host
//! sweeps LEAD (and flips DIR) while watching the velocity telemetry until the
//! rotor breaks into continuous rotation — which pins the encoder→electrical map
//! (direction sign + torque angle) empirically, at full commutation rate, with
//! zero reflashes per guess.
//!
//! Commutation each tick:  field = theta_e(enc, offset, dir) + LEAD
//!   dir 0:  theta_e = enc·PP − offset      (encoder counts WITH electrical)
//!   dir 1:  theta_e = offset − enc·PP      (encoder counts AGAINST electrical)
//! If dir/pole-pairs are right, some LEAD in (0°,180°) gives constant +torque →
//! spin; the mirror gives −spin; near 0/180° it parks. If dir is wrong, EVERY
//! lead parks (field−rotor ∝ sin(lead−2θ)). So: sweep lead for dir 0, then dir 1.
//!
//! Commands (0x610, byte0 = op):
//!   0x00 STOP              output off
//!   0x01 ENABLE            commutate with current params
//!   0x10 AMP   [i16 LE]    vector amplitude
//!   0x11 LEAD  [u16 LE]    field lead angle (electrical, u16 = full circle)
//!   0x12 DIR   [u8 0|1]    encoder direction sign
//!   0x13 REALIGN           drive field 0, settle, sample enc → offset (blocking)
//! Telemetry 0x181 @ ~40 Hz: [0xF0, dir, enc:u16, theta_e:u16, vel:i16] LE.
//! vel = Δenc over the telemetry interval (signed) — nonzero & growing = spinning.
//! Also answers the 0x1F51 enter-update for over-CAN reflash.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::encoder::POLE_PAIRS;
use servo4257_rs::motion::trig::sin_cos;

const CMD_ID: u16 = 0x610;
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
fn theta_e(enc: u16, offset: u16, dir: u8) -> u16 {
    let e = enc.wrapping_mul(POLE_PAIRS);
    if dir == 0 {
        e.wrapping_sub(offset)
    } else {
        offset.wrapping_sub(e)
    }
}

#[entry]
fn main() -> ! {
    w(0, 0xADD0_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(false);

    let mut enabled = false;
    let mut amp: i16 = 4000;
    let mut lead: u16 = 16384; // 90°
    let mut dir: u8 = 0;
    let mut offset: u16 = 0;
    let mut last_enc: u16 = 0;
    let mut tick: u32 = 0;

    loop {
        // ---- CAN command dispatch (also enter-update reflash) ----
        if let Some((id, d, len)) = board.can_recv() {
            if id == 0x601 && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                board.set_output_enable(false);
                board.reboot_to_bootloader();
            } else if id == CMD_ID && len >= 1 {
                match d[0] {
                    0x00 => enabled = false,
                    0x01 => enabled = true,
                    0x10 => amp = i16::from_le_bytes([d[1], d[2]]),
                    0x11 => lead = u16::from_le_bytes([d[1], d[2]]),
                    0x12 => dir = d[1] & 1,
                    0x13 => {
                        // Align: drive electrical zero, settle, sample encoder.
                        board.set_output_enable(true);
                        let (va, vb) = vector(0, amp);
                        board.apply_coil_voltages(va, vb);
                        delay_ms(600);
                        let enc0 = board.rotor_angle();
                        offset = enc0.wrapping_mul(POLE_PAIRS); // theta_e(enc0)=0
                        last_enc = enc0;
                        w(1, offset as u32);
                    }
                    _ => {}
                }
            }
        }

        if !enabled {
            board.set_output_enable(false);
            board.apply_coil_voltages(0, 0);
            tick = tick.wrapping_add(1);
            delay_ms(1);
            continue;
        }

        // ---- commutate: field held LEAD ahead of the measured rotor angle ----
        board.set_output_enable(true);
        let enc = board.rotor_angle();
        let th = theta_e(enc, offset, dir);
        let (va, vb) = vector(th.wrapping_add(lead), amp);
        board.apply_coil_voltages(va, vb);

        w(2, enc as u32);
        w(3, th as u32);

        if tick % 400 == 0 {
            let vel = enc.wrapping_sub(last_enc); // signed Δenc per telemetry interval
            last_enc = enc;
            let e = enc.to_le_bytes();
            let t = th.to_le_bytes();
            let v = vel.to_le_bytes();
            board.telemetry(TELEM_ID, &[0xF0, dir, e[0], e[1], t[0], t[1], v[0], v[1]]);
        }
        tick = tick.wrapping_add(1);

        cortex_m::asm::delay(2000); // pace loop; ~15–20 kHz with the SPI read
    }
}
