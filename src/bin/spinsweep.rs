#![no_std]
#![no_main]
//! spinsweep — find the correct commutation phase combination for clean rotation.
//!
//! All prior attempts librated or froze because the phase relationship between
//! the two coils, the encoder count direction, and the quadrature-lead sign was
//! wrong. There are only 3 binary axes → 8 combinations; exactly one commutates
//! correctly. This drives CLOSED-LOOP (field placed 90° ahead of the LIVE rotor
//! electrical angle — no pull-out limit, the only mode that sustains rotation)
//! with a commanded combination, so we can step through all 8 and watch which
//! makes the encoder advance steadily.
//!
//! theta_elec = (dir_enc ? +enc : -enc) * POLE_PAIRS
//! field      = theta_elec + (lead_sign ? +QUARTER : -QUARTER)
//! vb sign    = (invert_b ? -vb : vb)
//! The rotor chases the quadrature field → continuous rotation when the combo is
//! right; librates/freezes otherwise.
//!
//! Commands (0x614, byte0):
//!   0x00 OFF
//!   0x01 [amp:i16, combo:u8]  drive; combo bit0=invert_b, bit1=dir_enc,
//!                             bit2=lead_sign (0..7)
//! Telemetry 0x187: [enc:u16, combo:u8, 0, 0, 0, 0] LE. Answers 0x1F51.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::encoder::POLE_PAIRS;
use servo4257_rs::motion::trig::sin_cos;

const CMD_ID: u16 = 0x614;
const TELEM_ID: u16 = 0x187;
const SYSCLK_HZ: u32 = 64_000_000;
const QUARTER: u16 = 16384; // 90° electrical

#[inline]
fn i16le(a: u8, b: u8) -> i16 {
    i16::from_le_bytes([a, b])
}

#[inline]
fn delay_us(us: u32) {
    cortex_m::asm::delay((SYSCLK_HZ / 1_000_000) * us);
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(false);

    let mut amp: i16 = 0;
    let mut combo: u8 = 0;
    let mut on = false;
    let mut prev_on = false;
    // Startup align: on the enable edge, hold a FIXED field to seat the rotor at
    // a known electrical angle, THEN begin closed-loop commutation. From that
    // known start the +90° lead is always on the accelerating side → spins up
    // every time (no coin-flip libration from standstill).
    let mut align_ticks: u32 = 0;
    const ALIGN_TICKS: u32 = 8000; // ~0.4s of align hold at ~20kHz

    // Firmware-measured velocity: accumulate signed unwrapped enc delta over a
    // fixed window (immune to the host's aliased CAN sampling). Reported as a
    // signed i16 = net encoder counts moved per ~telemetry window.
    let mut last_enc: u16 = 0;
    let mut vel_acc: i32 = 0;

    let mut tick: u32 = 0;
    loop {
        if tick % 64 == 0 {
            while let Some((id, d, len)) = board.can_recv() {
                if id == 0x601 && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                    board.set_output_enable(false);
                    board.reboot_to_bootloader();
                } else if id == CMD_ID && len >= 1 {
                    match d[0] {
                        0x00 => on = false,
                        0x01 if len >= 4 => {
                            amp = i16le(d[1], d[2]);
                            combo = d[3];
                            on = true;
                        }
                        _ => {}
                    }
                }
            }
        }

        // enable edge → begin align hold
        if on && !prev_on {
            align_ticks = ALIGN_TICKS;
        }
        prev_on = on;

        board.set_output_enable(on);
        let enc = board.rotor_angle();
        vel_acc += (enc.wrapping_sub(last_enc)) as i16 as i32;
        last_enc = enc;

        if on {
            let invert_b = combo & 0x01 != 0;
            let dir_enc = combo & 0x02 != 0;
            let lead_pos = combo & 0x04 != 0;

            if align_ticks > 0 {
                // ALIGN: hold a fixed field (electrical 0) to seat the rotor.
                align_ticks -= 1;
                let (s, c) = sin_cos(0);
                let a = amp as f32;
                let va = (a * c) as i16;
                let mut vb = (a * s) as i16;
                if invert_b {
                    vb = -vb;
                }
                board.apply_coil_voltages(va, vb);
            } else {
                // RUN: closed-loop commutation from the seated start.
                let e = if dir_enc { enc } else { enc.wrapping_neg() };
                let theta = e.wrapping_mul(POLE_PAIRS);
                let field = if lead_pos {
                    theta.wrapping_add(QUARTER)
                } else {
                    theta.wrapping_sub(QUARTER)
                };
                let (s, c) = sin_cos(field);
                let a = amp as f32;
                let va = (a * c) as i16;
                let mut vb = (a * s) as i16;
                if invert_b {
                    vb = -vb;
                }
                board.apply_coil_voltages(va, vb);
            }
        } else {
            board.apply_coil_voltages(0, 0);
        }

        if tick % 512 == 0 {
            // report [enc:u16, combo:u8, _, vel:i16, _] — vel = net counts moved
            // over this 512-tick window (≈25ms). Sign = direction. This is the
            // alias-immune speed measurement to rank combos by.
            let e = enc.to_le_bytes();
            let v = (vel_acc.clamp(-32768, 32767) as i16).to_le_bytes();
            board.telemetry(TELEM_ID, &[e[0], e[1], combo, 0, v[0], v[1], 0, 0]);
            vel_acc = 0;
        }
        tick = tick.wrapping_add(1);
        delay_us(50); // ~20 kHz closed loop
    }
}
