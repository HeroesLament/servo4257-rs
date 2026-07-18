#![no_std]
#![no_main]
//! holdtest — the simplest possible drive-strength probe. Holds a STATIC field
//! vector at full amplitude so you can read the ACTUAL PSU current draw. No
//! commutation, no PID, no encoder — just steady DC into the coils at a fixed
//! electrical angle. Answers the question that's haunted the whole session:
//! when we command full drive, how many amps actually flow?
//!
//! Commands (0x613, byte0):
//!   0x00 OFF          de-energize
//!   0x01 [amp_lo,amp_hi, ang_lo,ang_hi]  hold field at electrical `ang` (u16),
//!                                         amplitude `amp` (i16). Default on
//!                                         boot: OFF (no heat).
//! Telemetry 0x186: [enc:u16, amp:u16, ang:u16, 0,0] LE.
//! Answers 0x1F51 enter-update.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

const CMD_ID: u16 = 0x613;
const TELEM_ID: u16 = 0x186;
const SYSCLK_HZ: u32 = 64_000_000;

#[inline]
fn vector(theta_e: u16, amp: i16) -> (i16, i16) {
    let (s, c) = sin_cos(theta_e);
    let amp = amp as f32;
    ((amp * c) as i16, (amp * s) as i16)
}

#[inline]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay((SYSCLK_HZ / 1000) * ms);
}

#[inline]
fn i16le(a: u8, b: u8) -> i16 {
    i16::from_le_bytes([a, b])
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(false);

    let mut amp: i16 = 0;
    // Field angle carried as a 32-bit fixed-point phase (high 16 bits = the u16
    // electrical angle, low 16 bits = fraction). This lets the field advance a
    // SMOOTH sub-unit amount every fast tick, instead of a coarse jump — the key
    // to visibly smooth rotation with no jerk.
    let mut phase: u32 = 0;
    let mut phase_inc: u32 = 0; // fixed-point units added per tick (speed)
    let mut on = false;
    // Coil-B (vb) polarity. If the B coil is wired/driven inverted relative to A,
    // a "rotating" field only ROCKS the rotor (libration) instead of walking it —
    // exactly the ~5° oscillation we see. Op 0x03 [inv] toggles this to test.
    let mut invert_b = false;

    let mut tick: u32 = 0;
    loop {
        // ---- service CAN only occasionally so it never stalls the field ----
        if tick % 64 == 0 {
            while let Some((id, d, len)) = board.can_recv() {
                if id == 0x601 && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                    board.set_output_enable(false);
                    board.reboot_to_bootloader();
                } else if id == CMD_ID && len >= 1 {
                    match d[0] {
                        0x00 => {
                            on = false;
                            phase_inc = 0;
                        }
                        0x01 if len >= 5 => {
                            amp = i16le(d[1], d[2]);
                            phase = (i16le(d[3], d[4]) as u16 as u32) << 16;
                            phase_inc = 0;
                            on = true;
                        }
                        // 0x02 [amp:i16, rate:i16] — CONTINUOUS smooth spin.
                        // `rate` = whole electrical-angle units advanced per
                        // ~20kHz tick (fixed-point high word). One MECHANICAL rev
                        // = 50 elec revs = 50·65536 elec-units; at 20kHz,
                        // rate=55 ≈ one mech rev / 3s. Small rate → slow smooth.
                        0x02 if len >= 5 => {
                            amp = i16le(d[1], d[2]);
                            phase_inc = (i16le(d[3], d[4]) as i32 as u32).wrapping_shl(16);
                            on = true;
                        }
                        // 0x03 [inv] — set coil-B polarity (0/1). Test whether a
                        // reverse-wired B coil is why a rotating field only rocks
                        // the rotor instead of walking it.
                        0x03 if len >= 2 => invert_b = d[1] != 0,
                        _ => {}
                    }
                }
            }
        }

        board.set_output_enable(on);
        if on {
            phase = phase.wrapping_add(phase_inc);
            let ang = (phase >> 16) as u16;
            let (va, mut vb) = vector(ang, amp);
            if invert_b {
                vb = -vb;
            }
            board.apply_coil_voltages(va, vb);
        } else {
            board.apply_coil_voltages(0, 0);
        }

        // telemetry infrequent + non-stalling
        if tick % 2048 == 0 {
            let e = board.rotor_angle().to_le_bytes();
            let a = (amp as u16).to_le_bytes();
            let g = ((phase >> 16) as u16).to_le_bytes();
            board.telemetry(TELEM_ID, &[e[0], e[1], a[0], a[1], g[0], g[1], 0, 0]);
        }
        tick = tick.wrapping_add(1);
        // tight ~20 kHz field update for smooth rotation (no delay_ms coarseness)
        cortex_m::asm::delay(3200);
    }
}
