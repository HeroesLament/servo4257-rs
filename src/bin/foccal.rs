#![no_std]
#![no_main]
//! foccal — faithful port of nano_stepper's StepperCtrl::calibrateEncoder.
//!
//! Builds the 200-point encoder linearization table that lets the field
//! commutate 90° ahead of the rotor EVERYWHERE (not just over the good arc —
//! the dead zones are why the motor spins when hand-helped then stalls/dithers).
//!
//! Capture (per nano_stepper, details preserved):
//!   for j in 0..200 full steps:
//!     settle 200ms, table[j] = mean of 200 encoder reads (encoder is noisy),
//!     advance ONE full step (=90° elec = 16384) as TWO half-steps (8192 each)
//!     to avoid the rotor jumping backward as current ramps between steps.
//!
//! One full step = 90° electrical because 200 full steps/rev ÷ 50 pole pairs =
//! 4 full steps per electrical rev. So the whole 200-step sweep = one MECHANICAL
//! revolution = 50 electrical revs = table maps enc→true angle over the circle.
//!
//! Streams each captured point over CAN (0x185) so the table is WATCHED as it
//! builds; when done, streams the full table on request. Answers 0x1F51.
//!
//! Commands (0x612, byte0): 0x01 START calibration; 0x02 DUMP table[idx] (idx in
//! byte1). Telemetry 0x185.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::encoder::POLE_PAIRS;
use servo4257_rs::motion::trig::sin_cos;

const CMD_ID: u16 = 0x612;
const TELEM_ID: u16 = 0x185;
const SYSCLK_HZ: u32 = 64_000_000;

const TABLE_SIZE: usize = 200;
const FULL_STEP_ELEC: u16 = 16384; // 90° electrical = one full mechanical step
const HALF_STEP_ELEC: u16 = 8192;

// Calibration drive amplitude — needs enough torque to firmly seat the rotor at
// each detent. Conservative; watch thermals over a full 200-step run (~50s).
const CAL_AMP: i16 = 15000;

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

/// Mean of `n` encoder reads — magnetic encoders are noisy, a single read
/// jitters (this was our error all session). Handles the u16 wrap by unwrapping
/// relative to the first sample so the average is correct across 0/65535.
fn sample_mean_encoder(board: &mut ActiveBoard, n: u32) -> u16 {
    let first = board.rotor_angle();
    let mut acc: i64 = 0;
    for _ in 0..n {
        let e = board.rotor_angle();
        // unwrap relative to first: shortest signed distance
        let d = (e.wrapping_sub(first)) as i16 as i64;
        acc += d;
        delay_ms(1);
    }
    let mean = (first as i64 + acc / n as i64) & 0xFFFF;
    mean as u16
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(false);

    let mut table: [u16; TABLE_SIZE] = [0; TABLE_SIZE];
    let mut have_table = false;

    loop {
        // ---- service CAN (drain all pending) ----
        let mut start_cal = false;
        let mut dump_idx: Option<usize> = None;
        while let Some((id, d, len)) = board.can_recv() {
            if id == 0x601 && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                board.set_output_enable(false);
                board.reboot_to_bootloader();
            } else if id == CMD_ID && len >= 1 {
                match d[0] {
                    0x01 => start_cal = true,
                    0x02 if len >= 2 => dump_idx = Some(d[1] as usize),
                    _ => {}
                }
            }
        }

        if let Some(idx) = dump_idx {
            if have_table && idx < TABLE_SIZE {
                let v = table[idx].to_le_bytes();
                let i = (idx as u16).to_le_bytes();
                board.telemetry(TELEM_ID, &[0xC2, i[0], i[1], v[0], v[1], 0, 0, 0]);
            }
        }

        if start_cal {
            // ---- SMOOTH open-loop calibration sweep (nano_stepper recipe) ----
            // Faithful to StepperCtrl::calibrateEncoder, with the fix that made
            // holdtest actually move the rotor: advance the field SMOOTHLY (many
            // tiny sub-degree increments), never in a jump. A jump lets the
            // strongly-detented rotor stay put (the 4-value dither); a smooth
            // ramp drags it along. Between steps, hold and let libration damp out
            // before sampling. Shaft must be FREE (it is).
            board.set_output_enable(true);

            // seat hard at electrical 0 and let any libration die
            let mut field: u32 = 0; // fixed-point: high 16 = elec angle, low 16 = frac
            let (va, vb) = vector(0, CAL_AMP);
            board.apply_coil_voltages(va, vb);
            delay_ms(600);

            // One full mechanical step = one full step = 16384 elec (90°). Advance
            // it smoothly over MICRO sub-steps.
            const STEP_ELEC: u32 = 16384;
            const MICRO: u32 = 256; // sub-steps per full step → 64 elec-units each
            let inc: u32 = (STEP_ELEC << 16) / MICRO;

            for j in 0..TABLE_SIZE {
                // smoothly ramp the field forward one full step
                for _ in 0..MICRO {
                    field = field.wrapping_add(inc);
                    let (va, vb) = vector((field >> 16) as u16, CAL_AMP);
                    board.apply_coil_voltages(va, vb);
                    delay_ms(1); // ~256ms per full step ramp → gentle, rotor tracks
                }
                // hold at the new detent, let libration damp, then sample
                delay_ms(150);
                let mean = sample_mean_encoder(&mut board, 100);
                table[j] = mean;

                let jj = (j as u16).to_le_bytes();
                let m = mean.to_le_bytes();
                let fa = ((field >> 16) as u16).to_le_bytes();
                board.telemetry(TELEM_ID, &[0xC1, jj[0], jj[1], m[0], m[1], fa[0], fa[1], 0]);
            }

            board.set_output_enable(false);
            board.apply_coil_voltages(0, 0);
            have_table = true;
            board.telemetry(TELEM_ID, &[0xCF, 0, 0, 0, 0, 0, 0, 0]);
            let _ = (FULL_STEP_ELEC, HALF_STEP_ELEC); // (open-loop consts retained)
        }

        delay_ms(2);
    }
}
