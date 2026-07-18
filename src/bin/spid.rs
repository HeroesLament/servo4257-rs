#![no_std]
#![no_main]
//! spid — nano_stepper's "Simple PID" servo, ported to SERVO57D (voltage bridge).
//!
//! The mode that actually turns this class of stepper+encoder into a servo, and
//! the one we should have built first. Encoder-only, NO current sensing.
//!
//! Per StepperCtrl::simpleFeedback (Misfittech/nano_stepper, Mechaduino lineage):
//!   y     = encoder mechanical angle (u16, 65536/rev)   [cal table: TODO]
//!   error = desired - y                                 (wrapping shortest path)
//!   u     = Kp·error + Ki·∫error + Kd·Δerror            (clamped ±fullStep)
//!   ma    = |u|/fullStep·(maxAmp - holdAmp) + holdAmp   (amplitude from error)
//!   apply field at electrical angle of (y + u), amplitude ma
//!
//! The KEY difference from every fixed-lead attempt: the field lead `u` is the
//! PID output, clamped to ±one full step (±90° elec), always in the direction
//! that reduces error. So the field self-commutates and CANNOT lock — at error 0
//! it holds on the rotor at holdAmp; under error it leads up to 90° at up to
//! maxAmp. Both lead and current scale with error.
//!
//! `desired` ramps at DESIRED_RATE so the motor spins continuously (a moving
//! setpoint the servo chases) — set rate 0 for a stationary position hold.
//!
//! Telemetry 0x184 (~cadence): [enc:u16, desired_lo:u16, u:i16, ma:u16] LE.
//! Answers 0x1F51 enter-update.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::encoder::POLE_PAIRS;
use servo4257_rs::motion::trig::sin_cos;

const TELEM_ID: u16 = 0x184;
const CMD_ID: u16 = 0x611;
const SYSCLK_HZ: u32 = 64_000_000;

// One full step in mechanical u16 units: 65536 / (200 full steps/rev) = 327.
const FULL_STEP: i32 = (65536 / 200) as i32;

/// Live control knobs (defaults; all settable over CAN on CMD_ID, and the exact
/// set the matrix sweep drives). Starts DISABLED so the board doesn't energize —
/// and heat — the instant it boots.
struct Knobs {
    enabled: bool,
    kp: f32,
    ki: f32,
    kd: f32,
    hold_amp: i32, // current floor at zero error (keep low — flows continuously)
    max_amp: i32,  // current ceiling under full-step error
    rate: i32,     // setpoint ramp: mech u16 counts/tick; 0 = position hold
}

impl Knobs {
    const fn default() -> Self {
        Self {
            enabled: false,
            kp: 4.0,
            ki: 0.02,
            kd: 40.0,
            hold_amp: 1200, // lowered from 3000 — this flows 100% of the time at rest
            max_amp: 14000,
            rate: 0,
        }
    }
}

// CAN command ops (byte0 on CMD_ID). Little-endian payloads.
const OP_ENABLE: u8 = 0x01; // [en]
const OP_RATE: u8 = 0x02; // [rate:i16]  spin speed (mech counts/tick), signed
const OP_GAINS: u8 = 0x03; // [kp:i16, ki_milli:i16, kd:i16]  (ki sent ×1000)
const OP_CURRENT: u8 = 0x04; // [hold:i16, max:i16]
const OP_STOP: u8 = 0x00; // disable + de-energize

#[inline]
fn vector(theta_e: u16, amp: i32) -> (i16, i16) {
    let (s, c) = sin_cos(theta_e);
    let amp = amp as f32;
    ((amp * c) as i16, (amp * s) as i16)
}

/// Wrapping signed difference on the u16 circle → shortest-path error in [-32768, 32767].
#[inline]
fn circ_err(desired: u16, y: u16) -> i32 {
    (desired.wrapping_sub(y) as i16) as i32
}

#[inline]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay((SYSCLK_HZ / 1000) * ms);
}

/// Signed i16 from two LE bytes.
#[inline]
fn i16le(a: u8, b: u8) -> i16 {
    i16::from_le_bytes([a, b])
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    // Start SAFE: output stage off, disabled. No coil current until commanded —
    // so booting spid never heats the motor.
    board.set_output_enable(false);

    let mut k = Knobs::default();
    delay_ms(50);
    let mut desired: u16 = board.rotor_angle(); // seat setpoint = current pos (no lurch)

    let mut i_term: f32 = 0.0;
    let mut last_err: i32 = 0;
    let mut tick: u32 = 0;

    loop {
        // ---- CAN: drain ALL pending frames this tick (single-frame-per-loop
        // starved commands under telemetry load — process the whole queue) ----
        while let Some((id, d, len)) = board.can_recv() {
            if id == 0x601 && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                board.set_output_enable(false);
                board.reboot_to_bootloader();
            } else if id == CMD_ID && len >= 1 {
                match d[0] {
                    OP_STOP => {
                        k.enabled = false;
                    }
                    OP_ENABLE if len >= 2 => {
                        k.enabled = d[1] != 0;
                        if k.enabled {
                            // reseat setpoint & clear integrator on (re)enable
                            desired = board.rotor_angle();
                            i_term = 0.0;
                            last_err = 0;
                        }
                    }
                    OP_RATE if len >= 3 => k.rate = i16le(d[1], d[2]) as i32,
                    OP_GAINS if len >= 7 => {
                        k.kp = i16le(d[1], d[2]) as f32;
                        k.ki = i16le(d[3], d[4]) as f32 / 1000.0; // ki sent ×1000
                        k.kd = i16le(d[5], d[6]) as f32;
                    }
                    OP_CURRENT if len >= 5 => {
                        k.hold_amp = i16le(d[1], d[2]) as i32;
                        k.max_amp = i16le(d[3], d[4]) as i32;
                    }
                    _ => {}
                }
            }
        }

        board.set_output_enable(k.enabled);
        if !k.enabled {
            board.apply_coil_voltages(0, 0);
            cortex_m::asm::delay(3200);
            tick = tick.wrapping_add(1);
            continue;
        }

        // ---- read position (raw encoder; cal-table linearization TODO) ----
        let y = board.rotor_angle();

        // ---- PID on the wrapping position error ----
        let error = circ_err(desired, y);
        i_term += k.ki * error as f32;
        let fs = FULL_STEP as f32;
        if i_term > fs {
            i_term = fs;
        } else if i_term < -fs {
            i_term = -fs;
        }
        let mut u = (k.kp * error as f32 + i_term + k.kd * (error - last_err) as f32) as i32;
        if u > FULL_STEP {
            u = FULL_STEP;
        } else if u < -FULL_STEP {
            u = -FULL_STEP;
        }

        // ---- amplitude scales with |error|: hold floor → max ceiling ----
        let mut ma = (u.abs() * (k.max_amp - k.hold_amp)) / FULL_STEP + k.hold_amp;
        if ma > k.max_amp {
            ma = k.max_amp;
        }

        // ---- apply the field at electrical angle of (y + u), amplitude ma ----
        let field_mech = (y as i32 + u) as u16;
        let theta_e = field_mech.wrapping_mul(POLE_PAIRS);
        let (va, vb) = vector(theta_e, ma);
        board.apply_coil_voltages(va, vb);

        last_err = error;

        // ---- advance the setpoint (moving target → continuous rotation) ----
        desired = ((desired as i32 + k.rate) as u16) & 0xFFFF;

        if tick % 200 == 0 {
            let e = y.to_le_bytes();
            let dd = desired.to_le_bytes();
            let uu = (u as i16).to_le_bytes();
            let m = (ma as u16).to_le_bytes();
            board.telemetry(TELEM_ID, &[e[0], e[1], dd[0], dd[1], uu[0], uu[1], m[0], m[1]]);
        }
        tick = tick.wrapping_add(1);

        cortex_m::asm::delay(3200); // ~50 µs → ~20 kHz (well above nano's 6 kHz)
    }
}
