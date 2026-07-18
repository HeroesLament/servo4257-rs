#![no_std]
#![no_main]
//! commlab — the Commutation Lab: a dedicated bring-up binary that exposes every
//! commutation knob and telemetry value over a nonstandard manufacturer CANopen
//! profile, so the motor can be tuned interactively from IEx with no reflash.
//!
//! Structure:
//!   * a tight loop (~tens of kHz) that reads `shared::PARAMS` each pass, drives
//!     the coils per the selected mode, and publishes a coherent telemetry
//!     snapshot to `shared::TELEM`;
//!   * a minimal expedited-SDO server that routes reads/writes of `0x2000` /
//!     `0x2001` through `canopen::od`, replying on `0x581` (node 1).
//!
//! Manufacturer objects (see canopen/od.rs):
//!   0x2000:01..09  params (enable/mode/amp/lead/dir/offset/pole_pairs/ol_angle/ol_rate)
//!   0x2001:01..06  telemetry (enc/theta/vel/iA/iB/liveness)
//!
//! Standard SDO framing (node 1): master → 0x601, node → 0x581. Also answers the
//! 0x1F51 enter-update poke for over-CAN reflash back into the bootloader.
//!
//! `layout-app` + `hw-can`: flashes over CAN and CRC-gates exactly like `instr`.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::canopen::mgmt::{self, Iwdg};
use servo4257_rs::canopen::od::{self, MgmtAction};
use servo4257_rs::display::{self, FrameBuf, Ssd1306};
use servo4257_rs::shared::{CommMode, Shape, Snapshot, PARAMS, TELEM};
use servo4257_rs::motion::trig::{sin_cos, sin_u16};

const NODE_ID: u16 = 1;
const RX_COBID: u16 = 0x600 + NODE_ID; // master → node
const TX_COBID: u16 = 0x580 + NODE_ID; // node → master

// ---- SDO command specifiers (CiA-301 expedited subset) ----
const CCS_DOWNLOAD: u8 = 0x20; // master writes (initiate download)
const CCS_UPLOAD: u8 = 0x40; // master reads (initiate upload)
const SCS_UPLOAD_OK: u8 = 0x40; // node → master, initiate-upload response base
const SCS_DOWNLOAD_OK: u8 = 0x60; // node → master, download acknowledged
const SCS_ABORT: u8 = 0x80;

/// Map encoder → electrical field angle using the live params.
#[inline]
fn theta_e(enc: u16, p: &Snapshot) -> u16 {
    let e = enc.wrapping_mul(p.pole_pairs);
    if p.direction == 0 {
        e.wrapping_sub(p.offset)
    } else {
        p.offset.wrapping_sub(e)
    }
}

/// One phase's normalized waveform value in [-1, 1] at electrical `angle`, per
/// the selected `Shape`. Sine is the baseline; the others trade smoothness for
/// torque/headroom (see `Shape`). All are peak-normalized to ±1 so `amplitude`
/// remains the true peak regardless of shape.
#[inline]
fn wave(angle: u16, shape: Shape) -> f32 {
    let s = sin_u16(angle);
    match shape {
        Shape::Sine => s,
        // Clip at ±sin(60°) then rescale so the clipped level maps back to ±1.
        // Flat-topped ("trapezoidal") drive: more fundamental per peak volt.
        Shape::Trapezoid => {
            const CLIP: f32 = 0.866_025_4; // sin(60°)
            let c = if s > CLIP { CLIP } else if s < -CLIP { -CLIP } else { s };
            c * (1.0 / CLIP)
        }
        // Third-harmonic injection: sin(θ) + (1/6)sin(3θ), then renormalize by
        // the resulting peak (~1.1547) so it still peaks at ±1. Flattens the
        // phase tops → ~15% more fundamental before the rail, without clipping.
        Shape::ThirdHarmonic => {
            let s3 = sin_u16(angle.wrapping_mul(3));
            (s + s3 * (1.0 / 6.0)) * 0.866_025_4
        }
    }
}

/// (v_a, v_b) for a vector at electrical `angle`, magnitude `amp`, waveform
/// `shape`. For `Shape::Sine` this is byte-for-byte the original proven drive:
/// `(amp*cos, amp*sin)` straight from the sin/cos LUT. Non-sine shapes reshape
/// each phase via `wave`, using the same cos=sin(θ+90°) quadrature.
#[inline]
fn vector(angle: u16, amp: i16, shape: Shape) -> (i16, i16) {
    let amp = amp as f32;
    match shape {
        Shape::Sine => {
            let (s, c) = sin_cos(angle);
            ((amp * c) as i16, (amp * s) as i16)
        }
        _ => {
            // Phase A carries cos(θ) = sin(θ+90°); phase B carries sin(θ).
            let a = wave(angle.wrapping_add(16384), shape);
            let b = wave(angle, shape);
            ((amp * a) as i16, (amp * b) as i16)
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(false);

    // Independent watchdog: if the main loop ever stalls (a blocking I2C write
    // to a non-ACKing OLED was exactly how commlab bricked itself once), the
    // MCU auto-resets within ~1 s → bootloader listen window catches it. The CAN
    // service + this feed run every pass, so the board is never unrecoverable.
    let mut dog = Iwdg::start_1s();

    // OLED framebuffer + ball. The display is brought up LAZILY inside the loop
    // (see below) so a missing/hung panel can never block startup before we've
    // serviced CAN and fed the dog. `oled` stays None until init succeeds.
    let mut oled: Option<Ssd1306<servo4257_rs::boards::servo57d::DisplayI2c>> = None;
    let mut display_i2c = board.take_display_i2c();
    let mut oled_tries: u8 = 0;
    let mut fb = FrameBuf::new();
    let mut ball = Ball::new();

    let mut tick: u32 = 0;
    let mut last_enc: u16 = 0;

    // ---- sensored startup state machine ----
    // A synchronous motor can't reliably start from a fixed quadrature field:
    // whether it accelerates or snaps to a stable lock depends on the rotor's
    // exact position at enable. Real drives ALIGN first (DC-inject a known angle
    // so the rotor seats deterministically), THEN commutate from that known
    // position so the +90° torque angle always lands on the driving side. This
    // handoff MUST be atomic in the control loop — doing it over CAN with sleeps
    // races the rotor and stays a coin-flip (confirmed on hardware). Here it is
    // in-loop: on the enable edge in Sensored mode we hold Align for
    // `ALIGN_TICKS`, then switch to running quadrature commutation.
    #[derive(Clone, Copy, PartialEq)]
    enum Startup { Idle, Aligning(u32), Running }
    let mut startup = Startup::Idle;
    let mut prev_enable = false;
    // Loop runs in the tens of kHz; ~40k ticks ≈ a solid fraction of a second of
    // align dwell, plenty for the rotor to seat and stop ringing.
    const ALIGN_TICKS: u32 = 40_000;
    // Electrical angle to DC-inject during align (rotor seats here). The RUN
    // phase then commutes from the live encoder, so this only sets the start
    // detent — 0 is fine.
    const ALIGN_ANGLE: u16 = 0;

    loop {
        // Always kick the watchdog first — a hang anywhere below self-heals.
        dog.feed();

        // ---- 1. service one CAN frame (SDO + reflash poke) ----
        if let Some((id, d, len)) = board.can_recv() {
            // 0x1F51 enter-update → drop back to the bootloader for reflash.
            if id == RX_COBID && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                board.set_output_enable(false);
                board.reboot_to_bootloader();
            } else if id == RX_COBID && len >= 8 {
                match handle_sdo(&d) {
                    SdoOutcome::Reply(resp) => board.telemetry(TX_COBID, &resp),
                    SdoOutcome::Mgmt(action, resp) => {
                        board.telemetry(TX_COBID, &resp);
                        exec_mgmt(&mut board, action);
                    }
                    SdoOutcome::Ignore => {}
                }
            }
        }

        // ---- 2. drive the coils per the live params ----
        let p = PARAMS.snapshot();
        board.set_output_enable(p.enable);

        let enc = board.rotor_angle();
        let th = theta_e(enc, &p);

        // Sensored startup edge: on the rising edge of `enable` (in Sensored
        // mode) begin the align dwell; on disable, reset so the next enable
        // re-aligns. Other modes don't use `startup`.
        if p.enable && !prev_enable && p.mode == CommMode::Sensored {
            startup = Startup::Aligning(ALIGN_TICKS);
        }
        if !p.enable {
            startup = Startup::Idle;
        }
        prev_enable = p.enable;

        if p.enable {
            match p.mode {
                CommMode::Sensored => {
                    const QUARTER: u16 = 16384; // 90° electrical
                    let torque = p.amplitude.unsigned_abs() as i16;

                    match startup {
                        // ALIGN: DC-inject a fixed electrical angle so the rotor
                        // seats to a known detent before we commutate. Field is a
                        // fixed vector (not encoder-derived) — this is what makes
                        // the subsequent quadrature start deterministic.
                        Startup::Aligning(0) => startup = Startup::Running,
                        Startup::Aligning(n) => {
                            let (va, vb) = vector(ALIGN_ANGLE, torque, p.shape);
                            board.apply_coil_voltages(va, vb);
                            startup = Startup::Aligning(n - 1);
                        }
                        // RUN: quadrature torque drive ("what FANUC does") — field
                        // held a fixed 90° AHEAD of the measured rotor angle, in
                        // the torque direction. Self-commutating, spins
                        // continuously; no stable field position to lock into.
                        // `amplitude` sign = direction, magnitude = torque;
                        // `lead_angle` is a fine trim around quadrature.
                        Startup::Running => {
                            let quad = if p.amplitude >= 0 {
                                QUARTER.wrapping_add(p.lead_angle)
                            } else {
                                QUARTER.wrapping_neg().wrapping_add(p.lead_angle)
                            };
                            let (va, vb) = vector(th.wrapping_add(quad), torque, p.shape);
                            board.apply_coil_voltages(va, vb);
                        }
                        // Idle shouldn't occur while enabled (edge sets Aligning),
                        // but hold safe if it does.
                        Startup::Idle => board.apply_coil_voltages(0, 0),
                    }
                }
                CommMode::OpenLoop => {
                    let (va, vb) = vector(p.ol_angle, p.amplitude, p.shape);
                    board.apply_coil_voltages(va, vb);
                    PARAMS.advance_ol_angle();
                }
                CommMode::Align => {
                    let (va, vb) = vector(p.ol_angle, p.amplitude, p.shape);
                    board.apply_coil_voltages(va, vb);
                }
                CommMode::Idle => board.apply_coil_voltages(0, 0),
            }
        } else {
            board.apply_coil_voltages(0, 0);
        }

        // ---- 3. publish telemetry (coherent seqlock snapshot) ----
        if tick % 32 == 0 {
            let vel = enc.wrapping_sub(last_enc) as i16;
            last_enc = enc;
            let (ia, ib) = board.read_coil_currents();
            TELEM.write(|t| {
                t.pos = enc as i32;
                t.theta_e = th;
                t.vel = vel as i32;
                t.iq = ia;
                t.id = ib;
                t.faults = (tick & 0xFFFF) as u16; // liveness
            });
        }

        // ---- 4. OLED: lazy init + animate (throttled; I2C flush is slow-ish) ----
        // The init is attempted a few times inside the loop rather than at
        // startup. If the panel never ACKs, the blocking HAL write would stall —
        // but the watchdog above turns that into a clean auto-reset instead of a
        // brick, and CAN was already serviced this pass. After a few failed
        // tries we give up and run headless (commutation + CAN unaffected).
        if tick % 4096 == 0 {
            if oled.is_none() && oled_tries < 3 {
                oled_tries += 1;
                if let Some(i2c) = display_i2c.take() {
                    match Ssd1306::new(i2c) {
                        Ok(d) => oled = Some(d),
                        // init failed but returned — bus is alive, retry later.
                        Err(_) => {}
                    }
                }
            }
            if let Some(d) = oled.as_mut() {
                ball.step();
                fb.clear();
                fb.rect(0, 0, (display::W - 1) as i32, (display::H - 1) as i32, true);
                fb.fill_circle(ball.x >> 4, ball.y >> 4, Ball::R, true);
                let _ = d.flush(&fb);
            }
        }

        tick = tick.wrapping_add(1);
        // No delay: run the loop as fast as it goes. At 64 MHz with a handful of
        // float ops + one SPI encoder read, this lands in the tens-of-kHz range,
        // which is what commutation wants.
    }
}

/// A bouncing ball in fixed-point (1/16 px) so motion is smooth per frame.
struct Ball {
    x: i32,
    y: i32,
    vx: i32,
    vy: i32,
}

impl Ball {
    const R: i32 = 5;
    // Fixed-point bounds (px << 4), inset by radius + the 1px border.
    const XMIN: i32 = (Self::R + 1) << 4;
    const XMAX: i32 = ((display::W as i32) - 1 - Self::R) << 4;
    const YMIN: i32 = (Self::R + 1) << 4;
    const YMAX: i32 = ((display::H as i32) - 1 - Self::R) << 4;

    const fn new() -> Self {
        Self {
            x: 40 << 4,
            y: 24 << 4,
            vx: 37, // coprime-ish speeds → long non-repeating path
            vy: 23,
        }
    }

    fn step(&mut self) {
        self.x += self.vx;
        self.y += self.vy;
        if self.x <= Self::XMIN {
            self.x = Self::XMIN;
            self.vx = self.vx.abs();
        } else if self.x >= Self::XMAX {
            self.x = Self::XMAX;
            self.vx = -self.vx.abs();
        }
        if self.y <= Self::YMIN {
            self.y = Self::YMIN;
            self.vy = self.vy.abs();
        } else if self.y >= Self::YMAX {
            self.y = Self::YMAX;
            self.vy = -self.vy.abs();
        }
    }
}

/// The result of handling one SDO frame.
enum SdoOutcome {
    /// Send this response frame.
    Reply([u8; 8]),
    /// Send the response, then execute the management action (reboot etc.).
    Mgmt(MgmtAction, [u8; 8]),
    /// Not an SDO we serve — drop.
    Ignore,
}

/// Handle one expedited SDO request.
fn handle_sdo(d: &[u8; 8]) -> SdoOutcome {
    let ccs = d[0] & 0xE0;
    let index = u16::from_le_bytes([d[1], d[2]]);
    let sub = d[3];

    match ccs {
        CCS_UPLOAD => SdoOutcome::Reply(match od::read(index, sub) {
            Ok((val, size)) => {
                // Expedited upload response: scs=2 (0x40), e=1, s=1, n=(4-size).
                let n = 4 - size;
                let cmd = SCS_UPLOAD_OK | ((n & 0x3) << 2) | 0b11;
                let v = val.to_le_bytes();
                [cmd, d[1], d[2], sub, v[0], v[1], v[2], v[3]]
            }
            Err(e) => abort(index, sub, e.abort_code()),
        }),

        CCS_DOWNLOAD => {
            // Expedited download: value is in bytes 4..8, low `4-n` significant.
            let val = u32::from_le_bytes([d[4], d[5], d[6], d[7]]);
            match od::write(index, sub, val) {
                Ok(None) => SdoOutcome::Reply([SCS_DOWNLOAD_OK, d[1], d[2], sub, 0, 0, 0, 0]),
                Ok(Some(action)) => {
                    // Ack the write first so the host sees success, THEN act.
                    SdoOutcome::Mgmt(action, [SCS_DOWNLOAD_OK, d[1], d[2], sub, 0, 0, 0, 0])
                }
                Err(e) => SdoOutcome::Reply(abort(index, sub, e.abort_code())),
            }
        }

        _ => SdoOutcome::Ignore,
    }
}

/// Execute a board-management action. Reboot/stay/boot never return; the app
/// can't invalidate its own meta (no flash driver here), so InvalidateApp falls
/// back to a stay-in-boot reset — the bootloader then owns flash for reflash.
fn exec_mgmt(board: &mut ActiveBoard, action: MgmtAction) {
    // Safe the output stage before any reset.
    board.set_output_enable(false);
    board.apply_coil_voltages(0, 0);
    match action {
        MgmtAction::Reboot => mgmt::reset(),
        MgmtAction::StayInBoot | MgmtAction::InvalidateApp => mgmt::reset_into_bootloader(),
        MgmtAction::BootApp => mgmt::reset_into_app(),
    }
}

/// Build an SDO abort response frame.
fn abort(index: u16, sub: u8, code: u32) -> [u8; 8] {
    let i = index.to_le_bytes();
    let c = code.to_le_bytes();
    [SCS_ABORT, i[0], i[1], sub, c[0], c[1], c[2], c[3]]
}
