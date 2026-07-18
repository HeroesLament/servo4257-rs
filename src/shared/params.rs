//! shared/params.rs — Commutation Lab tuning parameters (async -> ISR).
//!
//! Sibling to `setpoints.rs`: the async/CAN tier writes these live via the
//! manufacturer object dictionary (`0x2000:xx`), the ~20 kHz commutation ISR
//! reads them each tick. Per-field atomics — a torn read across two unrelated
//! knobs (e.g. lead vs amplitude) is harmless during interactive tuning, so no
//! seqlock is needed here. When the ISR needs a *coherent* view of all knobs at
//! once it calls `snapshot()`, which reads each field a single time into a plain
//! `Copy` struct; that's coherent enough for tuning (no field is written
//! mid-tick in a way that must be atomic with another).
//!
//! This backs the `commlab` bring-up binary, NOT the production servo firmware —
//! the winning constants get lifted into the real app once found.

use core::sync::atomic::{AtomicBool, AtomicI16, AtomicU16, AtomicU8, Ordering};

const RLX: Ordering = Ordering::Relaxed;

/// Commutation loop mode, selected via `0x2000:02`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum CommMode {
    /// Output off; coils de-energized regardless of `enable`.
    Idle = 0,
    /// Sensored: field angle = encoder→electrical map + `lead_angle`.
    Sensored = 1,
    /// Open loop: field angle = `ol_angle`, auto-advanced by `ol_rate`.
    OpenLoop = 2,
    /// Align: hold a fixed electrical angle (`ol_angle`) to seat the rotor.
    Align = 3,
}

impl CommMode {
    #[inline]
    pub fn from_u8(v: u8) -> CommMode {
        match v {
            1 => CommMode::Sensored,
            2 => CommMode::OpenLoop,
            3 => CommMode::Align,
            _ => CommMode::Idle,
        }
    }
}

/// Live commutation knobs. Each field is an independent atomic (mapped 1:1 to a
/// manufacturer OD sub-index). Writers: the SDO server. Reader: the comm ISR.
pub struct CommParams {
    /// `0x2000:01` — output stage enable (0/1).
    pub enable: AtomicBool,
    /// `0x2000:02` — `CommMode`.
    pub mode: AtomicU8,
    /// `0x2000:03` — vector amplitude (signed; sign flips drive direction too).
    pub amplitude: AtomicI16,
    /// `0x2000:04` — electrical lead angle (u16 wraps the electrical circle).
    pub lead_angle: AtomicU16,
    /// `0x2000:05` — encoder→electrical direction sign (0/1).
    pub direction: AtomicU8,
    /// `0x2000:06` — electrical alignment offset.
    pub offset: AtomicU16,
    /// `0x2000:07` — pole pairs (live-tunable; nominal 50 for this motor).
    pub pole_pairs: AtomicU16,
    /// `0x2000:08` — commanded open-loop field angle (mode OpenLoop/Align).
    pub ol_angle: AtomicU16,
    /// `0x2000:09` — open-loop auto-advance per tick (signed; 0 = hold).
    pub ol_rate: AtomicI16,
}

/// A coherent-enough single-read snapshot for one ISR tick.
#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub enable: bool,
    pub mode: CommMode,
    pub amplitude: i16,
    pub lead_angle: u16,
    pub direction: u8,
    pub offset: u16,
    pub pole_pairs: u16,
    pub ol_angle: u16,
    pub ol_rate: i16,
}

impl CommParams {
    pub const fn new() -> Self {
        Self {
            enable: AtomicBool::new(false),
            mode: AtomicU8::new(CommMode::Idle as u8),
            amplitude: AtomicI16::new(0),
            lead_angle: AtomicU16::new(0),
            direction: AtomicU8::new(1),
            offset: AtomicU16::new(0),
            // Sensible defaults for this motor; overridable live.
            pole_pairs: AtomicU16::new(50),
            ol_angle: AtomicU16::new(0),
            ol_rate: AtomicI16::new(0),
        }
    }

    /// Read every knob once into a plain struct for use across a single tick.
    #[inline]
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            enable: self.enable.load(RLX),
            mode: CommMode::from_u8(self.mode.load(RLX)),
            amplitude: self.amplitude.load(RLX),
            lead_angle: self.lead_angle.load(RLX),
            direction: self.direction.load(RLX),
            offset: self.offset.load(RLX),
            pole_pairs: self.pole_pairs.load(RLX),
            ol_angle: self.ol_angle.load(RLX),
            ol_rate: self.ol_rate.load(RLX),
        }
    }

    /// Advance the open-loop angle by `ol_rate` (called by the ISR each tick in
    /// OpenLoop mode). Kept here so the wrap is defined in one place.
    #[inline]
    pub fn advance_ol_angle(&self) {
        let rate = self.ol_rate.load(RLX);
        if rate != 0 {
            let a = self.ol_angle.load(RLX);
            self.ol_angle
                .store(a.wrapping_add(rate as u16), RLX);
        }
    }
}

/// The global commutation parameter block.
pub static PARAMS: CommParams = CommParams::new();
