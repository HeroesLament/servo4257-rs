//! CANopen object dictionary — Commutation Lab manufacturer profile.
//!
//! A deliberately nonstandard, manufacturer-specific object area (CiA reserves
//! `0x2000–0x5FFF` for exactly this) that exposes the commutation loop's knobs
//! and telemetry as SDO-addressable scalars, so the whole motor can be tuned
//! interactively from a CANopen master (IEx) with no reflash.
//!
//! Layout:
//!   * `0x2000:xx` — commutation parameters (RW) → `shared::PARAMS` atomics
//!   * `0x2001:xx` — telemetry (RO)             → `shared::TELEM` seqlock snapshot
//!   * `0x2002:xx` — actions (RW, write-triggers-behavior) — M2, stubbed here
//!
//! This dispatch is a **pure, non-blocking function** over shared atomics and
//! one seqlock read. It never allocates, never blocks, and is safe to call from
//! the SDO server running in the `commlab` main loop.
//!
//! All objects are ≤4-byte scalars, so every access is an *expedited* SDO
//! transfer. Values cross the wire zero-extended into a `u32`; `size` says how
//! many bytes are significant (1/2/4).

use crate::canopen::mgmt::{self, sub as msub};
use crate::shared::{PARAMS, TELEM};
use core::sync::atomic::Ordering;

const RLX: Ordering = Ordering::Relaxed;

/// Manufacturer object indices.
pub mod idx {
    pub const PARAMS: u16 = 0x2000;
    pub const TELEMETRY: u16 = 0x2001;
    pub const ACTIONS: u16 = 0x2002;
    pub const MGMT: u16 = 0x2F00; // board management (reboot/stay/boot/invalidate)
}

/// A board-management action requested via `0x2F00`. Returned by `write` for the
/// caller (bootloader or app) to execute — the actions need reset/flash context
/// each image supplies differently, so the OD only *classifies* the request.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MgmtAction {
    Reboot,
    StayInBoot,
    BootApp,
    InvalidateApp,
}

/// Why an OD access failed. Maps to CiA-301 SDO abort codes at the SDO layer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OdErr {
    /// No object at this index/sub — abort `0x0602_0000`.
    NoSuchObject,
    /// Object exists but is read-only — abort `0x0601_0002`.
    ReadOnly,
}

impl OdErr {
    /// CiA-301 SDO abort code for this error.
    pub const fn abort_code(self) -> u32 {
        match self {
            OdErr::NoSuchObject => 0x0602_0000,
            OdErr::ReadOnly => 0x0601_0002,
        }
    }
}

/// Read an object. Returns `(value, size_bytes)` on success.
///
/// `value` is the scalar zero-extended into a `u32`; the SDO server sends only
/// `size` bytes little-endian.
pub fn read(index: u16, sub: u8) -> Result<(u32, u8), OdErr> {
    match (index, sub) {
        // ---- 0x2000: parameters (read back current atomic value) ----
        (idx::PARAMS, 0x01) => Ok((PARAMS.enable.load(RLX) as u32, 1)),
        (idx::PARAMS, 0x02) => Ok((PARAMS.mode.load(RLX) as u32, 1)),
        (idx::PARAMS, 0x03) => Ok((PARAMS.amplitude.load(RLX) as u16 as u32, 2)),
        (idx::PARAMS, 0x04) => Ok((PARAMS.lead_angle.load(RLX) as u32, 2)),
        (idx::PARAMS, 0x05) => Ok((PARAMS.direction.load(RLX) as u32, 1)),
        (idx::PARAMS, 0x06) => Ok((PARAMS.offset.load(RLX) as u32, 2)),
        (idx::PARAMS, 0x07) => Ok((PARAMS.pole_pairs.load(RLX) as u32, 2)),
        (idx::PARAMS, 0x08) => Ok((PARAMS.ol_angle.load(RLX) as u32, 2)),
        (idx::PARAMS, 0x09) => Ok((PARAMS.ol_rate.load(RLX) as u16 as u32, 2)),
        (idx::PARAMS, 0x0A) => Ok((PARAMS.shape.load(RLX) as u32, 1)),

        // ---- 0x2001: telemetry (one coherent seqlock snapshot) ----
        (idx::TELEMETRY, sub) if (0x01..=0x06).contains(&sub) => {
            let t = TELEM.read();
            let (v, sz) = match sub {
                0x01 => (t.pos as u32, 4),           // enc_raw / position
                0x02 => (t.theta_e as u32, 2),       // theta_electrical
                0x03 => (t.vel as u32, 4),           // velocity
                0x04 => (t.iq as u16 as u32, 2),     // current A (iq slot)
                0x05 => (t.id as u16 as u32, 2),     // current B (id slot)
                0x06 => (t.faults as u32, 2),        // liveness/faults (tick TBD)
                _ => unreachable!(),
            };
            Ok((v, sz))
        }

        // ---- 0x2F00:05 status (RO): the bootloader's status marker word ----
        (idx::MGMT, msub::STATUS) => {
            let s = unsafe { core::ptr::read_volatile(mgmt::BOOT_FLAG.wrapping_sub(0)) };
            // Report the current stay-flag value; a fuller status word can be
            // packed here later (image id, uptime). 4 bytes.
            Ok((s, 4))
        }

        _ => Err(OdErr::NoSuchObject),
    }
}

/// Write an object. Only the low `size` bytes of `val` are significant; the SDO
/// server has already assembled the little-endian payload into `val`.
///
/// Returns `Ok(Some(action))` for a `0x2F00` management write the caller must
/// execute (reboot/stay/boot/invalidate), or `Ok(None)` for a plain parameter
/// write that took effect immediately.
pub fn write(index: u16, sub: u8, val: u32) -> Result<Option<MgmtAction>, OdErr> {
    match (index, sub) {
        // ---- 0x2000: parameters ----
        (idx::PARAMS, 0x01) => store(|| PARAMS.enable.store(val != 0, RLX)),
        (idx::PARAMS, 0x02) => store(|| PARAMS.mode.store(val as u8, RLX)),
        (idx::PARAMS, 0x03) => store(|| PARAMS.amplitude.store(val as i16, RLX)),
        (idx::PARAMS, 0x04) => store(|| PARAMS.lead_angle.store(val as u16, RLX)),
        (idx::PARAMS, 0x05) => store(|| PARAMS.direction.store(val as u8, RLX)),
        (idx::PARAMS, 0x06) => store(|| PARAMS.offset.store(val as u16, RLX)),
        (idx::PARAMS, 0x07) => store(|| PARAMS.pole_pairs.store(val as u16, RLX)),
        (idx::PARAMS, 0x08) => store(|| PARAMS.ol_angle.store(val as u16, RLX)),
        (idx::PARAMS, 0x09) => store(|| PARAMS.ol_rate.store(val as i16, RLX)),
        (idx::PARAMS, 0x0A) => store(|| PARAMS.shape.store(val as u8, RLX)),

        // ---- 0x2001: telemetry is read-only ----
        (idx::TELEMETRY, sub) if (0x01..=0x06).contains(&sub) => Err(OdErr::ReadOnly),

        // ---- 0x2F00: board management (write 1 to trigger) ----
        (idx::MGMT, msub::REBOOT) if val != 0 => Ok(Some(MgmtAction::Reboot)),
        (idx::MGMT, msub::STAY_IN_BOOT) if val != 0 => Ok(Some(MgmtAction::StayInBoot)),
        (idx::MGMT, msub::BOOT_APP) if val != 0 => Ok(Some(MgmtAction::BootApp)),
        (idx::MGMT, msub::INVALIDATE_APP) if val != 0 => Ok(Some(MgmtAction::InvalidateApp)),
        // write 0 to a trigger is a harmless no-op (ack, no action)
        (idx::MGMT, s) if (0x01..=0x04).contains(&s) => Ok(None),
        (idx::MGMT, msub::STATUS) => Err(OdErr::ReadOnly),

        _ => Err(OdErr::NoSuchObject),
    }
}

#[inline]
fn store(f: impl FnOnce()) -> Result<Option<MgmtAction>, OdErr> {
    f();
    Ok(None)
}
