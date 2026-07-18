//! shared/ — the lock-free contract between the real-time ISRs and the async
//! (embassy) tier. Nothing here blocks: the 25 kHz control ISR touches only
//! atomics and the telemetry seqlock, never an embassy critical section.
//!
//!   async tier ── SETPOINTS (atomics) ──▶ cascade/current ISR
//!   control ISR ── TELEM (seqlock)     ──▶ CANopen PDO + display
//!
//! Wire in with `pub mod shared;` in lib.rs.

pub mod params;
pub mod setpoints;
pub mod telemetry;
// pub mod events;   // Signals (SYNC / FAULT / CONFIG_CHANGED) — pending embassy-sync

pub use params::{CommMode, CommParams, Snapshot, PARAMS};
pub use setpoints::{Mode, Setpoints, SETPOINTS};
pub use telemetry::{SeqLock, Telem, TELEM};
