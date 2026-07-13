//! shared/setpoints.rs — async -> ISR command state, lock-free atomics.
//! Written by the CANopen tier; read by the cascade/current ISRs.

use core::sync::atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU16, AtomicU8, Ordering};

const RLX: Ordering = Ordering::Relaxed;

/// CiA 402 modes of operation we intend to support (subset).
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    Disabled = 0,
    ProfilePosition = 1,
    ProfileVelocity = 3,
    CyclicSyncPosition = 8,
    CyclicSyncVelocity = 9,
    CyclicSyncTorque = 10,
}

impl Mode {
    pub fn from_u8(v: u8) -> Option<Mode> {
        Some(match v {
            0 => Mode::Disabled,
            1 => Mode::ProfilePosition,
            3 => Mode::ProfileVelocity,
            8 => Mode::CyclicSyncPosition,
            9 => Mode::CyclicSyncVelocity,
            10 => Mode::CyclicSyncTorque,
            _ => return None,
        })
    }
}

/// Command state. Per-field atomics (each independently consistent); use TELEM's
/// seqlock when several values must be sampled coherently.
pub struct Setpoints {
    pub mode: AtomicU8,
    pub enabled: AtomicBool,
    pub target_pos: AtomicI32,
    pub target_vel: AtomicI32,
    pub target_torque: AtomicI16,
    pub current_limit_ma: AtomicU16,
}

impl Setpoints {
    pub const fn new() -> Self {
        Self {
            mode: AtomicU8::new(Mode::Disabled as u8),
            enabled: AtomicBool::new(false),
            target_pos: AtomicI32::new(0),
            target_vel: AtomicI32::new(0),
            target_torque: AtomicI16::new(0),
            current_limit_ma: AtomicU16::new(0),
        }
    }

    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.enabled.load(RLX)
    }
    #[inline]
    pub fn mode(&self) -> u8 {
        self.mode.load(RLX)
    }
    #[inline]
    pub fn target_pos(&self) -> i32 {
        self.target_pos.load(RLX)
    }
    #[inline]
    pub fn target_vel(&self) -> i32 {
        self.target_vel.load(RLX)
    }
    #[inline]
    pub fn target_torque(&self) -> i16 {
        self.target_torque.load(RLX)
    }
}

/// The global command state.
pub static SETPOINTS: Setpoints = Setpoints::new();
