//! shared/telemetry.rs — ISR -> async, coherent lock-free snapshot (seqlock).
//! Single writer (the control ISR), many readers (CANopen PDO, display).

use core::cell::UnsafeCell;
use core::sync::atomic::{fence, AtomicU32, Ordering};

/// Latest control-loop telemetry, published coherently each tick.
#[derive(Clone, Copy)]
pub struct Telem {
    pub pos: i32,
    pub vel: i32,
    pub iq: i16,
    pub id: i16,
    pub theta_e: u16,
    pub vbus: u16,
    pub temp: i16,
    pub faults: u16,
}

impl Telem {
    pub const ZERO: Telem = Telem {
        pos: 0,
        vel: 0,
        iq: 0,
        id: 0,
        theta_e: 0,
        vbus: 0,
        temp: 0,
        faults: 0,
    };
}

/// Single-writer / multi-reader seqlock over a `Copy` value. The writer must be
/// a single owner (the highest-priority control ISR); readers retry on a
/// straddled write. The memory ordering is the standard seqlock shape but the
/// fences deserve a review before trusting across an aggressive compiler.
pub struct SeqLock<T: Copy> {
    seq: AtomicU32,
    data: UnsafeCell<T>,
}

// SAFETY: single writer; readers take a value copy and re-check the sequence.
unsafe impl<T: Copy + Send> Sync for SeqLock<T> {}

impl<T: Copy> SeqLock<T> {
    pub const fn new(v: T) -> Self {
        Self {
            seq: AtomicU32::new(0),
            data: UnsafeCell::new(v),
        }
    }

    /// Writer (single owner, e.g. the control ISR).
    #[inline]
    pub fn write(&self, f: impl FnOnce(&mut T)) {
        let s = self.seq.load(Ordering::Relaxed);
        self.seq.store(s.wrapping_add(1), Ordering::Release);
        fence(Ordering::Release);
        // SAFETY: single writer.
        f(unsafe { &mut *self.data.get() });
        fence(Ordering::Release);
        self.seq.store(s.wrapping_add(2), Ordering::Release);
    }

    #[inline]
    pub fn store(&self, v: T) {
        self.write(|d| *d = v);
    }

    /// Reader: retries if a write straddled the read.
    #[inline]
    pub fn read(&self) -> T {
        loop {
            let s1 = self.seq.load(Ordering::Acquire);
            if s1 & 1 != 0 {
                core::hint::spin_loop();
                continue;
            }
            // SAFETY: value copy; the sequence re-check below rejects a torn read.
            let v = unsafe { core::ptr::read_volatile(self.data.get()) };
            fence(Ordering::Acquire);
            if self.seq.load(Ordering::Acquire) == s1 {
                return v;
            }
            core::hint::spin_loop();
        }
    }
}

/// The global telemetry snapshot.
pub static TELEM: SeqLock<Telem> = SeqLock::new(Telem::ZERO);
