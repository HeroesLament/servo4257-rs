//! Board-management manufacturer objects (`0x2F00`) + reset / watchdog helpers.
//!
//! Shared by the bootloader and the app so the board has a uniform lifecycle
//! control surface over CAN in *every* running state — the fix for "app hung,
//! must demount for SWD". Combined with the bootloader's boot-time CAN listen
//! window and the app's IWDG, a hung app is always recoverable over CAN:
//!
//!   * app hangs → IWDG resets it (~1 s) → bootloader listen window catches it;
//!   * or power-cycle → `Board.recover` spams stay-in-boot across the window.
//!
//! Management object area `0x2F00`:
//!   :01 reboot          (WO) write 1 → SCB::sys_reset()
//!   :02 stay_in_boot    (WO) write 1 → set stay flag + reset → bootloader
//!   :03 boot_app        (WO) write 1 → clear stay flag + reset → app (if valid)
//!   :04 invalidate_app  (WO) write 1 → erase app META → bootloader on next boot
//!   :05 status          (RO) packed boot state / marker
//!
//! `reboot`/`stay`/`boot`/`invalidate` are handled by the caller (they need
//! flash / reset context the bootloader and app supply differently); this module
//! defines the object numbers, the stay-flag magic, and the low-level actions
//! that are identical in both images.

use core::sync::atomic::{compiler_fence, Ordering};

/// Manufacturer index for board management.
pub const MGMT_INDEX: u16 = 0x2F00;

pub mod sub {
    pub const REBOOT: u8 = 0x01;
    pub const STAY_IN_BOOT: u8 = 0x02;
    pub const BOOT_APP: u8 = 0x03;
    pub const INVALIDATE_APP: u8 = 0x04;
    pub const STATUS: u8 = 0x05;
}

/// No-init RAM cell (top of RAM, shared with the bootloader's `_boot_flag`) and
/// the magics the boot path reads. Kept here so app and bootloader agree.
pub const BOOT_FLAG: *mut u32 = 0x2000_5FF8 as *mut u32;
pub const FLAG_STAY_IN_BOOT: u32 = 0xB007_57A4;
/// Written to explicitly request an app boot (clears any stale stay magic).
pub const FLAG_BOOT_APP: u32 = 0x0000_0000;

/// Set the stay-in-boot flag and reset. Lands in the bootloader.
pub fn reset_into_bootloader() -> ! {
    unsafe { core::ptr::write_volatile(BOOT_FLAG, FLAG_STAY_IN_BOOT) };
    compiler_fence(Ordering::SeqCst);
    cortex_m::peripheral::SCB::sys_reset()
}

/// Clear the stay flag and reset. Lands in the app (if valid).
pub fn reset_into_app() -> ! {
    unsafe { core::ptr::write_volatile(BOOT_FLAG, FLAG_BOOT_APP) };
    compiler_fence(Ordering::SeqCst);
    cortex_m::peripheral::SCB::sys_reset()
}

/// Plain MCU reset (honors whatever the boot flag currently says).
pub fn reset() -> ! {
    compiler_fence(Ordering::SeqCst);
    cortex_m::peripheral::SCB::sys_reset()
}

/// Independent watchdog (IWDG): free-running LSI-clocked reset timer. Start it
/// once, then `feed()` faster than the timeout or the MCU resets. Register-level
/// since the HAL lacks an IWDG driver.
///
/// LSI ≈ 40 kHz. With prescaler /256 the tick is ~6.4 ms; reload `RL` gives a
/// timeout of `(RL+1) * 256 / 40 kHz`. `RL = 156` → ~1.0 s.
pub struct Iwdg;

impl Iwdg {
    const KEY: *mut u32 = 0x4000_3000 as *mut u32; // IWDG_KEY
    const PR: *mut u32 = 0x4000_3004 as *mut u32; // IWDG_PREDIV
    const RLR: *mut u32 = 0x4000_3008 as *mut u32; // IWDG_RELV
    const SR: *const u32 = 0x4000_300C as *const u32; // IWDG_STS

    const KEY_FEED: u32 = 0xAAAA;
    const KEY_ACCESS: u32 = 0x5555;
    const KEY_START: u32 = 0xCCCC;

    const PR_DIV256: u32 = 0b110;

    /// Start the IWDG with a ~`timeout_ms` window (best-effort; LSI tolerance
    /// makes this approximate — size the reload generously).
    pub fn start(reload: u16) -> Self {
        unsafe {
            core::ptr::write_volatile(Self::KEY, Self::KEY_START); // enable
            core::ptr::write_volatile(Self::KEY, Self::KEY_ACCESS); // unlock PR/RLR
            core::ptr::write_volatile(Self::PR, Self::PR_DIV256);
            core::ptr::write_volatile(Self::RLR, (reload & 0x0FFF) as u32);
            // Wait for the register-update flags to clear (PVU/RVU in STS).
            while core::ptr::read_volatile(Self::SR) & 0b11 != 0 {}
            core::ptr::write_volatile(Self::KEY, Self::KEY_FEED); // reload
        }
        Iwdg
    }

    /// ~1 s window at LSI 40 kHz, /256 prescale (reload 156).
    pub fn start_1s() -> Self {
        Self::start(156)
    }

    /// Kick the dog. Call every main-loop pass.
    #[inline]
    pub fn feed(&mut self) {
        unsafe { core::ptr::write_volatile(Self::KEY, Self::KEY_FEED) };
    }
}
