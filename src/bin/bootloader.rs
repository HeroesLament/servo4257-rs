#![no_std]
#![no_main]
//! CAN bootloader — Step 2 skeleton: boot-flag check, app validity check,
//! and the bootloader->app jump (VTOR relocate + MSP/PC handoff).
//! Build: cargo build --release --bin bootloader --features board-57d,layout-boot
//!
//! Status (debug) published to RAM 0x20000000 before any jump, so SWD can see
//! what the bootloader decided:
//!   [0] 0xB007_0001  bootloader ran
//!   [1] boot_flag value read from _boot_flag
//!   [2] app SP   (word @ APP_BASE)
//!   [3] app reset vector (word @ APP_BASE+4)
//!   [4] decision: 0xJUMP=jump-to-app, 0x5TAY=stay-in-bootloader
//!   [5] meta state: 0=absent(heuristic used) 1=present+valid 2=present+BAD-CRC

use panic_halt as _;
use n32l4 as _;
use cortex_m_rt::entry;
use servo4257_rs::app_meta::AppMeta;

const APP_BASE: u32 = 0x0800_4000;
const STATUS: *mut u32 = 0x2000_0000 as *mut u32;

const FLAG_STAY_IN_BOOT: u32 = 0xB007_57A4; // "stay" magic the app writes
extern "C" {
    static mut _boot_flag: u32;
}

fn set(i: usize, v: u32) { unsafe { core::ptr::write_volatile(STATUS.add(i), v) } }

#[entry]
fn main() -> ! {
    set(0, 0xB007_0001);

    // Read (and consume) the boot-flag from no-init RAM.
    let flag = unsafe { core::ptr::read_volatile(&raw const _boot_flag) };
    set(1, flag);
    // Clear it so a future normal reset doesn't re-trap into the bootloader.
    unsafe { core::ptr::write_volatile(&raw mut _boot_flag, 0) };

    // Peek the app's vector table.
    let app_sp = unsafe { core::ptr::read_volatile(APP_BASE as *const u32) };
    let app_rv = unsafe { core::ptr::read_volatile((APP_BASE + 4) as *const u32) };
    set(2, app_sp);
    set(3, app_rv);

    // Validity decision. APP-META is authoritative when present: a populated
    // record means the image was shipped through xtask/the CAN download path,
    // so we trust its CRC over the heuristic. When NO meta record is present
    // (e.g. a plain SWD-flashed app during bring-up), fall back to the
    // structural heuristic so the board is still usable before xtask exists.
    let meta = unsafe { AppMeta::read_from_flash() };
    let app_valid = if meta.is_present() {
        let ok = unsafe { meta.verify_app() };
        set(5, if ok { 1 } else { 2 });
        ok
    } else {
        // Heuristic fallback: SP points into RAM, reset vector into app flash.
        set(5, 0);
        let sp_ok = (app_sp & 0xFFFF_0000) == 0x2000_0000;
        let rv_ok = app_rv >= APP_BASE && app_rv < 0x0802_0000;
        sp_ok && rv_ok
    };

    let stay = flag == FLAG_STAY_IN_BOOT || !app_valid;

    if stay {
        set(4, 0x5_7A4_000);
        run_bootloader_service();
    }

    // App is valid and no stay flag — but before jumping, open a short CAN
    // LISTEN WINDOW (~300 ms). If a management/enter-update command arrives, we
    // stay in the bootloader instead of booting. This is the recovery net: a
    // hung app (which never answers CAN) is caught by power-cycling and sending
    // a stay command during this window. Without it, a bricked app forces SWD.
    // CAN listen window (~300 ms) before jumping — the recovery net for a hung
    // app. The window brings up CAN (and thus reconfigures the clock tree); if it
    // elapses without a stay request we RESTORE THE RCC TO RESET DEFAULTS before
    // jumping, so the app's `rcc.freeze()` starts from the same clean state it
    // would see on a cold boot. (Root-caused: leaving the window's HSE/PLL clocks
    // in place hung the app in clock init before CAN came up.)
    #[cfg(feature = "hw-can")]
    {
        if listen_window_wants_stay() {
            set(4, 0x1157_E000); // window-stay decision
            run_bootloader_service();
        }
        set(4, 0x1157_5717); // window elapsed → restore clocks, then jump
        rcc_reset_to_default();
    }

    set(4, 0x0000_3001); // JUMP decision
    jump_to_app();
}

/// Restore RCC to its post-reset state: SYSCLK ← HSI, then clear the config so
/// PLL/HSE/prescalers are all back to defaults. Must run before jumping to an
/// app that will reconfigure clocks from scratch — a HAL `rcc.freeze()` assumes
/// a clean reset state and can hang if it inherits a live PLL. Mirrors the
/// classic STM32 "deinit RCC" sequence, adapted to the N32L40x register layout.
#[cfg(feature = "hw-can")]
fn rcc_reset_to_default() {
    use n32l4xx_hal::pac;
    let rcc = unsafe { &*pac::Rcc::ptr() };

    // 1. Ensure HSI is on and ready, then switch SYSCLK to it.
    rcc.ctrl().modify(|_, w| w.hsien().set_bit());
    while rcc.ctrl().read().hsirdf().bit_is_clear() {}
    rcc.cfg().modify(|_, w| unsafe { w.sclksw().bits(0b00) }); // 00 = HSI
    while rcc.cfg().read().sclksts().bits() != 0b00 {}

    // 2. Turn off HSE, PLL, CSS, and clear the bypass.
    rcc.ctrl()
        .modify(|_, w| w.hseen().clear_bit().pllen().clear_bit().clkssen().clear_bit().hsebp().clear_bit());
    while rcc.ctrl().read().pllrdf().bit_is_set() {}

    // 3. Clear CFG (prescalers, PLL source/mul, MCO) back to reset (0).
    rcc.cfg().reset();
}

/// Bring up CAN and listen for ~300 ms for a "stay in bootloader" request:
/// either the CiA-302 enter-update poke (`0x1F51`) or a board-management
/// stay/reboot write (`0x2F00:02`/`:01`). Returns true to stay.
///
/// This runs on the *bare reset clock* so it's fast to enter — but CAN needs a
/// known PCLK1 for its bit timing, so we set HSE→16 MHz PCLK1 first (matching
/// `BxCanTransport`'s BTR). Kept minimal: no flash driver, just a receive poll.
#[cfg(feature = "hw-can")]
fn listen_window_wants_stay() -> bool {
    use n32l4xx_hal::prelude::*;
    use n32l4xx_hal::can::Can;
    use servo4257_rs::canopen::transport::{BxCanTransport, CanTransport};

    let dp = unsafe { n32l4xx_hal::pac::Peripherals::steal() };

    // EXACT same clock config as run_bootloader_service (proven working): HSE
    // 8 MHz direct → sysclk 8, hclk/pclk1 4 MHz. No PLL. BxCanTransport's
    // BTR_500K_8MHZ (0x00050000) is derived for this 4 MHz PCLK1. A `sysclk(16M)`
    // here needs the PLL and made the HAL `freeze()` panic (→ panic_halt spin,
    // which read as "app won't boot"). Match the service to stay valid.
    let _clocks = dp
        .rcc
        .constrain()
        .cfgr
        .use_hse(8_000_000.Hz())
        .sysclk(8_000_000.Hz())
        .hclk(4_000_000.Hz())
        .pclk1(4_000_000.Hz())
        .freeze();

    let gpioa = dp.gpioa.split();
    let hal_can = Can::new(dp.can);
    hal_can.assign_pins((gpioa.pa11, gpioa.pa12));
    let mut can = BxCanTransport::new(hal_can);

    set(6, 0xCA_11_0000); // listen window open

    // ~300 ms of polling. The core is at 16 MHz here; a simple spin-count budget
    // bounds the window without a timer. RX_COBID = 0x601 (node 1).
    const RX_COBID: u16 = 0x601;
    // 16 MHz, ~a few cycles per poll iteration → ~300 ms budget.
    let mut budget: u32 = 1_200_000;
    while budget > 0 {
        budget -= 1;
        if let Ok(Some(frame)) = can.try_recv() {
            if frame.id == RX_COBID {
                let d = frame.payload();
                // enter-update poke: [.., idx=0x1F51, ..]
                let is_enter_update = d.len() >= 3 && d[1] == 0x51 && d[2] == 0x1F;
                // mgmt stay/reboot write: [0x2x, idx=0x2F00, sub in {01,02}]
                let is_mgmt_stay = d.len() >= 4
                    && (d[0] & 0xE0) == 0x20
                    && d[1] == 0x00
                    && d[2] == 0x2F
                    && (d[3] == 0x01 || d[3] == 0x02);
                if is_enter_update || is_mgmt_stay {
                    return true;
                }
            }
        }
    }
    false
}

/// The "stay in bootloader" service: bring up CAN and run the CANopen download
/// loop. On a complete, CRC-valid image it resets so the fresh app is taken;
/// on abort it parks (SWD-recoverable). Without the `hw-can` feature there's no
/// transport, so it simply parks.
#[cfg(feature = "hw-can")]
fn run_bootloader_service() -> ! {
    use n32l4xx_hal::prelude::*;
    use n32l4xx_hal::gpio::GpioExt;
    use n32l4xx_hal::can::Can;
    use n32l4xx_hal::fmc::FMCExt;
    use servo4257_rs::canopen::transport::BxCanTransport;
    use servo4257_rs::canopen::download::{run_download, DownloadOutcome};

    // SAFETY: the bootloader is the sole owner of the device here; the app has
    // not started. Steal is appropriate — we never constructed a HAL `take()`.
    let dp = unsafe { n32l4xx_hal::pac::Peripherals::steal() };

    // Clock the system from the external 8 MHz crystal (HSE) so CAN runs off a
    // precise, known PCLK1 = 8 MHz. The bare reset clock leaves PCLK1 at 4 MHz on
    // the internal RC -- wrong rate (125k not 500k) and too jittery for CAN. Only
    // in this download branch; the jump-to-app path keeps the reset clock.
    let _clocks = dp.rcc.constrain().cfgr
        .use_hse(8_000_000.Hz())
        .sysclk(8_000_000.Hz())
        .hclk(4_000_000.Hz())
        .pclk1(4_000_000.Hz())
        .freeze();

    // freeze() turns ON the flash instruction cache + prefetch buffer. On this
    // part, having them enabled during a flash erase/program wedges the FMC
    // busy-wait (the core hangs mid-erase). flashtest.rs erases fine with them
    // OFF (the reset default). Disable them before any download flash writes.
    unsafe {
        (*n32l4xx_hal::pac::Flash::ptr())
            .ac()
            .modify(|_, w| w.icahen().clear_bit().prftbfe().clear_bit());
    }

    // Bring up the flash driver (its own peripheral).
    let mut flash = dp.flash.constrain();

    // Configure CAN pins PA11 (RX, AF1) / PA12 (TX, AF1) and enable the CAN
    // peripheral clock. The bootloader stays on bare HSI (16 MHz PCLK1), which
    // is what BxCanTransport's 500 kbps BTR assumes.
    let gpioa = dp.gpioa.split();
    let hal_can = Can::new(dp.can);
    hal_can.assign_pins((gpioa.pa11, gpioa.pa12));

    let mut can = BxCanTransport::new(hal_can);

    set(6, 0xCA_00_0000); // CAN up, entering download service

    match run_download(&mut can, &mut flash, |code| set(7, code)) {
        DownloadOutcome::Complete { len } => {
            set(6, 0xCA_D0_0000 | (len & 0xFFFF));
            // Reset so the boot path re-runs and validates + jumps to the new app.
            cortex_m::peripheral::SCB::sys_reset();
        }
        DownloadOutcome::Aborted => {
            set(6, 0xCA_AB_0000);
            loop { cortex_m::asm::nop(); }
        }
    }
}

/// No CAN transport compiled in: nothing to serve, so park.
#[cfg(not(feature = "hw-can"))]
fn run_bootloader_service() -> ! {
    loop { cortex_m::asm::nop(); }
}

/// Hand control to the application: cortex_m::asm::bootload reads the app's
/// initial SP and reset vector from its vector table, sets VTOR=APP_BASE,
/// installs MSP, and branches. Never returns.
fn jump_to_app() -> ! {
    unsafe {
        let vt = APP_BASE as *const u32;
        cortex_m::asm::bootload(vt)
    }
}
