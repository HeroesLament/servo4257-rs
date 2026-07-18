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

    set(4, 0x0000_3001); // JUMP decision
    jump_to_app();
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
