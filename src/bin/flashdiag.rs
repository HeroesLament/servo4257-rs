//! v10 — the manual's fix, tested on the REAL HAL clock. Sets up the exact
//! bootloader clock (HSE 8MHz, sysclk 8MHz, HCLK/PCLK1 4MHz, cache+prefetch off)
//! via the HAL, then ENABLES HSI (RCC_CTRL.HSION), then raw program+erase.
//! N32L40x UM: "HSI must be turned on when flash is programmed (written/erased)."
//!   readable + erased => HSI-on is the fix; keep 4MHz CAN clock, no BTR change.
//!   wedged            => HSI-on insufficient at 4MHz; must raise HCLK to 16MHz.
//!
//! Status @0x2000_1000 (read 10):
//!  [0] 0xF1A5_000A  [1] RCC_CTRL (bit0 HSION=1, bit16/17 HSE)  [2] RCC_CFG
//!  [3] FMC_CTRL after unlock  [4] readback after program-0 (0)
//!  [5] erase count  [6] FMC_STS post  [7] readback after erase (0xFFFF_FFFF)
//!  [9] 0xF1A5_D1A6 sentinel
#![no_std]
#![no_main]

use panic_halt as _;
use n32l4xx_hal::prelude::*;
use n32l4xx_hal::pac;
use cortex_m_rt::entry;

const S: *mut u32 = 0x2000_1000 as *mut u32;
const RCC_CTRL: *mut u32 = 0x4002_1000 as *mut u32;
const RCC_CFG: *mut u32  = 0x4002_1004 as *mut u32;
const FMC_KEY: *mut u32  = 0x4002_2004 as *mut u32;
const FMC_STS: *mut u32  = 0x4002_200C as *mut u32;
const FMC_CTRL: *mut u32 = 0x4002_2010 as *mut u32;
const FMC_ADD: *mut u32  = 0x4002_2014 as *mut u32;
const PAGE: *mut u32 = 0x0801_F800 as *mut u32;

const STS_CLR: u32 = 0x7C;
const CTRL_PG: u32 = 0x01;
const CTRL_PER: u32 = 0x02;
const CTRL_START: u32 = 0x40;
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;
const MAX: u32 = 8_000_000;

#[inline(never)]
#[link_section = ".data"]
unsafe fn run_erase() -> ! {
    let rd = |p: *mut u32| core::ptr::read_volatile(p);
    let wr = |p: *mut u32, v: u32| core::ptr::write_volatile(p, v);
    let st = |i: usize, v: u32| core::ptr::write_volatile(S.add(i), v);

    st(0, 0xF1A5_000A);
    st(1, rd(RCC_CTRL));
    st(2, rd(RCC_CFG));

    wr(FMC_STS, STS_CLR);
    wr(FMC_KEY, KEY1);
    wr(FMC_KEY, KEY2);
    st(3, rd(FMC_CTRL));

    // program 0
    wr(FMC_STS, STS_CLR);
    wr(FMC_CTRL, rd(FMC_CTRL) | CTRL_PG);
    wr(PAGE, 0);
    cortex_m::asm::dsb();
    let mut a: u32 = 0; while rd(FMC_STS) & 1 != 0 { a += 1; if a >= MAX { break; } }
    wr(FMC_CTRL, rd(FMC_CTRL) & !CTRL_PG);
    st(4, if a < MAX { rd(PAGE) } else { 0xBADB_0002 });

    // erase
    wr(FMC_STS, STS_CLR);
    wr(FMC_CTRL, rd(FMC_CTRL) | CTRL_PER);
    wr(FMC_ADD, PAGE as u32);
    wr(FMC_CTRL, rd(FMC_CTRL) | CTRL_START);
    cortex_m::asm::dsb();
    let mut b: u32 = 0; while rd(FMC_STS) & 1 != 0 { b += 1; if b >= MAX { break; } }
    st(5, b);
    st(6, rd(FMC_STS));
    wr(FMC_CTRL, rd(FMC_CTRL) & !CTRL_PER);
    st(7, if b < MAX { rd(PAGE) } else { 0xBADB_0003 });

    st(9, 0xF1A5_D1A6);
    loop { cortex_m::asm::nop(); }
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let _clocks = dp.rcc.constrain().cfgr
        .use_hse(8_000_000.Hz())
        .sysclk(8_000_000.Hz())
        .hclk(4_000_000.Hz())
        .pclk1(4_000_000.Hz())
        .freeze();
    unsafe {
        (*pac::Flash::ptr()).ac().modify(|_, w| w.icahen().clear_bit().prftbfe().clear_bit());
        // enable HSI (the manual's requirement for flash program/erase)
        core::ptr::write_volatile(RCC_CTRL, core::ptr::read_volatile(RCC_CTRL) | 1);
        while core::ptr::read_volatile(RCC_CTRL) & 2 == 0 {}
        run_erase()
    }
}
