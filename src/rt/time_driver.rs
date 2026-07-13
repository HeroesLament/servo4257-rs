//! TIM2 16-bit embassy-time driver (1 MHz monotonic). now() = atomic PERIOD + CNT.
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;
use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use n32l4xx_hal::pac;
use n32l4xx_hal::rcc::Clocks;

const CC1_WINDOW: u64 = 0xFFFF; // max ticks we can arm CC1 directly (16-bit)

static PERIOD: AtomicU32 = AtomicU32::new(0);
pub static ISR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CC1_COUNT: AtomicU32 = AtomicU32::new(0);

struct TimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: TimeDriver = TimeDriver { queue: Mutex::new(RefCell::new(Queue::new())) }
);

#[inline(always)]
fn tim() -> &'static pac::tim2::RegisterBlock {
    unsafe { &*pac::Tim2::ptr() }
}

/// Lock-free 64-bit now(). Retries if a period rollover straddles the read.
#[inline]
fn now64() -> u64 {
    loop {
        let p1 = PERIOD.load(Ordering::Relaxed);
        let cnt = tim().cnt().read().cnt().bits() as u64;
        let p2 = PERIOD.load(Ordering::Relaxed);
        if p1 == p2 {
            return ((p1 as u64) << 16) | cnt;
        }
    }
}

/// Arm (or disarm) the CC1 alarm for absolute tick `at`. Must be called with
/// interrupts masked (from schedule_wake's critical section or the TIM2 ISR).
fn arm(at: u64) {
    let t = tim();
    if at == u64::MAX {
        t.dinten().modify(|_, w| w.cc1ien().clear_bit());
        return;
    }
    let now = now64();
    t.ccdat1().write(|w| unsafe { w.ccdat1().bits((at & 0xFFFF) as u16) });
    t.sts().modify(|_, w| w.cc1itf().clear_bit()); // clear stale match, keep UIF
    if at <= now || at - now <= CC1_WINDOW {
        t.dinten().modify(|_, w| w.cc1ien().set_bit());
    } else {
        // too far for a 16-bit compare; the overflow ISR re-arms as we approach.
        t.dinten().modify(|_, w| w.cc1ien().clear_bit());
    }
}

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        now64()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow(cs).borrow_mut();
            if q.schedule_wake(at, waker) {
                arm(q.next_expiration(now64()));
            }
        });
    }
}

/// Keep SWD/debug alive while the core WFE-sleeps, by setting DBG_CTRL.SLEEP
/// (UM 76029: keeps HCLK = FCLK during SLEEP). DEV AID ONLY: it costs power
/// (HCLK keeps running during sleep), so gate it out of production builds.
/// Verified: with this set, the sleeping embassy executor stays SWD-readable
/// and the CC1 alarm still wakes it.
pub fn enable_debug_sleep() {
    // DBG_CTRL @ 0xE0042004, SLEEP = bit 0 (external PPB; no clock-enable needed).
    unsafe { (*pac::Dbg::ptr()).ctrl().modify(|_, w| w.sleep().set_bit()) };
}

/// Initialize TIM2 as the 1 MHz monotonic base. `timer_clk_hz` is the TIM2 input
/// (APB1 timer) clock. NOTE: the N32L406 boots on MSI = 4 MHz (NOT HSI 16 MHz;
/// UM 7816 / DS 1340), so at reset pass 4_000_000; once the app runs on HSE/PLL,
/// pass the HAL Clocks-derived timer clock so the tick rate stays exact.
pub fn init(timer_clk_hz: u32) {
    let rcc = unsafe { &*pac::Rcc::ptr() };
    rcc.apb1pclken().modify(|_, w| w.tim2en().set_bit());

    let t = tim();
    t.ctrl1().modify(|_, w| w.cnten().clear_bit());
    let psc = (timer_clk_hz / 1_000_000).saturating_sub(1) as u16;
    t.psc().write(|w| unsafe { w.psc().bits(psc) });
    t.ar().write(|w| unsafe { w.ar().bits(0xFFFF) });
    t.cnt().write(|w| unsafe { w.cnt().bits(0) });
    t.evtgen().write(|w| w.udgn().set_bit()); // force update: load PSC
    t.sts().modify(|_, w| w.uditf().clear_bit().cc1itf().clear_bit());
    t.dinten().modify(|_, w| w.uien().set_bit()); // overflow interrupt (period ext)
    PERIOD.store(0, Ordering::Relaxed);
    t.ctrl1().modify(|_, w| w.cnten().set_bit()); // start

    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2) };
}

/// TIM2 (APB1) input clock derived from the frozen HAL `Clocks`.
///
/// TIM2 sits on APB1. Per the reference-manual timer-clock rule, the timer input
/// is PCLK1 when the APB1 prescaler is 1, and 2*PCLK1 when it is >1. `Clocks`
/// does not expose the prescaler, but PCLK1 == HCLK is exactly the "prescaler is
/// 1" condition, so we infer the doubling from that.
pub fn tim2_input_hz(clocks: &Clocks) -> u32 {
    let pclk1 = clocks.pclk1().raw();
    let hclk = clocks.hclk().raw();
    if pclk1 == hclk {
        pclk1
    } else {
        pclk1 * 2
    }
}

/// Initialize the 1 MHz time base, self-calibrating the TIM2 prescaler from the
/// frozen HAL `Clocks`. Prefer this over `init(hz)` once the app has configured
/// its clocks: it keeps the tick exact across any sysclk/HSE/PLL config without a
/// hand-passed magic number. (`init` remains for the pre-`freeze()` reset-clock
/// smoke tests, where no `Clocks` exists yet.)
pub fn init_from_clocks(clocks: &Clocks) {
    init(tim2_input_hz(clocks));
}

#[no_mangle]
pub extern "C" fn TIM2() {
    ISR_COUNT.fetch_add(1, Ordering::Relaxed);
    let t = tim();
    // Read-and-clear each flag independently so a rollover can't be lost.
    if t.sts().read().uditf().bit_is_set() {
        t.sts().modify(|_, w| w.uditf().clear_bit());
        PERIOD.fetch_add(1, Ordering::Relaxed);
    }
    if t.sts().read().cc1itf().bit_is_set() {
        t.sts().modify(|_, w| w.cc1itf().clear_bit());
        CC1_COUNT.fetch_add(1, Ordering::Relaxed);
    }
    critical_section::with(|cs| {
        let mut q = DRIVER.queue.borrow(cs).borrow_mut();
        arm(q.next_expiration(now64()));
    });
}
