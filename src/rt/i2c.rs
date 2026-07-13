//! Interrupt-driven async I2C master for the N32L406 (I2C1) — the "embassy-n32"
//! I2C brick.
//!
//! The N32's I2C is the STM32F1-era v1 cell: a byte-at-a-time EV/ER event
//! machine, no DMA request-mux on this part (a documented HAL gap), so this
//! driver advances the transfer from the `I2C1_EV` interrupt and reports errors
//! from `I2C1_ER`, waking an async `poll_fn` on completion. Same idiom as the
//! time and CAN drivers (ISR does the work, wakes a waker).
//!
//! Scope: single-flight master transactions. `write` is complete and is all a
//! display (SSD1306/SH1106, write-only) needs; `read` implements the standard
//! v1 sequence for probes/config reads (the 2-byte case carries the usual v1
//! caveat and is best-effort). Bytes are staged through a fixed [`CAP`]-byte
//! buffer so no caller borrow crosses the await — no cancellation soundness
//! hazard, at the cost of a copy and a per-transfer size cap.
//!
//! NOTE: pins must be pre-assigned to the I2C1 alternate function (open-drain)
//! by the caller before `init`, and the bus needs external pull-ups.

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::Poll;

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use n32l4xx_hal::pac;

/// Max bytes per transfer (staged through the internal buffer). A 128x64 OLED
/// page write is 1 control byte + 128 data = 129, so 256 is comfortable.
pub const CAP: usize = 256;

/// I2C transaction errors.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// Address or data byte was not acknowledged.
    Nack,
    /// Lost arbitration (multi-master).
    Arbitration,
    /// RX overrun / TX underrun.
    Overrun,
    /// Bus error (misplaced START/STOP).
    Bus,
    /// Transfer larger than [`CAP`].
    TooLong,
    /// A transaction is already in flight (single-flight driver).
    Busy,
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum Op {
    None,
    Write,
    Read,
}

struct State {
    op: Op,
    addr: u8,
    len: usize,
    idx: usize,
    buf: [u8; CAP],
    result: Option<Result<(), Error>>,
}

impl State {
    const fn new() -> Self {
        State {
            op: Op::None,
            addr: 0,
            len: 0,
            idx: 0,
            buf: [0; CAP],
            result: None,
        }
    }
}

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::new()));
static WAKER: AtomicWaker = AtomicWaker::new();

#[inline(always)]
fn i2c() -> &'static pac::i2c1::RegisterBlock {
    unsafe { &*pac::I2c1::ptr() }
}

/// Configure I2C1 in standard mode at `freq_hz` (from `pclk_hz` = PCLK1) and
/// unmask the EV/ER interrupt vectors. Pins must already be on the I2C1 AF
/// (open-drain) with bus pull-ups present. Mirrors the HAL's `i2c_init`.
pub fn init(pclk_hz: u32, freq_hz: u32) {
    let rcc = unsafe { &*pac::Rcc::ptr() };
    rcc.apb1pclken().modify(|_, w| w.i2c1en().set_bit());

    let i2c = i2c();
    i2c.ctrl1().modify(|_, w| w.en().clear_bit());

    let clc_mhz = pclk_hz / 1_000_000;
    i2c.ctrl2().modify(|_, w| unsafe { w.clkfreq().bits(clc_mhz as u8) });
    i2c.tmrise()
        .write(|w| unsafe { w.tmrise().bits((clc_mhz + 1) as u8) });
    // Standard-mode CCR: t_high = t_low = CCR / pclk; bit = 2*CCR/pclk.
    let ccr = (pclk_hz / (freq_hz * 2)).max(4);
    i2c.clkctrl().modify(|_, w| unsafe {
        w.fsmode()
            .clear_bit()
            .duty()
            .clear_bit()
            .clkctrl()
            .bits(ccr as u16)
    });

    i2c.ctrl1().modify(|_, w| w.en().set_bit());

    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::I2C1_EV);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::I2C1_ER);
    }
}

/// Write `bytes` to the 7-bit `addr`. Awaits the STOP.
pub async fn write(addr: u8, bytes: &[u8]) -> Result<(), Error> {
    if bytes.len() > CAP {
        return Err(Error::TooLong);
    }
    critical_section::with(|cs| {
        let mut s = STATE.borrow(cs).borrow_mut();
        if s.op != Op::None {
            return Err(Error::Busy);
        }
        s.op = Op::Write;
        s.addr = addr;
        s.len = bytes.len();
        s.idx = 0;
        s.buf[..bytes.len()].copy_from_slice(bytes);
        s.result = None;
        Ok(())
    })?;

    start(Op::Write);
    complete().await
}

/// Read `buf.len()` bytes from the 7-bit `addr` into `buf`. Awaits the STOP.
///
/// Primarily for device probes / register reads; the write path is the
/// display's hot path. The single-byte case is handled explicitly; multi-byte
/// uses the standard v1 ACK-until-penultimate sequence.
pub async fn read(addr: u8, buf: &mut [u8]) -> Result<(), Error> {
    let len = buf.len();
    if len == 0 || len > CAP {
        return Err(Error::TooLong);
    }
    critical_section::with(|cs| {
        let mut s = STATE.borrow(cs).borrow_mut();
        if s.op != Op::None {
            return Err(Error::Busy);
        }
        s.op = Op::Read;
        s.addr = addr;
        s.len = len;
        s.idx = 0;
        s.result = None;
        Ok(())
    })?;

    start(Op::Read);
    let r = complete().await;
    if r.is_ok() {
        critical_section::with(|cs| {
            let s = STATE.borrow(cs).borrow();
            buf.copy_from_slice(&s.buf[..len]);
        });
    }
    r
}

/// Enable the EV/ER + buffer interrupts and issue START.
fn start(_op: Op) {
    let i2c = i2c();
    i2c.ctrl2()
        .modify(|_, w| w.evtinten().set_bit().bufinten().set_bit().errinten().set_bit());
    // ACK enabled (matters for multi-byte reads); START.
    i2c.ctrl1()
        .modify(|_, w| w.acken().set_bit().startgen().set_bit());
}

/// Await the ISR-published result.
async fn complete() -> Result<(), Error> {
    poll_fn(|cx| {
        critical_section::with(|cs| {
            let mut s = STATE.borrow(cs).borrow_mut();
            if let Some(r) = s.result.take() {
                Poll::Ready(r)
            } else {
                WAKER.register(cx.waker());
                Poll::Pending
            }
        })
    })
    .await
}

/// Disable all I2C interrupts and publish the transfer result.
fn finish(i2c: &pac::i2c1::RegisterBlock, s: &mut State, r: Result<(), Error>) {
    i2c.ctrl2().modify(|_, w| {
        w.evtinten()
            .clear_bit()
            .bufinten()
            .clear_bit()
            .errinten()
            .clear_bit()
    });
    s.op = Op::None;
    s.result = Some(r);
}

/// I2C1 event interrupt: advance the byte-at-a-time master sequence.
#[no_mangle]
pub extern "C" fn I2C1_EV() {
    critical_section::with(|cs| {
        let i2c = i2c();
        let mut s = STATE.borrow(cs).borrow_mut();
        let sr1 = i2c.sts1().read();

        match s.op {
            Op::Write => {
                if sr1.startbf().bit_is_set() {
                    // EV5: send address (write = addr<<1 | 0).
                    i2c.dat().write(|w| unsafe { w.bits((s.addr as u32) << 1) });
                } else if sr1.addrf().bit_is_set() {
                    // EV6: clear ADDR by reading SR2.
                    let _ = i2c.sts2().read();
                    if s.len == 0 {
                        i2c.ctrl1().modify(|_, w| w.stopgen().set_bit());
                        finish(i2c, &mut s, Ok(()));
                    }
                } else if sr1.txdate().bit_is_set() {
                    if s.idx < s.len {
                        // EV8/EV8_1: feed next byte.
                        let b = s.buf[s.idx];
                        s.idx += 1;
                        i2c.dat().write(|w| unsafe { w.bits(b as u32) });
                    } else if sr1.bsf().bit_is_set() {
                        // EV8_2: last byte fully shifted out — STOP and finish.
                        i2c.ctrl1().modify(|_, w| w.stopgen().set_bit());
                        finish(i2c, &mut s, Ok(()));
                    } else {
                        // DR empty but shift still busy: stop TXE interrupts and
                        // wait for BTF (delivered via the event interrupt).
                        i2c.ctrl2().modify(|_, w| w.bufinten().clear_bit());
                    }
                }
            }
            Op::Read => {
                if sr1.startbf().bit_is_set() {
                    // EV5: send address (read = addr<<1 | 1).
                    i2c.dat()
                        .write(|w| unsafe { w.bits(((s.addr as u32) << 1) | 1) });
                } else if sr1.addrf().bit_is_set() {
                    // EV6. For a single byte, NACK it and arm STOP before ADDR
                    // is cleared; otherwise leave ACK on and read SR2.
                    if s.len == 1 {
                        i2c.ctrl1().modify(|_, w| w.acken().clear_bit());
                        let _ = i2c.sts2().read();
                        i2c.ctrl1().modify(|_, w| w.stopgen().set_bit());
                    } else {
                        let _ = i2c.sts2().read();
                    }
                } else if sr1.rxdatne().bit_is_set() {
                    // EV7: store byte.
                    let b = i2c.dat().read().bits() as u8;
                    let i = s.idx;
                    s.buf[i] = b;
                    s.idx += 1;
                    let remaining = s.len - s.idx;
                    if remaining == 1 {
                        // NACK the final byte and arm STOP.
                        i2c.ctrl1()
                            .modify(|_, w| w.acken().clear_bit().stopgen().set_bit());
                    } else if remaining == 0 {
                        finish(i2c, &mut s, Ok(()));
                    }
                }
            }
            Op::None => {
                // Spurious: mask everything so we don't spin.
                i2c.ctrl2().modify(|_, w| {
                    w.evtinten()
                        .clear_bit()
                        .bufinten()
                        .clear_bit()
                        .errinten()
                        .clear_bit()
                });
            }
        }
    });
    WAKER.wake();
}

/// I2C1 error interrupt: classify, release the bus with STOP, report.
#[no_mangle]
pub extern "C" fn I2C1_ER() {
    critical_section::with(|cs| {
        let i2c = i2c();
        let mut s = STATE.borrow(cs).borrow_mut();
        let sr1 = i2c.sts1().read();

        let err = if sr1.ackfail().bit_is_set() {
            i2c.sts1().modify(|_, w| w.ackfail().clear_bit());
            Error::Nack
        } else if sr1.arlost().bit_is_set() {
            i2c.sts1().modify(|_, w| w.arlost().clear_bit());
            Error::Arbitration
        } else if sr1.overrun().bit_is_set() {
            i2c.sts1().modify(|_, w| w.overrun().clear_bit());
            Error::Overrun
        } else {
            // BERR (and anything else): per the errata BERR can be spurious;
            // clear and report as a bus error.
            i2c.sts1().modify(|_, w| w.buserr().clear_bit());
            Error::Bus
        };

        // Release the bus.
        i2c.ctrl1().modify(|_, w| w.stopgen().set_bit());
        finish(i2c, &mut s, Err(err));
    });
    WAKER.wake();
}
