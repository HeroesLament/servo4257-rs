//! Async CAN driver for the N32L406 — the "embassy-n32" CAN brick.
//!
//! Interrupt-driven on top of bxcan, following the same idiom as the time
//! driver (ISR does the minimal hardware work and wakes; the async fn is a
//! `poll_fn` that registers a waker on "would block"):
//!
//!   * **RX** — `FIFO0_MESSAGE_PENDING` is enabled at init. The `CAN_RX0` ISR
//!     drains the hardware FIFO into a bounded [`Channel`]; `recv().await` pops
//!     it. The FIFO-pending interrupt is level-triggered, so draining to
//!     `WouldBlock` clears it.
//!   * **TX** — `send().await` tries `transmit()`; when all three mailboxes are
//!     busy it registers a waker, enables `TRANSMIT_MAILBOX_EMPTY`, and returns
//!     Pending. The `CAN_TX` ISR clears the completed-request flag, disables the
//!     TX interrupt, and wakes the sender, which retries.
//!
//! The driver speaks the portable [`Frame`] type so the CANopen layer above is
//! unchanged from the bootloader's polled transport. `send` is single-writer
//! (one TX waker): drive it from one task, or feed it from an outbound channel.
//!
//! This is the async counterpart to `canopen::transport::BxCanTransport` (which
//! stays polled for the bootloader, where there is no executor).

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::Poll;

use bxcan::{filter::Mask32, Fifo, Frame as BxFrame, Id, Interrupts, StandardId};
use canopen_proto::transport::{Frame, TransportError};
use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::waitqueue::AtomicWaker;
use n32l4xx_hal::can::Can as HalCan;
use n32l4xx_hal::pac;

/// Concrete bxcan type over the HAL's CAN instance (which impls
/// `bxcan::Instance` + `bxcan::FilterOwner`).
type BxCan = bxcan::Can<HalCan<pac::Can>>;

/// RX backlog depth. The hardware FIFO is only 3 deep; this is the software
/// cushion between an ISR burst and the consumer task draining it.
const RX_DEPTH: usize = 16;

static CAN: Mutex<RefCell<Option<BxCan>>> = Mutex::new(RefCell::new(None));
static RX: Channel<CriticalSectionRawMutex, Frame, RX_DEPTH> = Channel::new();
static TX_WAKER: AtomicWaker = AtomicWaker::new();

/// Bring up CAN at `btr` bit timing (accept-all filter, normal mode) and install
/// it as the async driver: enables the RX interrupt and unmasks the CAN vectors.
///
/// `btr` must match the current PCLK1 — a wrong value is the classic "powers up
/// but never ACKs" failure. For PCLK1 = 4 MHz / 500 kbps use
/// [`BTR_500K_PCLK4`]; recompute for any other clock.
///
/// Pins (PA11 RX / PA12 TX) must already be assigned to the CAN AF via the HAL.
pub fn init(instance: HalCan<pac::Can>, btr: u32) {
    let mut can = bxcan::Can::builder(instance)
        .set_bit_timing(btr)
        .leave_disabled();

    // Accept every standard frame; CANopen COB-ID filtering is done in software
    // above this layer.
    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    // Enter normal mode; spin until synced to the bus.
    nb::block!(can.enable_non_blocking()).ok();

    // RX interrupt on always; TX interrupt is armed on demand by `send`.
    can.enable_interrupts(Interrupts::FIFO0_MESSAGE_PENDING);

    critical_section::with(|cs| CAN.borrow(cs).replace(Some(can)));

    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::CAN_RX0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::CAN_TX);
    }
}

/// Send a standard-ID data frame, awaiting a free TX mailbox if all are busy.
///
/// Single-writer: only one task may be awaiting `send` at a time (one TX waker).
pub async fn send(frame: &Frame) -> Result<(), TransportError> {
    let id = StandardId::new(frame.id).ok_or(TransportError::NotReady)?;
    // payload() is always <= 8 bytes (Frame invariant), so Data::new never fails.
    let data = bxcan::Data::new(frame.payload()).unwrap_or_else(bxcan::Data::empty);
    let bx = BxFrame::new_data(id, data);

    poll_fn(|cx| {
        critical_section::with(|cs| {
            let mut slot = CAN.borrow(cs).borrow_mut();
            let can = match slot.as_mut() {
                Some(c) => c,
                None => return Poll::Ready(Err(TransportError::NotReady)),
            };
            match can.transmit(&bx) {
                Ok(_status) => Poll::Ready(Ok(())),
                Err(nb::Error::WouldBlock) => {
                    // Register BEFORE releasing the critical section so the ISR
                    // (which can only run after we return) cannot miss the wake.
                    TX_WAKER.register(cx.waker());
                    can.enable_interrupts(Interrupts::TRANSMIT_MAILBOX_EMPTY);
                    Poll::Pending
                }
                Err(nb::Error::Other(infallible)) => match infallible {},
            }
        })
    })
    .await
}

/// Await the next received standard-ID data frame. Non-standard and remote
/// frames are dropped by the ISR (the CANopen protocol uses neither).
pub async fn recv() -> Frame {
    RX.receive().await
}

/// Non-blocking RX poll: returns a queued frame if one is waiting, else `None`.
pub fn try_recv() -> Option<Frame> {
    RX.try_receive().ok()
}

/// Convert a received bxcan frame to our portable [`Frame`]; drop non-standard
/// and remote frames (returns `None`).
fn convert_rx(bx: &BxFrame) -> Option<Frame> {
    let id = match bx.id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return None,
    };
    let data = bx.data()?; // None == remote frame
    Frame::new(id, &data[..])
}

/// RX FIFO0 pending: drain every waiting frame into the RX channel and wake its
/// consumer. Bounded so a persistent overrun can't spin the ISR forever.
#[no_mangle]
pub extern "C" fn CAN_RX0() {
    critical_section::with(|cs| {
        let mut slot = CAN.borrow(cs).borrow_mut();
        if let Some(can) = slot.as_mut() {
            for _ in 0..8 {
                match can.receive() {
                    Ok(bx) => {
                        if let Some(f) = convert_rx(&bx) {
                            // Drop on backlog overflow rather than stall the ISR.
                            let _ = RX.try_send(f);
                        }
                    }
                    Err(nb::Error::WouldBlock) => break,
                    // Overrun: the read still released the mailbox; a frame was
                    // lost. Keep draining the rest.
                    Err(nb::Error::Other(_overrun)) => {}
                }
            }
        }
    });
}

/// TX mailbox completed: clear the request-complete flag, disarm the TX
/// interrupt (it only matters while a sender waits), and wake the sender.
#[no_mangle]
pub extern "C" fn CAN_TX() {
    critical_section::with(|cs| {
        let mut slot = CAN.borrow(cs).borrow_mut();
        if let Some(can) = slot.as_mut() {
            while can.clear_request_completed_flag().is_some() {}
            can.disable_interrupts(Interrupts::TRANSMIT_MAILBOX_EMPTY);
        }
    });
    TX_WAKER.wake();
}

/// Raw BTR for 500 kbps at PCLK1 = 4 MHz.
///
/// 4 MHz / (BRP=1) = 4 MHz tq clock; 4 MHz / 500 kHz = 8 tq/bit.
/// SYNC(1) + TSEG1(6) + TSEG2(1) = 8, sample point 87.5%.
/// bxCAN BTR stores value-1 per field: BRP=0, TS1=5, TS2=0, SJW=0
///   => 0<<24 | 0<<20 | 5<<16 | 0 = 0x0005_0000.
///
/// This is the timing the bootloader's `run_bootloader_service` runs (HSE 8 MHz,
/// PCLK1 = 4 MHz). If the app clocks PCLK1 differently, recompute.
pub const BTR_500K_PCLK4: u32 = 0x0005_0000;
