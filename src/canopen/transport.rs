//! bxCAN hardware transport for the CANopen layer.
//!
//! The portable protocol types ([`Frame`], [`CanTransport`], [`TransportError`])
//! and the host loopback mock live in the PAC-free `canopen-proto` crate, which
//! is host-testable. This module supplies only the hardware-bound piece: a
//! `CanTransport` implementation backed by bxcan-ng on the N32L40x.
//!
//! The bootloader uses **polled** RX (no interrupts, no queue): it has no
//! concurrent work and the SDO protocol is acknowledged, so the hardware RX
//! FIFO plus a poll loop is all the buffering needed.

#![allow(dead_code)]

pub use canopen_proto::transport::{
    recv_blocking, CanTransport, Frame, LoopbackTransport, TransportError,
};

#[cfg(feature = "hw-can")]
mod hw {
    use super::*;
    use bxcan::{Can, Frame as BxFrame, Id, StandardId};

    /// Raw BTR for 500 kbps at PCLK1 = 4 MHz (HSE 8 MHz crystal / 2 in run_bootloader_service; 4 MHz keeps the flash erase happy).
    ///
    /// 4 MHz / (BRP=1) = 4 MHz tq clock; 4 MHz / 500 kHz = 8 tq/bit.
    /// SYNC(1) + TSEG1(6) + TSEG2(1) = 8, sample point (1+6)/8 = 87.5%.
    /// bxCAN BTR fields store value-1: BRP=0, TS1=5, TS2=0, SJW=0.
    ///   BTR = (SJW-1)<<24 | (TS2-1)<<20 | (TS1-1)<<16 | (BRP-1)
    ///       = 0<<24 | 0<<20 | 5<<16 | 0  = 0x0005_0000
    ///
    /// NOTE: derived for 8 MHz PCLK1 (HSE). If the clock setup changes,
    /// recompute this constant — wrong BTR is the #1 "powers up but never
    /// ACKs" failure.
    pub const BTR_500K_8MHZ: u32 = 0x0005_0000;

    /// bxcan-backed transport. `I` is the HAL's `Can<pac::Can>` instance type
    /// (which impls `bxcan::Instance`).
    pub struct BxCanTransport<I: bxcan::Instance> {
        can: Can<I>,
    }

    impl<I: bxcan::Instance + bxcan::FilterOwner> BxCanTransport<I> {
        /// Bring up the peripheral at 500 kbps, normal mode, accept-all filter.
        /// Pins must already be assigned to the CAN AF (PA11/PA12) by the HAL.
        pub fn new(instance: I) -> Self {
            let mut can = Can::builder(instance)
                .set_bit_timing(BTR_500K_8MHZ)
                .leave_disabled();

            // Accept every standard frame; CANopen COB-ID filtering happens in
            // software above this layer (the bootloader wants all traffic).
            can.modify_filters()
                .enable_bank(0, bxcan::Fifo::Fifo0, bxcan::filter::Mask32::accept_all());

            // Enter normal mode; spin until synced to the bus.
            nb::block!(can.enable_non_blocking()).ok();

            BxCanTransport { can }
        }
    }

    impl<I: bxcan::Instance> CanTransport for BxCanTransport<I> {
        fn send(&mut self, frame: &Frame) -> Result<(), TransportError> {
            let id = StandardId::new(frame.id).ok_or(TransportError::NotReady)?;
            // payload() is always <=8 bytes (Frame's invariant), so Data::new
            // never returns None here.
            let data = bxcan::Data::new(frame.payload()).unwrap_or_else(bxcan::Data::empty);
            let bx = BxFrame::new_data(id, data);
            match self.can.transmit(&bx) {
                Ok(_status) => Ok(()),
                Err(nb::Error::WouldBlock) => Err(TransportError::TxBusy),
                Err(nb::Error::Other(infallible)) => match infallible {},
            }
        }

        fn try_recv(&mut self) -> Result<Option<Frame>, TransportError> {
            match self.can.receive() {
                Ok(bx) => Ok(convert_rx(&bx)),
                Err(nb::Error::WouldBlock) => Ok(None),
                Err(nb::Error::Other(_overrun)) => Err(TransportError::Overrun),
            }
        }
    }

    /// Convert a received bxcan frame to our [`Frame`]. Drops non-standard and
    /// remote frames (returns None) — the bootloader protocol uses neither.
    fn convert_rx(bx: &BxFrame) -> Option<Frame> {
        let id = match bx.id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(_) => return None,
        };
        let data = bx.data()?; // None == remote frame: ignore
        Frame::new(id, &data[..])
    }
}

#[cfg(feature = "hw-can")]
pub use hw::{BxCanTransport, BTR_500K_8MHZ};
