//! Bootloader CAN download — the firmware-side glue.
//!
//! The download *control loop* (poll → feed the SDO server → act → report →
//! send, including the fiddly last-segment-ack-plus-finalize path) lives in
//! `canopen_proto::sdo::driver::run`, where it is host-tested against a
//! loopback transport and a Vec-backed sink. This module supplies only the two
//! hardware-bound pieces the loop is generic over:
//!
//!   * a [`FlashSink`] backed by the silicon-proven `Flash` driver, and
//!   * the entry point that constructs the transport + sink and runs the loop.
//!
//! Keeping the loop in the tested crate means the untestable part here is just
//! three flash calls with an offset translation — not a state machine.

#![allow(dead_code)]

use canopen_proto::sdo::driver::{self, FlashSink, SdoCobIds};
pub use canopen_proto::sdo::driver::DownloadOutcome;
use canopen_proto::transport::CanTransport;
use embedded_storage::nor_flash::NorFlash;
use n32l4xx_hal::fmc::Flash;

use crate::app_meta::{
    AppMeta, APP_BASE, APP_META_MAGIC, APP_META_VERSION, APP_REGION_LEN, CRC, META_BASE,
};

/// Flash-base-relative offset of the app region. `Flash::write`/`erase` take
/// offsets relative to 0x08000000, but the SDO stream (and the driver's
/// `FlashSink` offsets) are image-relative (0 = first app byte).
const APP_FLASH_OFFSET: u32 = APP_BASE - 0x0800_0000; // 0x4000
/// Flash page size (all N32L406 pages are 2 KB).
const PAGE_SIZE: u32 = 2048;
const META_FLASH_OFFSET: u32 = META_BASE - 0x0800_0000; // 0x1F800

/// Node id for the bootloader (fixed for V1). SDO COB-IDs derive from it.
pub const NODE_ID: u16 = 0x01;

/// A [`FlashSink`] over the real N32 flash. Translates image-relative offsets
/// to flash-base-relative and performs erase/program/meta writes.
pub struct FlashBackend<'a> {
    flash: &'a mut Flash,
    /// Highest app-region page index (0-based from APP_FLASH_OFFSET) erased
    /// so far, or None. Segments arrive with monotonically increasing offset,
    /// so each page is erased lazily just before its first write.
    erased_through: Option<u32>,
}

impl<'a> FlashBackend<'a> {
    pub fn new(flash: &'a mut Flash) -> Self {
        FlashBackend { flash, erased_through: None }
    }
}

impl FlashSink for FlashBackend<'_> {
    fn erase(&mut self) -> Result<(), ()> {
        // Lazy erase: erasing the whole 108 KB region here blocks the initiate
        // response for >1 s and stalls the CPU across the entire multi-page
        // operation. Instead, arm per-page erase (done in `write`) and return
        // immediately so the node can ACK the initiate promptly.
        self.erased_through = None;
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), ()> {
        // Erase (in order) any pages this write touches that aren't yet erased.
        if !bytes.is_empty() {
            let last_page = (offset + bytes.len() as u32 - 1) / PAGE_SIZE;
            let mut page = match self.erased_through {
                Some(p) => p + 1,
                None => 0,
            };
            while page <= last_page {
                let page_off = APP_FLASH_OFFSET + page * PAGE_SIZE;
                self.flash.erase(page_off, page_off + PAGE_SIZE).map_err(|_| ())?;
                self.erased_through = Some(page);
                page += 1;
            }
        }
        let flash_off = APP_FLASH_OFFSET + offset;
        self.flash.write(flash_off, bytes).map_err(|_| ())
    }

    fn finalize(&mut self, total_len: u32) -> Result<(), ()> {
        // Compute the CRC over exactly `total_len` bytes read back from flash
        // (not the streamed bytes, which include the stager's trailing zero
        // pad). This is byte-identical to what the boot-time verify does.
        let meta = AppMeta {
            magic: APP_META_MAGIC,
            crc32: crc_app_region(total_len),
            length: total_len,
            fw_version: 0,
            meta_version: APP_META_VERSION,
            _reserved: 0,
            _reserved2: [0; 3],
        };
        let bytes = meta.to_bytes();
        // The app-region erase does NOT cover the meta page (separate page), so
        // erase it explicitly before stamping the record.
        self.flash
            .erase(META_FLASH_OFFSET, META_FLASH_OFFSET + AppMeta::SIZE as u32)
            .map_err(|_| ())?;
        self.flash.write(META_FLASH_OFFSET, &bytes).map_err(|_| ())
    }
}

/// CRC-32 over `len` bytes of the app region, read back from flash. Uses the
/// project CRC engine (`app_meta::CRC`), streamed in chunks like
/// `AppMeta::verify_app`, so the value matches xtask and the boot-time verify.
fn crc_app_region(len: u32) -> u32 {
    let mut digest = CRC.digest();
    let mut remaining = len as usize;
    let mut addr = APP_BASE as *const u8;
    let mut chunk = [0u8; 256];
    while remaining > 0 {
        let n = remaining.min(chunk.len());
        for (i, c) in chunk.iter_mut().enumerate().take(n) {
            *c = unsafe { core::ptr::read_volatile(addr.add(i)) };
        }
        digest.update(&chunk[..n]);
        addr = unsafe { addr.add(n) };
        remaining -= n;
    }
    digest.finalize()
}

/// Run the CANopen firmware download to completion or abort, polling `can`.
pub fn run_download<T, S>(can: &mut T, flash: &mut Flash, status: S) -> DownloadOutcome
where
    T: CanTransport,
    S: FnMut(u32),
{
    let mut sink = FlashBackend::new(flash);
    driver::run(can, &mut sink, SdoCobIds::for_node(NODE_ID), status)
}
