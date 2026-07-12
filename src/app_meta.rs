//! APP-META: the firmware-image descriptor record.
//!
//! One 2 KB flash page at [`META_BASE`] (the last page of the 128 K device)
//! holds a single [`AppMeta`] record describing the application image that
//! lives in the app region (`APP_BASE .. APP_BASE + APP_REGION_LEN`).
//!
//! This is the **single source of truth** for the image format. Three parties
//! agree on it byte-for-byte:
//!   * `cargo xtask dist` — computes `length` + `crc32` over `app.bin` and
//!     emits an `AppMeta` into the image it ships.
//!   * the bootloader (`src/bin/bootloader.rs`) — reads the record, validates
//!     magic/version, then CRC32s the app region and compares.
//!   * the app — may read its own meta (e.g. to report version over CANopen).
//!
//! ## What the CRC covers
//! The CRC32 covers the **app region only, exactly `length` bytes**, starting
//! at `APP_BASE`. The meta page is NOT included. Because the meta record lives
//! in a separate flash page from the bytes it protects, there is no
//! zero-the-field-then-backpatch dance: the stored CRC sits entirely outside
//! the region it covers.
//!
//! `length` is the exact byte count of the application image, never the padded
//! region size. Erased flash reads as `0xFF`; CRCing the whole padded region
//! would make the result depend on erase state. CRC precisely `length` bytes.

#![allow(dead_code)]

use crc::{Crc, Algorithm};

/// Base of the application region (just past the 16 K bootloader).
pub const APP_BASE: u32 = 0x0800_4000;

/// Length of the usable application region: 108 K. (110 K between the
/// bootloader and the meta page, minus the 2 K meta page itself.)
pub const APP_REGION_LEN: u32 = 108 * 1024;

/// Base of the APP-META page: the last 2 K page of the 128 K device.
pub const META_BASE: u32 = 0x0801_F800;

/// Size of the APP-META page (one flash page).
pub const META_PAGE_LEN: u32 = 2 * 1024;

/// Magic identifying a populated [`AppMeta`] record. Spells a recognizable
/// tag in a hexdump; distinct from the all-`0xFF` of an erased page and the
/// all-`0x00` of a zeroed one.
pub const APP_META_MAGIC: u32 = 0x4E33_324D; // "N32M"

/// Format version of this record layout. Bump on any field change.
pub const APP_META_VERSION: u16 = 1;

/// The CRC-32 algorithm used everywhere in this project.
///
/// CRC-32/ISO-HDLC (a.k.a. the zlib/PNG/Ethernet CRC): poly `0x04C11DB7`
/// reflected, init `0xFFFFFFFF`, xorout `0xFFFFFFFF`. This is the same
/// algorithm `crc32fast` / zlib / Python's `binascii.crc32` produce, so the
/// xtask side can cross-check against a host library trivially.
pub const CRC_ALG: Algorithm<u32> = crc::CRC_32_ISO_HDLC;

/// Construct the project CRC engine. Call `.checksum(bytes)` for a one-shot,
/// or `.digest()` to feed it incrementally (the bootloader streams the app
/// region a chunk at a time rather than materializing it).
pub const fn crc_engine() -> Crc<u32> {
    Crc::<u32>::new(&CRC_ALG)
}

/// The project CRC engine as a `const` so a borrowed `.digest()` outlives the
/// engine. Lives in flash, zero runtime construction cost.
pub const CRC: Crc<u32> = crc_engine();

/// The firmware-image descriptor. `#[repr(C)]` with explicit padding so the
/// layout is identical between this Rust definition and the bytes xtask
/// serializes on the host. Keep fields naturally aligned; total size 32 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AppMeta {
    /// Must equal [`APP_META_MAGIC`] for the record to be considered present.
    pub magic: u32,
    /// CRC32 (see [`CRC_ALG`]) over `APP_BASE .. APP_BASE + length`.
    pub crc32: u32,
    /// Exact application image length in bytes (NOT the padded region).
    pub length: u32,
    /// Application firmware version (semantic meaning owned by the app).
    pub fw_version: u32,
    /// Layout version of THIS record ([`APP_META_VERSION`]).
    pub meta_version: u16,
    /// Reserved; keeps the record 4-byte aligned and leaves room to grow
    /// without a layout-version bump for small additions. Write as 0.
    pub _reserved: u16,
    /// Reserved tail to round the record to 32 bytes. Write as 0.
    pub _reserved2: [u32; 3],
}

impl AppMeta {
    pub const SIZE: usize = core::mem::size_of::<Self>();

    /// Read the record out of the meta page over a memory-mapped pointer.
    ///
    /// # Safety
    /// `META_BASE` must point at readable flash containing a record written
    /// with this same layout (or erased flash, in which case `magic` will be
    /// `0xFFFF_FFFF` and [`is_present`] returns false).
    pub unsafe fn read_from_flash() -> Self {
        core::ptr::read_volatile(META_BASE as *const AppMeta)
    }

    /// True if the record carries the expected magic and a layout version this
    /// bootloader understands.
    pub fn is_present(&self) -> bool {
        self.magic == APP_META_MAGIC && self.meta_version == APP_META_VERSION
    }

    /// Validate the record against the actual app-region bytes.
    ///
    /// Returns `true` iff the record is present, `length` is sane (fits the
    /// region), and the CRC32 over exactly `length` bytes at `APP_BASE`
    /// matches `crc32`. Streams the region through the digest in chunks so no
    /// large buffer is needed.
    ///
    /// # Safety
    /// `APP_BASE .. APP_BASE + length` must be readable flash.
    pub unsafe fn verify_app(&self) -> bool {
        if !self.is_present() {
            return false;
        }
        if self.length == 0 || self.length > APP_REGION_LEN {
            return false;
        }
        let mut digest = CRC.digest();
        let mut remaining = self.length as usize;
        let mut addr = APP_BASE as *const u8;
        // Feed the region to the digest in page-sized bites.
        let mut chunk = [0u8; 256];
        while remaining > 0 {
            let n = remaining.min(chunk.len());
            for i in 0..n {
                chunk[i] = core::ptr::read_volatile(addr.add(i));
            }
            digest.update(&chunk[..n]);
            addr = addr.add(n);
            remaining -= n;
        }
        digest.finalize() == self.crc32
    }

    /// Serialize to the raw 32-byte little-endian record. Used by tests and
    /// (conceptually) mirrored by the xtask host serializer.
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut out = [0u8; Self::SIZE];
        out[0..4].copy_from_slice(&self.magic.to_le_bytes());
        out[4..8].copy_from_slice(&self.crc32.to_le_bytes());
        out[8..12].copy_from_slice(&self.length.to_le_bytes());
        out[12..16].copy_from_slice(&self.fw_version.to_le_bytes());
        out[16..18].copy_from_slice(&self.meta_version.to_le_bytes());
        out[18..20].copy_from_slice(&self._reserved.to_le_bytes());
        // _reserved2 stays zero.
        out
    }
}

/// Compile-time guarantee that the record is exactly 32 bytes. If a field
/// change trips this, the xtask serializer (tools/xtask/src/main.rs) MUST be
/// updated in lockstep and `APP_META_VERSION` bumped.
const _: () = assert!(AppMeta::SIZE == 32);
