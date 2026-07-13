//! xtask — build-pipeline automation for servo4257-rs.
//!
//! `cargo xtask dist` builds the application firmware, extracts a raw binary
//! from the ELF, computes the CRC-32 + length, and emits both the raw app
//! image (`app.bin`) and the full flashable image with the APP-META page
//! appended (`app.img`).
//!
//! ## Byte-layout contract (MUST match src/app_meta.rs)
//! The `AppMeta` record is 32 bytes, little-endian, `#[repr(C)]`:
//!   offset 0  u32   magic         = 0x4E33_324D ("N32M")
//!   offset 4  u32   crc32         = CRC-32/ISO-HDLC over app.bin
//!   offset 8  u32   length        = app.bin length in bytes
//!   offset 12 u32   fw_version
//!   offset 16 u16   meta_version  = 1
//!   offset 18 u16   _reserved     = 0
//!   offset 20 [u32;3] _reserved2  = 0
//! The firmware side has a compile-time assert that size_of == 32; if you
//! change this, change both and bump meta_version.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use object::{Object, ObjectSegment};

// --- APP-META layout constants: keep in lockstep with src/app_meta.rs ---
const APP_META_MAGIC: u32 = 0x4E33_324D; // "N32M"
const APP_META_VERSION: u16 = 1;
const META_SIZE: usize = 32;
const APP_REGION_LEN: u32 = 108 * 1024; // must match app_meta::APP_REGION_LEN
const META_PAGE_LEN: usize = 2 * 1024; // one flash page

/// CRC-32/ISO-HDLC — identical to app_meta::CRC_ALG (crc::CRC_32_ISO_HDLC).
const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC);

#[derive(Parser)]
#[command(name = "xtask", about = "servo4257-rs build pipeline")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Build the app and emit app.bin + app.img (with APP-META).
    Dist {
        /// Board feature: 57d or 42d.
        #[arg(long, default_value = "57d")]
        board: String,
        /// Firmware version to stamp into APP-META.
        #[arg(long, default_value_t = 0)]
        fw_version: u32,
        /// Build the debug profile instead of release.
        #[arg(long)]
        debug: bool,
        /// Output directory for app.bin / app.img.
        #[arg(long, default_value = "dist")]
        out: PathBuf,
    },
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Dist { board, fw_version, debug, out } => dist(&board, fw_version, debug, &out),
    }
}

fn dist(board: &str, fw_version: u32, debug: bool, out: &Path) -> Result<()> {
    let board_feat = match board {
        "57d" => "board-57d",
        "42d" => "board-42d",
        other => bail!("unknown board {other:?}; expected 57d or 42d"),
    };
    // The app bin name follows the board (src/bin/servo57d.rs etc).
    let bin_name = format!("servo{board}");

    // Firmware crate root = two levels up from tools/xtask/.
    let fw_root = workspace_root()?;
    let profile = if debug { "dev" } else { "release" };
    let profile_dir = if debug { "debug" } else { "release" };

    // 1. Build the application for the chip, app layout.
    eprintln!("==> building {bin_name} ({board_feat}, layout-app, {profile})");
    let status = Command::new("cargo")
        .current_dir(&fw_root)
        .args([
            "build",
            "--bin",
            &bin_name,
            "--no-default-features",
            "--features",
            &format!("{board_feat},layout-app"),
        ])
        .args(if debug { &[][..] } else { &["--release"][..] })
        .status()
        .context("failed to launch cargo build")?;
    if !status.success() {
        bail!("app build failed");
    }

    // 2. Locate the ELF and extract a raw binary image from its loadable
    //    segments. Using the `object` crate (not objcopy) so we control the
    //    exact byte range and never inherit objcopy's section-gap padding.
    let elf_path = fw_root
        .join("target/thumbv7em-none-eabihf")
        .join(profile_dir)
        .join(&bin_name);
    let app_bin = elf_to_bin(&elf_path)
        .with_context(|| format!("extracting raw image from {}", elf_path.display()))?;

    let length = app_bin.len() as u32;
    if length == 0 {
        bail!("extracted app image is empty");
    }
    if length > APP_REGION_LEN {
        bail!(
            "app image {length} bytes exceeds app region {APP_REGION_LEN} bytes",
        );
    }

    // 3. CRC-32 over exactly the app bytes (no padding). Matches the
    //    bootloader's verify_app(), which CRCs `length` bytes at APP_BASE.
    let crc32 = CRC.checksum(&app_bin);

    // 4. Serialize APP-META (little-endian, 32 bytes).
    let meta = serialize_meta(crc32, length, fw_version);

    // 5. Emit outputs.
    std::fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;
    let bin_out = out.join("app.bin");
    let img_out = out.join("app.img");
    std::fs::write(&bin_out, &app_bin)?;

    // app.img = the full flash picture of the app region + meta page:
    //   [app.bin][0xFF padding up to META page][32-byte meta][0xFF pad to page]
    // This is the byte image a flasher could write to 0x08004000 directly, and
    // also what the CAN download path streams (the master can slice the meta
    // out of the tail, or send region + meta separately).
    let mut img = Vec::with_capacity(APP_REGION_LEN as usize + META_PAGE_LEN);
    img.extend_from_slice(&app_bin);
    img.resize(APP_REGION_LEN as usize, 0xFF); // pad app region to its full extent
    let mut meta_page = vec![0xFFu8; META_PAGE_LEN];
    meta_page[..META_SIZE].copy_from_slice(&meta);
    img.extend_from_slice(&meta_page);
    std::fs::write(&img_out, &img)?;

    eprintln!("==> app.bin   {length} bytes");
    eprintln!("==> crc32     {crc32:#010x}  (CRC-32/ISO-HDLC)");
    eprintln!("==> fw_version {fw_version}");
    eprintln!("==> app.img   {} bytes ({} app region + {} meta page)",
        img.len(), APP_REGION_LEN, META_PAGE_LEN);
    eprintln!("    {}", bin_out.display());
    eprintln!("    {}", img_out.display());
    Ok(())
}

/// Serialize the 32-byte APP-META record, little-endian, matching the
/// firmware `#[repr(C)] AppMeta`.
fn serialize_meta(crc32: u32, length: u32, fw_version: u32) -> [u8; META_SIZE] {
    let mut m = [0u8; META_SIZE];
    m[0..4].copy_from_slice(&APP_META_MAGIC.to_le_bytes());
    m[4..8].copy_from_slice(&crc32.to_le_bytes());
    m[8..12].copy_from_slice(&length.to_le_bytes());
    m[12..16].copy_from_slice(&fw_version.to_le_bytes());
    m[16..18].copy_from_slice(&APP_META_VERSION.to_le_bytes());
    // bytes 18..32 stay zero (_reserved, _reserved2).
    m
}

/// Flatten an ELF's loadable segments into a contiguous raw binary, based at
/// the lowest loadable virtual address. Gaps between segments are filled with
/// 0xFF (erased-flash value) so the image matches what sits in flash.
fn elf_to_bin(path: &Path) -> Result<Vec<u8>> {
    let data = std::fs::read(path)?;
    let file = object::File::parse(&*data)?;

    let mut segs: Vec<(u64, &[u8])> = Vec::new();
    for seg in file.segments() {
        // Only PT_LOAD segments carry flashable bytes; object yields those.
        let bytes = seg.data()?;
        if bytes.is_empty() {
            continue;
        }
        segs.push((seg.address(), bytes));
    }
    if segs.is_empty() {
        bail!("no loadable segments in ELF");
    }
    segs.sort_by_key(|(addr, _)| *addr);

    let base = segs.first().unwrap().0;
    let end = segs
        .iter()
        .map(|(addr, b)| addr + b.len() as u64)
        .max()
        .unwrap();
    let mut out = vec![0xFFu8; (end - base) as usize];
    for (addr, bytes) in segs {
        let off = (addr - base) as usize;
        out[off..off + bytes.len()].copy_from_slice(bytes);
    }
    Ok(out)
}

/// servo4257-rs crate root, resolved relative to this xtask package.
fn workspace_root() -> Result<PathBuf> {
    // CARGO_MANIFEST_DIR for xtask = .../servo4257-rs/tools/xtask
    let here = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let root = here
        .parent()
        .and_then(|p| p.parent())
        .context("resolving servo4257-rs root from tools/xtask")?
        .to_path_buf();
    Ok(root)
}
