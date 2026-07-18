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
        /// Override the binary to package (default: `servo<board>`). Use e.g.
        /// `--bin commlab` to dist the Commutation Lab image instead of the app.
        #[arg(long)]
        bin: Option<String>,
        /// Extra comma-separated cargo features to add on top of
        /// `board-<board>,layout-app` (e.g. `hw-can` for commlab).
        #[arg(long)]
        features: Option<String>,
    },
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Dist { board, fw_version, debug, out, bin, features } => {
            dist(&board, fw_version, debug, &out, bin.as_deref(), features.as_deref())
        }
    }
}

fn dist(
    board: &str,
    fw_version: u32,
    debug: bool,
    out: &Path,
    bin: Option<&str>,
    extra_features: Option<&str>,
) -> Result<()> {
    let board_feat = match board {
        "57d" => "board-57d",
        "42d" => "board-42d",
        other => bail!("unknown board {other:?}; expected 57d or 42d"),
    };
    // The app bin name follows the board (src/bin/servo57d.rs etc) unless
    // overridden with --bin (e.g. --bin commlab).
    let bin_name = bin.map(String::from).unwrap_or_else(|| format!("servo{board}"));

    // Firmware crate root = two levels up from tools/xtask/.
    let fw_root = workspace_root()?;
    let profile = if debug { "dev" } else { "release" };
    let profile_dir = if debug { "debug" } else { "release" };

    // 1. Build the application for the chip, app layout.
    let feat = match extra_features {
        Some(extra) if !extra.is_empty() => format!("{board_feat},layout-app,{extra}"),
        _ => format!("{board_feat},layout-app"),
    };
    eprintln!("==> building {bin_name} ({feat}, {profile})");
    let status = Command::new("cargo")
        .current_dir(&fw_root)
        .args([
            "build",
            "--bin",
            &bin_name,
            "--no-default-features",
            "--features",
            &feat,
        ])
        .args(if debug { &[][..] } else { &["--release"][..] })
        .status()
        .context("failed to launch cargo build")?;
    if !status.success() {
        bail!("app build failed");
    }

    // 2. Locate the ELF and extract a raw flash image with objcopy. objcopy is
    //    the canonical, battle-tested ELF→bin tool: it walks PT_LOAD by *load*
    //    address (LMA), 0xFF-fills inter-segment gaps, and drops non-alloc
    //    sections — exactly the flash picture. A prior hand-rolled `object`-crate
    //    extractor got the LMA/offset math wrong and emitted the ELF header as
    //    "app.bin"; shelling out to objcopy removes that whole class of bug.
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

/// Extract a raw flash binary from an ELF via `objcopy -O binary`.
///
/// objcopy walks PT_LOAD by load address, 0xFF-fills the gaps between sections,
/// and emits exactly the bytes that belong in flash — the same image a debugger
/// programs. We do NOT hand-roll ELF parsing: a previous `object`-crate version
/// mis-computed segment offsets and emitted the ELF header itself as the "binary"
/// (a 24968-byte file starting with `\x7fELF` instead of the vector table),
/// which silently bricked every flash. objcopy is the canonical tool for this.
///
/// Picks the first available of `arm-none-eabi-objcopy` or `llvm-objcopy`.
fn elf_to_bin(path: &Path) -> Result<Vec<u8>> {
    let tool = objcopy_tool()?;

    // objcopy writes to a file, not stdout, so stage a temp path next to the ELF.
    let out_path = path.with_extension("objcopy.bin");
    let status = Command::new(&tool)
        .args(["-O", "binary"])
        .arg(path)
        .arg(&out_path)
        .status()
        .with_context(|| format!("failed to launch {tool}"))?;
    if !status.success() {
        bail!("{tool} -O binary failed on {}", path.display());
    }

    let bytes = std::fs::read(&out_path)
        .with_context(|| format!("reading objcopy output {}", out_path.display()))?;
    let _ = std::fs::remove_file(&out_path); // best-effort cleanup

    // Sanity-gate the result so a bad extraction can never slip through to flash
    // again: the first word is the initial SP (must point into RAM, 0x2000_xxxx)
    // and the second is the reset vector (must point into the app flash region,
    // odd for Thumb). This is exactly the check that would have caught the
    // ELF-as-bin bug immediately.
    if bytes.len() < 8 {
        bail!("objcopy image is only {} bytes — too small to be valid", bytes.len());
    }
    let sp = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    let reset = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
    if sp & 0xFFFF_0000 != 0x2000_0000 {
        bail!(
            "objcopy image bad vector table: initial SP {sp:#010x} is not in RAM \
             (0x2000_xxxx) — extraction likely produced non-image bytes"
        );
    }
    if reset & 0xFFF0_0000 != 0x0800_0000 || reset & 1 == 0 {
        bail!(
            "objcopy image bad vector table: reset vector {reset:#010x} is not an \
             odd (Thumb) address in flash (0x08xx_xxxx)"
        );
    }

    Ok(bytes)
}

/// Locate an objcopy on PATH: prefer the ARM GNU toolchain's, fall back to LLVM.
fn objcopy_tool() -> Result<String> {
    for tool in ["arm-none-eabi-objcopy", "llvm-objcopy"] {
        let found = Command::new(tool)
            .arg("--version")
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        if found {
            return Ok(tool.to_string());
        }
    }
    bail!(
        "no objcopy found on PATH (looked for arm-none-eabi-objcopy, llvm-objcopy). \
         Install the ARM GNU toolchain or LLVM tools."
    )
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
