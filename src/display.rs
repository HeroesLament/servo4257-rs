//! display — a tiny SSD1306 (128×64, I2C) driver + monochrome framebuffer.
//!
//! Write-only, blocking, no_std, no alloc. Just enough to init the panel, push a
//! full 1 KiB framebuffer, and draw filled circles / rectangles — for the
//! commutation-lab bouncing ball and (later) live telemetry glyphs.
//!
//! Panel: 128×64, page-addressed. The framebuffer is 8 pages of 128 bytes; bit
//! `y & 7` of byte `[page*128 + x]` is pixel (x, y), page = y/8. Flushing sends
//! the whole buffer after setting the column/page address range to full.

/// Minimal blocking I2C write the driver needs. Implemented for the HAL's
/// `I2c` (which has an inherent `write(addr, &[u8])`) in the binary that owns
/// the bus, so `display` stays independent of any specific HAL/eh version.
pub trait I2cWrite {
    type Error;
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error>;
}

pub const W: usize = 128;
pub const H: usize = 64;
const PAGES: usize = H / 8;
const FB_LEN: usize = W * PAGES; // 1024

/// SSD1306 default 7-bit I2C address (SA0 low).
pub const ADDR: u8 = 0x3C;

// Co-byte prefixes: 0x00 = command stream, 0x40 = data stream.
const CMD: u8 = 0x00;
const DATA: u8 = 0x40;

/// The panel init sequence (charge-pump on, 128×64, horizontal addressing).
const INIT: &[u8] = &[
    0xAE, // display off
    0xD5, 0x80, // clock div
    0xA8, 0x3F, // multiplex = 63 (64 rows)
    0xD3, 0x00, // display offset 0
    0x40, // start line 0
    0x8D, 0x14, // charge pump on
    0x20, 0x00, // memory mode = horizontal
    0xA1, // segment remap (col 127 → SEG0)
    0xC8, // COM scan direction remapped
    0xDA, 0x12, // COM pins config
    0x81, 0x7F, // contrast
    0xD9, 0xF1, // pre-charge
    0xDB, 0x40, // VCOMH deselect
    0xA4, // resume to RAM content
    0xA6, // normal (non-inverted)
    0xAF, // display on
];

/// A 128×64 monochrome framebuffer.
pub struct FrameBuf {
    buf: [u8; FB_LEN],
}

impl FrameBuf {
    pub const fn new() -> Self {
        Self { buf: [0; FB_LEN] }
    }

    #[inline]
    pub fn clear(&mut self) {
        self.buf = [0; FB_LEN];
    }

    /// Set/clear one pixel (bounds-checked; out-of-range is a no-op).
    #[inline]
    pub fn set(&mut self, x: i32, y: i32, on: bool) {
        if x < 0 || y < 0 || x as usize >= W || y as usize >= H {
            return;
        }
        let (x, y) = (x as usize, y as usize);
        let idx = (y / 8) * W + x;
        let bit = 1u8 << (y & 7);
        if on {
            self.buf[idx] |= bit;
        } else {
            self.buf[idx] &= !bit;
        }
    }

    /// Filled circle centered at (cx, cy), radius r.
    pub fn fill_circle(&mut self, cx: i32, cy: i32, r: i32, on: bool) {
        let r2 = r * r;
        for dy in -r..=r {
            for dx in -r..=r {
                if dx * dx + dy * dy <= r2 {
                    self.set(cx + dx, cy + dy, on);
                }
            }
        }
    }

    /// One-pixel border rectangle.
    pub fn rect(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, on: bool) {
        for x in x0..=x1 {
            self.set(x, y0, on);
            self.set(x, y1, on);
        }
        for y in y0..=y1 {
            self.set(x0, y, on);
            self.set(x1, y, on);
        }
    }

    /// Filled horizontal bar from x0..x1 at rows y0..=y1 (for telemetry gauges).
    pub fn fill_rect(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, on: bool) {
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.set(x, y, on);
            }
        }
    }
}

/// SSD1306 over any blocking-`Write` I2C.
pub struct Ssd1306<I> {
    i2c: I,
    addr: u8,
}

impl<I: I2cWrite> Ssd1306<I> {
    /// Wrap an I2C bus and run the panel init sequence.
    pub fn new(i2c: I) -> Result<Self, I::Error> {
        let mut d = Self { i2c, addr: ADDR };
        d.cmds(INIT)?;
        Ok(d)
    }

    /// Send a command stream (each byte a command; co-byte 0x00 prefix).
    fn cmds(&mut self, cmds: &[u8]) -> Result<(), I::Error> {
        // Stage into a small buffer: [0x00, c0, c1, ...]. INIT is < 32 bytes.
        let mut frame = [0u8; 64];
        frame[0] = CMD;
        let n = cmds.len().min(63);
        frame[1..=n].copy_from_slice(&cmds[..n]);
        self.i2c.write(self.addr, &frame[..=n])
    }

    /// Push the whole framebuffer to the panel.
    pub fn flush(&mut self, fb: &FrameBuf) -> Result<(), I::Error> {
        // Address the full 128×64 window: column 0..127, page 0..7.
        self.i2c.write(self.addr, &[CMD, 0x21, 0, (W as u8) - 1])?;
        self.i2c.write(self.addr, &[CMD, 0x22, 0, (PAGES as u8) - 1])?;

        // Stream the framebuffer in data chunks (co-byte 0x40 each chunk).
        // 128-byte chunks keep each I2C transfer modest.
        let mut chunk = [0u8; 1 + W];
        chunk[0] = DATA;
        for page in fb.buf.chunks(W) {
            chunk[1..1 + page.len()].copy_from_slice(page);
            self.i2c.write(self.addr, &chunk[..1 + page.len()])?;
        }
        Ok(())
    }
}
