#![no_std]
#![no_main]
//! adcdump — measure the ACTUAL per-coil currents on demand (software-triggered
//! regular ADC reads, bypassing the broken CC4 injected trigger), while driving
//! raw half-bridge duties. Lets the host drive each coil (or a full vector, by
//! computing raw duties) and read back iA (ch3=PA2) and iB (ch2=PA1) directly —
//! to see whether a commanded rotating vector produces a rotating CURRENT vector
//! (circle) or a degenerate one (line), and whether the two coils are symmetric.
//!
//! Commands (0x610, byte0=op):
//!   0x00 STOP               output off
//!   0x01 ENABLE [1]         output stage enable
//!   0x03 RAW [ap,am,bp,bm]  raw half-bridge duties (×max_duty/255)
//! Telemetry 0x182: [iA:u16, iB:u16, encLo, encHi, 0, 0] LE — iA/iB are 12-bit
//! ADC (centered ~2043, ~165 counts/A), enc is the rotor angle.
//! Answers 0x1F51 enter-update for over-CAN reflash.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;

const CMD_ID: u16 = 0x610;
const TELEM_ID: u16 = 0x182;
const SYSCLK_HZ: u32 = 64_000_000;

#[inline]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay((SYSCLK_HZ / 1000) * ms);
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    let maxd = board.max_duty() as u32;
    let scale = |x: u8| ((x as u32 * maxd) / 255) as u16;

    let mut enabled = false;
    let mut raw: (u16, u16, u16, u16) = (0, 0, 0, 0);

    board.set_output_enable(false);

    loop {
        if let Some((id, d, len)) = board.can_recv() {
            if id == 0x601 && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                board.set_output_enable(false);
                board.reboot_to_bootloader();
            } else if id == CMD_ID && len >= 1 {
                match d[0] {
                    0x00 => {
                        enabled = false;
                        raw = (0, 0, 0, 0);
                    }
                    0x01 => enabled = d[1] != 0,
                    0x03 => raw = (scale(d[1]), scale(d[2]), scale(d[3]), scale(d[4])),
                    _ => {}
                }
            }
        }

        board.set_output_enable(enabled);
        board.set_raw_duties(raw.0, raw.1, raw.2, raw.3);

        // on-demand regular reads: ch3 = coil A, ch2 = coil B
        let ia = board.read_channel_now(3);
        let ib = board.read_channel_now(2);
        let enc = board.rotor_angle();
        let a = ia.to_le_bytes();
        let b = ib.to_le_bytes();
        let e = enc.to_le_bytes();
        board.telemetry(TELEM_ID, &[a[0], a[1], b[0], b[1], e[0], e[1], 0, 0]);

        delay_ms(5);
    }
}
