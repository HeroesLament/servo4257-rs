#![no_std]
#![no_main]
//! Characterization instrument — turns the board into a CAN-controlled bench
//! tool. It idles with the output stage off and takes COMMANDS on COB-ID 0x610,
//! streaming MEASUREMENTS (encoder + both coil currents) on 0x181 at ~100 Hz so
//! the host can compute statistics and fit models. No commutation logic — every
//! probe drives the hardware directly and reads it back, so experiments are
//! deterministic, parameterized, and repeatable without reflashing.
//!
//! Commands (frame to 0x610, byte 0 = opcode):
//!   0x00 STOP                       — duties 0, output disabled
//!   0x01 ENABLE  [1]=on(0/1)        — output stage enable
//!   0x02 VECTOR  [1..3]=angle u16, [3..5]=amp i16  — apply sinusoidal vector
//!   0x03 RAW     [1..5]=ap,am,bp,bm u8 (0..255 → 0..max_duty) — raw half-bridges
//!
//! Telemetry (0x181): [0xE1, mode, enc:u16, iA:u16, iB:u16] little-endian.
//! Also answers the 0x1F51 enter-update for over-CAN reflash.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

const CMD_ID: u16 = 0x610;
const TELEM_ID: u16 = 0x181;
const SYSCLK_HZ: u32 = 64_000_000;
const M: *mut u32 = 0x2000_4000 as *mut u32;

#[derive(Clone, Copy)]
enum Mode {
    Idle,
    Vector { angle: u16, amp: i16 },
    Raw { ap: u16, am: u16, bp: u16, bm: u16 },
}

#[inline]
fn w(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(M.add(i), v) };
}

#[inline]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay((SYSCLK_HZ / 1000) * ms);
}

#[inline]
fn vector(a: u16, amp: i16) -> (i16, i16) {
    let (s, c) = sin_cos(a);
    let amp = amp as f32;
    ((amp * c) as i16, (amp * s) as i16)
}

#[entry]
fn main() -> ! {
    w(0, 0x1257_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    let maxd = board.max_duty() as u32;
    let scale = |x: u8| ((x as u32 * maxd) / 255) as u16;

    let mut mode = Mode::Idle;
    let mut enabled = false;
    let mut tag: u8 = 0; // opcode of last command, echoed in telemetry

    board.set_output_enable(false);

    loop {
        // ---- receive & dispatch (also handles enter-update reflash) ----
        if let Some((id, d, len)) = board.can_recv() {
            if id == 0x601 && len >= 3 && d[1] == 0x51 && d[2] == 0x1F {
                board.reboot_to_bootloader();
            } else if id == CMD_ID && len >= 1 {
                match d[0] {
                    0x00 => {
                        mode = Mode::Idle;
                        enabled = false;
                    }
                    0x01 => enabled = d[1] != 0,
                    0x02 => {
                        mode = Mode::Vector {
                            angle: u16::from_le_bytes([d[1], d[2]]),
                            amp: i16::from_le_bytes([d[3], d[4]]),
                        }
                    }
                    0x03 => {
                        mode = Mode::Raw {
                            ap: scale(d[1]),
                            am: scale(d[2]),
                            bp: scale(d[3]),
                            bm: scale(d[4]),
                        }
                    }
                    _ => {}
                }
                tag = d[0];
            }
        }

        // ---- apply the commanded drive ----
        board.set_output_enable(enabled);
        match mode {
            Mode::Idle => board.apply_coil_voltages(0, 0),
            Mode::Vector { angle, amp } => {
                let (va, vb) = vector(angle, amp);
                board.apply_coil_voltages(va, vb);
            }
            Mode::Raw { ap, am, bp, bm } => board.set_raw_duties(ap, am, bp, bm),
        }

        // ---- sample & stream ----
        let enc = board.rotor_angle();
        let (ia, ib) = board.read_coil_currents();
        w(2, enc as u32);
        w(3, ia as u32 & 0xFFFF);
        w(4, ib as u32 & 0xFFFF);
        let e = enc.to_le_bytes();
        let ca = (ia as u16).to_le_bytes();
        let cb = (ib as u16).to_le_bytes();
        board.telemetry(TELEM_ID, &[0xE1, tag, e[0], e[1], ca[0], ca[1], cb[0], cb[1]]);

        delay_ms(10); // ~100 Hz
    }
}
