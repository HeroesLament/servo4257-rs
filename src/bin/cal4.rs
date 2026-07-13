#![no_std]
#![no_main]
//! Electrical-angle calibration by static holds. Parks the applied field at
//! 0°, 90°, 180°, 270° electrical (each a distinct rotor detent, one full step
//! apart), holding each long enough to settle, and telemeters the encoder. The
//! four (field, enc) points give the encoder→electrical DIRECTION (does enc rise
//! or fall as the field advances) and the OFFSET, and confirm ~328 enc counts
//! per 90° electrical (POLE_PAIRS = 50). Uses static holds, which the rotor
//! reliably snaps to. OTA app: telemetry 0x181 phase 0xC4 [quadrant:u8, enc:u16].

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::trig::sin_cos;

const AMP: i16 = 6000;
const HOLD_MS: u32 = 1200;
const FIELDS: [u16; 4] = [0, 16384, 32768, 49152]; // 0°, 90°, 180°, 270°

const TELEM_ID: u16 = 0x181;
const SYSCLK_HZ: u32 = 64_000_000;
const M: *mut u32 = 0x2000_4000 as *mut u32;

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
    w(0, 0x0C40_0001);

    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);
    board.set_output_enable(true);

    loop {
        for (q, &field) in FIELDS.iter().enumerate() {
            let (va, vb) = vector(field, AMP);
            board.apply_coil_voltages(va, vb);
            // hold and settle, telemetering the encoder the whole time
            for _ in 0..(HOLD_MS / 30) {
                board.poll_reflash();
                delay_ms(30);
                let enc = board.rotor_angle();
                w(2, enc as u32);
                w(3, q as u32);
                let e = enc.to_le_bytes();
                board.telemetry(TELEM_ID, &[0xC4, q as u8, e[0], e[1], 0, 0, 0, 0]);
            }
        }
    }
}
