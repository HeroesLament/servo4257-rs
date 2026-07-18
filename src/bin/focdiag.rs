#![no_std]
#![no_main]
//! focdiag — the current-sense truth test AND the scaffold the current loop
//! drops into. Runs the KNOWN-GOOD sensored spin (align → commutate a voltage
//! vector 90° ahead of the reversed-encoder electrical angle), and every
//! telemetry tick streams the raw injected ADC current slots PLUS the
//! Park-transformed (id, iq). If current sense works, a spinning rotor yields
//! sinusoidally varying iA/iB (a rotating current vector) and roughly constant
//! (id, iq); if a channel is dead or mis-slotted, it shows immediately.
//!
//! Telemetry (0x183, ~200-tick cadence):
//!   phase 0xD0: [enc:u16, theta_e:u16, iA:i16]     raw angle + coil A current
//!   phase 0xD1: [iB:i16, id:i16, iq:i16, jA?:u8..]  coil B + dq + slot marker
//! Two frames per report so we fit the 8-byte CAN payload. Answers 0x1F51
//! enter-update for over-CAN reflash.

use cortex_m_rt::entry;
use n32l4 as _;
use n32l4xx_hal::pac;
use panic_halt as _;
use servo4257_rs::board::Board;
use servo4257_rs::boards::ActiveBoard;
use servo4257_rs::motion::encoder::{offset_from_alignment, POLE_PAIRS};
use servo4257_rs::motion::foc::{clarke, park};
use servo4257_rs::motion::trig::sin_cos;

/// Applied vector amplitude (voltage-mode). 12000 developed real torque on the
/// bench; 3000 was too weak to break static friction.
const AMP: i16 = 12000;
/// True quadrature in this angle convention (see foc_spin): 32768, not 16384.
const LEAD: u16 = 32768;

const TELEM_ID: u16 = 0x183;
const SYSCLK_HZ: u32 = 64_000_000;

/// Rotor electrical angle with the encoder direction REVERSED (MT6816 counts
/// opposite to electrical rotation): `offset − enc·pole_pairs`.
#[inline]
fn theta_e_rev(enc: u16, offset: u16) -> u16 {
    offset.wrapping_sub(enc.wrapping_mul(POLE_PAIRS))
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

/// Center-subtracted coil current: the GS8632 amp idles at ~vREF/2 (~2043 raw)
/// and swings around it, so real signed current = raw − bias.
#[inline]
fn signed(raw: i16, bias: i16) -> i16 {
    raw.wrapping_sub(bias)
}

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };
    let mut board = ActiveBoard::init(dp);

    // FANUC-style: own the current sample instant. Switch the injected group to
    // software trigger so `sample_currents_now()` fires at a fixed loop phase,
    // decoupled from the CC4/coil-B duty edge (the cause of "rotating field,
    // static sampled current").
    board.use_sw_current_trigger();

    // ---- Align: field at electrical zero, rotor settles, sample enc → offset ----
    let (va, vb) = vector(0, AMP);
    board.apply_coil_voltages(va, vb);
    board.set_output_enable(true);
    delay_ms(800);
    let enc0 = board.rotor_angle();
    let offset = offset_from_alignment(enc0);

    // Sample the current-sense bias (should be ~2043 on both) while still held at
    // the align vector — used to center the readings into signed current.
    let (bias_a, bias_b) = board.sample_currents_now();

    let _ = offset; // (sensored path retained above for reference)

    // OPEN-LOOP FORCED ROTATION: advance the APPLIED field angle at a fixed rate,
    // independent of the rotor. This is the definitive current-sense test — a
    // rotating applied voltage vector must produce a rotating CURRENT vector
    // (iA,iB sinusoidal, 90° apart) if the low-side-shunt sense works, whether or
    // not the rotor stays synchronized. Slow rate so any following rotor tracks.
    let mut field: u16 = 0;
    const FIELD_RATE: u16 = 8; // electrical-angle units per ~50µs tick

    let mut tick: u32 = 0;
    let mut phase_toggle = false;
    loop {
        board.poll_reflash();

        let enc = board.rotor_angle();
        let theta_e = field; // report the COMMANDED field angle for the Park math
        let (va, vb) = vector(field, AMP);
        board.apply_coil_voltages(va, vb);
        field = field.wrapping_add(FIELD_RATE);

        if tick % 200 == 0 {
            // Software-fired injected sample at a fixed loop phase.
            let (ra, rb) = board.sample_currents_now();
            let ia = signed(ra, bias_a);
            let ib = signed(rb, bias_b);

            // Park: measured coil currents → (id, iq) at this electrical angle.
            // id≈0, iq≈const while spinning ⇒ sense + commutation are coherent.
            let (s, c) = sin_cos(theta_e);
            let (alpha, beta) = clarke(ia as f32, ib as f32);
            let idf = alpha * c + beta * s;
            let iqf = -alpha * s + beta * c;
            let id = idf as i16;
            let iq = iqf as i16;

            // Alternate the two report frames on successive ticks: sending both
            // back-to-back drops the second (TX mailbox still busy). Toggling
            // guarantees each gets a clear mailbox.
            if !phase_toggle {
                let e = enc.to_le_bytes();
                let t = theta_e.to_le_bytes();
                let ea = ia.to_le_bytes();
                board.telemetry(TELEM_ID, &[0xD0, e[0], e[1], t[0], t[1], ea[0], ea[1], 0]);
            } else {
                let eb = ib.to_le_bytes();
                let di = id.to_le_bytes();
                let qi = iq.to_le_bytes();
                board.telemetry(TELEM_ID, &[0xD1, eb[0], eb[1], di[0], di[1], qi[0], qi[1], 0]);
            }
            phase_toggle = !phase_toggle;
        }
        tick = tick.wrapping_add(1);

        cortex_m::asm::delay(3200); // ~50 µs → ~20 kHz commutation
    }
}
