//! motion/encoder.rs — rotor electrical-angle mapping from the raw encoder.
//!
//! The encoder reports the MECHANICAL angle as a `u16` spanning one revolution
//! (`0..=u16::MAX`). The FOC/commutation math needs the ELECTRICAL angle, which
//! cycles `POLE_PAIRS` times per mechanical revolution. Because both are `u16`
//! wrapping the full circle, the conversion is a wrapping multiply — the natural
//! overflow gives exactly `POLE_PAIRS` electrical cycles per mechanical turn.
//!
//! An `offset` accounts for the arbitrary alignment between the encoder's zero
//! and the rotor's electrical zero; it is measured once at startup by driving
//! the rotor to a known electrical angle and reading the encoder there.

/// Electrical cycles per mechanical revolution. A 1.8°/step hybrid stepper has
/// 200 full steps/rev = 50 electrical cycles.
pub const POLE_PAIRS: u16 = 50;

/// Raw encoder reading (`u16`, one mechanical rev) → rotor electrical angle
/// (`u16`, one electrical cycle), given the alignment `offset`.
#[inline]
pub fn electrical_angle(enc: u16, offset: u16) -> u16 {
    enc.wrapping_mul(POLE_PAIRS).wrapping_sub(offset)
}

/// Compute the alignment `offset` from an encoder reading taken while the rotor
/// is held at electrical zero (d-axis driven). Pass the result as `offset` to
/// [`electrical_angle`], which will then read 0 at that physical position.
#[inline]
pub fn offset_from_alignment(enc_at_align: u16) -> u16 {
    enc_at_align.wrapping_mul(POLE_PAIRS)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aligned_reads_zero() {
        for enc in (0..=65535u32).step_by(257) {
            let e = enc as u16;
            let off = offset_from_alignment(e);
            assert_eq!(electrical_angle(e, off), 0);
        }
    }
}
