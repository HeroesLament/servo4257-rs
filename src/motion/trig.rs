//! motion/trig.rs — fast `u16`-angle → (sin, cos) for the hot path.
//!
//! The electrical angle is carried as a `u16` wrapping the full circle
//! (`0..=u16::MAX` == `0..2π`), matching `board::RotorAngle`. The current loop
//! needs `sin`/`cos` of that angle every tick, so this must be cheap on the
//! Cortex-M4F: a parabolic sine approximation (Nicolas Capens' form, with the
//! precision-refinement step) — a few FMAs, no table, ~1.1e-3 max abs error
//! (≈0.06° equivalent), well below the cogging and current-sense noise floor.
//! Pure `f32`, host-testable against `std`. If a future need demands tighter
//! commutation, swap in a higher-order minimax poly or an interpolated LUT.

use core::f32::consts::PI;

const TWO_PI: f32 = 2.0 * PI;
/// u16 angle units per radian-scaled full turn.
const RAD_PER_UNIT: f32 = TWO_PI / 65536.0;
/// Quarter turn in u16 units — a 90° phase shift for deriving cos from sin.
const QUARTER: u16 = 16384;

/// sin of `x` reduced to `[-π, π]`. Parabola + one refinement pass.
#[inline(always)]
fn sin_reduced(x: f32) -> f32 {
    // Base parabola: y = (4/π)x − (4/π²)x·|x|  (exact at 0, ±π/2, ±π).
    const B: f32 = 4.0 / PI;
    const C: f32 = -4.0 / (PI * PI);
    let y = B * x + C * x * x.abs();
    // Refinement toward minimax: y = P·(y·|y| − y) + y, P = 0.225.
    0.225 * (y * y.abs() - y) + y
}

/// sin of a `u16` electrical angle.
#[inline]
pub fn sin_u16(angle: u16) -> f32 {
    // Map [0, 2π) then fold into [-π, π) so the approximation stays accurate.
    let mut x = angle as f32 * RAD_PER_UNIT;
    if x > PI {
        x -= TWO_PI;
    }
    sin_reduced(x)
}

/// (sin, cos) of a `u16` electrical angle. cos(θ) = sin(θ + 90°), via a
/// wrapping quarter-turn add — no second reduction special-case needed.
#[inline]
pub fn sin_cos(angle: u16) -> (f32, f32) {
    (sin_u16(angle), sin_u16(angle.wrapping_add(QUARTER)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_std_across_the_circle() {
        let mut max_err = 0.0f32;
        for a in (0..=65535u32).step_by(37) {
            let (s, c) = sin_cos(a as u16);
            let theta = a as f32 * (TWO_PI / 65536.0);
            max_err = max_err.max((s - theta.sin()).abs());
            max_err = max_err.max((c - theta.cos()).abs());
        }
        // Refined-parabola bound is ~1.1e-3; assert the documented envelope.
        assert!(max_err < 1.5e-3, "max abs error {max_err} too large");
    }

    #[test]
    fn cardinal_points() {
        let approx = |a: u16| sin_cos(a);
        // 0°, 90°, 180°, 270°
        let (s0, c0) = approx(0);
        assert!(s0.abs() < 2e-3 && (c0 - 1.0).abs() < 2e-3);
        let (s90, c90) = approx(QUARTER);
        assert!((s90 - 1.0).abs() < 2e-3 && c90.abs() < 2e-3);
        let (s180, c180) = approx(QUARTER * 2);
        assert!(s180.abs() < 2e-3 && (c180 + 1.0).abs() < 2e-3);
        let (s270, c270) = approx(QUARTER * 3);
        assert!((s270 + 1.0).abs() < 2e-3 && c270.abs() < 2e-3);
    }

    #[test]
    fn unit_magnitude() {
        for a in (0..65536u32).step_by(101) {
            let (s, c) = sin_cos(a as u16);
            let mag = (s * s + c * c).sqrt();
            assert!((mag - 1.0).abs() < 3e-3, "‖(sin,cos)‖ = {mag} at {a}");
        }
    }
}
