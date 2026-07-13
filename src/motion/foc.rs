//! motion/foc.rs — pure field-oriented-control math for a 2-phase drive.
//!
//! The MKS servo motor is a 2-phase bipolar hybrid stepper run as a PMSM. Its
//! two windings ARE the alpha/beta axes, so Clarke and inverse-Clarke are
//! identities and only the Park rotation is real work.
//!
//! Everything here is pure `f32` and takes `sin`/`cos` of the electrical angle
//! as arguments (a LUT keyed by the u16 angle in the hot ISR; real trig in host
//! tests). No hardware, no float-trig dependency: fully host-testable.

/// Clarke for a 2-phase motor: identity (windings == alpha/beta).
#[inline(always)]
pub fn clarke(ia: f32, ib: f32) -> (f32, f32) { (ia, ib) }

/// Inverse Clarke for a 2-phase motor: identity.
#[inline(always)]
pub fn inv_clarke(valpha: f32, vbeta: f32) -> (f32, f32) { (valpha, vbeta) }

/// Park: stationary (alpha,beta) -> rotating (d,q), given sin/cos of theta_e.
#[inline(always)]
pub fn park(alpha: f32, beta: f32, sin: f32, cos: f32) -> (f32, f32) {
    (alpha * cos + beta * sin, -alpha * sin + beta * cos)
}

/// Inverse Park: rotating (d,q) -> stationary (alpha,beta).
#[inline(always)]
pub fn inv_park(d: f32, q: f32, sin: f32, cos: f32) -> (f32, f32) {
    (d * cos - q * sin, d * sin + q * cos)
}

/// Clamp the (vd,vq) voltage vector into the circle of radius `vmax`.
/// Returns (vd', vq', saturated) — the flag drives current-loop anti-windup.
#[inline]
pub fn clamp_circle(vd: f32, vq: f32, vmax: f32) -> (f32, f32, bool) {
    let mag2 = vd * vd + vq * vq;
    if mag2 <= vmax * vmax {
        (vd, vq, false)
    } else {
        let s = vmax / sqrtf(mag2);
        (vd * s, vq * s, true)
    }
}

/// PI controller with integrator clamping (anti-windup).
#[derive(Clone, Copy)]
pub struct Pi {
    pub kp: f32,
    pub ki: f32,
    pub i: f32,
    pub out_min: f32,
    pub out_max: f32,
}

impl Pi {
    pub const fn new(kp: f32, ki: f32, out_min: f32, out_max: f32) -> Self {
        Self { kp, ki, i: 0.0, out_min, out_max }
    }

    /// One PI step, `dt` in seconds. Simple integrator-clamp anti-windup; for the
    /// current loop the (d,q) pair should additionally honor `clamp_circle`'s
    /// saturation flag — see hot/current.rs.
    #[inline]
    pub fn step(&mut self, err: f32, dt: f32) -> f32 {
        self.i = clampf(self.i + self.ki * err * dt, self.out_min, self.out_max);
        clampf(self.kp * err + self.i, self.out_min, self.out_max)
    }

    pub fn reset(&mut self) { self.i = 0.0; }
}

/// The FOC current (torque) loop core: (iA,iB) + angle + iq_ref -> (vA,vB),
/// with id regulated to 0 (max torque-per-amp). Pure; call once per PWM tick.
pub struct CurrentLoop {
    pub pi_d: Pi,
    pub pi_q: Pi,
    pub last_id: f32,
    pub last_iq: f32,
}

impl CurrentLoop {
    pub const fn new(pi_d: Pi, pi_q: Pi) -> Self {
        Self { pi_d, pi_q, last_id: 0.0, last_iq: 0.0 }
    }

    #[inline]
    pub fn step(&mut self, ia: f32, ib: f32, sin: f32, cos: f32,
                iq_ref: f32, vmax: f32, dt: f32) -> (f32, f32) {
        let (alpha, beta) = clarke(ia, ib);
        let (id, iq) = park(alpha, beta, sin, cos);
        self.last_id = id;
        self.last_iq = iq;
        let vd = self.pi_d.step(0.0 - id, dt);
        let vq = self.pi_q.step(iq_ref - iq, dt);
        let (vd, vq, _sat) = clamp_circle(vd, vq, vmax);
        let (valpha, vbeta) = inv_park(vd, vq, sin, cos);
        inv_clarke(valpha, vbeta)
    }
}

#[inline(always)]
fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo { lo } else if x > hi { hi } else { x }
}

/// Dependency-free f32 sqrt (bit-trick seed + 2 Newton steps). Adequate for the
/// voltage-circle clamp; swap for FPU VSQRT / `libm::sqrtf` later.
#[inline]
fn sqrtf(x: f32) -> f32 {
    if x <= 0.0 { return 0.0; }
    let mut y = f32::from_bits(0x1fbd_1df5 + (x.to_bits() >> 1));
    y = 0.5 * (y + x / y);
    y = 0.5 * (y + x / y);
    y
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sc(t: f32) -> (f32, f32) { (t.sin(), t.cos()) }

    #[test]
    fn park_roundtrip() {
        let (s, c) = sc(0.9);
        let (d, q) = park(0.3, -0.7, s, c);
        let (a, b) = inv_park(d, q, s, c);
        assert!((a - 0.3).abs() < 1e-5 && (b + 0.7).abs() < 1e-5);
    }

    #[test]
    fn circle_clamp_scales_onto_radius() {
        let (vd, vq, sat) = clamp_circle(3.0, 4.0, 2.5);
        assert!(sat);
        assert!(((vd * vd + vq * vq).sqrt() - 2.5).abs() < 1e-4);
    }

    #[test]
    fn sqrt_matches_std() {
        for x in [0.25f32, 1.0, 2.0, 100.0, 12345.0] {
            assert!((sqrtf(x) - x.sqrt()).abs() / x.sqrt() < 1e-3);
        }
    }
}
