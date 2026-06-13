//! Board trait: the abstract motor-drive contract.
//!
//! This trait is the firmware<->hardware boundary. Control code (the current
//! loop, microstepping, CANopen position/velocity loops) depends ONLY on this
//! trait, never on a concrete HAL, PAC, pin, or peripheral type. Each supported
//! board provides an implementor in `crate::boards`.
//!
//! Scope: the MKS SERVO42D / SERVO57D family. Both are 2-phase bipolar steppers
//! driven as two independent full H-bridges (coil A, coil B), one single-ended
//! PWM per half-bridge into an EG3013 gate driver. The EG3013 owns dead-time
//! (200 ns fixed) and cross-conduction lockout, so the firmware drives four
//! plain PWM duties and never stages shoot-through protection in software.
//! (This is why the bridge timer being a general-purpose TIM3 with no
//! complementary/dead-time hardware is a non-issue. See docs/HAL_INTERFACE.md
//! and docs/HARDWARE.md.)
//!
//! This trait deliberately does NOT cover 3-phase BLDC / SimpleFOC hardware:
//! that is a different control topology (space-vector, 3 half-bridges) and
//! would warrant its own trait, not contortions here.
//!
//! HAL-agnostic by construction: nothing in these signatures names an
//! n32l4xx-hal or n32l4-pac type. Values are physical/logical (signed bridge
//! commands, currents, rotor angle). That keeps the boundary honest even though
//! only the N32-based boards are implemented today.

/// Signed per-coil bridge command, full-scale `i16`.
///
/// `+i16::MAX` = full forward drive across the coil, `i16::MIN` = full reverse,
/// `0` = no net voltage. The implementor maps this onto its two half-bridge PWM
/// duties using whatever H-bridge convention it prefers (bipolar or
/// sign-magnitude); callers do not care which.
pub type CoilCommand = i16;

/// Signed measured coil current, full-scale `i16` mapped to the board's
/// `MAX_CURRENT_MA`, so control code is current-scale-agnostic and the
/// per-board shunt/gain lives entirely in the implementor.
pub type CoilCurrent = i16;

/// Rotor angle as a `u16` wrapping the full circle (`0..=u16::MAX` == `0..2pi`).
/// Source is the on-board encoder (MT6816 over SPI1 on the MKS boards); the
/// implementor owns the transport and any zero-offset / direction correction.
pub type RotorAngle = u16;

/// The abstract motor-drive contract for a single servo board.
///
/// An implementor owns its configured peripherals (PWM timer, ADC, encoder bus,
/// enable line) after construction and exposes behavior through these methods.
/// The hot path holds a `&mut impl Board` and calls `read_coil_currents` /
/// `apply_coil_voltages` once per current-loop tick.
pub trait Board {
    // ---- Static, per-board characteristics -------------------------------

    /// Shunt resistance in ohms (42D = 0.05, 57D = 0.02). TODO: confirm 42D
    /// value against its power sheet (57D shunt R10/R22 = 0.02 ohm, verified).
    const SHUNT_OHMS: f32;

    /// Max continuous phase current in milliamps; full-scale for `CoilCurrent`
    /// and `CoilCommand`. TODO: confirm from datasheet/thermal (42D ~3 A,
    /// 57D ~5.2 A placeholders).
    const MAX_CURRENT_MA: u32;

    /// Human-readable board name for logs / CANopen identity.
    const NAME: &'static str;

    // ---- Motor-drive primitives (the hot-path contract) ------------------

    /// Apply signed bridge commands to the two coils. Realized as the four PWM
    /// duties (two half-bridges per coil). Must be cheap and ISR-safe: no
    /// allocation, no blocking, no interrupt masking. Called from the
    /// current-loop ISR.
    fn apply_coil_voltages(&mut self, v_a: CoilCommand, v_b: CoilCommand);

    /// Most recent coil currents `(i_a, i_b)`. On the MKS boards these come
    /// from the injected ADC group triggered off the PWM timer; this returns
    /// the latched conversion, it does NOT spin-wait for one. ISR-safe.
    fn read_coil_currents(&self) -> (CoilCurrent, CoilCurrent);

    /// Current rotor angle from the on-board encoder. May involve a (fast,
    /// non-blocking-from-ISR) SPI read; see implementor notes for whether this
    /// is safe to call directly in the hot path or must be pipelined.
    fn rotor_angle(&mut self) -> RotorAngle;

    /// Enable or disable the output stage (the `nEN` line, PB7 on the MKS
    /// boards, gating the EG3013s). `false` must leave the bridge safe (off).
    fn set_output_enable(&mut self, enabled: bool);

    // ---- Optional subsystems (presence varies by board) ------------------

    /// Whether this board exposes an RS485 / second serial channel (57D yes,
    /// 42D no -- the 32-pin package doesn't break out USART3). Control code
    /// that wants RS485 checks this rather than #[cfg]-ing on board.
    const HAS_RS485: bool = false;
}
