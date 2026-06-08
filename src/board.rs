//! Board trait: the per-board deltas. Everything else is shared.
//! Implementors supply shunt scaling, current limits, and the PAC device.
pub trait Board {
    /// Shunt resistance in ohms (42D = 0.05, 57D = 0.02).
    fn shunt_ohms() -> f32;
    /// Max phase current in milliamps.
    fn max_current_ma() -> u32;
}
