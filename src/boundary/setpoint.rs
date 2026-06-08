//! DOWN path: double-buffered SPSC bundle, async writer -> ISR reader.
//! {target_pos, vel_ff, torque_ff, seq/valid}. Writer fills inactive buffer,
//! flips atomic index; reader always sees a complete published buffer.
