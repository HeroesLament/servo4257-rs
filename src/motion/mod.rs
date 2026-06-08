//! Pure control math. NO hardware/timing deps -> host-testable in `#[test]`.
//! Get this correct in unit tests BEFORE touching silicon or ISRs.
pub mod foc;
pub mod encoder;
pub mod commutation;
