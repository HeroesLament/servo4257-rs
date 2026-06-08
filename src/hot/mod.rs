//! Hot path — the interrupt domain. Mission-critical.
//! Two-tier: `current` (top priority) preempts `cascade` (2nd priority).
//! Nothing below may mask interrupts longer than the current-loop slack.
pub mod current;
pub mod cascade;
pub mod interp;
pub mod nvic;
