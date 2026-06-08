//! Async tier (embassy). ADVISORY, not safety-critical: if it stalls, the
//! ISRs detect stale data and fault-handle on their own.
pub mod od;
pub mod cia402;
pub mod pdo;
pub mod transport;
