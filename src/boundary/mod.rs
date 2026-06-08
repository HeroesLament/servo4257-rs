//! Sync primitives between tiers. Single-core: hazard is preemption
//! mid-update (torn READ), not parallelism. Per-item primitive by size:
//! scalar -> atomic; bundle -> double-buffer SPSC (lock-free, never stalls ISR).
pub mod setpoint;
pub mod feedback;
pub mod ipbuf;
