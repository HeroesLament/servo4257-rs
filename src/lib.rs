#![no_std]
//! servo4257-rs — CANopen CiA 402 (csp/ip) firmware for MKS SERVO42D/57D.
//! See AGENTS.md and docs/ARCHITECTURE.md. Tiers: hot (ISRs) / boundary /
//! motion / canopen (async). Dependency direction is strictly downward.

pub mod app_meta;
pub mod boundary;
pub mod motion;
pub mod hot;
pub mod canopen;
pub mod board;
pub mod boards;
pub mod shared;
pub mod rt;

/// Shared firmware entry point. Each src/bin/<board>.rs selects its board
/// profile + PAC device feature and calls run().
pub fn run() -> ! {
    // TODO: init clocks/peripherals via HAL, configure NVIC priorities
    // (current loop TOP, cascade below, embassy/PendSV below that),
    // start embassy executor for the async tier, arm the current-loop timer.
    loop {}
}
