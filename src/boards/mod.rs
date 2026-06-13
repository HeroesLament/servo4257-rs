//! Board implementors. Exactly one is selected by a cargo feature
//! (`board-42d` or `board-57d`), each pulling its own PAC device. The shared,
//! schematic-verified pin/peripheral map lives in `hw_map`; the per-board
//! modules supply only the genuine deltas (PAC device, shunt/current consts,
//! optional subsystems) and delegate the common wiring to `hw_map`.

pub mod hw_map;

#[cfg(feature = "board-42d")]
pub mod servo42d;
#[cfg(feature = "board-57d")]
pub mod servo57d;

/// The active board type for this build, re-exported as `ActiveBoard` so the
/// rest of the firmware names one concrete type without #[cfg] sprinkled
/// everywhere.
#[cfg(feature = "board-42d")]
pub use servo42d::Servo42D as ActiveBoard;
#[cfg(feature = "board-57d")]
pub use servo57d::Servo57D as ActiveBoard;
