//! Concrete board profiles, selected by cargo feature.
#[cfg(feature = "board-42d")]
pub mod servo42d;
#[cfg(feature = "board-57d")]
pub mod servo57d;
