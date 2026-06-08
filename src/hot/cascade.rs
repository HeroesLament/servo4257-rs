//! 2nd-priority cascade ISR (decimated, ~1-2 kHz), preemptible by `current`.
//! Position loop -> velocity loop, with feed-forward injection at each stage,
//! plus `next_target()` (csp: read buffer; ip: interpolate). Publishes the
//! target d/q current for the current loop via the boundary cell.
