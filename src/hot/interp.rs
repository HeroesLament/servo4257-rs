//! ip interpolation, consumed by the cascade at loop rate.
//! Drains the ip ring buffer; underflow detected in-loop for CiA 402 fault
//! handling. Order (linear vs cubic) is an OPEN decision — measure cycle cost.
