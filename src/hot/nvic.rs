//! NVIC priority map — the load-bearing invariant.
//! current-loop > cascade > everything async (PendSV/SysTick/embassy).
//! REQUIRED: audit that no dep masks interrupts beyond current-loop slack.
