# Decision Log

Terse record of what is DECIDED (with the deciding reason) vs OPEN. A future
session should treat DECIDED items as settled unless a new constraint appears.

## DECIDED

- **Replace stock firmware, not shim.** Stock CAN protocol only offers
  profiled point-to-point moves; csp needs per-SYNC target overwrite with no
  drive-side profile. Can't synthesize csp through stock firmware.
- **Target the boards we own (N32L403 + N32L406).** No pivot to nicer silicon
  (e.g. STM32G4) — firmware is for existing hardware.
- **In-place port on N32L403/406**, not a companion MCU.
- **PAC: dual-device single crate** (`n32l403` + `n32l406` features), mirroring
  the n32g4 family layout. Build from vendor SVDs (they exist! in the Keil
  N32L40x_DFP) via idempotent Python prepass -> svdtools -> svd2rust -> form.
- **svd2rust pinned to 0.31.5** to match the n32g4 family API shape.
- **Prepass idempotency by defect-pattern, not markers.** Absence of the defect
  IS the applied state; re-runs are byte-identical no-ops. Vendor SVD never
  mutated; corrected copy written to build/.
- **HAL: fork guineawheek/n32g4xx-hal** (0BSD, blocking embedded-hal HAL), not
  build a fresh async HAL.
- **embassy for the async tier only** (CANopen/CiA402/PDO), via embassy-executor
  + custom embassy-time driver. NOT for control loops.
- **Control loops in hardware ISRs**, not async. (Same reason AtomVM was
  rejected: no scheduler in the us-critical hot path.)
- **Split C: full cascade (current+velocity+position) in the ISR.** Forced by
  feed-forwards being mandatory — FF injection requires co-located loops.
- **Feed-forwards mandatory** (velocity + torque FF streamed each SYNC).
- **ip interpolation ISR-side**, behind uniform `next_target()` shared with csp.
  Mission-criticality -> guaranteed-by-construction over guaranteed-if-async.
- **Two-tier interrupts:** current loop top priority; cascade 2nd priority,
  pended + preemptible by current loop. Current-loop timing immune to cascade
  cost; scales with FF/interp richness.
- **Boundary primitives:** double-buffered SPSC bundles both directions; ring
  buffer for ip. Atomics for any single-word shared scalar.
- **One firmware, two boards.** Per-board cargo features select PAC device +
  calibration consts. Separate cargo invocations per board. Board deltas only
  in src/boards/ behind a Board trait.
- **Single repo for firmware**, separate infra repo for PAC + HAL.

## OPEN (need a decision or measurement later)

- **Decimation ratio** (current : cascade). Start ~16:1 (20kHz : ~1.25kHz);
  tune on silicon.
- **Interpolation order for ip** (linear vs cubic). Affects ISR cycle cost;
  measure against the 3200-cycle budget.
- **Exact NVIC priority numbers** and confirming no dep (embassy/HAL/
  critical-section) masks interrupts beyond the current-loop slack. REQUIRED
  audit before trusting the two-tier guarantee.
- **Whether N32L403 needs additional prepass rules** beyond the 406's access
  fix (unknown until run through the pipeline).
- **PAC upstreaming**: the n32g4 PAC's SOURCE repo is not public (only the
  generated crate). So an upstream target for n32l4 may not exist; likely
  publish our own family-consistent crate rather than PR.
- **CANopen stack choice**: CANopenNode (C, portable) vs a Rust stack vs
  hand-rolled. Not yet decided.
- **probe-rs chip support** for N32L403/406 (non-mainstream part; may need a
  custom chip definition for flashing over SWD).

## REJECTED (with reason, so they don't get re-proposed)

- AtomVM/bytecode-VM for FOC — no us-timing guarantee.
- CiA 402 shim on companion MCU — can't create a control mode the stock
  firmware doesn't expose.
- Pivot to STM32G4 / custom board — firmware is for boards already owned.
- Fully-async n32l4 HAL — far larger than forking the blocking HAL; unnecessary
  since control loops are ISRs anyway.
- async-side ip interpolation — per-tick deadline across the embassy boundary
  isn't guaranteed; would require ISR policing async liveness.
- One-ISR-inline cascade — injects jitter into the current loop on decimation
  ticks; doesn't scale with FF richness.
