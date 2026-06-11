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
- **svd2rust pinned to 0.37.1.** The n32g4-family API shape the HAL targets
  (`Sclksw`/`SclkswR`/`CfgSpec`, PascalCase peripherals like `Rcc`) is the
  svd2rust 0.32+ default theme. svd2rust <=0.31 emits the legacy shape
  (`SCLKSW_A`/`SCLKSW_R`/`CFG_SPEC`, CONSTANT_CASE `RCC`), which the HAL does
  NOT resolve against. CORRECTS the earlier "pinned to 0.31.5 to match the
  n32g4 shape" note: 0.31.5 does the opposite -- it was the root cause of the
  bulk of the HAL name-resolution errors. 0.37.1 also gives PascalCase
  peripheral names, so the planned separate `RCC`->`Rcc` casing patch is
  unnecessary.
- **Prepass idempotency by defect-pattern, not markers.** Absence of the defect
  IS the applied state; re-runs are byte-identical no-ops. Vendor SVD never
  mutated; corrected copy written to build/.
- **svdtools Stage 2 normalization (the pipeline is prepass -> svd patch ->
  svd2rust -> form).** The raw NSING SVD names registers with a peripheral
  prefix (`RCC_CFG`) and carries no field `<enumeratedValues>`. The n32g4
  family PAC (which the HAL targets) has neither defect. Stage 2 strips the
  per-peripheral register prefixes and adds the field enums, so the PAC
  presents the n32g4 dialect and the HAL stays close to upstream. COMP is NOT
  prefix-stripped (mixed COMP_/COMP1_/COMP2_ would collide; firmware doesn't
  use COMP).
- **Enum enrichment lives at the PAC layer, not the HAL (Path A, Option 1).**
  The n32g4 HAL matches on field enums (`Sclksw::Pll`, `Ahbpres::Div2`); our
  raw SVD emits bare bit accessors. We add the enums via svdtools Stage 2
  rather than rewriting HAL register access to raw bits. Keeps every HAL
  module upstream-shaped. Recurs across modules (rcc, timer, spi, can) — same
  treatment each time. CORRECTS the earlier provenance rule:
  enum values are NOT to be trusted from the n32g4 PAC / vendor SDK headers.
  Those are STARTING POINTS; the N32L40x User Manual / Datasheet is the source
  of truth and every enriched value must be verified against it. This is not
  hypothetical -- the ADC trigger enum (EXTRSEL/EXTJSEL) was inherited from
  stm32f4xx-hal, matched the old "trust the upstream values" rule, and was
  WRONG for this part (caught only by reading UM Tables 17-5/17-6). The clock
  registers do happen to share the n32g4 layout, but that is a fact to confirm
  per-field, not assumed. See `docs/HAL_INTERFACE.md` (provenance discipline).
- **Toolchain: nightly, pinned to a fixed date** (`nightly-2026-06-07`) via
  `rust-toolchain.toml` in both the firmware repo and the PAC. REQUIRED: the
  HAL uses 8 unstable features (adt_const_params, min_specialization,
  impl_trait_in_assoc_type, ...). All 8 verified present on the pinned nightly.
  Supersedes the earlier "Rust 1.92 stable" note (which predated the HAL dep).
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
- ~~Whether N32L403 needs additional prepass rules~~ RESOLVED: no. N32L403 has
  the identical 6-site bad-`<access>`-enum signature as the 406; the same
  single prepass rule covers both. Both devices build.
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
