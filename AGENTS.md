# AGENTS.md — servo4257-rs

Operating brief for an AI agent (or human) picking up this project cold.
Read this first, then `docs/ARCHITECTURE.md` and `docs/DECISIONS.md`.

## What this is

Open-source firmware for the **MKS SERVO42D and SERVO57D** closed-loop
stepper driver boards, rewritten from scratch in Rust to add what the stock
firmware does not expose: **CANopen CiA 402 with Interpolated Position (ip,
mode 7) and Cyclic Synchronous Position (csp, mode 8), including
feed-forwards and PDO feedback.** That capability is effectively absent from
the open-source space; it is the entire reason this project exists.

We are NOT modifying or reverse-engineering stock MKS firmware (there is no
leaked source; the vendor repos are host-side examples + a CAN command
manual only). We are writing replacement firmware on hardware we own.

## Hardware (verified from schematics, not assumed)

- **MCU:** NSING (Nationstech) N32L40x family, single-core Cortex-M4F @ 64 MHz,
  FPU + DSP, 128KB flash / 24KB SRAM.
  - SERVO42D board -> **N32L403KBQ7** (LQFP48)
  - SERVO57D board -> **N32L406CBL7** (LQFP48, adds RS485 + opto I/O)
- **Encoder:** MT6816, 14-bit magnetic, over SPI.
- **Gate drivers:** EG3013 x4 (two H-bridges, stepper = 2-phase PMSM FOC).
- **Power FETs:** 42D = AP4008QD dual; 57D = AP050N03Q (8 discrete, beefier).
- **Current sense:** dual shunts + GS8632 amps into ADC.
  - 42D shunts = 0.05 ohm; 57D shunts = 0.02 ohm  (CALIBRATION DIFFERS)
- **CAN:** TJA1051 transceiver + on-chip bxCAN (classical CAN, 8-byte frames,
  NOT CAN-FD -> PDO payloads capped at 8 bytes).
- **Timers:** N32L406 SVD exposes TIM1..TIM9; TIM1 & TIM8 are advanced-control
  (complementary outputs + dead-time / BKDT register) -> two adv timers
  available. ADC is single, ~4.4-4.5 Msps, 12-bit (sequential phase sampling).
- **Single core.** No second core. Concurrency = NVIC interrupt priorities,
  not multicore.

## Repo relationships (IMPORTANT)

This is the **firmware** repo. The **chip-support** layer lives in a SEPARATE
repo (the "infra repo") and is depended on, never edited from here:

- `n32l4`      — PAC (svd2rust). Dual-device: features `n32l403`, `n32l406`.
- `n32l4xx-hal`— HAL, forked from `guineawheek/n32g4xx-hal` (0BSD), retargeted.

Dependency direction is strictly downward and must never invert:
`bin -> lib -> {hot, canopen} -> boundary -> motion -> hal -> pac`

## Toolchain (version-pinned across the workspace)

- **Rust: `nightly-2026-06-07`**, pinned via `rust-toolchain.toml` in this
  repo AND in `n32l4-pac`. REQUIRED: the HAL (n32l4xx-hal, forked from
  n32g4xx-hal) uses 8 unstable features (`adt_const_params`,
  `min_specialization`, `impl_trait_in_assoc_type`, etc.), so the whole
  dependency tree builds on nightly. (Was "Rust 1.92 stable" — that predated
  the HAL dependency and is no longer correct.)
- target `thumbv7em-none-eabihf` (installed for the pinned nightly).
- `svd2rust` **0.31.5** (pinned — newer versions change the PAC API shape)
- `form` 0.13.0, `svdtools` 0.1.27 (CLI is `svd`, not `svdtools`)
- `probe-rs` NOT yet installed (needed to flash; SWD header J5 is broken out)
- zsh autocorrect bites unknown commands: `unsetopt correct correct_all` in
  fresh shells before installs.

## Control architecture (decided — do not relitigate without cause)

Two-tier interrupt structure + advisory async. See docs/ARCHITECTURE.md for
the full derivation; the short version:

1. **Current loop** — TOP-priority timer/ADC ISR, ~20 kHz. FOC inner loop.
   Uninterruptible by anything below it. ~3200 cycle budget @ 64MHz/50us.
2. **Cascade** — 2nd-priority ISR, pended by the current loop on its
   decimation tick (~1-2 kHz). Velocity loop + position loop + feed-forward
   injection at each stage + ip interpolation. PREEMPTIBLE by the current
   loop, so heavy outer math never injects jitter into the current loop.
3. **Async (embassy)** — CANopen, CiA 402 state machine, PDO/SYNC marshalling.
   ADVISORY ONLY: if it stalls, the ISRs detect stale data and fault-handle
   on their own. Never safety-critical.

Tier boundaries are explicit sync primitives:
- DOWN: double-buffered SPSC bundle {target_pos, vel_ff, torque_ff, seq/valid}
- UP:   double-buffered SPSC bundle {actual_pos, actual_vel, actual_iq, status}
- ip:   ring buffer, async producer -> ISR-side interpolator consumer

csp and ip share the hot-path call site via a uniform `next_target()`; the
only difference is whether async writes raw master setpoints (csp) or the ip
ring is interpolated (ip). Underflow/stale detection is IN-LOOP for
self-contained CiA 402 fault handling.

### The load-bearing invariant

The current-loop ISR must be strictly highest priority and NOTHING may mask
interrupts longer than its slack (~50us). This includes embassy internals,
HAL critical sections, and any `critical-section::with`. **Audit every
interrupt-masking critical section in all deps.** This is what makes the
two-tier guarantee real; violating it silently destroys current-loop timing.

## Feed-forwards are mandatory

The master streams velocity + torque feed-forward alongside position each
SYNC. This is WHY the full cascade lives in the ISR (Split C): feed-forward
injection only works if the loops being fed are co-located in one timing
domain. Do not split the cascade across the async/ISR boundary.

## Build model: one firmware, two boards

Single codebase, per-board cargo features (`board-42d`, `board-57d`) selecting
the matching PAC device feature + calibration consts (shunt scale, current
limits). The two boards build as SEPARATE cargo invocations (mutually
exclusive PAC device features can't co-compile). Board deltas live ONLY in
`src/boards/` behind a `Board` trait; everything else is shared.

## Where to start (suggested order, lowest-risk first)

1. ~~Finish the PAC in the infra repo.~~ DONE — both devices build; see
   `n32l4-pac/README.md`. Pipeline is now prepass -> svd patch (svdtools) ->
   svd2rust -> form.
2. **Fork + retarget the HAL — IN PROGRESS.** Fork created and building
   against the PAC; peripheral modules being ported module-by-module. See
   `n32l4xx-hal/PORT_STATUS.md` for the live state and plan. This is the
   current critical path: the firmware can't drop its commented-out PAC/HAL
   deps until the HAL compiles.
3. Fill `src/motion/` FIRST — it's pure math, host-testable, no hardware. Get
   FOC/encoder/interpolation correct in unit tests before touching silicon.
   (Can proceed in parallel with the HAL port — no hardware deps.)
4. Then `boundary/`, then `hot/` ISRs (current first, cascade second), then
   `canopen/` async last.
5. Install probe-rs; bring up on hardware via SWD (J5).

## Status at handoff

- Architecture fully decided (this file + docs/).
- **PAC (`n32l4-pac`): DONE.** Both N32L403 and N32L406 build for
  thumbv7em-none-eabihf. Three-stage pipeline: prepass -> svdtools patch
  (strip register prefixes, add field enums) -> svd2rust 0.31.5 -> form. The
  svdtools Stage 2 makes the PAC present the n32g4 dialect so the HAL stays
  upstream-shaped. See `n32l4-pac/README.md`.
- **HAL (`n32l4xx-hal`): IN PROGRESS.** Forked from n32g4xx-hal, retargeted
  onto the PAC, toolchain pinned. Builds attempted; ~717 errors characterized
  and a module-by-module port plan is underway (rcc first). The recurring
  issue — the HAL expects an enum-enriched PAC — is being solved by enriching
  the SVD (Stage 2), not by diverging the HAL. See `n32l4xx-hal/PORT_STATUS.md`.
- **Toolchain: nightly-2026-06-07**, pinned in this repo and the PAC.
- This firmware repo itself is still scaffolding only — module stubs, no real
  logic yet. `src/motion/` (pure math) can be started in parallel with the HAL
  port since it has no hardware/PAC deps.
