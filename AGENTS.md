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

## Toolchain (installed & version-pinned to match the n32g4 family)

- `svd2rust` **0.31.5** (pinned — newer versions change the PAC API shape)
- `form` 0.13.0, `svdtools` 0.1.27 (CLI is `svd`, not `svdtools`)
- Rust 1.92, target `thumbv7em-none-eabihf` (installed)
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

1. Finish the PAC in the infra repo: run N32L403 through the same pipeline
   (prepass -> svd2rust -> form -> build) that already works for N32L406; add
   any new prepass rules it surfaces; assemble the dual-device crate mirroring
   the n32g4 template (per-device modules, build.rs/device.x, features).
2. Fork + retarget the HAL.
3. Fill `src/motion/` FIRST — it's pure math, host-testable, no hardware. Get
   FOC/encoder/interpolation correct in unit tests before touching silicon.
4. Then `boundary/`, then `hot/` ISRs (current first, cascade second), then
   `canopen/` async last.
5. Install probe-rs; bring up on hardware via SWD (J5).

## Status at handoff

- Architecture fully decided (this file + docs/).
- PAC pipeline PROVEN end-to-end for N32L406: vendor SVD -> idempotent
  Python prepass (1 rule, 6 sites: bad access enums) -> svd2rust ->
  form (73 modules) -> `cargo build --target thumbv7em-none-eabihf` COMPILES.
- This repo is scaffolding only — module stubs, no real logic yet.
