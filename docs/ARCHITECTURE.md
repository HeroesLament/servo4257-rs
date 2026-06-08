# Architecture

This document captures the *reasoning*, not just the conclusions. If you are
tempted to change a decision, read the corresponding rationale here first —
most of these were derived from constraints, not chosen by preference.

## The goal, precisely

CiA 402 drive profile with **ip (mode 7)** and **csp (mode 8)**, with
feed-forwards, over CANopen, with PDO feedback. Open drives that implement
csp properly (real interpolation, SYNC-locked PDOs) are nearly nonexistent in
open source; that gap is the point of the project.

## Why replace firmware at all (not a shim)

The stock MKS firmware already does FOC well on this silicon — FOC was never
the reason to rewrite. The reason is **control mode**: the stock CAN command
protocol exposes only profiled point-to-point moves (its own trapezoidal
generator), which is the opposite of csp, where the master overwrites the
target every SYNC and the drive must NOT run its own profile. You cannot
synthesize csp through a firmware that only offers profiled moves, so you must
own the layer that holds the position loop. That — not FOC performance — is
the justification for replacement.

A CiA 402 shim on a companion MCU was considered and rejected for exactly this
reason: it can't create a control mode the underlying firmware doesn't expose.

## Why not embassy/async for the control loops

A 20 kHz current loop needs deterministic, jitter-bounded, microsecond-budget
execution. A cooperative async executor (embassy) — like a bytecode VM
(AtomVM, the original idea, rejected for the same reason) — cannot guarantee
that in the hot path. So:

- Current loop + cascade: **hardware interrupts**, no executor, no await.
- Everything else (CANopen, CiA 402, PDO/SYNC, comms): **embassy async**,
  because it's event-driven, I/O-bound, concurrent, and NOT us-critical.

This split (async outer + ISR hot path) is the standard architecture for
serious async-Rust motor control, not a compromise we invented.

embassy needs only `embassy-executor` + `embassy-time` with a CUSTOM time
driver (off a spare timer — the 406 has TIM1..9). We do NOT have a fully async
n32l4 HAL and don't need one; peripherals are accessed via the blocking HAL /
raw PAC from async context. Building a fully-async HAL would be a much larger
project than the forked blocking HAL we're using.

## The concurrency hazard (get the model right)

Single-core Cortex-M: there is NO parallel execution. The ISR PREEMPTS main/
async code, runs to completion, returns. The hazard is therefore NOT a "torn
thread pointer" and the fix is NOT immutability. The hazard is **preemption
mid-update**: main code is halfway through writing a multi-word shared bundle
when the ISR fires and reads a half-old/half-new value (a torn READ of data).

Three tools, chosen per shared item by size & direction:
- Single word, one CPU store -> **atomic** (no torn read possible).
- Multi-field bundle -> **double-buffer / seqlock (SPSC)**: writer fills the
  inactive buffer, flips an atomic index; reader always sees a complete
  published buffer. Lock-free, never stalls the ISR. PREFERRED for the csp
  setpoint path so the current loop is never delayed.
- (Alternative for bundles: short `critical-section` — but it briefly masks
  interrupts, so only where a few cycles of ISR delay is acceptable. Avoid on
  the current-loop path.)

Rust forces this boundary into the open: you cannot share state between ISR
and async without an explicit sync primitive, so the seam becomes small,
typed, and auditable.

## Where to split the cascade (Split A/B/C)

Three candidate boundaries between ISR and outer world:
- **A — boundary at torque:** ISR runs only current loop; pos+vel loops
  outside. Crosses boundary: one scalar (q-current). Leanest ISR.
- **B — boundary at velocity:** ISR runs current+velocity; pos loop outside.
- **C — boundary at position:** ISR runs the full cascade; master streams
  position targets. Classic csp shape.

**Feed-forwards being mandatory forces Split C.** The point of master-supplied
velocity/torque feed-forward is that they are INJECTED into the velocity and
current loops (added to each loop's output so the loops trim error rather than
generate the bulk command). Injection only works if the loops being fed are
co-located in one timing domain. You cannot inject velocity feed-forward into
a velocity loop running in a different domain than the position loop. So the
whole cascade goes in the ISR.

Consequence: the ISR runs current + velocity + position + 3 feed-forward
injections. On a 64 MHz M4F this is feasible but requires DECIMATION: current
loop every tick (~20 kHz); velocity/position loops every Nth tick (~1-2 kHz).

## ip interpolation: ISR-side (decided)

ip buffers master setpoints and interpolates between them at loop rate.
The math is cheap; the hard part is "a fresh target ready every tick, on time,
forever."

- ISR-side interpolation: the guarantee is FREE (it runs in the domain that
  already has it). Underflow detected in-loop -> correct CiA 402 fault
  handling. Cost: interpolation cursor/segment state in the hot path, and a
  mode difference (csp reads directly, ip interpolates).
- Async-side interpolation: hot path stays mode-agnostic, but the per-tick
  deadline now crosses the async boundary, where embassy's cooperative
  scheduler can't guarantee it -> risk of stale reads, requiring the ISR to
  police async liveness (cross-domain coupling we want to avoid).

**Decided: ISR-side**, for mission-criticality — guaranteed-by-construction
beats guaranteed-if-async-behaves. Since Split C already put the full cascade
in the ISR, the marginal cost of the interpolation cursor is small. csp and ip
share the call site behind a uniform `next_target()`; the mode difference is
one function body, not a sprawling hot-path branch.

(Escape hatch: if profiling on silicon shows the hot path over budget, the
boundary is structured so interpolation COULD move to async. Don't
pre-optimize; measure first.)

## Two-tier interrupts (decided)

Decimated cascade options:
- **One ISR, cascade inline:** simplest NVIC (one interrupt, one priority) but
  every decimation tick does current+cascade in one invocation. The heavy tick
  (~2500-3500 cycles) brushes/exceeds the 3200-cycle budget and DELAYS the next
  current sample -> jitter injected into the current loop every Nth tick. You
  hand-balance to stay under budget, and re-balance every time feed-forward/
  interpolation richness grows.
- **Two-tier (CHOSEN):** current loop = own interrupt, TOP priority. Cascade =
  second, LOWER-priority interrupt, pended by the current loop and PREEMPTIBLE
  by it. The current loop's timing becomes IMMUNE to cascade cost — the cascade
  gets sliced by the current loop instead of delaying it. Costs: more NVIC
  plumbing, explicit priority map, and the cascade must be preemptible (clean
  here — current loop only reads the cascade's published target via the
  boundary cell, never the cascade's internal integrator/cursor state).

Two-tier matches every prior mission-critical choice and SCALES with
feed-forward/interpolation richness instead of forcing rationing. Jitter moves
from where it hurts (current loop) to where it doesn't (outer loop @ ~1-2 kHz).

## Timing budget reference

- 64 MHz, 20 kHz current loop -> 50 us / 3200 cycles per tick (HARD deadline).
- Current loop work alone: ~ few hundred to ~1000 cycles (SPI encoder read is
  the variable cost).
- Cascade (decimated): heavier; runs in the 2nd-tier ISR, preemptible.

## CiA 402 mode mapping

- mode 8 (csp): async writes raw master setpoint bundle to the DOWN buffer
  each SYNC. ISR runs cascade toward it with injected feed-forwards.
- mode 7 (ip):  async fills the ip ring buffer from PDOs; ISR-side interpolator
  produces the per-tick target via `next_target()`. Same cascade afterward.
- Feedback (TxPDO): ISR publishes actual pos/vel/iq to the UP buffer; async
  marshals it into PDOs on SYNC. Classical CAN = 8-byte frames, so PDO
  mappings must fit 8 bytes; may need multiple PDOs per cycle.
