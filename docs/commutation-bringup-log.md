# Commutation bring-up — log

How the motor was made to spin, and the dead-ends along the way. The headline:
the whole difficulty was **one wrong commutation phase**, not weak drive, dead
coils, a bad current loop, or a supply limit — all of which were investigated
and ruled out.

## The answer (verified on hardware, ~475 RPM)

Closed-loop sensored commutation — the field placed at a fixed quadrature ahead
of the *live* rotor electrical angle, each control tick:

```
theta = (−enc) · POLE_PAIRS       # NEGATE the encoder direction
field =  theta + 16384            # +90° electrical (quadrature ahead)
va = amp·cos(field),  vb = amp·sin(field)
```

`POLE_PAIRS = 50`. Because it references the live encoder every tick, there is
no open-loop pull-out limit — a moving rotor stays locked to the field.

Getting the **encoder-direction sign** or the **lead sign** wrong is what made
every earlier attempt librate (rock ±5°) or hard-lock instead of turning. There
are only three binary axes — {encoder direction, lead sign, coil-B polarity} —
so eight combinations; a firmware sweep (`bin/spinsweep.rs`) with firmware-side
velocity telemetry found the spinning one unambiguously. Coil-B polarity turned
out to be a don't-care for this 2-phase motor.

## What was ruled out (each cost real time)

- **Drive strength / PSU limit:** a static field on either coil pulls 1.8 A and
  locks the shaft hard. Drive and supply were always fine.
- **Dead coil B:** both coils develop full holding torque individually.
- **Current sensing:** the default working mode is encoder-only; a current loop
  is a later performance upgrade, not a prerequisite (per the nano_stepper
  reference — see `nano-stepper-port.md`).
- **Open-loop stepping / calibration-first:** open-loop synchronous drive from
  standstill librates or stalls (pull-out); it is not the path to clean spin
  here. The encoder-referenced closed loop is.

## Measurement lesson

Sampling rotor position over CAN **aliases** a fast spin — a slow-rate host loop
reads near-identical values and looks frozen even while the shaft flies. It sent
an earlier (wrong) conclusion that a different combo was correct. Always measure
velocity **in firmware** (accumulate Δenc over a fixed window) and report that.

## Still open

Reliable **spin-up from standstill**. The commutation phase is correct, but a
closed-loop start from rest is a coin-flip without a known rotor position. The
standard fix is an open-loop acceleration ramp that drags the rotor up to a
modest speed, then a handoff to the closed loop — both halves are demonstrated;
they need sequencing.

## Bugs fixed during bring-up (all verified on silicon)

1. **CAN drought (slcan open race).** The `can_ex` `CANable.open_channel` wrote
   `C`/`S6`/`O` back-to-back; a cold/re-enumerated adapter coalesced them into
   one USB transfer and came up with the CAN channel closed → TX went nowhere,
   the board's TEC climbed, RF0R stayed 0. Fixed with inter-command settles.
   Root-caused from bxCAN registers over SWD.
2. **ADC scan mode off.** `servo57d.rs` configured two injected channels but
   `AdcConfig::default()` leaves scan disabled, so only slot One (coil A)
   converted; coil B's `JDAT2` read 0 forever. Fixed: `.scan(Scan::Enabled)`.
   Confirmed against the Nationstech SVD (ADC base `0x40020800`,
   `CTRL1.SCANMD` bit 8). Also added a software-triggered current sample so the
   sample instant is decoupled from the CC4/coil-B PWM edge.
3. **xtask `elf_to_bin` broken.** A hand-rolled `object`-crate extractor emitted
   the ELF header itself as `app.bin` (wrong bytes and wrong size), bricking
   every flash. Replaced with `arm-none-eabi-objcopy -O binary` plus a
   vector-table sanity gate (rejects any image whose first words aren't a RAM
   stack pointer + a Thumb reset vector in flash).
4. **CAN download contract.** `download`/`reflash` must stream the raw `app.bin`
   (≤ the app region), never `app.img` (region padded to full size + the meta
   page) — streaming the latter overruns into the meta page and the bootloader
   stamps an inconsistent CRC. Added a size guard.
5. **commlab shape knob (OD `0x2000:0A`).** Sine / trapezoid / third-harmonic
   field shaping, settable live over CAN.

## Bring-up binaries (in `src/bin/`)

Kept for reuse; each module header documents its command/telemetry protocol.

- `spinsweep` — closed-loop commutation-phase combination sweep with
  firmware-measured velocity telemetry. Found the correct phase.
- `holdtest` — static / continuous field drive; PSU-current and coil-polarity
  probe.
- `commlab` — the interactive tuning app behind the CANopen manufacturer profile.
- `foccal` — encoder calibration sweep (nano_stepper port, WIP).
- `spid` — nano_stepper "simple positional PID" servo (WIP).
- `focdiag` — current-during-rotation diagnostic.
