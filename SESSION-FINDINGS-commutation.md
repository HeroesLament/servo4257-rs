# Commutation bring-up — session findings

A long session that killed a stack of bugs and confirmed the motor spins, but
landed on a real architectural wall: **voltage-mode commutation can't sustain
clean rotation — the current-loop FOC ISR (task #41) is required.**

## Confirmed on silicon

- **Motor is healthy.** No board fault (per bench call, and consistent with all
  measurements). It develops real torque and has spun continuously (~25–30 RPM,
  clean encoder ramps) in several runs.
- **POLE_PAIRS = 50.** Sharp resonance: pp=50 gave a ~22k–27k-count spin vs a
  few hundred at every neighbor (48,49,51,52). This kills the earlier "250.5"
  ghost — that estimate was corrupted by the bugs below.
- **Encoder counts REVERSED vs electrical rotation.** `theta_e = offset −
  enc·pp` (MT6816 direction). `direction: 0` in the OD gives this.
- **True quadrature is LEAD = 32768**, not 16384, in this angle convention.
  16384 lands the field ON the rotor → stable servo hold (locks). +another 90°
  → 32768 = max constant torque. (Documented in `bin/foc_spin.rs` header.)
- **Commutation offset ≈ from align:** drive field at a fixed angle, rotor
  seats, `offset = enc_at_align · pp`. `motion/encoder.rs::offset_from_alignment`.
- **split_duty scaling:** `mag = |v|·max_duty / i16::MAX`. So amp 3000 ≈ 9% duty
  (too weak — rotor holds align), amp 12000 ≈ 37% (real torque). Amplitude must
  be ~12000+ to break static friction/detent.

## The wall

Voltage-mode fixed-lead commutation — in BOTH `commlab` sensored mode and the
purpose-built `foc_spin` — produces torque and motion but **hunts and slips near
synchronism**: the rotor either drifts while oscillating ± (weak sustained
torque) or hard-locks (wrong start position). A fixed voltage vector can't hold
torque through the slip. Host-side align→commutate handoff is a coin-flip
because it races the rotor over CAN latency; an in-firmware align-start state
machine (added to commlab this session) seats the rotor deterministically but
the fixed-lead run still can't sustain rotation.

This is exactly the problem **current-regulated FOC** solves and is the honest
next step. `foc_spin`'s own header hedges "no current sensing needed (it's
unreliable here)" — that shortcut is *why* it hunts.

## Bugs fixed this session (all verified)

1. **CAN drought (slcan open race)** — `can_ex` `CANable.open_channel` wrote
   `C`/`S6`/`O` back-to-back; a cold/re-enumerated adapter coalesced them into
   one USB transfer and came up with the channel closed → TX went nowhere, board
   TEC climbed, RF0R=0. Fixed with 25 ms inter-command settles. Root-caused from
   bxCAN registers over SWD.
2. **ADC scan mode off** — `servo57d.rs` configured two injected channels but
   `AdcConfig::default()` leaves scan disabled, so only slot One (coil A)
   converted; coil B `JDAT2` read 0 forever. Fixed: `.scan(Scan::Enabled)`.
   Confirmed against Nationstech SVD (ADC base 0x40020800, CTRL1.SCANMD bit 8).
   This is a prerequisite for the current loop (need both iA and iB).
3. **xtask elf_to_bin broken** — hand-rolled `object`-crate extraction emitted
   the ELF header as "app.bin" (wrong bytes AND wrong size). Replaced with
   `arm-none-eabi-objcopy -O binary` + a vector-table sanity gate.
4. **CAN download contract** — `download`/`reflash` must stream `app.bin` (raw,
   ≤ app region), never `app.img` (padded + meta page). Added a size guard.
5. **commlab shape knob (OD 0x2000:0A)** — sine/trapezoid/3rd-harmonic field
   shaping, live over CAN. Works.

## Known-good spin recipe (voltage-mode, hunts but moves)

    mode: sensored, pole_pairs: 50, direction: 0,
    offset: from align (~16384–20480 range observed), lead: 32768,
    amp: 12000, shape: sine

## Next: task #41 — current-loop FOC ISR

`hot/current.rs` is an empty stub. `motion/foc.rs` already has host-tested
park/inv_park/clamp_circle/Pi/CurrentLoop::step. Wire into a PWM-rate ISR:
measure (iA,iB) → Clarke/Park with encoder angle → PI(id→0)+PI(iq→iq_ref) →
inv Park → voltage vector; velocity outer loop ramps iq_ref. Watch the ADC
injected trigger (TIM3-CC4 shares the phaseB PWM compare — sample point coupled
to duty; may want a dedicated CC channel).
