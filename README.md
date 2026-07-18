# servo4257-rs

Open-source replacement firmware for the **MKS SERVO42D / SERVO57D** closed-loop
stepper drivers, written from scratch in Rust. The goal is to expose what the
stock firmware does not: **CANopen CiA 402 Interpolated Position (ip, mode 7)
and Cyclic Synchronous Position (csp, mode 8)** with feed-forwards and PDO
feedback — a capability effectively absent from the open-source space.

This is replacement firmware for hardware we own; it is not a modification or
reverse-engineering of the stock MKS firmware.

## Status

Bring-up is well underway and the core loop works on silicon:

- **Toolchain:** builds clean; the `n32l4` PAC and `n32l4xx-hal` build on both
  N32L403 (SERVO42D) and N32L406 (SERVO57D).
- **Over-CAN dev loop:** a CANopen bootloader flashes the app entirely over the
  CAN bus (no SWD, no BOOT0) with a CRC gate. See `docs/over-can-dev-loop.md`.
- **Commutation:** the motor spins under closed-loop sensored commutation. The
  correct commutation law (encoder direction + quadrature lead) was found and
  verified on hardware — see `docs/commutation-profile.md`.
- **Interactive tuning:** commutation knobs and telemetry are exposed over a
  manufacturer CANopen profile so the motor can be tuned live from a CANopen
  master with no reflash.

In progress: reliable spin-up from standstill (open-loop start → closed-loop
handoff), the cascaded current/velocity/position control loops, and the
CiA 402 ip/csp application on top.

## Start here

- `AGENTS.md` — operating brief; read first.
- `docs/ARCHITECTURE.md` — full derivation of the control architecture.
- `docs/DECISIONS.md` — what's decided / open / rejected.
- `docs/HARDWARE.md` — verified board + MCU reference.
- `docs/HAL_INTERFACE.md` — canonical firmware↔HAL/PAC interface contract.
- `docs/over-can-dev-loop.md` — reflash-over-CAN protocol, flash layout, CRC gate.
- `docs/commutation-profile.md` — the CANopen tuning profile + commutation notes.

## Layout

```
src/hot/       interrupt domain (two-tier: current loop + cascade) — mission-critical
src/boundary/  sync primitives between tiers (double-buffer SPSC, ring buffer)
src/motion/    pure FOC/encoder/interp math — host-testable
src/canopen/   embassy async tier (CiA 402, PDO/SYNC) — advisory
src/boards/    per-board calibration behind a Board trait
src/bin/       per-board + bring-up binaries (build separately, mutually
               exclusive features)
```

## Building

The app binaries build for their board, in the app flash layout:

```
cargo build --bin servo42d --features board-42d,layout-app   # N32L403
cargo build --bin servo57d --features board-57d,layout-app   # N32L406
```

`cargo xtask dist --bin <name> [--features <extra>]` produces the raw `app.bin`
plus the `app.img` with an APP-META (CRC + length) page for the over-CAN
bootloader. Several bring-up binaries live in `src/bin/` (e.g. `commlab`,
`spinsweep`, `holdtest`); see their module docs for what each probes.

## License

MIT OR Apache-2.0.
