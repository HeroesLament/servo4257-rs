# servo4257-rs

Open-source replacement firmware for the **MKS SERVO42D / SERVO57D** closed-loop
stepper drivers, adding **CANopen CiA 402 Interpolated Position (ip) and Cyclic
Synchronous Position (csp)** with feed-forwards and PDO feedback -- a capability
effectively absent from the open-source space.

**Status:** architecture decided, repo scaffolded (stubs only). Not yet
buildable -- depends on the `n32l4` PAC and `n32l4xx-hal` from the infra repo,
which are in progress.

## Start here

- `AGENTS.md` -- operating brief; read first.
- `docs/ARCHITECTURE.md` -- full derivation of the control architecture.
- `docs/DECISIONS.md` -- what's decided / open / rejected.
- `docs/HARDWARE.md` -- verified board + MCU reference.

## Layout

```
src/hot/       interrupt domain (two-tier: current loop + cascade) -- mission-critical
src/boundary/  sync primitives between tiers (double-buffer SPSC, ring buffer)
src/motion/    pure FOC/encoder/interp math -- host-testable
src/canopen/   embassy async tier (CiA 402, PDO/SYNC) -- advisory
src/boards/    per-board calibration behind a Board trait
src/bin/       per-board binaries (build separately, mutually exclusive features)
```

## Building (once infra deps exist)

```
cargo build --bin servo42d --features board-42d   # N32L403
cargo build --bin servo57d --features board-57d   # N32L406
```

## License

MIT OR Apache-2.0.
