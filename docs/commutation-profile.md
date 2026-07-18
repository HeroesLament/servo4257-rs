# Commutation Lab — manufacturer CANopen profile

A deliberately **nonstandard, manufacturer-specific** object area (CiA reserves
`0x2000–0x5FFF`) that exposes the commutation loop's knobs, telemetry, and board
lifecycle over expedited SDO — so the whole motor can be tuned interactively
from a CANopen master (IEx) with **no reflash**, and a wedged board can be
recovered over CAN with **no SWD**.

Served by the `commlab` bring-up binary (app side) and, for the management
objects, also by the bootloader (in its listen window / download service). The
host side is the Elixir `Commutation` and `Board` modules in `can_ex`.

All objects are ≤4-byte scalars → every access is an **expedited** SDO transfer.
Standard SDO framing for node 1: master → `0x601`, node → `0x581`.

Firmware source of truth: `src/canopen/od.rs`, `src/shared/params.rs`,
`src/canopen/mgmt.rs`. Keep this doc in sync with those.

---

## 0x2000 — Commutation parameters (RW)

Written by the SDO server → `shared::PARAMS` atomics; read each tick by the
commutation loop. Torn reads across unrelated knobs are harmless during tuning,
so per-field atomics (no seqlock).

| sub | name         | type | notes |
|-----|--------------|------|-------|
| 01  | `enable`     | u8   | output-stage enable (0/1) |
| 02  | `mode`       | u8   | 0 idle · 1 sensored · 2 open-loop · 3 align |
| 03  | `amplitude`  | i16  | vector magnitude (sign also flips drive direction) |
| 04  | `lead_angle` | u16  | electrical lead — the sweep knob (wraps the electrical circle) |
| 05  | `direction`  | u8   | encoder→electrical sign (0/1) |
| 06  | `offset`     | u16  | electrical alignment offset |
| 07  | `pole_pairs` | u16  | live-tunable; nominal 50 for this motor |
| 08  | `ol_angle`   | u16  | commanded open-loop field angle (mode 2/3) |
| 09  | `ol_rate`    | i16  | open-loop auto-advance per tick (0 = hold) |

Field angle in sensored mode: `theta_e = enc·pole_pairs ∓ offset`
(sign per `direction`), then `+ lead_angle`.

## 0x2001 — Telemetry (RO)

One coherent `shared::TELEM` seqlock snapshot per read (all fields from the same
tick).

| sub | name    | type | source |
|-----|---------|------|--------|
| 01  | `enc`   | i32  | rotor encoder / position |
| 02  | `theta` | u16  | electrical angle |
| 03  | `vel`   | i32  | velocity (Δenc/window) |
| 04  | `ia`    | i16  | coil A current (ADC, ~2043 = 0 A) |
| 05  | `ib`    | i16  | coil B current |
| 06  | `live`  | u16  | liveness counter (proves the loop runs) |

## 0x2F00 — Board management (WO triggers + RO status)

Answered by **both** the running app and the bootloader, so the board's
lifecycle is controllable in every state. A trigger is a write of `1`. Reboot /
stay / boot self-reset the MCU (so they usually send no SDO response — the host
treats a timeout on these as success).

| sub | name             | action |
|-----|------------------|--------|
| 01  | `reboot`         | soft reset (honors current boot flag) |
| 02  | `stay_in_boot`   | set stay flag + reset → bootloader |
| 03  | `boot_app`       | clear stay flag + reset → app |
| 04  | `invalidate_app` | (bootloader) erase app META → boot to loader next time; (app) falls back to stay-in-boot |
| 05  | `status` (RO)    | boot-flag / state word |

---

## Recovery model (no SWD once provisioned)

Three layers, so a hung app is never a demount:

1. **Bootloader CAN listen window (~300 ms).** Every boot, before jumping to the
   app, the bootloader brings up CAN and listens. A stay/enter-update command in
   that window holds it in the bootloader. After the window it restores the RCC
   to reset defaults, then jumps (so the app inits clocks from a clean state —
   a `sysclk(16MHz)` window config once panicked the HAL `freeze()`; the window
   now uses the proven HSE-8 / PCLK1-4 MHz service config).
2. **App IWDG (~1 s).** The commutation loop kicks it every pass. Any stall
   (e.g. a blocking I2C write to a dead OLED) auto-resets the MCU within ~1 s →
   bootloader → listen window. Most hangs self-heal with no intervention.
3. **Bounded/lazy peripheral init.** The OLED is brought up lazily inside the
   loop, never before the CAN service, so a missing panel can't wedge startup.

Host helpers (`can_ex` `Board` module):

```elixir
b = Board.open(device: "/dev/cu.usbmodem…")
Board.status(b)         # boot state
Board.reboot(b)         # soft reset
Board.stay_in_boot(b)   # reboot into bootloader
Board.boot_app(b)       # reboot into app
Board.recover(device: …) # spam stay across the window while you power-cycle
```

## Host tuning session (`Commutation` module)

```elixir
c = Commutation.open(device: "/dev/cu.usbmodem…")
Commutation.set(c, mode: :openloop, amp: 3000, ol_rate: 40, enable: true)
Commutation.telemetry(c)   #=> %{enc:, theta:, vel:, ia:, ib:, live:}
Commutation.stop(c)
```

## Adding a knob

1. Add the atomic field to `CommParams` (`src/shared/params.rs`).
2. Add the read+write arms to `od.rs` under `0x2000`.
3. Add the `{name, sub, size, kind}` row to `@params` in `can_ex`
   `lib/commutation.ex`.

That's it — the profile is intentionally table-driven so extension is one row
per layer.
