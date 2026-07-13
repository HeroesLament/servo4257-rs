# The over-CAN development loop

This board can be reflashed entirely over the CAN bus — no SWD probe, no BOOT0
jumper — once the CANopen bootloader is resident. This document describes how
that loop works end to end, the on-wire protocol, the flash layout and the CRC
gate that makes it safe, the RAM marker maps you can read over SWD while
debugging, the one-call workflow from the Elixir master, and how to recover a
board if something goes wrong.

The design follows CiA-302 (program download) with a CiA-402 drive application
on top. The bootloader is the CANopen managed node; the PC-side `CANopen.SDO.Master`
in `can_ex` is the manager.

## The loop in one paragraph

A running application, when it receives the CiA-302 "stop program" command over
CAN, latches a stay-in-boot flag in no-init RAM and resets itself. The
bootloader runs, sees the flag, and stays resident, bringing up CAN and serving
a segmented SDO download into flash. When a complete image has been received it
stamps a CRC record for the new application and resets. On that reset the boot
path re-runs, validates the application's CRC, and — because the CRC is now
valid — jumps into the freshly-flashed application. The whole cycle is driven by
a single `Master.reflash/2` call and touches no debug hardware.

## On-wire protocol

The node is CANopen node 1. SDO COB-IDs follow the standard mapping: the manager
writes to the node on `0x600 + node_id` (`0x601`) and the node replies on
`0x580 + node_id` (`0x581`). Two CiA-302 objects are used:

- **`0x1F50` Program Data** — the firmware image is streamed here with a
  segmented SDO download (initiate, then toggle-alternated 7-byte data segments,
  each acknowledged before the next is sent). The node defers each acknowledgement
  until its flash write completes, so the manager physically cannot outrun the
  flash.
- **`0x1F51` Program Control** — an expedited SDO write of `0` ("stop program")
  to sub-index 1 tells the running application to reboot into its bootloader.

The enter-update write is fire-and-forget: the application resets before it can
send an SDO response, so the manager does not wait for an acknowledgement.

## Reliability

Two layers cooperate. On the wire, the MCP2515 runs in **One-Shot Mode** (OSM):
each frame is transmitted exactly once with no hardware auto-retransmit, so
`send_frame` returns deterministically. Reliability is provided one layer up: the
SDO master retransmits a segment if its acknowledgement does not arrive within
the timeout, and the node is **idempotent to duplicate segments** — it re-acks a
repeated segment without re-writing flash. A lost segment or a lost ack both
recover by re-sending the identical segment. OSM is the default in
`CAN.Transport.MCP2515.open/1`; pass `one_shot: false` for a contended
multi-master bus where hardware retransmit is preferable.

RX-overflow recovery is automatic: on the MCP2515 erratum where `RX0IF` fails to
set despite a message sitting in `RXB0`, `Ch347.MCP2515.receive_frame/1` detects
`EFLG.RX0OVR`, reads the buffer, and clears the overflow.

## Flash layout and the CRC gate

```
0x0800_0000  bootloader        (layout-boot)
0x0800_4000  application base   APP_BASE
0x0801_F800  AppMeta record     META_BASE  (page-aligned, PAGE_SIZE = 2048)
```

`AppMeta` is a 32-byte record: a magic (`"M23N"`), the application's CRC-32, its
length, and version fields. The CRC is **CRC-32/ISO-HDLC** (the zlib/Ethernet
polynomial), which matches Elixir's `:erlang.crc32` and Python's
`binascii.crc32` byte-for-byte, so the host and node agree without a custom
implementation.

The boot decision is: if a populated `AppMeta` is present, the bootloader
verifies the application CRC and jumps only if it matches; if no meta record is
present (e.g. a plain SWD-flashed app during bring-up), it falls back to a
structural heuristic (initial SP points into RAM, reset vector into app flash).
It stays in the bootloader if the stay-in-boot flag is set **or** the application
is invalid. This is the gate that makes an interrupted or corrupt download safe:
a half-written image fails its CRC and the bootloader simply stays resident and
serviceable rather than jumping into garbage.

The download writes the image page-by-page, then in `finalize` erases the
meta page and writes the fresh `AppMeta`. (Historical note: the meta-page erase
must be `PAGE_SIZE`-aligned — erasing only `AppMeta::SIZE` bytes returns
`NotAligned` and the meta stamp silently never lands, leaving the CRC gate
dormant. That bug is fixed; the erase spans a full page.)

## RAM marker maps

Both the bootloader and the application publish decision/progress words to fixed
no-init RAM so you can watch the loop over SWD without a debugger halt.

Bootloader status block at `0x2000_0000`:

| Word | Address       | Meaning |
|------|---------------|---------|
| [0]  | `0x2000_0000` | `0xB007_0001` — bootloader ran |
| [1]  | `0x2000_0004` | boot-flag value read from `_boot_flag` |
| [2]  | `0x2000_0008` | application initial SP (word @ `APP_BASE`) |
| [3]  | `0x2000_000C` | application reset vector (word @ `APP_BASE+4`) |
| [4]  | `0x2000_0010` | decision: `0x0000_3001` jump-to-app, `0x057A_4000` stay-in-boot |
| [5]  | `0x2000_0014` | meta state: `0` absent (heuristic used), `1` present+valid, `2` present+BAD-CRC |
| [6]  | `0x2000_0018` | CAN/service state: `0xCA00_0000` up, `0xCAD0_xxxx` download complete (len in low bits), `0xCAAB_0000` aborted |
| [7]  | `0x2000_001C` | live download progress code |

The application (the `appstub` handoff test image) uses a status block at
`0x2000_4000`:

| Word | Address       | Meaning |
|------|---------------|---------|
| [0]  | `0x2000_4000` | `0x0A99_0001` running, `0xB007_0000` handed off to bootloader |
| [6]  | `0x2000_4018` | `0xCA00_0000` — CAN up |
| [7]  | `0x2000_401C` | received-frame count |

The stay-in-boot flag lives in no-init RAM at `_boot_flag` = `0x2000_5FF8`; the
magic value is `FLAG_STAY_IN_BOOT` = `0xB007_57A4`. The bootloader reads and
clears it on entry so a subsequent normal reset does not re-trap.

## Clocking

The bootloader's download service clocks from the external 8 MHz crystal (HSE):
`sysclk` 8 MHz, `hclk` 4 MHz, `pclk1` 4 MHz, giving CAN a precise 500 kbps. The
jump-to-app path leaves the reset clock untouched. Note the reset clock is
**MSI at 4 MHz** (not HSI 16 MHz) — relevant when timing anything before
`freeze()`. The Embassy time driver derives its prescaler from the frozen HAL
`Clocks` via `rt::init_time_driver_from_clocks`, so the 1 MHz tick stays exact
across clock configs; the raw `rt::init_time_driver(hz)` remains for pre-`freeze()`
smoke tests where no `Clocks` exists yet.

## The workflow

From an `iex -S mix` session in `can_ex`, the whole loop is one call:

```elixir
alias CAN.Transport.MCP2515
alias CANopen.SDO.Master

image = File.read!("/path/to/app.bin")
{:ok, bus} = MCP2515.open([])              # OSM + normal mode, no register pokes
m = Master.new(MCP2515, bus, 1, timeout: 800)
:ok = Master.reflash(m, image)             # enter-update -> settle -> download
MCP2515.close(bus)
```

`reflash/3` sends the enter-update write, waits `:settle_ms` (default 500) for
the application to reset and the bootloader to bring up CAN, then downloads. The
bootloader stamps the CRC and self-resets; the boot path validates and jumps.
No SWD, no BOOT0.

If the node is already sitting in its bootloader (e.g. after a manual
stay-in-boot), skip the enter-update and call `Master.download(m, image)`
directly — there is no running app to reset.

The image is a raw binary, not an ELF. Produce it with, e.g.:

```
cargo build --release --bin servo57d --features board-57d,layout-app
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/servo57d app.bin
```

## Reading markers with probe-rs

The N32L406 needs a custom chip description. Reads (no halt) look like:

```
probe-rs read b32 0x2000_0000 8 \
  --chip N32L406CB --chip-description-path n32l40x.target.yaml   # bootloader block
probe-rs read b32 0x2000_4000 8 \
  --chip N32L406CB --chip-description-path n32l40x.target.yaml   # app block
```

## Recovery

If a board wedges (bad bootloader, or a download that somehow bricked the boot
path), recover over SWD:

1. Pull **BOOT0 high** and reset to enter the on-chip ROM bootloader (or use
   probe-rs directly if SWD is still responsive).
2. Force stay-in-boot by writing the flag over SWD, then reset:
   `probe-rs write b32 0x2000_5FF8 0xB007_57A4 …` then `probe-rs reset …`.
3. Reflash the bootloader with probe-rs, then resume the over-CAN loop.

The ST-Link clones in use are flaky; an intermittent `SwdApWdataError` or
`JtagNoDeviceConnected` is usually cleared by re-plugging the dongle. A read
issued immediately after a self-reset can also transiently error simply because
the chip is mid-reset — retry once before treating it as a real fault.

## Provisioning new boards

A fresh 42D/57D board joins the over-CAN fleet after a one-time SWD flash of the
bootloader. After that, every subsequent update is over CAN via the loop above.
