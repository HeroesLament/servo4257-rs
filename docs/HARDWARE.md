# Hardware Reference

Consolidated from the official MKS schematics
(MKS_SERVO42D_CAN_V1.0_003 and MKS_SERVO57D_CAN_V1.1_001) and the NSING
N32L40x datasheet. Verified, not assumed.

## MCU

| Board | MCU            | Pkg    | Notes                          |
|-------|----------------|--------|--------------------------------|
| 42D   | N32L403KBQ7    | LQFP48 | 128KB flash (KB suffix)        |
| 57D   | N32L406CBL7    | LQFP48 | adds RS485 + opto I/O          |

- Core: ARM Cortex-M4F @ 64 MHz, FPU + DSP, MPU, 2KB I-cache, 24KB SRAM.
- NSING = Nationstech international brand (same vendor).
- Single core. No SVD distinction between 403/406 at register level — vendor
  treats N32L40X as one family; per-part differences are peripheral instances
  + trim, not register map.
- SWD programming header J5 broken out (3V3/GND/SWCLK/SWDIO) — reflashable
  without desoldering.

## Power stage (both boards: 2 H-bridges, stepper = 2-phase PMSM)

| Part            | 42D         | 57D                  |
|-----------------|-------------|----------------------|
| Gate drivers    | EG3013 x4   | EG3013 x4            |
| FETs            | AP4008QD    | AP050N03Q x8 (disc.) |
| Shunts          | 0.05 ohm    | 0.02 ohm             |
| Fuse            | 63V/3A      | 32V/5A               |
| Current amps    | GS8632      | GS8632               |

The shunt delta (0.05 vs 0.02) is the key calibration difference — sets the
current-sense scaling. 57D is the higher-current NEMA23 board.

## Shared signal chain

- Encoder: MT6816 14-bit magnetic angle sensor, SPI
  (SPI_CS/CLK/MISO/MOSI). 16384 counts/rev equivalent resolution domain.
- Current sense: 2 shunts -> GS8632 amps -> ADC (currentA, currentB).
- CAN: TJA1051T/3 transceiver + on-chip bxCAN. Classical CAN only (8-byte
  frames). 120 ohm termination present; 57D has a termination DIP switch.
- 57D extras: RS485 (second/third UART), opto-isolated digital I/O
  (M_IN1/2, M_OUT1/2 via EL357N/ELQ3H7).

## FOC-relevant peripherals (from N32L406 SVD)

- Timers TIM1..TIM9. **TIM1 and TIM8 are advanced-control** (complementary
  outputs + dead-time via BKDT register) -> two adv timers available; one for
  the H-bridge PWM, one spare for ADC trigger / embassy-time driver.
- ADC: single, 12-bit, ~4.4-4.5 Msps. SINGLE adc -> phase currents sampled
  SEQUENTIALLY (not simultaneous dual-ADC). Fast enough, but the current-loop
  timing model must account for the two samples not being co-incident.
- Dead-time: hardware (BKDT) -> no software dead-time needed for the bridges.

## Constraints that shaped the firmware

- Single ADC -> sequential current sampling.
- Classical CAN (no FD) -> 8-byte PDOs -> may need multiple PDO mappings/cycle.
- Single core -> concurrency via NVIC priorities (-> two-tier interrupts).
- 64 MHz M4F -> cascade must be decimated; FPU/DSP make FOC feasible.

## Stock CAN command protocol (for reference / interop, NOT what we implement)

Vendor protocol is NOT CANopen. Standard 11-bit frames; CAN ID = motor slave
address (0=broadcast, 1..n configurable). Payload: [function-code, data...,
checksum]. Checksum = (ID + bytes) & 0xFF (an additive 8-bit sum that INCLUDES
the CAN ID, despite being labelled CRC). Profiled point-to-point moves only —
which is exactly why we replace it (see DECISIONS.md). Reference host-side
implementations: salvamarce/MKS_SERVO42D (Arduino),
DzymFardreamer/mks-servo-can (Python).
