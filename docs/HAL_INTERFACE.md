# HAL Interface Contract

**Canonical reference for how `servo4257-rs` (firmware) talks to
`n32l4xx-hal` (HAL) and `n32l4-pac` (PAC).**

This is the single source of truth for the firmware<->library boundary. When a
later session reasons about *any* HAL or PAC interaction -- call shape, a
register field, an ISR usage rule, a peripheral's instance model -- it refers
here first, and updates this doc when the contract changes. If something below
conflicts with memory or with an older doc, this doc wins (and the older doc
should be corrected to point here).

## Provenance discipline (read this first)

The HAL is a port of `guineawheek/n32g4xx-hal` (STM32-F4/L4-shaped). The PAC is
generated from the NSING N32L40x vendor SVD, normalized via svdtools.

The danger this creates, and the rule that follows from it:

> Inherited values are guilty until proven innocent. Any hardware-meaning
> constant that rode in from the n32g4/STM32 source -- a field encoding, a
> pin's alternate-function number, a trigger-source code, a DMA request map --
> is UNVERIFIED until checked against the N32L40x User Manual or Datasheet.
> The n32g4 PAC and the STM32 HAL are starting points to verify against the
> docs, NOT sources of truth.

This is not hypothetical. The ADC external-trigger enum was inherited verbatim
from `stm32f4xx-hal` (a 4-bit EXTSEL layout), compiled cleanly, and was wrong
for this part: the N32L40x has two distinct 3-bit trigger maps (EXTRSEL /
EXTJSEL) where the same code selects different timer events. It was caught only
by reading UM Tables 17-5/17-6, not by any build or test.

The correct fix pattern, used throughout: enrich the SVD with UM-verified
enumeratedValues (`n32l4-pac/tools/svd_patch.yaml`), regenerate, and have the
HAL use the generated PAC variant (`.variant(...)`) instead of a hand-rolled
enum written through raw `.bits()`. That puts the meaning in the type system,
where a wrong choice is a compile error rather than a silent mis-write.

## Library layout & how the firmware depends on it

- `n32l4-pac` -- register access. Dual-device single crate; select the device
  with a feature (`n32l403` OR `n32l406`, mutually exclusive -- different
  register maps). Re-exported by the HAL as `crate::pac`.
- `n32l4xx-hal` -- the blocking `embedded-hal` HAL the firmware builds on.
  Depends on the PAC as a path dependency.
- Both pin the SAME nightly as the firmware (`nightly-2026-06-07`); the HAL
  uses 8 unstable features, so the pin is required, not a preference.
- The firmware accesses peripherals via the blocking HAL or raw PAC from async
  (embassy) context. There is intentionally NO async HAL -- control loops are
  ISRs, so an async HAL would be a much larger project for no hot-path benefit.

Per-board selection (one firmware, two boards) is a cargo feature that picks
the PAC device + calibration constants in `src/boards/`; everything else shared.

## Post-refactor call shape (what changed, what to write)

The HAL was rewritten from the n32g4 baseline onto the STM32-F4/L4 model. These
are the CURRENT shapes; older docs/examples showing the other form are stale.

### GPIO alternate functions -- const-generic, not method-per-AF

The Remap model is GONE. Alternate functions are selected by a const-generic
AF number:

    // CORRECT (current):
    let tx = gpioa.pa9.into_alternate::<7>();            // AF7
    let od = gpiob.pb8.into_alternate_open_drain::<4>(); // AF4, open-drain

    // STALE (pre-refactor, do NOT use):
    //   gpioa.pa9.into_alternate_af7()
    //   anything mentioning Remap / .remap()

The pin carries the AF in its type: `Pin<P, N, Alternate<A, PushPull>>`.
Peripheral constructors are bounded on the right `Alternate<A, _>`, so a wrong
AF number is a compile error -- but only if the AF bound in the HAL's pin table
is itself correct (see Verify-on-contact).

### Single-instance peripherals -- no numeric suffix

The N32L40x has one ADC and one bxCAN:

    let adc = Adc::adc(dp.ADC, true, AdcConfig::default()); // pac::Adc, not ADC1
    let can = Can::new(dp.CAN);                             // single bxCAN

There is no ADC1/ADC2/CAN1. Docs referring to numbered instances describe
STM32/n32g4, not this part.

### DMA -- clustered `Ch` model

DMA channels are a uniform cluster (`dma::Ch`, accessed `ch(n)`), not 40 flat
per-channel register types. Deliberate decision during the port (dissolved a
type-system wall, matches ST's own SVD shape). Do NOT reintroduce per-channel
register types. The DMA request mux (`chmap.rs`) is currently n32g4-gated and
UNBUILT for n32l4 -- if the firmware needs DMA request mapping, that is new UM
territory (the N32L40x mechanism differs from n32g4's CHMAPEN).

## ISR usage contract (the hot path) -- MANDATORY

The current loop runs in a top-priority ISR with a hard 50 us / 3200-cycle
budget at 64 MHz. The HAL has blocking paths that are FATAL there:

1. Never call the spin-waiting ADC paths from an ISR. `Adc::convert()`,
   `wait_for_regular_conversion_sequence()`, and
   `wait_for_injected_conversion_sequence()` busy-loop on status bits and
   `panic!` if no conversion is in flight. In a 20 kHz ISR a busy-loop is a
   blown deadline and a panic is dead silicon.

2. Current sampling uses the injected group, hardware-triggered. Arm an
   injected conversion triggered off the PWM advanced timer (see ADC
   triggering), enable the injected EOC interrupt, and read with
   `adc.injected_sample(seq)` in the handler. The injected path has per-channel
   offset registers (`set_injected_offset` / `shift_injected_offset`) suited to
   shunt-bias calibration.

3. No interrupt masking beyond the current-loop slack. Any `critical-section`
   / `interrupt::free` -- in HAL, embassy, or firmware -- that masks longer than
   the current-loop slack (~tens of us) breaks the two-tier guarantee. The csp
   setpoint path uses the lock-free double-buffer/seqlock specifically to avoid
   masking. An audit of every dependency's critical sections against this bound
   is still OPEN (see DECISIONS.md) and must happen before the two-tier timing
   guarantee can be trusted.

## ADC triggering (the current-loop linchpin)

The control design hinges on ADC conversions triggered by the PWM timer. The
machinery exists and the trigger codes are UM-verified:

- Two distinct trigger maps, each its own 3-bit field and enum:
  - `config::RegularTrigger` -> EXTRSEL (CTRL2[19:17]), UM Table 17-5, regular.
  - `config::InjectedTrigger` -> EXTJSEL (CTRL2[14:12]), UM Table 17-6, injected.
- The SAME code means different events in each table (e.g. code 0 = TIM1_CC1
  regular vs TIM1_TRGO injected). Different types on purpose -- you cannot use a
  regular-trigger value on the injected field.
- Set with:
  - `adc.set_regular_channel_external_trigger((edge, RegularTrigger::...))`
  - `adc.set_injected_channel_external_trigger((edge, InjectedTrigger::...))`
- `AdcConfig` carries only the REGULAR trigger (auto-applied by `apply_config`).
  The injected trigger is set imperatively after init; not part of `AdcConfig`.

For FOC current sampling the natural choice is an injected group triggered off
the H-bridge advanced timer's trigger output -- e.g. `InjectedTrigger::Tim1Trgo`
(code 0 on EXTJSEL). Confirm the exact timer event against your PWM config.

## Known-verified facts (checked against UM/Datasheet, safe to rely on)

- ADC EXTRSEL / EXTJSEL trigger codes -- UM Tables 17-5 / 17-6. Enriched in the
  SVD; HAL uses generated variants.
- ADC `Resolution` -- UM ADC_CTRL3.RES[1:0]: 00=6-bit, 01=8-bit, 10=10-bit,
  11=12-bit. HAL enum matches. (Still a hand-enum via `.bits()`; correct today,
  candidate for SVD hardening.)
- ADC channel map (pin -> channel) -- Datasheet Table 3-1 + UM ch.17, via
  `n32l4xx-hal/tools/gpio_af/adc_channel_um.tsv`. Single ADC; PA0-PA7->1-8,
  PB0/1->9/10, PC0-C5->11-16, temp=17, vref=18.
- GPIO alternate-function table -- UM AF tables via the
  `tools/gpio_af/af_table_um.tsv` pipeline, with firmware-critical peripherals
  (SPI1, USART1, CAN, TIM1) manually re-verified. "Never guess a missing AF" is
  enforced by the generator.
- Five vendor-SVD bugs fixed during the port (see PORT_STATUS.md / PAC patch
  comments).

## Verify-on-contact (NOT yet proven; check before trusting)

Per the provenance rule, confirm against the doc / on silicon first:

- PWM pin<->AF for the specific MKS gate-driver pins. The HAL's TIM channel pin
  table is datasheet-derived, but WHICH TIM1/TIM8 channels wire to the EG3013
  gate drivers on the 42D/57D is a BOARD fact -- cross-check the MKS schematics.
  Datasheet-correct != board-correct.
- Dead-time tick math at 64 MHz. `pwm.rs` dead-time helper comment references a
  200 MHz M7; recompute the register value for 64 MHz.
- Injected multi-channel sequence indexing. `configure_injected_channel` uses
  right-aligned slot arithmetic (`3 - jlen + sequence`) inherited from STM32.
  Plausibly correct but UNTESTED for multi-channel injected sequences;
  unit-test the slot mapping when sampling >1 phase current via injection.
- Anything else inherited. A broad sweep found the trigger enum was the only
  wrong-value instance of the inherited-enum-via-`.bits()` pattern, but new code
  paths may surface more. Default to verifying.

## Cross-references

- `docs/ARCHITECTURE.md` -- control architecture & timing budget (the *why*;
  this doc is the *how* of the library boundary).
- `docs/HARDWARE.md` -- board/MCU/peripheral facts from schematics.
- `docs/DECISIONS.md` -- settled decisions; the interrupt-masking audit OPEN
  item is load-bearing for this doc's ISR contract.
- `n32l4xx-hal/PORT_STATUS.md` -- port completion state & per-driver notes.
- `n32l4-pac/tools/svd_patch.yaml` -- where every UM-verified enum enrichment
  lives. The place to add the next one.
- `n32l4xx-hal/tools/gpio_af/` -- the AF/channel extraction pipeline + README.
