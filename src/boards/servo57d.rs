//! MKS SERVO57D board implementor.
//!
//! MCU: N32L406CBL7 (48-pin). Shares the entire motor-drive/encoder/CAN pin
//! map with the 57D (see super::hw_map); differs only in MCU/package, the
//! electrical constants below, and it ADDS USART3/RS485 + extra opto IO
//! that the 48-pin package breaks out.
//!
//! Stage 2 complete: all four Board methods are wired against the HAL --
//! TIM3 4-channel PWM (apply_coil_voltages), injected ADC current sense
//! (read_coil_currents), MT6816/SPI1 encoder (rotor_angle), and the nEN
//! output-stage enable (set_output_enable). See docs/HAL_INTERFACE.md.

use crate::board::{Board, CoilCommand, CoilCurrent, RotorAngle};
use n32l4xx_hal::pac;
use n32l4xx_hal::prelude::*;
use n32l4xx_hal::pwm::{Pwm, PwmExt};
use embedded_hal_02::PwmPin;
use n32l4xx_hal::gpio::{Alternate, Output, PushPull,
    gpioa::{PA6, PA7}, gpiob::{PB0, PB1, PB6, PB7}};
use n32l4xx_hal::spi::{Spi, Mode, Phase, Polarity};
use n32l4xx_hal::adc::{Adc, config::{AdcConfig, InjectedSequence, InjectedTrigger, SampleTime, TriggerMode}};
use n32l4xx_hal::pwm::{C1, C2, C3, C4, ComplementaryImpossible};
use n32l4xx_hal::can::Can as HalCan;
use bxcan::{filter::Mask32, Fifo, Frame as BxFrame, Id, StandardId};

/// No-init RAM cell the bootloader reads to decide "stay in boot" (app.x
/// `_boot_flag`); the magic value that means stay. Shared with the bootloader.
const BOOT_FLAG: *mut u32 = 0x2000_5FF8 as *mut u32;
const FLAG_STAY_IN_BOOT: u32 = 0xB007_57A4;

/// bxCAN BTR for 500 kbps at PCLK1 = 16 MHz (this board's full-speed clock).
/// tq = PCLK1/(BRP+1) = 16M/2 = 8 MHz → 125 ns; bit = SYNC(1)+TSEG1(13)+TSEG2(2)
/// = 16 tq = 2 µs = 500 kbps, sample point 87.5%. bxCAN stores each field −1:
/// BRP=1, TS1=12, TS2=1, SJW=0 → (1<<20)|(12<<16)|1 = 0x001C_0001.
pub const BTR_500K_PCLK16: u32 = 0x001C_0001;

/// PWM carrier frequency for the H-bridges. TODO: confirm against the current
/// loop rate; 20 kHz is the design target (above audible, fits the budget).
const PWM_HZ: u32 = 20_000;

/// MT6816 SPI clock. Datasheet TSCK min 64 ns => ~15 MHz ceiling; run
/// conservative. Mode 3 (CPOL=1/IdleHigh, CPHA=1/2nd edge).
const ENCODER_HZ: u32 = 4_000_000;

/// Output-stage enable polarity on the nEN line (PB7).
/// VERIFY: schematic -- does nEN enable the stage when driven LOW or HIGH?
/// The `n` prefix and MKS convention suggest ACTIVE-LOW (drive low to
/// enable), but this is UNCONFIRMED. init() commands the stage DISABLED
/// regardless, so a wrong value here can only fail safe (stays off), never
/// energize the bridge unexpectedly. Flip this once confirmed on the sheet.
const EN_ACTIVE_LOW: bool = true;

// The four TIM3 PWM channels on this board, in coil order.
type Ch1 = Pwm<pac::Tim3, C1, ComplementaryImpossible, n32l4xx_hal::pwm::ActiveHigh, n32l4xx_hal::pwm::ActiveHigh>;
type Ch2 = Pwm<pac::Tim3, C2, ComplementaryImpossible, n32l4xx_hal::pwm::ActiveHigh, n32l4xx_hal::pwm::ActiveHigh>;
type Ch3 = Pwm<pac::Tim3, C3, ComplementaryImpossible, n32l4xx_hal::pwm::ActiveHigh, n32l4xx_hal::pwm::ActiveHigh>;
type Ch4 = Pwm<pac::Tim3, C4, ComplementaryImpossible, n32l4xx_hal::pwm::ActiveHigh, n32l4xx_hal::pwm::ActiveHigh>;

/// SERVO57D board handle. Owns the configured peripherals.
pub struct Servo57D {
    // Coil A = (a_plus CH1, a_minus CH2); Coil B = (b_plus CH3, b_minus CH4).
    a_plus: Ch1,
    a_minus: Ch2,
    b_plus: Ch3,
    b_minus: Ch4,
    max_duty: u16,
    adc: Adc<pac::Adc>,
    spi: Spi<pac::Spi1>,
    enc_cs: PB6<Output<PushPull>>,
    en: PB7<Output<PushPull>>,
    can: bxcan::Can<HalCan<pac::Can>>,
}

impl Servo57D {
    /// Construct and configure board peripherals from the raw device.
    /// Stage 2: clocks + TIM3 PWM only; other peripherals added incrementally.
    pub fn init(mut dp: pac::Peripherals) -> Self {
        // Full-speed clock tree: 8 MHz HSE crystal -> PLL -> 64 MHz SYSCLK
        // (SYSCLK_MAX). PCLK1 pinned to its 16 MHz ceiling, so the APB1
        // prescaler is 4 and the APB1 timers (TIM3, the bridge PWM) clock at
        // 2*PCLK1 = 32 MHz -> 32M/20kHz = 1600 PWM steps. This replaces the
        // bare reset clock (MSI 4 MHz), which the PWM/ADC timing and the
        // current-loop cycle budget all assume is gone.
        let rcc = dp.rcc.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8_000_000.Hz())
            .sysclk(64_000_000.Hz())
            .pclk1(16_000_000.Hz())
            .freeze();

        let gpioa = dp.gpioa.split();
        let gpiob = dp.gpiob.split();

        // TIM3 CH1..4 on PA6/PA7/PB0/PB1, AF2 (see super::hw_map::pwm).
        let pa6: PA6<Alternate<2>> = gpioa.pa6.into_alternate::<2>();
        let pa7: PA7<Alternate<2>> = gpioa.pa7.into_alternate::<2>();
        let pb0: PB0<Alternate<2>> = gpiob.pb0.into_alternate::<2>();
        let pb1: PB1<Alternate<2>> = gpiob.pb1.into_alternate::<2>();

        let (mut a_plus, mut a_minus, mut b_plus, mut b_minus) =
            dp.tim3.pwm((pa6, pa7, pb0, pb1), PWM_HZ.Hz(), &clocks);

        let max_duty = a_plus.get_max_duty();

        // Start centered (zero differential = no coil current) and enabled.
        let mid = max_duty / 2;
        a_plus.set_duty(mid);
        a_minus.set_duty(mid);
        b_plus.set_duty(mid);
        b_minus.set_duty(mid);
        a_plus.enable();
        a_minus.enable();
        b_plus.enable();
        b_minus.enable();

        // ---- Current-sense ADC (injected group, TIM3-triggered) ----
        // currentA = PA2 = ch3, currentB = PA1 = ch2 (see hw_map::adc).
        let cur_a = gpioa.pa2.into_analog();
        let cur_b = gpioa.pa1.into_analog();

        let mut adc = Adc::adc(dp.adc, true, AdcConfig::default());
        adc.calibrate();
        // Injected sequence: slot One = coil A (PA2), slot Two = coil B (PA1).
        adc.configure_injected_channel(&cur_a, InjectedSequence::One, SampleTime::Cycles_7p5);
        adc.configure_injected_channel(&cur_b, InjectedSequence::Two, SampleTime::Cycles_7p5);
        // Trigger the injected group off the bridge timer.
        // TODO(hot-loop): Tim3Cc4 shares CH4 with the phaseB2 PWM output --
        // sample timing is coupled to that compare value. Revisit when the
        // current-loop ISR is built (reserve a CC channel for triggering, or
        // confirm the sample point is acceptable). EXTJSEL has no TIM3_TRGO
        // option on this part (UM Table 17-6), so CC4 is the only TIM3 source.
        adc.set_injected_channel_external_trigger((TriggerMode::RisingEdge, InjectedTrigger::Tim3cc4));
        adc.enable();

        // ---- Encoder: MT6816 on SPI1 (see hw_map::encoder) ----
        // PB3=SCK, PB4=MISO, PB5=MOSI @ AF; CS=PB6 driven as GPIO (the MT6816
        // frames each transfer with CSN, so we toggle it manually).
        let sck = gpiob.pb3.into_alternate::<1>();
        let miso = gpiob.pb4.into_alternate::<1>();
        let mosi = gpiob.pb5.into_alternate::<0>();
        let mut enc_cs = gpiob.pb6.into_push_pull_output();
        let _ = enc_cs.set_high(); // idle high (CSN inactive)
        // MT6816 = SPI mode 3.
        let mode = Mode { polarity: Polarity::IdleHigh, phase: Phase::CaptureOnSecondTransition };
        let spi = dp.spi1.spi((sck, miso, mosi), mode, ENCODER_HZ.Hz(), &clocks, &mut dp.afio);

        // ---- Output-stage enable (nEN = PB7, see hw_map::gpio) ----
        // Come up in the INACTIVE (disabled) state regardless of polarity.
        let en = gpiob.pb7.into_push_pull_output();

        // ---- CAN (PA11 RX / PA12 TX, AF1; see hw_map::can) ----
        // 500 kbps at the 16 MHz PCLK1. Accept-all filter — COB-ID filtering is
        // software, above this layer. Brought up here (the drive owns its bus):
        // telemetry/diagnostics during bring-up, PDO/SDO in the running app.
        let hal_can = HalCan::new(dp.can);
        hal_can.assign_pins((gpioa.pa11, gpioa.pa12));
        let mut can = bxcan::Can::builder(hal_can)
            .set_bit_timing(BTR_500K_PCLK16)
            .leave_disabled();
        can.modify_filters()
            .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
        nb::block!(can.enable_non_blocking()).ok();

        let mut board = Self {
            a_plus, a_minus, b_plus, b_minus, max_duty, adc, spi, enc_cs, en, can,
        };
        // Fail-safe: explicitly command the output stage OFF at boot.
        board.set_output_enable(false);
        board
    }

    /// Map a signed coil command to a (plus, minus) duty pair using
    /// **sign-magnitude** drive: energize one half-bridge with |v| and hold the
    /// other at GND, so the coil is driven unipolar-switched in the direction
    /// set by the sign.
    ///
    /// NOTE: this replaced locked-antiphase (both half-bridges at mid ± v/2).
    /// On hardware, locked-antiphase failed to reverse the coil — both bridges
    /// chopping ±Vbus draws large ripple current that pins a current-limited
    /// supply and collapses the net drive, so the motor only ever saw two
    /// positions and twitched. Sign-magnitude (verified with `bridgetest`)
    /// commutates cleanly. All four half-bridges are healthy.
    #[inline]
    fn split_duty(&self, v: CoilCommand) -> (u16, u16) {
        let mag = ((v.unsigned_abs() as u32 * self.max_duty as u32) / (i16::MAX as u32)) as u16;
        if v >= 0 {
            (mag, 0)
        } else {
            (0, mag)
        }
    }

    /// Diagnostic: set the four half-bridge PWM duties directly (a+, a−, b+, b−),
    /// bypassing the locked-antiphase mapping — for bridge/hardware bring-up.
    pub fn set_raw_duties(&mut self, ap: u16, am: u16, bp: u16, bm: u16) {
        self.a_plus.set_duty(ap);
        self.a_minus.set_duty(am);
        self.b_plus.set_duty(bp);
        self.b_minus.set_duty(bm);
    }

    /// Full-scale PWM duty (100%).
    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }

    /// Best-effort CAN transmit for telemetry / diagnostics: non-blocking, drops
    /// the frame if all TX mailboxes are busy (telemetry is lossy by design).
    /// `id` is an 11-bit standard COB-ID; `data` is 0..=8 bytes.
    pub fn telemetry(&mut self, id: u16, data: &[u8]) {
        if let Some(sid) = StandardId::new(id) {
            let d = bxcan::Data::new(data).unwrap_or_else(bxcan::Data::empty);
            let _ = self.can.transmit(&BxFrame::new_data(sid, d));
        }
    }

    /// Poll CAN for the CiA-302 "enter update" command — an SDO access to
    /// `0x1F51` (Program Control) on the node's RX COB-ID `0x601`. On a match:
    /// **safe the output stage** (coils to zero, nEN off), latch the
    /// stay-in-boot flag, and reset into the bootloader for an over-CAN reflash.
    /// Returns normally when no such frame is pending; never returns on a match.
    ///
    /// Call this every main-loop iteration so the over-CAN dev loop stays live —
    /// it is the app's only escape hatch back to the bootloader without SWD.
    pub fn poll_reflash(&mut self) {
        let f = match self.can.receive() {
            Ok(f) => f,
            Err(_) => return, // WouldBlock / overrun: nothing to act on
        };
        let id = match f.id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(_) => return,
        };
        let data = match f.data() {
            Some(d) => d,
            None => return, // remote frame
        };
        if id == 0x601 && data.len() >= 3 && data[1] == 0x51 && data[2] == 0x1F {
            // Safe the bridge before handing control to the bootloader.
            self.apply_coil_voltages(0, 0);
            self.set_output_enable(false);
            unsafe { core::ptr::write_volatile(BOOT_FLAG, FLAG_STAY_IN_BOOT) };
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

impl Board for Servo57D {
    const SHUNT_OHMS: f32 = 0.02; // R10/R22 = 0.02 ohm, verified on power sheet
    const MAX_CURRENT_MA: u32 = 5200; // TODO confirm from datasheet/thermal
    const NAME: &'static str = "MKS SERVO57D (N32L406)";
    const HAS_RS485: bool = true; // 48-pin package breaks out USART3

    fn apply_coil_voltages(&mut self, v_a: CoilCommand, v_b: CoilCommand) {
        let (ap, am) = self.split_duty(v_a);
        let (bp, bm) = self.split_duty(v_b);
        self.a_plus.set_duty(ap);
        self.a_minus.set_duty(am);
        self.b_plus.set_duty(bp);
        self.b_minus.set_duty(bm);
    }

    fn read_coil_currents(&self) -> (CoilCurrent, CoilCurrent) {
        // Latched injected results; does NOT spin-wait (ISR-safe per
        // docs/HAL_INTERFACE.md). Slot One = coil A, slot Two = coil B.
        let i_a = self.adc.injected_sample(InjectedSequence::One);
        let i_b = self.adc.injected_sample(InjectedSequence::Two);
        (i_a, i_b)
    }

    fn rotor_angle(&mut self) -> RotorAngle {
        // MT6816 angle read (datasheet 7.6.5): 16-bit frame returns
        //   [Angle<13:6>][Angle<5:0> No_Mag PC]
        // i.e. word = (angle14 << 2) | (no_mag << 1) | parity.
        // NOTE(hot-loop): this is a BLOCKING SPI transfer (~tens of SCK at
        // 4 MHz). Per docs/HAL_INTERFACE.md it must NOT be called inline from
        // the current-loop ISR -- pipeline it (kick off / read next tick).
        // TODO: verify even-parity (bit0) and No_Mag_Warning (bit1) and
        // reject/flag bad reads; the read command byte may need adjusting to
        // the consolidated angle-read opcode once confirmed on hardware.
        // MT6816 register read: two frames. Each frame = [0x80|addr, dummy];
        // the register byte comes back in the 2nd byte. Reg 0x03 = Angle<13:6>,
        // reg 0x04 = Angle<5:0> in bits 7:2 (bit1 No_Mag, bit0 parity).
        let mut hi = [0x83u8, 0x00u8];
        let _ = self.enc_cs.set_low();
        let _ = self.spi.transfer_in_place(&mut hi);
        let _ = self.enc_cs.set_high();

        let mut lo = [0x84u8, 0x00u8];
        let _ = self.enc_cs.set_low();
        let _ = self.spi.transfer_in_place(&mut lo);
        let _ = self.enc_cs.set_high();

        let angle14 = ((hi[1] as u16) << 6) | ((lo[1] as u16) >> 2);
        // Map 14-bit (0..16384) to full-circle u16 (0..=65535): << 2.
        angle14 << 2
    }

    fn set_output_enable(&mut self, enabled: bool) {
        // Drive nEN to its active level when enabling. Polarity is isolated in
        // EN_ACTIVE_LOW (see its doc -- still to be confirmed on the schematic).
        let drive_high = enabled ^ EN_ACTIVE_LOW;
        if drive_high {
            let _ = self.en.set_high();
        } else {
            let _ = self.en.set_low();
        }
    }
}
