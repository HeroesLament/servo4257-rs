//! MKS SERVO42D board implementor.
//!
//! MCU: N32L403KBQ7 (32-pin). Shares the entire motor-drive/encoder/CAN pin
//! map with the 57D (see super::hw_map); differs only in MCU/package, the
//! electrical constants below, and the absence of USART3/RS485 (the 32-pin
//! package does not break it out).
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

/// SERVO42D board handle. Owns the configured peripherals.
pub struct Servo42D {
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
}

impl Servo42D {
    /// Construct and configure board peripherals from the raw device.
    /// Stage 2: clocks + TIM3 PWM only; other peripherals added incrementally.
    pub fn init(mut dp: pac::Peripherals) -> Self {
        let rcc = dp.rcc.constrain();
        let clocks = rcc.cfgr.freeze();

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

        let mut board = Self {
            a_plus, a_minus, b_plus, b_minus, max_duty, adc, spi, enc_cs, en,
        };
        // Fail-safe: explicitly command the output stage OFF at boot.
        board.set_output_enable(false);
        board
    }

    /// Map a signed coil command to a (plus, minus) duty pair using
    /// locked-antiphase: plus = mid + v/2, minus = mid - v/2, so the
    /// differential across the coil is proportional to v and its sign sets
    /// direction. Saturates at the rails.
    #[inline]
    fn split_duty(&self, v: CoilCommand) -> (u16, u16) {
        let mid = (self.max_duty / 2) as i32;
        // Scale i16 full-scale to +/- mid.
        let half = (v as i32 * mid) / (i16::MAX as i32);
        let plus = (mid + half).clamp(0, self.max_duty as i32) as u16;
        let minus = (mid - half).clamp(0, self.max_duty as i32) as u16;
        (plus, minus)
    }
}

impl Board for Servo42D {
    const SHUNT_OHMS: f32 = 0.05; // R9 = 0.05 ohm, verified on 42D power sheet
    const MAX_CURRENT_MA: u32 = 3000; // TODO confirm from datasheet/thermal
    const NAME: &'static str = "MKS SERVO42D (N32L403)";
    const HAS_RS485: bool = false; // 32-pin package: no USART3 breakout

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
        let mut buf = [0u8, 0u8];
        let _ = self.enc_cs.set_low();
        let _ = self.spi.transfer_in_place(&mut buf);
        let _ = self.enc_cs.set_high();
        let word = ((buf[0] as u16) << 8) | (buf[1] as u16);
        let angle14 = word >> 2; // drop No_Mag + parity
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
