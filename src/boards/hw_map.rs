//! Verified hardware pin/peripheral map shared by the MKS SERVO42D and 57D.
//!
//! PROVENANCE: every assignment here is read off the MKS schematic (MCU sheet
//! + power sheet) AND cross-checked against the N32L40x AF table
//! (n32l4xx-hal/tools/gpio_af/af_table_um.tsv). Do not edit from memory; if a
//! future board remaps these, that board gets its OWN map rather than mutating
//! this one. See docs/HAL_INTERFACE.md (verify-on-contact) for the rule.
//!
//! Verified identical across 42D (N32L403KBQ7, 32-pin) and 57D (N32L406CBL7,
//! 48-pin): the entire motor-drive + encoder + CAN core is on the same pins.
//! The boards differ only in MCU/package, shunt/current constants, and which
//! OPTIONAL peripherals the package exposes (57D adds USART3/RS485 + extra IO).
//!
//! Topology: 2-phase bipolar stepper, two full H-bridges (coil A, coil B),
//! four single-ended PWM nets into EG3013 gate drivers (dead-time in-chip).

/// Motor PWM bridge timer and its four channels (all AF2 = TIM3).
///   phaseA1 = PA6 = TIM3_CH1   (coil A, + side half-bridge)
///   phaseA2 = PA7 = TIM3_CH2   (coil A, - side half-bridge)
///   phaseB1 = PB0 = TIM3_CH3   (coil B, + side half-bridge)
///   phaseB2 = PB1 = TIM3_CH4   (coil B, - side half-bridge)
pub mod pwm {
    pub const TIMER: &str = "TIM3";
    pub const AF: u8 = 2;
    // (pin, TIM3 channel index 1..=4)
    pub const PHASE_A1: (&str, u8) = ("PA6", 1);
    pub const PHASE_A2: (&str, u8) = ("PA7", 2);
    pub const PHASE_B1: (&str, u8) = ("PB0", 3);
    pub const PHASE_B2: (&str, u8) = ("PB1", 4);
}

/// Coil current sense -> ADC. Differential GS8632 amp across a low-side shunt
/// per coil, single-ended into the ADC. Sampled via the injected group,
/// triggered off TIM3 (EXTJSEL has a TIM3 option -- see HAL_INTERFACE.md).
///   currentA = PA2 = ADC channel 3
///   currentB = PA1 = ADC channel 2
pub mod adc {
    // (pin, ADC channel)
    pub const CURRENT_A: (&str, u8) = ("PA2", 3);
    pub const CURRENT_B: (&str, u8) = ("PA1", 2);
}

/// On-board encoder (MT6816) on SPI1. CS is driven as a GPIO output, not
/// hardware NSS.
///   SPI_CLK  = PB3 (AF1)
///   SPI_MISO = PB4 (AF1)
///   SPI_MOSI = PB5 (AF0)
///   SPI_CS   = PB6 (GPIO out)
pub mod encoder {
    pub const SPI: &str = "SPI1";
    pub const CLK: (&str, u8) = ("PB3", 1);
    pub const MISO: (&str, u8) = ("PB4", 1);
    pub const MOSI: (&str, u8) = ("PB5", 0);
    pub const CS: &str = "PB6"; // GPIO output
}

/// CAN (bxCAN) -- the CANopen / CiA-402 bus.
///   CAN_RX = PA11 (AF1)
///   CAN_TX = PA12 (AF1)
pub mod can {
    pub const RX: (&str, u8) = ("PA11", 1);
    pub const TX: (&str, u8) = ("PA12", 1);
}

/// Step/dir/enable discrete GPIO (legacy step interface + output-stage gate).
///   nSTP = PA0   nDIR = PA8   nEN = PB7 (gates the EG3013s; active level TBD)
pub mod gpio {
    pub const NSTP: &str = "PA0";
    pub const NDIR: &str = "PA8";
    pub const NEN: &str = "PB7";
}
