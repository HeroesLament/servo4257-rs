//! MKS SERVO57D board implementor.
//!
//! MCU: N32L406CBL7 (48-pin). Shares the entire motor-drive/encoder/CAN pin
//! map with the 42D (see super::hw_map); differs only in MCU/package, the
//! electrical constants below, and it ADDS USART3/RS485 + extra opto IO that
//! the 48-pin package breaks out.
//!
//! NOTE: peripheral handles are not yet owned here. The HAL/PAC path deps in
//! Cargo.toml are still commented out. This implementor currently provides the
//! static characteristics and the method shape; the method bodies are todo!()
//! until the HAL deps go live and init() wires the real peripherals via
//! hw_map. See docs/HAL_INTERFACE.md.

use crate::board::{Board, CoilCommand, CoilCurrent, RotorAngle};

/// SERVO57D board handle. Will own its configured peripherals once the HAL
/// deps are enabled.
pub struct Servo57D {
    // TODO: pwm, adc_currents, encoder, en handles wired from super::hw_map.
    _private: (),
}

impl Servo57D {
    /// Construct and configure all board peripherals from the raw device.
    /// TODO: take the PAC Peripherals, configure TIM3 PWM / injected ADC /
    /// SPI1 encoder / nEN per super::hw_map, return the ready handle.
    pub fn init() -> Self {
        Self { _private: () }
    }
}

impl Board for Servo57D {
    const SHUNT_OHMS: f32 = 0.02; // R10/R22 = 0.02 ohm, verified on power sheet
    const MAX_CURRENT_MA: u32 = 5200; // TODO confirm from datasheet/thermal
    const NAME: &'static str = "MKS SERVO57D (N32L406)";
    const HAS_RS485: bool = true; // 48-pin package breaks out USART3

    fn apply_coil_voltages(&mut self, _v_a: CoilCommand, _v_b: CoilCommand) {
        todo!("map coil voltages to TIM3 CH1..4 duties via hw_map::pwm")
    }

    fn read_coil_currents(&self) -> (CoilCurrent, CoilCurrent) {
        todo!("read latched injected ADC samples (hw_map::adc)")
    }

    fn rotor_angle(&mut self) -> RotorAngle {
        todo!("MT6816 read over SPI1 (hw_map::encoder)")
    }

    fn set_output_enable(&mut self, _enabled: bool) {
        todo!("drive nEN (hw_map::gpio); active level TBD from EG3013 EN")
    }
}
