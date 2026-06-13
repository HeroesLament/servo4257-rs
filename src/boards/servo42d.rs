//! MKS SERVO42D board implementor.
//!
//! MCU: N32L403KBQ7 (32-pin). Shares the entire motor-drive/encoder/CAN pin
//! map with the 57D (see super::hw_map); differs only in MCU/package, the
//! electrical constants below, and the absence of USART3/RS485 (the 32-pin
//! package does not break it out).
//!
//! NOTE: peripheral handles are not yet owned here. The HAL/PAC path deps in
//! Cargo.toml are still commented out. This implementor currently provides the
//! static characteristics and the method shape; the method bodies are todo!()
//! until the HAL deps go live and init() wires the real peripherals via
//! hw_map. See docs/HAL_INTERFACE.md.

use crate::board::{Board, CoilCommand, CoilCurrent, RotorAngle};

/// SERVO42D board handle. Will own its configured peripherals once the HAL
/// deps are enabled.
pub struct Servo42D {
    // TODO: pwm, adc_currents, encoder, en handles wired from super::hw_map.
    _private: (),
}

impl Servo42D {
    /// Construct and configure all board peripherals from the raw device.
    /// TODO: take the PAC Peripherals, configure TIM3 PWM / injected ADC /
    /// SPI1 encoder / nEN per super::hw_map, return the ready handle.
    pub fn init() -> Self {
        Self { _private: () }
    }
}

impl Board for Servo42D {
    const SHUNT_OHMS: f32 = 0.05; // R9 = 0.05 ohm, verified on 42D power sheet
    const MAX_CURRENT_MA: u32 = 3000; // TODO confirm from datasheet/thermal
    const NAME: &'static str = "MKS SERVO42D (N32L403)";
    const HAS_RS485: bool = false; // 32-pin package: no USART3 breakout

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
