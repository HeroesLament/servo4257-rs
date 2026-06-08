//! SERVO57D: N32L406, 0.02 ohm shunts (higher-current NEMA23).
use crate::board::Board;
pub struct Servo57D;
impl Board for Servo57D {
    fn shunt_ohms() -> f32 { 0.02 }
    fn max_current_ma() -> u32 { 5200 } // TODO confirm from datasheet/thermal
}
