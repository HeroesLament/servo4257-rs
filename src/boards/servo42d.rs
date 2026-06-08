//! SERVO42D: N32L403, 0.05 ohm shunts.
use crate::board::Board;
pub struct Servo42D;
impl Board for Servo42D {
    fn shunt_ohms() -> f32 { 0.05 }
    fn max_current_ma() -> u32 { 3000 } // TODO confirm from datasheet/thermal
}
