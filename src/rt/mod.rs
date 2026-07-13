//! rt/ — the "embassy-n32" seed shim: the minimal platform glue that lets the
//! Embassy async tier run on the N32L406 on top of the blocking n32l4xx-hal.
//! Grows peripheral-by-peripheral (time driver now; async CAN + I2C next).

pub mod can;
pub mod i2c;
pub mod time_driver;

pub use time_driver::init as init_time_driver;
pub use time_driver::init_from_clocks as init_time_driver_from_clocks;
