#![no_std]

//! Bosch BMM350 magnetometer driver. The device keeps sensor state while the
//! caller lends it an I2C bus for initialization and measurement.

pub mod device;
pub mod i2c_bus;
mod registers;

pub use device::{
    Bmm350, Error, Measurement, ALTERNATE_I2C_ADDRESS, AVERAGING_4, AXES_XYZ, DEFAULT_I2C_ADDRESS,
    ODR_100_HZ, POWER_MODE_NORMAL,
};
pub use registers::Register;
