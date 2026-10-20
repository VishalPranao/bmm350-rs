//! Small bus and delay interfaces for code that borrows shared hardware.

/// I2C operations supplied by an application or hardware abstraction layer.
pub trait I2CBus {
    type Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error>;

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8])
        -> Result<(), Self::Error>;
}

/// Delays needed while initializing a sensor or waiting for a conversion.
pub trait Delay {
    fn delay_us(&mut self, microseconds: u16);
    fn delay_ms(&mut self, milliseconds: u16);
}

impl<T: embedded_hal::i2c::I2c> I2CBus for T {
    type Error = T::Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        embedded_hal::i2c::I2c::write(self, address, bytes)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        embedded_hal::i2c::I2c::write_read(self, address, write, read)
    }
}

impl<T: embedded_hal::delay::DelayNs> Delay for T {
    fn delay_us(&mut self, microseconds: u16) {
        embedded_hal::delay::DelayNs::delay_us(self, u32::from(microseconds));
    }

    fn delay_ms(&mut self, milliseconds: u16) {
        embedded_hal::delay::DelayNs::delay_ms(self, u32::from(milliseconds));
    }
}
