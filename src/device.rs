//! BMM350 setup and compensated measurements.
//!
//! The register behavior, startup sequence, OTP layout, and compensation
//! decoding are adapted from the Bosch Sensortec BMM350 SensorAPI. The license
//! notice is stored in this crate's `LICENSE-BOSCH` file.

use crate::i2c_bus::{Delay, I2CBus};
use crate::Register;

pub const DEFAULT_I2C_ADDRESS: u8 = 0x14;
pub const ALTERNATE_I2C_ADDRESS: u8 = 0x15;

const OTP_DIRECT_READ: u8 = 0x20;
const OTP_POWER_OFF: u8 = 0x80;
const OTP_WORD_COUNT: usize = 32;
const OTP_STATUS_DONE: u8 = 0x01;
const OTP_STATUS_ERROR_MASK: u8 = 0xe0;
const OTP_POLL_ATTEMPTS: usize = 100;

const POWER_MODE_SUSPEND: u8 = 0x00;
pub const POWER_MODE_NORMAL: u8 = 0x01;
const PMU_CMD_UPDATE_ODR_AVERAGING: u8 = 0x02;
const PMU_CMD_FLUX_GUIDE_RESET: u8 = 0x05;
const PMU_CMD_BIT_RESET: u8 = 0x07;

const PMU_STATUS_BUSY: u8 = 0x01;
const PMU_STATUS_NORMAL: u8 = 0x08;
const PMU_STATUS_ILLEGAL_COMMAND: u8 = 0x10;
const PMU_STATUS_COMMAND_SHIFT: u8 = 5;

pub const ODR_100_HZ: u8 = 0x04;
pub const AVERAGING_4: u8 = 0x02;
pub const AXES_XYZ: u8 = 0x07;

const MAX_REGISTER_READ_LENGTH: usize = 12;
const I2C_DUMMY_BYTE_COUNT: usize = 2;
const MEASUREMENT_DATA_LENGTH: usize = 12;

// Default conversion factors from the Bosch BMM350 SensorAPI. Magnetic
// results are expressed in microtesla and temperature in degrees Celsius.
const XY_LSB_TO_MICROTESLA: f32 = 0.007_069_979;
const Z_LSB_TO_MICROTESLA: f32 = 0.007_174_964;
const TEMPERATURE_LSB_TO_CELSIUS: f32 = 0.000_981_282;
const TEMPERATURE_ZERO_OFFSET_CELSIUS: f32 = 25.49;

#[derive(Debug, PartialEq)]
pub enum Error<E> {
    Bus(E),
    NotInitialized,
    InvalidAddress(u8),
    WrongChipId(u8),
    RegisterReadTooLong,
    OtpTimeout(u8),
    OtpStatus(u8),
    PmuCommand { expected: u8, status: u8 },
    ConfigurationRejected,
}

/// Register values captured when the sensor rejects a PMU command.
/// Reading ERR_REG clears its flags, so it is captured first.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PmuDiagnostic {
    pub error: Option<u8>,
    pub requested: Option<u8>,
    pub command: Option<u8>,
    pub effective: Option<u8>,
    pub status_after: Option<u8>,
}

/// One factory-compensated BMM350 sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Measurement {
    /// X-axis magnetic field in microtesla.
    pub x: f32,
    /// Y-axis magnetic field in microtesla.
    pub y: f32,
    /// Z-axis magnetic field in microtesla.
    pub z: f32,
    /// Sensor temperature in degrees Celsius.
    pub temperature: f32,
}

#[derive(Copy, Clone)]
struct RawMeasurement {
    x: i32,
    y: i32,
    z: i32,
    temperature: i32,
}

#[derive(Copy, Clone)]
struct Compensation {
    temperature_offset: f32,
    offset_x: f32,
    offset_y: f32,
    offset_z: f32,
    temperature_sensitivity: f32,
    sensitivity_x: f32,
    sensitivity_y: f32,
    sensitivity_z: f32,
    tco_x: f32,
    tco_y: f32,
    tco_z: f32,
    tcs_x: f32,
    tcs_y: f32,
    tcs_z: f32,
    reference_temperature: f32,
    cross_x_y: f32,
    cross_y_x: f32,
    cross_z_x: f32,
    cross_z_y: f32,
}

/// Persistent state for one BMM350 sensor.
///
/// The bus is passed to each operation rather than stored here, so any
/// application can lend its I2C peripheral only when it is needed.
pub struct Bmm350 {
    address: u8,
    compensation: Option<Compensation>,
    variant_id: Option<u8>,
    initialized: bool,
}

impl Bmm350 {
    pub const fn new(address: u8) -> Self {
        Self {
            address,
            compensation: None,
            variant_id: None,
            initialized: false,
        }
    }

    pub const fn address(&self) -> u8 {
        self.address
    }

    pub const fn variant_id(&self) -> Option<u8> {
        self.variant_id
    }

    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn reset_runtime_state(&mut self) {
        self.compensation = None;
        self.variant_id = None;
        self.initialized = false;
    }

    /// Capture PMU registers after a failed command. ERR_REG clears on read.
    /// Missing values indicate that a diagnostic I2C read also failed.
    pub fn pmu_diagnostic<B: I2CBus>(&self, bus: &mut B) -> PmuDiagnostic {
        PmuDiagnostic {
            error: self.read_register(bus, Register::ERR_REG).ok(),
            requested: self.read_register(bus, Register::PMU_CMD_AGGR_SET).ok(),
            command: self.read_register(bus, Register::PMU_CMD).ok(),
            effective: self.read_register(bus, Register::PMU_CMD_STATUS_1).ok(),
            status_after: self.read_register(bus, Register::PMU_CMD_STATUS_0).ok(),
        }
    }

    /// Reset, identify, calibrate, and configure the sensor for normal mode.
    pub fn initialize<B: I2CBus + Delay>(&mut self, bus: &mut B) -> Result<(), Error<B::Error>> {
        self.reset_runtime_state();

        if !matches!(self.address, DEFAULT_I2C_ADDRESS | ALTERNATE_I2C_ADDRESS) {
            return Err(Error::InvalidAddress(self.address));
        }

        bus.delay_ms(3);
        self.write_register(bus, Register::CMD, Register::CMD_SOFT_RESET)?;
        bus.delay_ms(24);

        let chip_id = self.read_register(bus, Register::CHIP_ID)?;
        if chip_id != Register::BMM350_CHIP_ID {
            return Err(Error::WrongChipId(chip_id));
        }

        let otp = self.read_otp(bus)?;
        let compensation = Compensation::from_otp(&otp);
        let variant_id = ((otp[30] & 0x7f00) >> 9) as u8;

        self.write_register(bus, Register::OTP_CMD, OTP_POWER_OFF)?;
        self.magnetic_reset(bus)?;
        self.configure_sensor(bus)?;

        self.compensation = Some(compensation);
        self.variant_id = Some(variant_id);
        self.initialized = true;
        Ok(())
    }

    /// Read one complete sample and apply the sensor's factory compensation.
    pub fn measure<B: I2CBus>(&self, bus: &mut B) -> Result<Measurement, Error<B::Error>> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        let compensation = self.compensation.ok_or(Error::NotInitialized)?;
        let raw = self.read_raw_measurement(bus)?;
        Ok(compensation.apply(raw))
    }

    fn write_register<B: I2CBus>(
        &self,
        bus: &mut B,
        register: u8,
        value: u8,
    ) -> Result<(), Error<B::Error>> {
        bus.write(self.address, &[register, value])
            .map_err(Error::Bus)
    }

    fn read_register<B: I2CBus>(&self, bus: &mut B, register: u8) -> Result<u8, Error<B::Error>> {
        let mut value = [0; 1];
        self.read_registers(bus, register, &mut value)?;
        Ok(value[0])
    }

    fn read_registers<B: I2CBus>(
        &self,
        bus: &mut B,
        register: u8,
        values: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        if values.len() > MAX_REGISTER_READ_LENGTH {
            return Err(Error::RegisterReadTooLong);
        }

        let mut response = [0; MAX_REGISTER_READ_LENGTH + I2C_DUMMY_BYTE_COUNT];
        let response_length = values.len() + I2C_DUMMY_BYTE_COUNT;
        bus.write_read(self.address, &[register], &mut response[..response_length])
            .map_err(Error::Bus)?;

        values.copy_from_slice(&response[I2C_DUMMY_BYTE_COUNT..response_length]);
        Ok(())
    }

    fn read_raw_measurement<B: I2CBus>(
        &self,
        bus: &mut B,
    ) -> Result<RawMeasurement, Error<B::Error>> {
        let mut data = [0; MEASUREMENT_DATA_LENGTH];
        self.read_registers(bus, Register::MAG_X_XLSB, &mut data)?;

        Ok(RawMeasurement {
            x: decode_signed_24(&data[0..3]),
            y: decode_signed_24(&data[3..6]),
            z: decode_signed_24(&data[6..9]),
            temperature: decode_signed_24(&data[9..12]),
        })
    }

    fn read_otp_word<B: I2CBus + Delay>(
        &self,
        bus: &mut B,
        word_address: u8,
    ) -> Result<u16, Error<B::Error>> {
        self.write_register(
            bus,
            Register::OTP_CMD,
            OTP_DIRECT_READ | (word_address & 0x1f),
        )?;

        let mut command_done = false;
        for _ in 0..OTP_POLL_ATTEMPTS {
            bus.delay_us(300);
            let status = self.read_register(bus, Register::OTP_STATUS)?;
            let otp_error = status & OTP_STATUS_ERROR_MASK;
            if otp_error != 0 {
                return Err(Error::OtpStatus(otp_error));
            }
            if status & OTP_STATUS_DONE != 0 {
                command_done = true;
                break;
            }
        }

        if !command_done {
            return Err(Error::OtpTimeout(word_address));
        }

        let msb = self.read_register(bus, Register::OTP_DATA_MSB)?;
        let lsb = self.read_register(bus, Register::OTP_DATA_LSB)?;
        Ok(u16::from_be_bytes([msb, lsb]))
    }

    fn read_otp<B: I2CBus + Delay>(
        &self,
        bus: &mut B,
    ) -> Result<[u16; OTP_WORD_COUNT], Error<B::Error>> {
        let mut otp = [0; OTP_WORD_COUNT];
        for (address, word) in otp.iter_mut().enumerate() {
            *word = self.read_otp_word(bus, address as u8)?;
        }
        Ok(otp)
    }

    fn verify_pmu_command<B: I2CBus>(
        &self,
        bus: &mut B,
        expected_command: u8,
    ) -> Result<u8, Error<B::Error>> {
        let status = self.read_register(bus, Register::PMU_CMD_STATUS_0)?;
        let completed_command = status >> PMU_STATUS_COMMAND_SHIFT;
        if status & (PMU_STATUS_BUSY | PMU_STATUS_ILLEGAL_COMMAND) != 0
            || completed_command != expected_command
        {
            return Err(Error::PmuCommand {
                expected: expected_command,
                status,
            });
        }
        Ok(status)
    }

    fn magnetic_reset<B: I2CBus + Delay>(&self, bus: &mut B) -> Result<(), Error<B::Error>> {
        let status = self.read_register(bus, Register::PMU_CMD_STATUS_0)?;
        if status & PMU_STATUS_NORMAL != 0 {
            self.write_register(bus, Register::PMU_CMD, POWER_MODE_SUSPEND)?;
            bus.delay_ms(6);
            self.verify_pmu_command(bus, POWER_MODE_SUSPEND)?;
        }

        self.write_register(bus, Register::PMU_CMD, PMU_CMD_BIT_RESET)?;
        bus.delay_ms(14);
        self.verify_pmu_command(bus, PMU_CMD_BIT_RESET)?;

        self.write_register(bus, Register::PMU_CMD, PMU_CMD_FLUX_GUIDE_RESET)?;
        bus.delay_ms(18);
        self.verify_pmu_command(bus, PMU_CMD_FLUX_GUIDE_RESET)?;
        Ok(())
    }

    fn configure_sensor<B: I2CBus + Delay>(&self, bus: &mut B) -> Result<(), Error<B::Error>> {
        let odr_and_averaging = (AVERAGING_4 << 4) | ODR_100_HZ;
        self.write_register(bus, Register::PMU_CMD_AGGR_SET, odr_and_averaging)?;
        self.write_register(bus, Register::PMU_CMD, PMU_CMD_UPDATE_ODR_AVERAGING)?;
        bus.delay_ms(1);
        self.verify_pmu_command(bus, PMU_CMD_UPDATE_ODR_AVERAGING)?;

        let stored_odr_and_averaging = self.read_register(bus, Register::PMU_CMD_AGGR_SET)?;
        if stored_odr_and_averaging & 0x3f != odr_and_averaging {
            return Err(Error::ConfigurationRejected);
        }

        self.write_register(bus, Register::PMU_CMD_AXIS_EN, AXES_XYZ)?;
        if self.read_register(bus, Register::PMU_CMD_AXIS_EN)? & AXES_XYZ != AXES_XYZ {
            return Err(Error::ConfigurationRejected);
        }

        self.write_register(bus, Register::PMU_CMD, POWER_MODE_NORMAL)?;
        bus.delay_ms(38);
        let status = self.verify_pmu_command(bus, POWER_MODE_NORMAL)?;
        if status & PMU_STATUS_NORMAL == 0 {
            return Err(Error::ConfigurationRejected);
        }
        Ok(())
    }
}

impl Compensation {
    fn from_otp(otp: &[u16; OTP_WORD_COUNT]) -> Self {
        let offset_x = otp[0x0e] & 0x0fff;
        let offset_y = ((otp[0x0e] & 0xf000) >> 4) | (otp[0x0f] & 0x00ff);
        let offset_z = (otp[0x0f] & 0x0f00) | (otp[0x10] & 0x00ff);

        Self {
            temperature_offset: sign_extend((otp[0x0d] & 0x00ff) as u32, 8) as f32 / 5.0,
            offset_x: sign_extend(offset_x as u32, 12) as f32,
            offset_y: sign_extend(offset_y as u32, 12) as f32,
            offset_z: sign_extend(offset_z as u32, 12) as f32,
            temperature_sensitivity: sign_extend((otp[0x0d] >> 8) as u32, 8) as f32 / 512.0,
            sensitivity_x: sign_extend((otp[0x10] >> 8) as u32, 8) as f32 / 256.0,
            sensitivity_y: sign_extend((otp[0x11] & 0x00ff) as u32, 8) as f32 / 256.0,
            sensitivity_z: sign_extend((otp[0x11] >> 8) as u32, 8) as f32 / 256.0,
            tco_x: sign_extend((otp[0x12] & 0x00ff) as u32, 8) as f32 / 32.0,
            tco_y: sign_extend((otp[0x13] & 0x00ff) as u32, 8) as f32 / 32.0,
            tco_z: sign_extend((otp[0x14] & 0x00ff) as u32, 8) as f32 / 32.0,
            tcs_x: sign_extend((otp[0x12] >> 8) as u32, 8) as f32 / 16384.0,
            tcs_y: sign_extend((otp[0x13] >> 8) as u32, 8) as f32 / 16384.0,
            tcs_z: sign_extend((otp[0x14] >> 8) as u32, 8) as f32 / 16384.0,
            reference_temperature: sign_extend(otp[0x18] as u32, 16) as f32 / 512.0 + 23.0,
            cross_x_y: sign_extend((otp[0x15] & 0x00ff) as u32, 8) as f32 / 800.0,
            cross_y_x: sign_extend((otp[0x15] >> 8) as u32, 8) as f32 / 800.0,
            cross_z_x: sign_extend((otp[0x16] & 0x00ff) as u32, 8) as f32 / 800.0,
            cross_z_y: sign_extend((otp[0x16] >> 8) as u32, 8) as f32 / 800.0,
        }
    }

    fn apply(&self, raw: RawMeasurement) -> Measurement {
        let mut temperature =
            raw.temperature as f32 * TEMPERATURE_LSB_TO_CELSIUS - TEMPERATURE_ZERO_OFFSET_CELSIUS;
        temperature = (1.0 + self.temperature_sensitivity) * temperature + self.temperature_offset;

        let temperature_delta = temperature - self.reference_temperature;
        let mut x = compensate_axis(
            raw.x as f32 * XY_LSB_TO_MICROTESLA,
            self.sensitivity_x,
            self.offset_x,
            self.tco_x,
            self.tcs_x,
            temperature_delta,
        );
        let mut y = compensate_axis(
            raw.y as f32 * XY_LSB_TO_MICROTESLA,
            self.sensitivity_y,
            self.offset_y,
            self.tco_y,
            self.tcs_y,
            temperature_delta,
        );
        let mut z = compensate_axis(
            raw.z as f32 * Z_LSB_TO_MICROTESLA,
            self.sensitivity_z,
            self.offset_z,
            self.tco_z,
            self.tcs_z,
            temperature_delta,
        );

        // Bosch applies cross-axis compensation only after offset,
        // sensitivity, TCO, and TCS compensation. Keep the pre-cross-axis X
        // and Y values because all three equations use the same inputs.
        let pre_cross_x = x;
        let pre_cross_y = y;
        let cross_xy_denominator = 1.0 - self.cross_y_x * self.cross_x_y;

        x = (pre_cross_x - self.cross_x_y * pre_cross_y) / cross_xy_denominator;
        y = (pre_cross_y - self.cross_y_x * pre_cross_x) / cross_xy_denominator;
        z += (pre_cross_x * (self.cross_y_x * self.cross_z_y - self.cross_z_x)
            - pre_cross_y * (self.cross_z_y - self.cross_x_y * self.cross_z_x))
            / cross_xy_denominator;

        Measurement {
            x,
            y,
            z,
            temperature,
        }
    }
}

fn compensate_axis(
    value: f32,
    sensitivity: f32,
    offset: f32,
    tco: f32,
    tcs: f32,
    temperature_delta: f32,
) -> f32 {
    let corrected = value * (1.0 + sensitivity) + offset + tco * temperature_delta;
    corrected / (1.0 + tcs * temperature_delta)
}

fn decode_signed_24(bytes: &[u8]) -> i32 {
    let value = u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16);
    sign_extend(value, 24)
}

fn sign_extend(value: u32, bit_count: u8) -> i32 {
    let shift = 32 - u32::from(bit_count);
    ((value << shift) as i32) >> shift
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_TOLERANCE: f32 = 0.001;

    struct FakeBus {
        address: u8,
        register: u8,
    }

    struct MeasurementBus {
        address: u8,
        register: u8,
        read_length: usize,
        response: [u8; MEASUREMENT_DATA_LENGTH + I2C_DUMMY_BYTE_COUNT],
    }

    impl I2CBus for FakeBus {
        type Error = ();

        fn write(&mut self, _address: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write_read(
            &mut self,
            address: u8,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.address = address;
            self.register = write[0];
            read.copy_from_slice(&[0x00, 0x00, 0x12, 0x34][..read.len()]);
            Ok(())
        }
    }

    impl Delay for FakeBus {
        fn delay_us(&mut self, _microseconds: u16) {}
        fn delay_ms(&mut self, _milliseconds: u16) {}
    }

    impl I2CBus for MeasurementBus {
        type Error = ();

        fn write(&mut self, _address: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write_read(
            &mut self,
            address: u8,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.address = address;
            self.register = write[0];
            self.read_length = read.len();
            read.copy_from_slice(&self.response[..read.len()]);
            Ok(())
        }
    }

    fn identity_compensation() -> Compensation {
        Compensation {
            temperature_offset: 0.0,
            offset_x: 0.0,
            offset_y: 0.0,
            offset_z: 0.0,
            temperature_sensitivity: 0.0,
            sensitivity_x: 0.0,
            sensitivity_y: 0.0,
            sensitivity_z: 0.0,
            tco_x: 0.0,
            tco_y: 0.0,
            tco_z: 0.0,
            tcs_x: 0.0,
            tcs_y: 0.0,
            tcs_z: 0.0,
            reference_temperature: 0.0,
            cross_x_y: 0.0,
            cross_y_x: 0.0,
            cross_z_x: 0.0,
            cross_z_y: 0.0,
        }
    }

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() <= TEST_TOLERANCE,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn register_reads_use_the_bus_and_remove_i2c_dummy_bytes() {
        let sensor = Bmm350::new(DEFAULT_I2C_ADDRESS);
        let mut bus = FakeBus {
            address: 0,
            register: 0,
        };
        let mut values = [0; 2];

        sensor.read_registers(&mut bus, 0x40, &mut values).unwrap();

        assert_eq!(bus.address, DEFAULT_I2C_ADDRESS);
        assert_eq!(bus.register, 0x40);
        assert_eq!(values, [0x12, 0x34]);
    }

    #[test]
    fn invalid_address_is_rejected_before_the_bus_is_used() {
        let mut sensor = Bmm350::new(0x16);
        let mut bus = FakeBus {
            address: 0,
            register: 0,
        };

        assert_eq!(
            sensor.initialize(&mut bus),
            Err(Error::InvalidAddress(0x16))
        );
        assert!(!sensor.is_initialized());
    }

    #[test]
    fn sign_extension_handles_otp_field_widths() {
        assert_eq!(sign_extend(0x7f, 8), 127);
        assert_eq!(sign_extend(0x80, 8), -128);
        assert_eq!(sign_extend(0x7ff, 12), 2047);
        assert_eq!(sign_extend(0x800, 12), -2048);
        assert_eq!(sign_extend(0xffff, 16), -1);
    }

    #[test]
    fn compensation_decodes_packed_signed_offsets() {
        let mut otp = [0; OTP_WORD_COUNT];
        otp[0x0e] = 0x7800;
        otp[0x0f] = 0x08ff;
        otp[0x10] = 0x0000;

        let compensation = Compensation::from_otp(&otp);
        assert_eq!(compensation.offset_x, -2048.0);
        assert_eq!(compensation.offset_y, 2047.0);
        assert_eq!(compensation.offset_z, -2048.0);
    }

    #[test]
    fn signed_24_bit_decoding_handles_full_range() {
        assert_eq!(decode_signed_24(&[0xff, 0xff, 0x7f]), 8_388_607);
        assert_eq!(decode_signed_24(&[0x00, 0x00, 0x80]), -8_388_608);
        assert_eq!(decode_signed_24(&[0xff, 0xff, 0xff]), -1);
    }

    #[test]
    fn measure_reads_one_block_and_converts_all_four_channels() {
        let mut sensor = Bmm350::new(DEFAULT_I2C_ADDRESS);
        sensor.compensation = Some(identity_compensation());
        sensor.initialized = true;

        // Two I2C dummy bytes, then X=1, Y=-1, Z=256, temperature=0.
        let mut bus = MeasurementBus {
            address: 0,
            register: 0,
            read_length: 0,
            response: [
                0x00, 0x00, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
            ],
        };

        let measurement = sensor.measure(&mut bus).unwrap();

        assert_eq!(bus.address, DEFAULT_I2C_ADDRESS);
        assert_eq!(bus.register, Register::MAG_X_XLSB);
        assert_eq!(
            bus.read_length,
            MEASUREMENT_DATA_LENGTH + I2C_DUMMY_BYTE_COUNT
        );
        assert_close(measurement.x, XY_LSB_TO_MICROTESLA);
        assert_close(measurement.y, -XY_LSB_TO_MICROTESLA);
        assert_close(measurement.z, 256.0 * Z_LSB_TO_MICROTESLA);
        assert_close(measurement.temperature, -25.49);
    }

    #[test]
    fn compensation_matches_bosch_operation_order() {
        let compensation = Compensation {
            temperature_offset: -1.5,
            offset_x: 2.0,
            offset_y: -3.0,
            offset_z: 1.0,
            temperature_sensitivity: 0.02,
            sensitivity_x: 0.01,
            sensitivity_y: -0.02,
            sensitivity_z: 0.03,
            tco_x: 0.1,
            tco_y: -0.05,
            tco_z: 0.02,
            tcs_x: 0.001,
            tcs_y: -0.002,
            tcs_z: 0.0005,
            reference_temperature: 23.0,
            cross_x_y: 0.01,
            cross_y_x: -0.015,
            cross_z_x: 0.02,
            cross_z_y: -0.01,
        };
        let raw = RawMeasurement {
            x: 100_000,
            y: -50_000,
            z: 25_000,
            temperature: 50_000,
        };

        let measurement = compensation.apply(raw);

        assert_close(measurement.x, 719.7309);
        assert_close(measurement.y, -338.2930);
        assert_close(measurement.z, 168.0109);
        assert_close(measurement.temperature, 22.5456);
    }

    #[test]
    fn measurement_before_setup_is_rejected_without_bus_access() {
        let sensor = Bmm350::new(DEFAULT_I2C_ADDRESS);
        let mut bus = MeasurementBus {
            address: 0,
            register: 0,
            read_length: 0,
            response: [0; MEASUREMENT_DATA_LENGTH + I2C_DUMMY_BYTE_COUNT],
        };

        assert_eq!(sensor.measure(&mut bus), Err(Error::NotInitialized));
        assert_eq!(bus.read_length, 0);
    }
}
