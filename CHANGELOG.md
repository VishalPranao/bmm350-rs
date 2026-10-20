# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-20
### Changed
- Use one BMM350 implementation in `device.rs` with a borrowed `I2CBus` and `Delay`.
- Return factory-compensated XYZ magnetic field and temperature from `measure()`.
- Replace the incomplete owned-peripheral API; callers now pass a bus to `initialize()` and `measure()`.

## [0.0.2] - 2025-04-06
### Added
- Initial release of the BMM350 Rust driver.
- Support for both I2C and SPI interfaces (SPI not validated).
- Configurable magnetometer settings.
- Functions to read raw sensor data.
- Error handling and device initialization.
- README with usage examples and API overview.
