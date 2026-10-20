# BMM350 Rust driver

This crate reads the Bosch BMM350 magnetometer over I2C. It initializes the sensor, reads its factory OTP calibration, and returns X, Y, and Z magnetic field in microtesla plus temperature in degrees Celsius.

The device owns its address and calibration state. The caller lends an I2C bus to each operation, so a firmware application can share one peripheral with other sensors. The bus must implement [`I2CBus`](src/i2c_bus.rs) and [`Delay`](src/i2c_bus.rs). An `embedded-hal` I2C or delay implementation gets the corresponding trait implementation automatically; an application can also write a small adapter for its board API.

```rust
use bmm350::{i2c_bus::{Delay, I2CBus}, Bmm350, Error, Measurement, DEFAULT_I2C_ADDRESS};

fn sample<B: I2CBus + Delay>(
    sensor: &mut Bmm350,
    bus: &mut B,
) -> Result<Measurement, Error<B::Error>> {
    if !sensor.is_initialized() {
        sensor.initialize(bus)?;
    }
    sensor.measure(bus)
}
```

Create the sensor with `Bmm350::new(DEFAULT_I2C_ADDRESS)` and keep it between samples so its OTP calibration is read only during initialization. The supported I2C addresses are `0x14` and `0x15`. Initialization configures normal mode, 100 Hz output rate, four-sample averaging, and all three axes. Polls have fixed limits, and a read before successful initialization returns an error.

The register and compensation behavior is adapted from Bosch's BMM350 SensorAPI. See [LICENSE](LICENSE) and [LICENSE-BOSCH](LICENSE-BOSCH).

## References

- [BMM350 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf)
- [Bosch BMM350 SensorAPI](https://github.com/boschsensortec/BMM350_SensorAPI)
