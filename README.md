# ICM-20948 Driver

[![Crates.io](https://img.shields.io/crates/v/icm20948-rs.svg)](https://crates.io/crates/icm20948-rs)
[![Documentation](https://docs.rs/icm20948-rs/badge.svg)](https://docs.rs/icm20948-rs)
[![CI](https://github.com/1-rafael-1/ICM-20948-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/1-rafael-1/ICM-20948-rs/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](README.md#license)

A platform-agnostic Rust driver for the **ICM-20948** 9-axis IMU (Inertial Measurement Unit), featuring:
- 3-axis Gyroscope
- 3-axis Accelerometer  
- 3-axis Magnetometer (AK09916)

This driver is built using the [`device-driver`](https://crates.io/crates/device-driver) toolkit and supports both blocking and async operation modes.

**Note**: This driver draws extensively on the [SparkFun ICM-20948 Arduino Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) for guidance on register interactions and initialization sequences. The DMP firmware is also sourced from `SparkFun`'s MIT-licensed library.

**Note**: Another invaluable source extensively used is the [ICM-20948 component for ESP-IDF](https://github.com/cybergear-robotics/icm20948) by Cybergear Robotics, which provided deep insights into the device's operation and configuration.

**Note**: This driver has been created with the help of agentic AI. Code has been self-reviewed but I am a hobbyist developer and have very little prior experience with IMUs, so I will not have spotted everything. To validate viability so far I have created a number of examples and ran them on hardware. Additionally I have made sure the agent added tests and CI, that run on every PR and every push to `main`. 

I was hoping to make DMP work and was looking for some christmas holidays entertainment making it. I failed at DMP but was duly entertained. In its current form this driver has somewhat more features than other existing ICM-20948 drivers, so I decided to publish it anyway. Somebody somewhere may find it useful.

That being said, this driver works for me and my hobby needs so far but WILL contain bugs and imperfections.
In case YOU find any issues, please consider submitting a PR or opening an issue so I can improve this driver over time. Thank you!

## Features

### Sensors
- ✅ **3-axis Accelerometer** - Configure range (±2g to ±16g), read in g-force or raw values
- ✅ **3-axis Gyroscope** - Configure range (±250°/s to ±2000°/s), read in °/s, rad/s, or raw values
- ✅ **3-axis Magnetometer (AK09916)** - Full 9-axis IMU support, read in µT
- ✅ **Temperature sensor** - Read in °C or raw values

### Communication & Modes
- ✅ **I2C interface** - Fully tested with embedded-hal 1.0
- ⚠️ **SPI interface** - Implemented but untested
- ✅ **Blocking and async operation** - Choose based on your application needs (embedded-hal-async)
- ✅ **`no_std` compatible** - Works in embedded environments without standard library

### Data Management
- ✅ **Physical unit conversions** - Direct output in g, °/s, rad/s, °C, µT (or raw 16-bit values)
- ✅ **Calibration support** - Offset calibration for accelerometer, gyroscope, and magnetometer with motion detection
- ✅ **Hardware self-test** - Built-in diagnostic verification for all sensors
- ✅ **FIFO management** - Batch reading with configurable watermarks for efficient data collection

### Configuration & Power
- ✅ **Flexible sensor configuration** - Full-scale ranges, digital low-pass filters, sample rate dividers
- ✅ **Power management** - Sleep, low-power, and cycle modes for battery-powered applications
- ✅ **Optional defmt support** - Logging support for embedded debugging

### Known Limitations
- ❌ **Interrupts** - API exists but hardware interrupt pin does not trigger (not functional)
- ❌ **Wake-on-Motion** - API exists but motion detection does not trigger interrupts (not functional). Related to interrupt system issues.
- ❌ **DMP (Digital Motion Processor)** - Not functional despite extensive attempts. Use software sensor fusion instead.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
icm20948-rs = "0.1"
```

The driver includes blocking I2C/SPI support by default. See the [Features](#features) section below for optional features like `async`, `defmt`, and `dmp`.

### Features

- `async` - Enable `AsyncRegisterInterface` trait implementations for embedded-hal-async 1.0
- `defmt` - Enable defmt formatting support for types
- `dmp` - Enable Digital Motion Processor (DMP) firmware loading (**not functional yet!**)

**Note:** The crate uses `embedded-hal` for blocking I2C/SPI traits and core functionality like `DelayNs` used by magnetometer operations. The `async` feature is additive and provides async trait implementations alongside the blocking ones.

### Basic Example (I2C)

```rust,ignore
use icm20948::{Icm20948Driver, I2cInterface};
use icm20948::sensors::{AccelConfig, AccelFullScale, AccelDlpf};
use icm20948::sensors::{GyroConfig, GyroFullScale, GyroDlpf};

// Create I2C interface with default address (0x68, AD0 pin LOW)
let interface = I2cInterface::default(i2c);

// Create driver and initialize
let mut imu = Icm20948Driver::new(interface)?;
imu.init()?;

// Configure accelerometer: ±2g range, 246 Hz DLPF
let accel_config = AccelConfig {
    full_scale: AccelFullScale::G2,
    dlpf: AccelDlpf::Hz246,
    dlpf_enable: true,
    sample_rate_div: 10,
};
imu.configure_accelerometer(accel_config)?;

// Configure gyroscope: ±250°/s range, 197 Hz DLPF
let gyro_config = GyroConfig {
    full_scale: GyroFullScale::Dps250,
    dlpf: GyroDlpf::Hz197,
    dlpf_enable: true,
    sample_rate_div: 10,
};
imu.configure_gyroscope(gyro_config)?;

// Read sensor data
let accel = imu.read_accelerometer()?;  // Returns data in g
let gyro = imu.read_gyroscope()?;       // Returns data in °/s
let temp = imu.read_temperature_celsius()?; // Returns °C

println!("Accel: x={:.2}g y={:.2}g z={:.2}g", accel.x, accel.y, accel.z);
println!("Gyro: x={:.2}°/s y={:.2}°/s z={:.2}°/s", gyro.x, gyro.y, gyro.z);
println!("Temp: {:.2}°C", temp);
```

For complete working examples with async Embassy, magnetometer, FIFO, self-test diagnostics, and more, see the [`examples/rp2350-blocking/`](examples/rp2350-blocking/) and [`examples/rp2350-async/`](examples/rp2350-async/) directories.

## Register Bank System

The ICM-20948 organizes registers into 4 banks:

- **Bank 0**: Primary configuration, sensor data, interrupts, FIFO
- **Bank 1**: Self-test registers, time base correction
- **Bank 2**: Gyroscope and accelerometer configuration
- **Bank 3**: I2C master configuration (for magnetometer)

The driver automatically handles bank switching, so you don't need to worry about it in normal usage.

## Physical Units

The driver provides high-level APIs that return sensor data in physical units:

- **Accelerometer**: Returns data in g-force (gravity units)
- **Gyroscope**: Returns data in degrees per second (°/s) or radians per second (rad/s)
- **Magnetometer**: Returns data in microtesla (µT)
- **Temperature**: Returns data in degrees Celsius (°C)

For raw 16-bit values, use the `_raw()` variants like `read_accel_raw()` and `read_gyro_raw()`.

## Hardware Connections

### I2C Interface

**I2C Addresses:**
- `0x68` when AD0 is LOW (default) - use `I2cInterface::default(i2c)`
- `0x69` when AD0 is HIGH (alternative) - use `I2cInterface::alternative(i2c)`

### SPI Interface

**⚠️ SPI Status:** The SPI interface is implemented but has not been tested with hardware. Pull Requests with fixes and/or examples are welcome!

**SPI Configuration:**
- Mode: 0 or 3 (CPOL=0, CPHA=0 or CPOL=1, CPHA=1)
- Max clock: 7 MHz
- Bit order: MSB first

## Testing

The driver includes unit tests, async tests, and integration tests with mock hardware interfaces.

```bash
# Run library unit tests
cargo test --lib

# Run async tests
cargo test --test async_tests --features std,async

# Run FIFO efficiency tests
cargo test --test fifo_efficiency_test --features std
```

**Continuous Integration:** See [`.github/workflows/ci.yml`](.github/workflows/ci.yml) for the CI test suite, which includes:
- Library unit tests (sensors, FIFO, interrupts, power management, self-test)
- Async tests
- FIFO efficiency tests
- Example builds for both blocking and async variants (RP2350)
- Format and clippy checks

All tests run automatically on every push and pull request.

## Development Status

### Implemented ✅

**Core Functionality:**
- [x] Complete register definitions (all 4 banks)
- [x] I2C and SPI interfaces (blocking and async)
- [x] Device initialization and `WHO_AM_I` verification
- [x] Automatic bank switching

**Sensors:**
- [x] Accelerometer (with configuration, calibration, physical units)
- [x] Gyroscope (with configuration, calibration, physical units)
- [x] Temperature sensor
- [x] Magnetometer AK09916 (via I2C master, with calibration)

**Advanced Features:**
- [x] FIFO management (configuration, batch reading, parsing)
- [x] Power management (sleep, low-power, cycle modes)
- [x] Hardware self-test (accelerometer, gyroscope, magnetometer with internal test signals)

### Known Non-Functional Features ❌

**Interrupts:**
- Interrupt configuration API exists in code
- **Status:** Not functional - hardware interrupt pin does not trigger
- Likely requires deep hardware-level debugging or may have fundamental issues
- **Recommendation:** Do not use interrupt features

**Wake-on-Motion:**
- Low-power mode configuration API exists
- **Status:** Not functional - motion detection does not trigger interrupts
- Related to interrupt system issues
- **Recommendation:** Use continuous polling if motion detection is needed

**Digital Motion Processor (DMP):**
- Firmware loading implementation exists (behind `dmp` feature flag)
- **Status:** Not functional despite extensive tinkering
- **Recommendation:** Use software sensor fusion (see `ahrs_euler.rs` example) instead
- DMP code remains in codebase for reference only

## Documentation

- [API Documentation](https://docs.rs/icm20948-rs)
- [ICM-20948 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
- [device-driver toolkit documentation](https://diondokter.github.io/device-driver/)

## License

Licensed under the MIT License ([LICENSE](./LICENSE) or <http://opensource.org/licenses/MIT>).

This project extensively references the [SparkFun ICM-20948 Arduino Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) (also MIT licensed) for register sequences and DMP firmware. The DMP firmware binary is included from `SparkFun`'s library under the MIT License (Copyright (c) 2016 `SparkFun` Electronics). See `src/dmp/firmware.rs` for the complete license text.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you shall be licensed under the MIT license, without any additional terms or conditions.
