# ICM-20948 Blocking Examples for Raspberry Pi Pico 2 (RP2350)

This directory contains blocking examples for using the ICM-20948 9-axis IMU with the Raspberry Pi Pico 2 board using the Embassy framework.

**For async examples**, see the `../rp2350-async` directory.

**Note:** Examples for interrupts and DMP have been removed as these features are not functional in hardware.

## Hardware Requirements

- Raspberry Pi Pico 2 (RP2350-based board)
- ICM-20948 9-axis IMU breakout board
- 4 jumper wires for I2C connection
- USB cable for programming and power

## Wiring

Connect the ICM-20948 to your Raspberry Pi Pico 2 as follows:

```
ICM-20948    →    Raspberry Pi Pico 2
─────────────────────────────────────
VCC          →    3V3 (Pin 36)
GND          →    GND (Pin 38)
SDA          →    GP12 (Pin 16) - I2C0 SDA
SCL          →    GP13 (Pin 17) - I2C0 SCL
AD0          →    GND (for I2C address 0x68)
```

**Note**: The AD0 pin determines the I2C address (GND = 0x68, VCC = 0x69).

## Building and Running

Build and run a specific example:
```bash
cargo run --release --bin basic_reading
```

Build all examples:
```bash
cargo build --release
```

## Available Examples

Each example includes documentation in its source file. Check the file's doc comment for configuration details, tuning parameters, and expected output.

### Reading Individual Sensors

- **`basic_reading`** - Simple accelerometer, gyroscope, and temperature reading to get started
- **`sensor_configuration`** - Demonstrates different configuration presets for various use cases
- **`magnetometer`** - Reads the magnetometer and calculates compass heading
- **`nine_axis_reading`** - Reads all 9 axes (accelerometer + gyroscope + magnetometer)

### Calibration and Testing

- **`calibration`** - Interactive calibration procedure for accelerometer and gyroscope
- **`magnetometer_calibration`** - Interactive hard-iron calibration for accurate magnetometer readings
- **`self_test`** - Hardware self-test verification (accel, gyro, mag)
- **`sample_rate_test`** - Sample rate configuration testing

### Advanced Features

- **`fifo_batch`** - Efficient batch reading using FIFO buffer for low-power applications
- **`fifo_data_validation`** - Comprehensive FIFO validation suite comparing direct reads with FIFO-extracted data
- **`low_power_mode`** - Low-power mode demonstration with power consumption measurement

### Sensor Fusion

- **`ahrs_euler`** - Madgwick AHRS filter for smooth Euler angle orientation tracking with noise rejection

## Running Examples

```bash
cargo run --release --bin basic_reading
cargo run --release --bin nine_axis_reading
cargo run --release --bin ahrs_euler
cargo run --release --bin magnetometer_calibration
cargo run --release --bin self_test
cargo run --release --bin sample_rate_test
cargo run --release --bin fifo_batch
cargo run --release --bin fifo_data_validation
cargo run --release --bin low_power_mode
```

## Troubleshooting

- **Device not detected**: Check wiring, verify 3.3V power, confirm I2C address matches AD0 pin setting
- **Communication errors**: Reduce I2C frequency to 100kHz, add 4.7kΩ pull-up resistors, shorten wires
- **Unstable readings**: Run calibration example, enable DLPF, check for EMI and stable power supply
- **Build errors**: Ensure ARM toolchain installed (`rustup target add thumbv8m.main-none-eabihf`)

## Resources

- [ICM-20948 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Embassy Framework](https://embassy.dev/)
- [ICM-20948-rs Library Documentation](../../README.md)

## License

MIT