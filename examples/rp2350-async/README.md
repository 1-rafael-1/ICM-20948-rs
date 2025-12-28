# ICM-20948 Async Examples for Raspberry Pi Pico 2 (RP2350)

This directory contains async examples for using the ICM-20948 9-axis IMU with the Raspberry Pi Pico 2 board using the Embassy async framework.

**For blocking examples**, see the `../rp2350-blocking` directory.

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
cargo run --release --bin basic_reading_async
```

Build all examples:
```bash
cargo build --release
```

## Available Examples

Each example includes documentation in its source file. Check the file's doc comment for configuration details, tuning parameters, and expected output.

### Reading Individual Sensors

- **`basic_reading_async`** - Simple accelerometer, gyroscope, and temperature reading
- **`sensor_configuration_async`** - Configuration presets for various use cases
- **`magnetometer_async`** - Magnetometer and compass heading
- **`nine_axis_reading_async`** - All 9 axes (accel + gyro + mag)

### Calibration and Testing

- **`calibration_async`** - Accelerometer and gyroscope calibration
- **`magnetometer_calibration_async`** - Magnetometer hard-iron calibration
- **`self_test_async`** - Hardware self-test verification (accel, gyro, mag)
- **`sample_rate_test_async`** - Sample rate configuration testing

### Advanced Features

- **`fifo_batch_async`** - FIFO batch reading
- **`fifo_data_validation_async`** - FIFO data validation and testing
- **`low_power_mode_async`** - Low-power mode demonstration with power consumption measurement

### Sensor Fusion

- **`ahrs_euler_async`** - Madgwick AHRS filter for orientation tracking

## Running Examples

```bash
cargo run --release --bin basic_reading_async
cargo run --release --bin nine_axis_reading_async
cargo run --release --bin ahrs_euler_async
cargo run --release --bin magnetometer_calibration_async
cargo run --release --bin self_test_async
cargo run --release --bin sample_rate_test_async
cargo run --release --bin fifo_batch_async
cargo run --release --bin fifo_data_validation_async
cargo run --release --bin low_power_mode_async
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