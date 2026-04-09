# ICM-20948 Async Examples for STM32L431 (SPI)

This directory contains asynchronous examples for using the ICM-20948 9-axis IMU with an **STM32L431** microcontroller using the **Embassy** async framework over **SPI**.

**Note:** Examples for DMP (Digital Motion Processor) are currently excluded or marked as work-in-progress as stable quaternion output is still being debugged.

## Hardware Requirements

- STM32L431-based development board
- ICM-20948 9-axis IMU breakout board
- Logic Level Converter (e.g., TXB0106PWR)
- Jumper wires for SPI and external interrupt (EXTI) connections
- USB cable for programming and power/logging

## Wiring

Connect the ICM-20948 to your STM32L431 using SPI as follows. Ensure the level shifter is properly powered with 3.3V on the high side and 1.8V on the low side.

```text
ICM-20948  → Logic Level Shifter →  STM32L431
─────────────────────────────────────────────────
VCC          →    1.8V LDO or 3.3V
VDDIO        →    1.8V LDO
GND          →    GND               → GND
INT1         →    Level Shifter     → PA3 (EXTI)
NSS / CS     →    Level Shifter     → PA2 (Output)
SCL / SCLK   →    Level Shifter     → PB13 (SPI SCK)
SDA / SDI    →    Level Shifter     → PB15 (SPI MOSI)
SDO / AD0    →    Level Shifter     → PB14 (SPI MISO)
````

## Building and Running

Ensure you have the correct ARM target installed for the STM32L431 (Cortex-M4F):

```bash
rustup target add thumbv7em-none-eabihf
```

Build and run a specific example:

```bash
cargo run --release --bin basic_reading_async
```

## Available Examples

Each example includes documentation in its source file. Check the file's doc comment for configuration details, tuning parameters, and expected output.

### Advanced Hardware Features

  - **`hardware_interrupt_async`** - Demonstrates configuring the `INT1` pin for hardware-driven `Data Ready` triggers, allowing the MCU to sleep between samples for zero-latency fetching.
  - **`wake_on_motion_async`** - Demonstrates the ultra-low power Wake-on-Motion (WoM) feature. The MCU sleeps deeply until the IMU detects a physical tap or shake. *(Note: Includes the workaround for the ICM-20948 hardware limitation where axis-detection must be done via raw ADC delta comparison).*

## Running Specific Examples

```bash
cargo run --release --bin wake_on_motion_async
cargo run --release --bin hardware_interrupt_async
```

## Troubleshooting

  - **Device not detected (`WHO_AM_I` check failed)**:
      - Double-check SPI wiring (MOSI/MISO swapped is a common issue).
      - Verify your 1.8V regulator and level shifter are working. The ICM-20948 will burn if 3.3V is applied directly to its IO pins.
  - **SPI Communication Errors / Garbage Data**:
      - Reduce SPI frequency (start at 1MHz, max 7MHz).
      - Check CPOL/CPHA settings (SPI Mode 0 or 3).
  - **Interrupts not firing**:
      - Verify EXTI configuration on `PA3`.
      - Ensure the `INT1` pin on the IMU is correctly configured (push-pull vs open-drain, active high/low) matching your MCU's GPIO pull settings.

## Resources

  - [ICM-20948 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
  - [Embassy Framework](https://embassy.dev/)
  - [ICM-20948-rs Library Documentation](../../README.md)
## License

MIT