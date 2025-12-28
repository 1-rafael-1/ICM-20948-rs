//! FIFO Batch Reading Example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates how to configure and use the FIFO buffer
//! to efficiently read multiple sensor samples in a batch operation.
//!
//! FIFO benefits:
//! - Reduces interrupt frequency
//! - Enables burst reading of multiple samples
//! - Supports low-power operation by allowing host MCU to sleep
//! - Prevents data loss during brief periods when host is busy
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
//! - GND: GND
//! - AD0: GND (for address 0x68)
//! - INT (optional): GPIO14 (for interrupt-driven reading)

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    block::ImageDef,
    config::Config,
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::I2C0,
};
use embassy_time::{Delay, Timer};
use icm20948::{
    fifo::{FifoConfig, FifoMode},
    AccelConfig, AccelFullScale, GyroConfig, GyroFullScale, I2cInterface, Icm20948Driver,
};
use panic_probe as _;

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Bind I2C interrupts
bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("ICM-20948 FIFO Batch Reading Example (Async)");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    // Create ICM-20948 driver with I2C interface (uses default address 0x68, AD0 pin LOW)
    info!("Initializing ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface).await {
        Ok(imu) => {
            info!("ICM-20948 detected!");
            imu
        }
        Err(e) => {
            error!("Failed to detect ICM-20948: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    // Initialize the device
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("Failed to initialize ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    Timer::after_millis(100).await;
    info!("ICM-20948 initialized successfully!");

    // Configure accelerometer: ±2g range, ~102 Hz sample rate
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        sample_rate_div: 10, // ~102 Hz sample rate
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_accelerometer(accel_config).await {
        error!("Failed to configure accelerometer: {:?}", e);
    } else {
        info!("Accelerometer configured: ±2g, ~102Hz sample rate");
    }

    // Configure gyroscope: ±250°/s range, ~102 Hz sample rate
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        sample_rate_div: 10, // ~102 Hz sample rate
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config).await {
        error!("Failed to configure gyroscope: {:?}", e);
    } else {
        info!("Gyroscope configured: ±250°/s, ~102Hz sample rate");
    }

    // Configure FIFO to store accelerometer and gyroscope data
    info!("Configuring FIFO...");
    let fifo_config = FifoConfig {
        enable_accel: true,
        enable_gyro: true,
        enable_temp: false,
        enable_mag: false,
        mode: FifoMode::Stream, // Stream mode: oldest data dropped when full
    };

    if let Err(e) = imu.fifo_configure(&fifo_config).await {
        error!("Failed to configure FIFO: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    // Reset FIFO to clear any existing data
    if let Err(e) = imu.fifo_reset().await {
        error!("Failed to reset FIFO: {:?}", e);
    }

    // Enable FIFO
    if let Err(e) = imu.fifo_enable(true).await {
        error!("Failed to enable FIFO: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    info!("FIFO configured and enabled!");
    info!("Each sample: accel (6 bytes) + gyro (6 bytes) = 12 bytes");
    info!("Starting batch reading every 500ms...");
    info!("");

    let bytes_per_sample = 12u16;
    let mut batch_count = 0u32;

    // Get conversion factors
    let accel_sensitivity = accel_config.full_scale.sensitivity();
    let gyro_sensitivity = gyro_config.full_scale.sensitivity();

    // Main loop: periodically read FIFO data in batches
    loop {
        // Wait for data to accumulate
        Timer::after_millis(500).await;

        // Check FIFO count
        let count = match imu.fifo_count().await {
            Ok(c) => c,
            Err(e) => {
                error!("Failed to read FIFO count: {:?}", e);
                continue;
            }
        };

        if count == 0 {
            info!("FIFO empty, waiting...");
            continue;
        }

        let num_samples = count / bytes_per_sample;

        if num_samples == 0 {
            info!("Incomplete sample in FIFO, waiting...");
            continue;
        }

        batch_count += 1;
        info!(
            "Batch {}: FIFO count: {} bytes ({} samples)",
            batch_count, count, num_samples
        );

        // Read FIFO data
        let mut buffer = [0u8; 512];
        let bytes_to_read = ((num_samples * bytes_per_sample) as usize).min(buffer.len());
        let actual_samples = bytes_to_read / bytes_per_sample as usize;

        if actual_samples < num_samples as usize {
            warn!(
                "Buffer too small for {} samples, reading only {} samples",
                num_samples, actual_samples
            );
        }

        let bytes_read = match imu.fifo_read(&mut buffer[..bytes_to_read]).await {
            Ok(n) => n,
            Err(e) => {
                error!("Failed to read FIFO: {:?}", e);
                continue;
            }
        };

        info!("Read {} bytes from FIFO", bytes_read);

        // Parse FIFO data
        match imu.fifo_parse(&buffer[..bytes_read], &fifo_config.into()) {
            Ok(records) => {
                info!("Parsed {} records", records.len());

                // Process each record
                for (i, record) in records.iter().enumerate() {
                    if let (Some((ax, ay, az)), Some((gx, gy, gz))) = (record.accel, record.gyro) {
                        // Convert to physical units
                        let ax_g = ax as f32 / accel_sensitivity;
                        let ay_g = ay as f32 / accel_sensitivity;
                        let az_g = az as f32 / accel_sensitivity;

                        let gx_dps = gx as f32 / gyro_sensitivity;
                        let gy_dps = gy as f32 / gyro_sensitivity;
                        let gz_dps = gz as f32 / gyro_sensitivity;

                        if i < 3 || i >= records.len() - 1 {
                            // Print first 3 and last sample
                            info!(
                                "  Sample {}: Accel({}, {}, {})g Gyro({}, {}, {})°/s",
                                i, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps
                            );
                        } else if i == 3 {
                            info!("  ... ({} more samples) ...", records.len() - 4);
                        }
                    }
                }
            }
            Err(e) => {
                error!("Failed to parse FIFO data: {:?}", e);
            }
        }

        // Check for FIFO overflow
        match imu.fifo_overflow_status().await {
            Ok(overflow) => {
                if overflow.any_overflow() {
                    warn!("Warning: FIFO overflow detected!");
                    if let Err(e) = imu.fifo_reset().await {
                        error!("Failed to reset FIFO after overflow: {:?}", e);
                    } else {
                        info!("FIFO reset after overflow");
                    }
                }
            }
            Err(e) => {
                error!("Failed to read overflow status: {:?}", e);
            }
        }

        info!("");
    }
}
