//! FIFO Batch Reading Example for ICM-20948 on Raspberry Pi Pico 2 (Blocking version)
//!
//! This example demonstrates how to configure and use the FIFO buffer
//! to efficiently read multiple sensor samples in a batch operation,
//! including magnetometer data.
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
use embassy_time::Delay;
use icm20948::{
    fifo::{FifoConfig, FifoMode},
    AccelConfig, AccelFullScale, GyroConfig, GyroFullScale, I2cInterface, Icm20948Driver,
    MagConfig, MagMode,
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
    info!("ICM-20948 FIFO Batch Reading Example");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400kHz on pins 12(SDA)/13(SCL)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_13, p.PIN_12, i2c_config);

    // Create and initialize ICM-20948 driver (address 0x68)
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface) {
        Ok(imu) => imu,
        Err(e) => {
            error!("Failed to detect ICM-20948: {:?}", e);
            loop {
                embassy_time::block_for(embassy_time::Duration::from_millis(1000));
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay) {
        error!("Failed to initialize: {:?}", e);
        loop {
            embassy_time::block_for(embassy_time::Duration::from_millis(1000));
        }
    }

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    // Configure accelerometer: ±2g, ~102Hz sample rate
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        sample_rate_div: 10,
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_accelerometer(accel_config) {
        error!("Failed to configure accelerometer: {:?}", e);
    }

    // Configure gyroscope: ±250°/s, ~102Hz sample rate
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        sample_rate_div: 10,
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config) {
        error!("Failed to configure gyroscope: {:?}", e);
    }

    // Configure magnetometer at 100Hz
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    if let Err(e) = imu.init_magnetometer(mag_config, &mut delay) {
        error!("Failed to initialize magnetometer: {:?}", e);
        loop {
            embassy_time::block_for(embassy_time::Duration::from_millis(1000));
        }
    }

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    // Configure FIFO to store accelerometer, gyroscope, and magnetometer data
    let fifo_config = FifoConfig {
        enable_accel: true,
        enable_gyro: true,
        enable_temp: false,
        enable_mag: true,
        mode: FifoMode::Stream, // Stream mode: oldest data dropped when full
    };

    if let Err(e) = imu.fifo_configure(&fifo_config) {
        error!("Failed to configure FIFO: {:?}", e);
        loop {
            embassy_time::block_for(embassy_time::Duration::from_millis(1000));
        }
    }

    // Reset FIFO to clear any existing data
    if let Err(e) = imu.fifo_reset() {
        error!("Failed to reset FIFO: {:?}", e);
    }

    // Enable FIFO
    if let Err(e) = imu.fifo_enable(true) {
        error!("Failed to enable FIFO: {:?}", e);
        loop {
            embassy_time::block_for(embassy_time::Duration::from_millis(1000));
        }
    }

    info!("FIFO configured: accel(6B) + gyro(6B) + mag(8B) = 20B/sample");
    info!("Reading batches every 500ms");
    info!("");

    let bytes_per_sample = 20u16;
    let mut batch_count = 0u32;

    // Get conversion factors for displaying physical units
    let accel_sensitivity = accel_config.full_scale.sensitivity();
    let gyro_sensitivity = gyro_config.full_scale.sensitivity();

    // Main loop: periodically read FIFO data in batches
    loop {
        // Wait for data to accumulate
        embassy_time::block_for(embassy_time::Duration::from_millis(500));

        // Check FIFO count
        let count = match imu.fifo_count() {
            Ok(c) => c,
            Err(e) => {
                error!("FIFO count error: {:?}", e);
                continue;
            }
        };

        if count == 0 {
            continue;
        }

        let num_samples = count / bytes_per_sample;

        if num_samples == 0 {
            continue;
        }

        batch_count += 1;
        info!(
            "Batch {}: {} bytes, {} samples",
            batch_count, count, num_samples
        );

        // Read FIFO data
        let mut buffer = [0u8; 512];
        let bytes_to_read = ((num_samples * bytes_per_sample) as usize).min(buffer.len());
        let actual_samples = bytes_to_read / bytes_per_sample as usize;

        if actual_samples < num_samples as usize {
            warn!(
                "Buffer limited: reading {} of {} samples",
                actual_samples, num_samples
            );
        }

        let bytes_read = match imu.fifo_read(&mut buffer[..bytes_to_read]) {
            Ok(n) => n,
            Err(e) => {
                error!("FIFO read error: {:?}", e);
                continue;
            }
        };

        // Parse FIFO data
        match imu.fifo_parse(&buffer[..bytes_read], &fifo_config.into()) {
            Ok(records) => {
                // Show first 2 and last sample from batch
                for (i, record) in records.iter().enumerate() {
                    if let (Some((ax, ay, az)), Some((gx, gy, gz))) = (record.accel, record.gyro) {
                        // Convert to physical units
                        let ax_g = ax as f32 / accel_sensitivity;
                        let ay_g = ay as f32 / accel_sensitivity;
                        let az_g = az as f32 / accel_sensitivity;

                        let gx_dps = gx as f32 / gyro_sensitivity;
                        let gy_dps = gy as f32 / gyro_sensitivity;
                        let gz_dps = gz as f32 / gyro_sensitivity;

                        if i < 2 || i >= records.len() - 1 {
                            if let Some((mx, my, mz)) = record.mag {
                                info!(
                                    "  [{}] A:({},{},{})g G:({},{},{})°/s M:({},{},{})μT",
                                    i, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, mx, my, mz
                                );
                            } else {
                                info!(
                                    "  [{}] A:({},{},{})g G:({},{},{})°/s",
                                    i, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps
                                );
                            }
                        } else if i == 2 {
                            info!("  ... {} more samples ...", records.len() - 3);
                        }
                    }
                }
            }
            Err(e) => {
                error!("FIFO parse error: {:?}", e);
            }
        }

        // Check for FIFO overflow
        match imu.fifo_overflow_status() {
            Ok(overflow) => {
                if overflow.any_overflow() {
                    warn!("FIFO overflow detected! Resetting...");
                    if let Err(e) = imu.fifo_reset() {
                        error!("Failed to reset FIFO: {:?}", e);
                    }
                }
            }
            Err(e) => {
                error!("Overflow status error: {:?}", e);
            }
        }

        info!("");
    }
}
