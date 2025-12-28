//! Magnetometer reading example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates:
//! - Async device initialization
//! - Async magnetometer configuration via I2C master
//! - Async reading magnetic field data
//! - Converting to microteslas (μT)
//! - Applying magnetometer calibration for accurate readings
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
//! - GND: GND
//! - AD0: GND (for address 0x68)
//!
//! Note: For best results, run the magnetometer_calibration_async example first to
//! determine your specific calibration values, then update the MagCalibration
//! configuration below with your values.

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
use icm20948::{I2cInterface, Icm20948Driver, MagCalibration, MagConfig, MagMode};
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
    info!("ICM-20948 Magnetometer Example (Async)");
    info!("========================================");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    info!("I2C configured at 400kHz on pins 12(SDA)/13(SCL)");
    info!("Waiting for ICM-20948 to power up...");
    Timer::after_millis(100).await;

    // Create ICM-20948 driver with I2C interface (uses default address 0x68, AD0 pin LOW)
    info!("Attempting to detect ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface).await {
        Ok(imu) => {
            info!("✓ ICM-20948 detected successfully!");
            imu
        }
        Err(e) => {
            error!("✗ Failed to detect ICM-20948: {:?}", e);
            error!("This usually means:");
            error!("  1. Device not connected or powered");
            error!("  2. Wrong I2C address (check AD0 pin)");
            error!("  3. I2C pins swapped");
            error!("  4. Previous run left device in bad state - try power cycling");
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    // Initialize the device
    info!("");
    info!("Initializing ICM-20948 (soft reset + configuration)...");
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("✗ Failed to initialize ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    // Wait for device to stabilize after reset
    Timer::after_millis(100).await;
    info!("✓ ICM-20948 initialized successfully!");
    info!("");

    // Configure magnetometer
    info!("Configuring magnetometer...");
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz, // Continuous measurement at 100Hz
    };

    if let Err(e) = imu.init_magnetometer(mag_config, &mut delay).await {
        error!("Failed to initialize magnetometer: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    // Wait for magnetometer to stabilize
    Timer::after_millis(100).await;

    // Example: Apply magnetometer calibration
    // These values were obtained from running the magnetometer_calibration_async example.
    // For best accuracy, run that example yourself to get calibration values specific
    // to your sensor and environment.
    //
    // See: examples/rp2350-async/magnetometer_calibration_async.rs
    info!("Applying magnetometer calibration...");
    let mag_cal = MagCalibration {
        offset_x: -43.800003,
        offset_y: -107.7,
        offset_z: -20.400002,
        // Soft-iron correction (advanced): scaling factors to compensate for
        // distortion from nearby ferromagnetic materials. Leave these at 1.0
        // unless you have computed per-axis scale factors (e.g. from an
        // ellipsoid-fit calibration); 1.0 means "no additional scaling".
        scale_x: 1.0,
        scale_y: 1.0,
        scale_z: 1.0,
    };
    imu.set_magnetometer_calibration(mag_cal);
    info!("✓ Calibration applied!");
    info!("");

    info!("Magnetometer initialized successfully");
    info!("Starting magnetometer readings (rotate sensor to see heading changes)...");
    info!("");

    // Wait for first data to be available
    Timer::after_millis(100).await;

    let mut sample_count = 0;

    // Main reading loop
    loop {
        // Read magnetometer data in microteslas (μT)
        // Note: Readings are automatically calibrated using the values set above
        match imu.read_magnetometer().await {
            Ok(mag_data) => {
                // Calculate magnetic field magnitude
                let magnitude = mag_data.magnitude();

                // Calculate heading (yaw) assuming the sensor is level
                let heading_deg = match imu.read_magnetometer_heading().await {
                    Ok(h) => h,
                    Err(_) => {
                        let heading_rad = libm::atan2(mag_data.y as f64, mag_data.x as f64);
                        let h = heading_rad * 180.0 / core::f64::consts::PI;
                        (if h < 0.0 { h + 360.0 } else { h }) as f32
                    }
                };

                info!(
                    "Mag: X={}μT Y={}μT Z={}μT |Mag|={}μT Heading={}°",
                    mag_data.x, mag_data.y, mag_data.z, magnitude, heading_deg
                );

                sample_count += 1;
                if sample_count % 50 == 0 {
                    info!("--- {} samples collected ---", sample_count);
                }
            }
            Err(e) => error!("Failed to read magnetometer: {:?}", e),
        }

        // Read at 1 Hz to allow magnetometer buffer to update reliably
        Timer::after_millis(1000).await;
    }
}
