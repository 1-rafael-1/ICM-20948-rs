//! Magnetometer calibration example for ICM-20948 on Raspberry Pi Pico 2 (Blocking version)
//!
//! This example performs hard-iron calibration for the magnetometer by rotating
//! the sensor through all orientations and recording min/max values.
//!
//! Hard-iron calibration corrects for constant magnetic field offsets caused by
//! nearby ferromagnetic materials (e.g., PCB components, metal case).
//!
//! This example demonstrates:
//! - Magnetometer initialization
//! - Hard-iron calibration data collection
//! - Computing calibration offsets
//! - Saving calibration for later use
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
//! - GND: GND
//! - AD0: GND (for address 0x68)
//!
//! Instructions:
//! 1. Start the program
//! 2. When prompted, rotate the sensor slowly through ALL orientations:
//!    - Spin it flat (like a compass) - full 360°
//!    - Tilt it vertically and spin again - full 360°
//!    - Flip it upside down and spin again - full 360°
//!    - Roll it on all sides
//! 3. Goal: Get the magnetic field vector to point in every possible direction
//! 4. Continue for ~60 seconds or until min/max values stabilize
//! 5. Calibration offsets will be displayed at the end
//! 6. Copy the offsets and use them in your application

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
use icm20948::{I2cInterface, Icm20948Driver, MagConfig, MagMode};
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
    info!("ICM-20948 Magnetometer Calibration");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400kHz on pins 12(SDA)/13(SCL)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_13, p.PIN_12, i2c_config);

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

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

    // Display calibration instructions
    info!("INSTRUCTIONS:");
    info!("  1. Rotate sensor slowly through ALL orientations for 60 seconds");
    info!("  2. Required movements (~15s each):");
    info!("     - Spin flat (like a compass) - full 360°");
    info!("     - Tilt vertical and spin - full 360°");
    info!("     - Flip upside down and spin - full 360°");
    info!("     - Roll on all sides");
    info!("  3. Stay away from magnetic interference sources");
    info!("     (laptops, phones, speakers, metal desks)");
    info!("");

    // Countdown
    for i in (1..=5).rev() {
        info!("Starting in {}...", i);
        embassy_time::block_for(embassy_time::Duration::from_millis(1000));
    }
    info!("");
    info!("START ROTATING NOW!");
    info!("");

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    // Initialize min/max tracking for hard-iron calibration
    let mut min_x = f32::MAX;
    let mut max_x = f32::MIN;
    let mut min_y = f32::MAX;
    let mut max_y = f32::MIN;
    let mut min_z = f32::MAX;
    let mut max_z = f32::MIN;

    let num_samples = 600; // 60 seconds at 10Hz
    let mut valid_samples = 0;
    let mut failed_reads = 0;

    // Collect calibration data
    for i in 0..num_samples {
        match imu.read_magnetometer() {
            Ok(mag_data) => {
                // Track min/max for each axis
                if mag_data.x < min_x {
                    min_x = mag_data.x;
                }
                if mag_data.x > max_x {
                    max_x = mag_data.x;
                }
                if mag_data.y < min_y {
                    min_y = mag_data.y;
                }
                if mag_data.y > max_y {
                    max_y = mag_data.y;
                }
                if mag_data.z < min_z {
                    min_z = mag_data.z;
                }
                if mag_data.z > max_z {
                    max_z = mag_data.z;
                }

                valid_samples += 1;

                // Show progress every second
                if (i + 1) % 10 == 0 {
                    let elapsed = (i + 1) / 10;
                    let remaining = (num_samples - i - 1) / 10;

                    let offset_x = (max_x + min_x) * 0.5;
                    let offset_y = (max_y + min_y) * 0.5;
                    let offset_z = (max_z + min_z) * 0.5;

                    info!(
                        "[{}s] Offsets: X={} Y={} Z={} ({}s remaining)",
                        elapsed, offset_x, offset_y, offset_z, remaining
                    );
                }
            }
            Err(_) => {
                failed_reads += 1;
            }
        }

        embassy_time::block_for(embassy_time::Duration::from_millis(100));
    }

    info!("");
    info!("CALIBRATION COMPLETE!");
    info!("");

    // Calculate hard-iron offsets
    let offset_x = (max_x + min_x) * 0.5;
    let offset_y = (max_y + min_y) * 0.5;
    let offset_z = (max_z + min_z) * 0.5;

    let range_x = max_x - min_x;
    let range_y = max_y - min_y;
    let range_z = max_z - min_z;

    // Display results
    info!("═══════════════════════════════════════════════════════════");
    info!("CALIBRATION RESULTS");
    info!("═══════════════════════════════════════════════════════════");
    info!("Valid samples: {}/{}", valid_samples, num_samples);
    info!("Failed reads:  {}", failed_reads);
    info!("");
    info!("Copy these values to your code:");
    info!("");
    info!("let mag_cal = MagCalibration {{");
    info!("    offset_x: {},", offset_x);
    info!("    offset_y: {},", offset_y);
    info!("    offset_z: {},", offset_z);
    info!("    scale_x: 1.0,");
    info!("    scale_y: 1.0,");
    info!("    scale_z: 1.0,");
    info!("}};");
    info!("");
    info!("═══════════════════════════════════════════════════════════");
    info!("");

    // Quality assessment
    let avg_range = (range_x + range_y + range_z) / 3.0;
    let max_offset = libm::fmaxf(
        libm::fabsf(offset_x),
        libm::fmaxf(libm::fabsf(offset_y), libm::fabsf(offset_z)),
    );

    info!("Quality: Range={}μT MaxOffset={}μT", avg_range, max_offset);

    if avg_range < 60.0 {
        warn!("⚠ Low range - may need more rotation coverage");
    } else if avg_range > 150.0 {
        warn!("⚠ High range - possible magnetic interference");
    } else {
        info!("✓ Good calibration quality");
    }

    if max_offset > 100.0 {
        warn!("⚠ Large offset - strong magnetic interference detected");
    }

    info!("");

    // Apply calibration and show demo
    let mag_cal = icm20948::MagCalibration {
        offset_x,
        offset_y,
        offset_z,
        scale_x: 1.0,
        scale_y: 1.0,
        scale_z: 1.0,
    };
    imu.set_magnetometer_calibration(mag_cal);

    info!("Demo readings (calibrated):");
    info!("");

    for i in 0..10 {
        if let Ok(mag_data) = imu.read_magnetometer() {
            let magnitude = mag_data.magnitude();

            let heading_deg = match imu.read_magnetometer_heading() {
                Ok(h) => h,
                Err(_) => {
                    let heading_rad = libm::atan2(mag_data.y as f64, mag_data.x as f64);
                    let h = heading_rad * 180.0 / core::f64::consts::PI;
                    (if h < 0.0 { h + 360.0 } else { h }) as f32
                }
            };

            info!(
                "[{}] X={}μT Y={}μT Z={}μT Mag={}μT Heading={}°",
                i + 1,
                mag_data.x,
                mag_data.y,
                mag_data.z,
                magnitude,
                heading_deg
            );
        }

        embassy_time::block_for(embassy_time::Duration::from_millis(500));
    }
}
