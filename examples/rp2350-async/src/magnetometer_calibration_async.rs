//! Magnetometer calibration example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
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
use embassy_time::{Delay, Timer};
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
    info!("===================================");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    info!("I2C configured at 400kHz on pins 12(SDA)/13(SCL)");
    info!("Waiting for ICM-20948 to power up...");
    Timer::after_millis(100).await;

    // Create ICM-20948 driver with I2C interface
    info!("Attempting to detect ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface).await {
        Ok(imu) => {
            info!("✓ ICM-20948 detected successfully!");
            imu
        }
        Err(e) => {
            error!("✗ Failed to detect ICM-20948: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    // Initialize the device
    info!("");
    info!("Initializing ICM-20948...");
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("✗ Failed to initialize ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    Timer::after_millis(100).await;
    info!("✓ ICM-20948 initialized successfully!");
    info!("");

    // Configure magnetometer
    info!("Configuring magnetometer...");
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    if let Err(e) = imu.init_magnetometer(mag_config, &mut delay).await {
        error!("Failed to initialize magnetometer: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    Timer::after_millis(100).await;
    info!("✓ Magnetometer initialized successfully!");
    info!("");

    // Instructions
    info!("╔════════════════════════════════════════════════════════════════╗");
    info!("║          MAGNETOMETER CALIBRATION INSTRUCTIONS                ║");
    info!("╠════════════════════════════════════════════════════════════════╣");
    info!("║ 1. When the countdown finishes, slowly rotate the sensor      ║");
    info!("║    through ALL possible orientations                          ║");
    info!("║                                                                ║");
    info!("║ 2. Required movements (spend ~15 seconds on each):            ║");
    info!("║    → Spin flat (like a compass) - full 360°                   ║");
    info!("║    → Tilt vertical and spin again - full 360°                 ║");
    info!("║    → Flip upside down and spin - full 360°                    ║");
    info!("║    → Roll on all sides                                        ║");
    info!("║                                                                ║");
    info!("║ 3. Continue for 60 seconds or until ranges stabilize          ║");
    info!("║                                                                ║");
    info!("║ 4. IMPORTANT: Stay away from:                                 ║");
    info!("║    ✗ Laptop/phone (magnetic field interference)               ║");
    info!("║    ✗ Speakers, motors, power supplies                         ║");
    info!("║    ✗ Metal desks, steel cabinets                              ║");
    info!("║                                                                ║");
    info!("║ 5. Calibration will complete automatically                    ║");
    info!("╚════════════════════════════════════════════════════════════════╝");
    info!("");

    // Countdown
    for i in (1..=5).rev() {
        info!("Starting in {} seconds...", i);
        Timer::after_millis(1000).await;
    }
    info!("");
    info!("🔄 CALIBRATION STARTED - START ROTATING THE SENSOR NOW!");
    info!("");

    // Wait for first data
    Timer::after_millis(100).await;

    // Initialize min/max tracking
    let mut min_x = f32::MAX;
    let mut max_x = f32::MIN;
    let mut min_y = f32::MAX;
    let mut max_y = f32::MIN;
    let mut min_z = f32::MAX;
    let mut max_z = f32::MIN;

    let num_samples = 600; // 60 seconds at 10Hz
    let mut valid_samples = 0;
    let mut failed_reads = 0;

    for i in 0..num_samples {
        match imu.read_magnetometer().await {
            Ok(mag_data) => {
                // Update min/max
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

                // Display progress every 10 samples (1 second)
                if (i + 1) % 10 == 0 {
                    let elapsed = (i + 1) / 10;
                    let remaining = (num_samples - i - 1) / 10;

                    // Calculate current ranges
                    let range_x = max_x - min_x;
                    let range_y = max_y - min_y;
                    let range_z = max_z - min_z;

                    // Calculate current offsets
                    let offset_x = (max_x + min_x) * 0.5;
                    let offset_y = (max_y + min_y) * 0.5;
                    let offset_z = (max_z + min_z) * 0.5;

                    info!(
                        "[{}s/60s] X:[{} to {}] Y:[{} to {}] Z:[{} to {}]",
                        elapsed, min_x, max_x, min_y, max_y, min_z, max_z
                    );
                    info!(
                        "         Ranges: X={}μT Y={}μT Z={}μT | Offsets: X={} Y={} Z={}",
                        range_x, range_y, range_z, offset_x, offset_y, offset_z
                    );
                    info!(
                        "         Remaining: {}s | Keep rotating through all orientations!",
                        remaining
                    );
                    info!("");
                }
            }
            Err(_) => {
                failed_reads += 1;
            }
        }

        // Sample at ~10Hz
        Timer::after_millis(100).await;
    }

    info!("");
    info!("✓ CALIBRATION COMPLETE!");
    info!("");

    // Calculate calibration offsets (hard-iron correction)
    let offset_x = (max_x + min_x) * 0.5;
    let offset_y = (max_y + min_y) * 0.5;
    let offset_z = (max_z + min_z) * 0.5;

    // Calculate ranges (for quality assessment)
    let range_x = max_x - min_x;
    let range_y = max_y - min_y;
    let range_z = max_z - min_z;

    info!("╔════════════════════════════════════════════════════════════════╗");
    info!("║                    CALIBRATION RESULTS                         ║");
    info!("╠════════════════════════════════════════════════════════════════╣");
    info!(
        "║ Valid samples: {}/{}                                      ║",
        valid_samples, num_samples
    );
    info!(
        "║ Failed reads:  {}                                           ║",
        failed_reads
    );
    info!("║                                                                ║");
    info!("║ Min/Max Values (μT):                                           ║");
    info!(
        "║   X: {} to {}                              ║",
        min_x, max_x
    );
    info!(
        "║   Y: {} to {}                              ║",
        min_y, max_y
    );
    info!(
        "║   Z: {} to {}                              ║",
        min_z, max_z
    );
    info!("║                                                                ║");
    info!("║ Ranges (μT):                                                   ║");
    info!(
        "║   X: {} μT                                            ║",
        range_x
    );
    info!(
        "║   Y: {} μT                                            ║",
        range_y
    );
    info!(
        "║   Z: {} μT                                            ║",
        range_z
    );
    info!("║                                                                ║");
    info!("║ ★ HARD-IRON OFFSETS (copy these to your code):                ║");
    info!("║                                                                ║");
    info!(
        "║   offset_x: {} μT                                    ║",
        offset_x
    );
    info!(
        "║   offset_y: {} μT                                    ║",
        offset_y
    );
    info!(
        "║   offset_z: {} μT                                    ║",
        offset_z
    );
    info!("╚════════════════════════════════════════════════════════════════╝");
    info!("");

    // Quality assessment
    info!("Quality Assessment:");
    info!("──────────────────");

    // Ideal range for Earth's magnetic field is ~50-100μT
    let avg_range = (range_x + range_y + range_z) / 3.0;

    if avg_range < 60.0 {
        warn!("⚠ WARNING: Average range ({} μT) is LOW", avg_range);
        warn!("   You may not have rotated the sensor through all orientations.");
        warn!("   Ideal range: 80-120 μT for good calibration coverage.");
    } else if avg_range > 150.0 {
        warn!("⚠ WARNING: Average range ({} μT) is HIGH", avg_range);
        warn!("   This may indicate magnetic interference or very large offsets.");
        warn!("   Try calibrating far from magnetic sources.");
    } else {
        info!("✓ Average range: {} μT - GOOD", avg_range);
    }

    // Check if offsets are reasonable (should be < 100μT for typical sensors)
    let max_offset = libm::fmaxf(
        libm::fabsf(offset_x),
        libm::fmaxf(libm::fabsf(offset_y), libm::fabsf(offset_z)),
    );
    if max_offset > 100.0 {
        warn!("⚠ WARNING: Large offset detected ({} μT)", max_offset);
        warn!("   This suggests strong magnetic interference nearby.");
        warn!("   Consider recalibrating in a different location.");
    } else {
        info!("✓ Maximum offset: {} μT - REASONABLE", max_offset);
    }

    info!("");
    info!("Usage Instructions:");
    info!("──────────────────");
    info!("1. Copy the offset values above");
    info!("2. In your application code, create a MagCalibration:");
    info!("");
    info!("   use icm20948::MagCalibration;");
    info!("");
    info!("   let mag_cal = MagCalibration {{");
    info!("       offset_x: {},", offset_x);
    info!("       offset_y: {},", offset_y);
    info!("       offset_z: {},", offset_z);
    info!("       // Soft-iron correction (advanced): per-axis scaling factors");
    info!("       // Hard-iron: constant offsets from ferromagnetic materials (above)");
    info!("       // Soft-iron: axis-dependent distortion from nearby metal");
    info!("       // Leave at 1.0 unless you've computed ellipsoid-fit scale factors");
    info!("       scale_x: 1.0,");
    info!("       scale_y: 1.0,");
    info!("       scale_z: 1.0,");
    info!("   }};");
    info!("");
    info!("3. Apply calibration to your IMU:");
    info!("   imu.set_magnetometer_calibration(mag_cal);");
    info!("");
    info!("4. All magnetometer readings will now be automatically corrected!");
    info!("");

    // Apply calibration and show corrected readings
    let mag_cal = icm20948::MagCalibration {
        offset_x,
        offset_y,
        offset_z,
        scale_x: 1.0,
        scale_y: 1.0,
        scale_z: 1.0,
    };
    imu.set_magnetometer_calibration(mag_cal);

    info!("Demo: Showing calibrated readings (keep sensor still)...");
    info!("");

    for i in 0..10 {
        if let Ok(mag_data) = imu.read_magnetometer().await {
            let magnitude = mag_data.magnitude();

            // Calculate heading
            let heading_deg = match imu.read_magnetometer_heading().await {
                Ok(h) => h,
                Err(_) => {
                    let heading_rad = libm::atan2(mag_data.y as f64, mag_data.x as f64);
                    let h = heading_rad * 180.0 / core::f64::consts::PI;
                    (if h < 0.0 { h + 360.0 } else { h }) as f32
                }
            };

            info!(
                "[{}] Calibrated: X={}μT Y={}μT Z={}μT |Mag|={}μT Heading={}°",
                i + 1,
                mag_data.x,
                mag_data.y,
                mag_data.z,
                magnitude,
                heading_deg
            );
        }

        Timer::after_millis(500).await;
    }

    info!("");
    info!("Calibration complete! You can now use these offsets in your application.");

    loop {
        Timer::after_millis(1000).await;
    }
}
