//! Sensor calibration example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates sensor calibration in a typical household environment
//! where some vibration from appliances, foot traffic, and ambient activity is expected.
//!
//! Configuration optimized for household calibration:
//! - Low-pass filter: 6Hz (filters out high-frequency vibrations)
//! - Motion detection: Only for accelerometer (gyro bias drift is normal, not motion)
//! - Sample rate: ~20Hz (stable readings with good averaging)
//!
//! This example demonstrates:
//! - Gyroscope calibration (zero-rate offset calibration)
//! - Accelerometer calibration (level surface with Z-axis up)
//! - Automatic application of calibration offsets
//! - Validation of calibration quality
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
//! - GND: GND
//! - AD0: GND (for address 0x68)
//!
//! Instructions:
//! 1. Place the device on a level surface
//! 2. Keep it reasonably still during calibration (normal household vibrations are OK)
//! 3. Follow the on-screen prompts

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
    AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale, I2cInterface,
    Icm20948Driver,
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
    info!("ICM-20948 Calibration Example (Async) - Household Environment");
    info!("==============================================================");
    info!("");

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

    // Initialize the device with async delay
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("Failed to initialize ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    // Wait for device to stabilize after reset
    Timer::after_millis(100).await;
    info!("ICM-20948 initialized successfully!");
    info!("");
    info!("Configuration optimized for household environments:");
    info!("- Filters out vibrations from appliances, foot traffic, etc.");
    info!("- Tolerates normal household activity during calibration");
    info!("");

    // Configure accelerometer with household-friendly settings
    // - 6Hz DLPF filters out high-frequency vibrations
    // - Slower sample rate provides stable, averaged readings
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        dlpf: AccelDlpf::Hz6, // Low frequency filters vibrations
        dlpf_enable: true,
        sample_rate_div: 50, // ~20Hz sample rate for stability
    };

    if let Err(e) = imu.configure_accelerometer(accel_config).await {
        error!("Failed to configure accelerometer: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }
    info!("Accelerometer configured: ±2g, 6Hz DLPF");

    // Configure gyroscope with household-friendly settings
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        dlpf: GyroDlpf::Hz6, // Low frequency filters vibrations
        dlpf_enable: true,
        sample_rate_div: 50, // ~20Hz sample rate for stability
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config).await {
        error!("Failed to configure gyroscope: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }
    info!("Gyroscope configured: ±250°/s, 6Hz DLPF");
    info!("");

    // Wait for low-pass filters to stabilize
    // With 6Hz DLPF, allow extra time for the filter to fully settle
    info!("Waiting for sensors to stabilize...");
    Timer::after_millis(5000).await;

    // ========================================================================
    // STEP 1: Show uncalibrated readings
    // ========================================================================
    info!("=== UNCALIBRATED READINGS ===");
    info!("Taking 10 samples to show uncalibrated state...");
    info!("");

    for i in 0..10 {
        if let Ok(accel_data) = imu.read_accelerometer().await {
            info!(
                "Sample {}: Accel X={}g Y={}g Z={}g",
                i + 1,
                accel_data.x,
                accel_data.y,
                accel_data.z
            );
        }

        if let Ok(gyro_data) = imu.read_gyroscope().await {
            info!(
                "Sample {}: Gyro  X={}°/s Y={}°/s Z={}°/s",
                i + 1,
                gyro_data.x,
                gyro_data.y,
                gyro_data.z
            );
        }

        Timer::after_millis(100).await;
    }

    info!("");
    info!("Note: Ideally, gyro should read near 0 and accel Z should be near 1g");
    info!("");

    // ========================================================================
    // STEP 2: Calibrate Gyroscope
    // ========================================================================
    info!("=== GYROSCOPE CALIBRATION ===");
    info!("Place device on a stable surface (normal household vibrations are OK)");
    info!("Calibration starts in 3 seconds...");
    Timer::after_millis(1000).await;
    info!("3...");
    Timer::after_millis(1000).await;
    info!("2...");
    Timer::after_millis(1000).await;
    info!("1...");
    Timer::after_millis(1000).await;
    info!("");
    info!("Calibrating gyroscope (200 samples, ~10 seconds)...");

    // Use very lenient threshold (100) for gyroscope calibration in household environments.
    // MEMS gyros exhibit significant bias drift (0.5-1°/s) even when perfectly stationary,
    // plus household vibrations add more variance. A threshold of 100 allows ~1.3 LSB variance
    // at ±250°/s, which effectively disables motion detection while still catching gross movement.
    // This is appropriate because we're measuring bias drift, not detecting motion.
    let gyro_cal = match imu.calibrate_gyroscope_with_threshold(200, 100).await {
        Ok(cal) => {
            info!("Gyroscope calibration complete!");
            info!(
                "Offsets: X={} Y={} Z={}",
                cal.offset_x, cal.offset_y, cal.offset_z
            );
            info!("Gyroscope calibration automatically applied!");
            cal
        }
        Err(e) => {
            error!("Failed to calibrate gyroscope: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };
    info!("");

    // ========================================================================
    // STEP 3: Calibrate Accelerometer
    // ========================================================================
    info!("=== ACCELEROMETER CALIBRATION ===");
    info!("Ensure device is level with Z-axis pointing UP");
    info!("Calibration starts in 3 seconds...");
    Timer::after_millis(1000).await;
    info!("3...");
    Timer::after_millis(1000).await;
    info!("2...");
    Timer::after_millis(1000).await;
    info!("1...");
    Timer::after_millis(1000).await;
    info!("");
    info!("Calibrating accelerometer (200 samples, ~10 seconds)...");

    // Use 10% threshold (divisor=10) - accelerometer has motion detection enabled
    // because gravity provides a stable reference. Calibration waits for data ready
    // to ensure independent samples at the configured sensor update rate.
    let accel_cal = match imu.calibrate_accelerometer_with_threshold(200, 10).await {
        Ok(cal) => {
            info!("Accelerometer calibration complete!");
            info!(
                "Offsets: X={} Y={} Z={}",
                cal.offset_x, cal.offset_y, cal.offset_z
            );
            info!(
                "Scales:  X={} Y={} Z={}",
                cal.scale_x, cal.scale_y, cal.scale_z
            );
            info!("Accelerometer calibration automatically applied!");
            cal
        }
        Err(e) => {
            error!("Failed to calibrate accelerometer: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };
    info!("");

    // ========================================================================
    // STEP 4: Validate calibration
    // ========================================================================
    info!("=== CALIBRATION VALIDATION ===");
    info!("Taking 20 samples to validate calibration...");
    info!("");

    Timer::after_millis(500).await;

    let mut accel_sum_x = 0.0;
    let mut accel_sum_y = 0.0;
    let mut accel_sum_z = 0.0;
    let mut gyro_sum_x = 0.0;
    let mut gyro_sum_y = 0.0;
    let mut gyro_sum_z = 0.0;
    let validation_samples = 20;

    for i in 0..validation_samples {
        if let Ok(accel_data) = imu.read_accelerometer().await {
            accel_sum_x += accel_data.x;
            accel_sum_y += accel_data.y;
            accel_sum_z += accel_data.z;

            info!(
                "Sample {}: Accel X={}g Y={}g Z={}g",
                i + 1,
                accel_data.x,
                accel_data.y,
                accel_data.z
            );
        }

        if let Ok(gyro_data) = imu.read_gyroscope().await {
            gyro_sum_x += gyro_data.x;
            gyro_sum_y += gyro_data.y;
            gyro_sum_z += gyro_data.z;

            info!(
                "Sample {}: Gyro  X={}°/s Y={}°/s Z={}°/s",
                i + 1,
                gyro_data.x,
                gyro_data.y,
                gyro_data.z
            );
        }

        Timer::after_millis(100).await;
    }

    info!("");
    info!("=== CALIBRATION RESULTS ===");

    let accel_avg_x = accel_sum_x / validation_samples as f32;
    let accel_avg_y = accel_sum_y / validation_samples as f32;
    let accel_avg_z = accel_sum_z / validation_samples as f32;
    let gyro_avg_x = gyro_sum_x / validation_samples as f32;
    let gyro_avg_y = gyro_sum_y / validation_samples as f32;
    let gyro_avg_z = gyro_sum_z / validation_samples as f32;

    info!("Accelerometer averages:");
    info!("  X: {}g (should be ~0.00)", accel_avg_x);
    info!("  Y: {}g (should be ~0.00)", accel_avg_y);
    info!("  Z: {}g (should be ~1.00)", accel_avg_z);
    info!("");

    info!("Gyroscope averages:");
    info!("  X: {}°/s (should be ~0.0)", gyro_avg_x);
    info!("  Y: {}°/s (should be ~0.0)", gyro_avg_y);
    info!("  Z: {}°/s (should be ~0.0)", gyro_avg_z);
    info!("");

    // Calculate RMS errors
    let accel_error_x = accel_avg_x.abs();
    let accel_error_y = accel_avg_y.abs();
    let accel_error_z = (accel_avg_z - 1.0).abs();
    let gyro_error_x = gyro_avg_x.abs();
    let gyro_error_y = gyro_avg_y.abs();
    let gyro_error_z = gyro_avg_z.abs();

    info!("Calibration quality:");
    info!("  Accel X error: {}g", accel_error_x);
    info!("  Accel Y error: {}g", accel_error_y);
    info!("  Accel Z error: {}g", accel_error_z);
    info!("  Gyro X error:  {}°/s", gyro_error_x);
    info!("  Gyro Y error:  {}°/s", gyro_error_y);
    info!("  Gyro Z error:  {}°/s", gyro_error_z);
    info!("");

    // Quality assessment
    let accel_good = accel_error_x < 0.02 && accel_error_y < 0.02 && accel_error_z < 0.02;
    let gyro_good = gyro_error_x < 1.0 && gyro_error_y < 1.0 && gyro_error_z < 1.0;

    if accel_good && gyro_good {
        info!("✓ Calibration quality: EXCELLENT");
    } else if accel_good || gyro_good {
        info!("✓ Calibration quality: GOOD");
        if !accel_good {
            warn!("  Accelerometer calibration could be improved");
            warn!("  Ensure device is level during calibration");
        }
        if !gyro_good {
            warn!("  Gyroscope calibration could be improved");
            warn!("  Try to minimize movement during calibration");
        }
    } else {
        warn!("✗ Calibration quality: NEEDS IMPROVEMENT");
        warn!("  Try recalibrating in a quieter location");
    }

    info!("");
    info!("=== CONTINUOUS MONITORING ===");
    info!("Monitoring calibrated sensor readings...");
    info!("(Press reset to recalibrate)");
    info!("");

    info!("Calibration data:");
    info!("  Accel: {:?}", accel_cal);
    info!("  Gyro:  {:?}", gyro_cal);
    info!("");

    // ========================================================================
    // STEP 5: Continuous monitoring with calibration applied
    // ========================================================================
    loop {
        if let Ok(accel_data) = imu.read_accelerometer().await {
            let magnitude = accel_data.magnitude();
            info!(
                "Accel: X={}g Y={}g Z={}g |Mag|={}g",
                accel_data.x, accel_data.y, accel_data.z, magnitude
            );
        }

        if let Ok(gyro_data) = imu.read_gyroscope().await {
            info!(
                "Gyro:  X={}°/s Y={}°/s Z={}°/s",
                gyro_data.x, gyro_data.y, gyro_data.z
            );
        }

        info!("");
        Timer::after_millis(500).await;
    }
}
