//! Low-Power Mode Example for ICM-20948 on Raspberry Pi Pico 2
//!
//! This example demonstrates entering and using the ICM-20948 low-power mode.
//! Low-power mode reduces power consumption by duty-cycling the accelerometer
//! and disabling the gyroscope and magnetometer.
//!
//! In low-power mode:
//! - Accelerometer: Duty-cycled at 31.25 Hz (configurable)
//! - Gyroscope: DISABLED (to save power)
//! - Magnetometer: DISABLED (to save power)
//!
//! Potential use cases:
//! 1. Battery-powered applications requiring extended runtime
//! 2. Motion monitoring with reduced sampling requirements
//! 3. Applications that only need periodic accelerometer data
//! 4. Standby/idle states between active measurement periods
//!
//! Test procedure (this example):
//! 1. Device starts in normal mode (all sensors active)
//! 2. Enters low-power mode (accelerometer only at 31.25 Hz)
//! 3. Prints "MEASURE CURRENT NOW" - connect ammeter
//! 4. Reads accelerometer periodically to verify operation
//! 5. Exits low-power mode and returns to normal
//!

//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V (measure current on this line)
//! - GND: GND
//! - AD0: GND (for address 0x68)

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
    AccelConfig, AccelFullScale, GyroConfig, GyroFullScale, I2cInterface, Icm20948Driver,
    power::{LowPowerConfig, LowPowerRate, WomMode},
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
    info!("=== ICM-20948 Low-Power Mode Example ===");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400kHz
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_13, p.PIN_12, i2c_config);

    // Create and initialize driver
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

    // ========== PHASE 1: NORMAL MODE BASELINE ==========
    info!("PHASE 1: Normal Mode Operation");
    info!("Configuring sensors for normal operation...");

    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        sample_rate_div: 10,
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_accelerometer(accel_config) {
        error!("Failed to configure accelerometer: {:?}", e);
    }

    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        sample_rate_div: 10,
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config) {
        error!("Failed to configure gyroscope: {:?}", e);
    }

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    info!("Reading baseline data (normal mode, ~3.4 mA)...");
    info!("Sample rate: ~102 Hz");
    info!("Showing continuous data output for comparison:");
    info!("");

    // Read data for 10 seconds in normal mode with more frequent output
    for i in 0..20 {
        embassy_time::block_for(embassy_time::Duration::from_millis(500));

        match (imu.read_accelerometer(), imu.read_gyroscope()) {
            (Ok(accel), Ok(gyro)) => {
                info!(
                    "[{}] NORMAL MODE: Accel=({}, {}, {})g  Gyro=({}, {}, {})°/s",
                    i, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
                );
            }
            _ => warn!("Failed to read sensors"),
        }
    }

    info!("");
    info!("PHASE 1 complete. Normal mode baseline established.");
    embassy_time::block_for(embassy_time::Duration::from_millis(1000));

    // ========== PHASE 2: ENTER LOW-POWER MODE ==========
    info!("");
    info!("PHASE 2: Low-Power Mode");
    info!("Entering low-power mode...");

    let lp_config = LowPowerConfig {
        accel_rate: LowPowerRate::Hz31_25,
        enable_wake_on_motion: false, // Disable WoM for pure low-power test
        wom_threshold: 0,
        wom_mode: WomMode::CompareCurrentSample,
    };

    if let Err(e) = imu.enter_low_power_mode(&lp_config, &mut delay) {
        error!("Failed to enter low-power mode: {:?}", e);
        loop {
            embassy_time::block_for(embassy_time::Duration::from_millis(1000));
        }
    }

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    info!("Low-power mode active - measure current consumption now");
    info!("Accel: 31.25 Hz duty-cycled | Gyro/Mag: OFF");
    info!("");
    embassy_time::block_for(embassy_time::Duration::from_millis(2000));

    let mut sample_count = 0u32;
    let mut last_reading = (0.0f32, 0.0f32, 0.0f32);

    // Read for 20 seconds with more frequent output
    for i in 0..40 {
        embassy_time::block_for(embassy_time::Duration::from_millis(500));

        match imu.read_accelerometer() {
            Ok(accel) => {
                sample_count += 1;
                last_reading = (accel.x, accel.y, accel.z);

                // Print every sample for comparison (only accel is active)
                info!(
                    "[{}] LP MODE:    Accel=({}, {}, {})g  [Gyro/Mag: DISABLED]",
                    i, accel.x, accel.y, accel.z
                );
            }
            Err(e) => {
                warn!("[{}] Failed to read accelerometer: {:?}", i, e);
            }
        }
    }

    info!("");
    info!("Low-power mode test complete!");
    info!("  Samples read: {}", sample_count);
    info!(
        "  Last reading: ({}, {}, {})g",
        last_reading.0, last_reading.1, last_reading.2
    );

    info!("");
    info!("Exiting low-power mode...");

    if let Err(e) = imu.exit_low_power_mode() {
        error!("Failed to exit low-power mode: {:?}", e);
    }

    // Reconfigure sensors for normal mode
    if let Err(e) = imu.configure_accelerometer(accel_config) {
        error!("Failed to reconfigure accelerometer: {:?}", e);
    }

    if let Err(e) = imu.configure_gyroscope(gyro_config) {
        error!("Failed to reconfigure gyroscope: {:?}", e);
    }

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    info!("Normal mode restored");
    info!("");

    // Read data for 10 seconds to verify normal operation restored
    for i in 0..20 {
        embassy_time::block_for(embassy_time::Duration::from_millis(500));

        match (imu.read_accelerometer(), imu.read_gyroscope()) {
            (Ok(accel), Ok(gyro)) => {
                info!(
                    "[{}] NORMAL MODE: Accel=({}, {}, {})g  Gyro=({}, {}, {})°/s",
                    i, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
                );
            }
            _ => warn!("Failed to read sensors"),
        }
    }

    info!("");
    info!(
        "Low-power mode test complete - {} samples read",
        sample_count
    );

    loop {
        embassy_time::block_for(embassy_time::Duration::from_millis(5000));
        info!("Idle in normal mode...");
    }
}
