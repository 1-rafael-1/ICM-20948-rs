//! Magnetometer reading example for ICM-20948 on Raspberry Pi Pico 2 (Blocking version)
//!
//! This example demonstrates:
//! - Device initialization
//! - Magnetometer configuration via I2C master
//! - Reading magnetic field data in microteslas (μT)
//! - Applying magnetometer calibration for accurate readings
//! - Computing heading/yaw angle
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
//! - GND: GND
//! - AD0: GND (for address 0x68)
//!
//! Note: For best results, run the magnetometer_calibration example first to
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
use embassy_time::Delay;
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
    info!("ICM-20948 Magnetometer Example");
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

    // Configure magnetometer at 100Hz continuous mode
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

    // Apply magnetometer calibration
    // Run magnetometer_calibration example to get values specific to your sensor
    let mag_cal = MagCalibration {
        offset_x: -43.800003,
        offset_y: -107.7,
        offset_z: -20.400002,
        scale_x: 1.0,
        scale_y: 1.0,
        scale_z: 1.0,
    };
    imu.set_magnetometer_calibration(mag_cal);

    info!("Reading started (100Hz magnetometer, 1Hz output, calibrated)");
    info!("Rotate sensor to see heading changes");
    info!("");

    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    let mut sample_count = 0;

    // Main reading loop - read magnetometer and display values with heading
    loop {
        match imu.read_magnetometer() {
            Ok(mag_data) => {
                let magnitude = mag_data.magnitude();

                // Calculate heading (yaw) assuming sensor is level
                let heading_deg = match imu.read_magnetometer_heading() {
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
                    info!("--- {} samples ---", sample_count);
                }
            }
            Err(e) => error!("Mag read error: {:?}", e),
        }

        // Read at 1Hz to allow magnetometer buffer to update
        embassy_time::block_for(embassy_time::Duration::from_millis(1000));
    }
}
