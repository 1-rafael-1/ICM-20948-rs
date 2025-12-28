//! Nine-axis sensor reading example for ICM-20948 on Raspberry Pi Pico 2 (Blocking version)
//!
//! This example demonstrates:
//! - Async reading of all 9 axes (accelerometer, gyroscope, magnetometer)
//! - Async temperature monitoring
//! - Combined sensor fusion data output
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
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
use embassy_time::{Delay};
use icm20948::{
    AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale, I2cInterface,
    Icm20948Driver, MagConfig, MagMode,
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
    info!("ICM-20948 Nine-Axis Reading Example (Async)");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_13, p.PIN_12, i2c_config);

    // Create ICM-20948 driver with I2C interface
    // Using default address (0x68, AD0 pin LOW) - most common configuration
    // If your board has AD0 pulled HIGH, use:
    //   let i2c_interface = I2cInterface::alternative(i2c);
    info!("Initializing ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface) {
        Ok(imu) => {
            info!("ICM-20948 detected!");
            imu
        }
        Err(e) => {
            error!("Failed to detect ICM-20948: {:?}", e);
            loop {
                embassy_time::block_for(embassy_time::Duration::from_millis(1000));
            }
        }
    };

    // Initialize the device with async delay
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay) {
        error!("Failed to initialize ICM-20948: {:?}", e);
        loop {
            embassy_time::block_for(embassy_time::Duration::from_millis(1000));
        }
    }

    // Wait for device to stabilize after reset
    embassy_time::block_for(embassy_time::Duration::from_millis(100));
    info!("ICM-20948 initialized successfully!");

    // Configure accelerometer: ±2g range, 246 Hz DLPF
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        dlpf: AccelDlpf::Hz246,
        dlpf_enable: true,
        sample_rate_div: 10, // ~102 Hz sample rate
    };

    if let Err(e) = imu.configure_accelerometer(accel_config) {
        error!("Failed to configure accelerometer: {:?}", e);
    } else {
        info!("Accelerometer configured: ±2g, 246Hz DLPF");
    }

    // Configure gyroscope: ±250°/s range, 197 Hz DLPF
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        dlpf: GyroDlpf::Hz197,
        dlpf_enable: true,
        sample_rate_div: 10, // ~102 Hz sample rate
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config) {
        error!("Failed to configure gyroscope: {:?}", e);
    } else {
        info!("Gyroscope configured: ±250°/s, 197Hz DLPF");
    }

    // Configure magnetometer: Continuous 100Hz mode
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    if let Err(e) = imu.init_magnetometer(mag_config, &mut delay) {
        error!("Failed to initialize magnetometer: {:?}", e);
    } else {
        info!("Magnetometer configured: Continuous 100Hz");
    }

    // Wait for magnetometer to initialize
    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    info!("");
    info!("All sensors configured successfully!");
    info!("Starting 9-axis sensor readings...");
    info!("");

    // Wait for all sensors to stabilize
    embassy_time::block_for(embassy_time::Duration::from_millis(200));

    let mut sample_count = 0;

    // Main reading loop
    loop {
        sample_count += 1;

        // Read accelerometer
        let accel_data = match imu.read_accelerometer() {
            Ok(data) => {
                let magnitude = data.magnitude();
                info!(
                    "Accel: X={}g Y={}g Z={}g |Mag|={}g",
                    data.x, data.y, data.z, magnitude
                );
                Some(data)
            }
            Err(e) => {
                error!("Failed to read accelerometer: {:?}", e);
                None
            }
        };

        // Read gyroscope
        let gyro_data = match imu.read_gyroscope() {
            Ok(data) => {
                info!("Gyro:  X={}°/s Y={}°/s Z={}°/s", data.x, data.y, data.z);
                Some(data)
            }
            Err(e) => {
                error!("Failed to read gyroscope: {:?}", e);
                None
            }
        };

        // Read magnetometer
        let _mag_data = match imu.read_magnetometer() {
            Ok(data) => {
                let magnitude = data.magnitude();

                // Calculate heading (assuming sensor is level)
                let heading_rad = libm::atan2(data.y as f64, data.x as f64);
                let mut heading_deg = heading_rad * 180.0 / core::f64::consts::PI;
                if heading_deg < 0.0 {
                    heading_deg += 360.0;
                }

                info!(
                    "Mag:   X={}μT Y={}μT Z={}μT |Mag|={}μT Heading={}°",
                    data.x, data.y, data.z, magnitude, heading_deg
                );
                Some(data)
            }
            Err(e) => {
                error!("Failed to read magnetometer: {:?}", e);
                None
            }
        };

        // Read temperature
        match imu.read_temperature_celsius() {
            Ok(temp_c) => {
                info!("Temp:  {}°C", temp_c);
            }
            Err(e) => error!("Failed to read temperature: {:?}", e),
        }

        // Calculate tilt angles if we have accelerometer data
        if let Some(accel) = accel_data {
            // Roll (rotation around X-axis)
            let roll_rad = libm::atan2(accel.y as f64, accel.z as f64);
            let roll_deg = roll_rad * 180.0 / core::f64::consts::PI;

            // Pitch (rotation around Y-axis)
            let pitch_rad = libm::atan2(
                -accel.x as f64,
                libm::sqrt((accel.y * accel.y + accel.z * accel.z) as f64),
            );
            let pitch_deg = pitch_rad * 180.0 / core::f64::consts::PI;

            info!("Tilt:  Roll={}° Pitch={}°", roll_deg, pitch_deg);
        }

        // Show motion detection
        if let (Some(accel), Some(gyro)) = (accel_data, gyro_data) {
            let accel_mag = accel.magnitude();
            let gyro_mag = libm::sqrt((gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z) as f64);

            // Simple motion detection thresholds
            let is_moving = (accel_mag - 1.0).abs() > 0.1; // More than 0.1g deviation from 1g
            let is_rotating = gyro_mag > 10.0; // More than 10°/s rotation

            if is_moving || is_rotating {
                if is_moving && is_rotating {
                    info!("Status: MOVING + ROTATING");
                } else if is_moving {
                    info!("Status: MOVING");
                } else {
                    info!("Status: ROTATING");
                }
            } else {
                info!("Status: STATIONARY");
            }
        }

        info!("");

        // Show periodic summary
        if sample_count % 50 == 0 {
            info!("=== {} samples collected ===", sample_count);
            info!("");
        }

        // Read at ~10 Hz
        embassy_time::block_for(embassy_time::Duration::from_millis(100));
    }
}
