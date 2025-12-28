//! Basic sensor reading example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates:
//! - Async device initialization
//! - Async accelerometer configuration and reading
//! - Async gyroscope configuration and reading
//! - Async temperature reading
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
    info!("ICM-20948 Basic Reading Example (Async)");

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

    // Configure accelerometer: ±2g range, 246 Hz DLPF
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        dlpf: AccelDlpf::Hz246,
        dlpf_enable: true,
        sample_rate_div: 10, // ~102 Hz sample rate
    };

    if let Err(e) = imu.configure_accelerometer(accel_config).await {
        error!("Failed to configure accelerometer: {:?}", e);
    } else {
        info!("Accelerometer configured: ±2g, 246Hz DLPF, ~102Hz sample rate");
    }

    // Configure gyroscope: ±250°/s range, 197 Hz DLPF
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        dlpf: GyroDlpf::Hz197,
        dlpf_enable: true,
        sample_rate_div: 10, // ~102 Hz sample rate
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config).await {
        error!("Failed to configure gyroscope: {:?}", e);
    } else {
        info!("Gyroscope configured: ±250°/s, 197Hz DLPF, ~102Hz sample rate");
    }

    info!("Starting sensor readings...");
    info!("");

    // Main reading loop
    loop {
        // Read accelerometer data in g-force
        match imu.read_accelerometer().await {
            Ok(accel_data) => {
                let magnitude = accel_data.magnitude();
                info!(
                    "Accel: X={}g Y={}g Z={}g |Mag|={}g",
                    accel_data.x, accel_data.y, accel_data.z, magnitude
                );
            }
            Err(e) => error!("Failed to read accelerometer: {:?}", e),
        }

        // Read gyroscope data in degrees per second
        match imu.read_gyroscope().await {
            Ok(gyro_data) => {
                info!(
                    "Gyro:  X={}°/s Y={}°/s Z={}°/s",
                    gyro_data.x, gyro_data.y, gyro_data.z
                );
            }
            Err(e) => error!("Failed to read gyroscope: {:?}", e),
        }

        // Read temperature
        match imu.read_temperature_celsius().await {
            Ok(temp_c) => {
                info!("Temp:  {}°C", temp_c);
            }
            Err(e) => error!("Failed to read temperature: {:?}", e),
        }

        info!("");

        // Read at ~10 Hz
        Timer::after_millis(100).await;
    }
}
