//! Basic sensor reading example for ICM-20948 on Raspberry Pi Pico 2 (Blocking version)
//!
//! This example demonstrates:
//! - Device initialization
//! - Accelerometer configuration and reading
//! - Gyroscope configuration and reading
//! - Temperature reading
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
use embassy_time::Delay;
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
    info!("ICM-20948 Basic Reading Example");

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

    // Configure accelerometer: ±2g, 246Hz DLPF, ~102Hz sample rate
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        dlpf: AccelDlpf::Hz246,
        dlpf_enable: true,
        sample_rate_div: 10,
    };

    if let Err(e) = imu.configure_accelerometer(accel_config) {
        error!("Failed to configure accelerometer: {:?}", e);
    }

    // Configure gyroscope: ±250°/s, 197Hz DLPF, ~102Hz sample rate
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        dlpf: GyroDlpf::Hz197,
        dlpf_enable: true,
        sample_rate_div: 10,
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config) {
        error!("Failed to configure gyroscope: {:?}", e);
    }

    info!("Reading started (±2g accel, ±250°/s gyro, ~10Hz output)");
    info!("");

    // Main reading loop - read all sensors and display values
    loop {
        match imu.read_accelerometer() {
            Ok(accel_data) => {
                let magnitude = accel_data.magnitude();
                info!(
                    "Accel: X={}g Y={}g Z={}g |Mag|={}g",
                    accel_data.x, accel_data.y, accel_data.z, magnitude
                );
            }
            Err(e) => error!("Accel read error: {:?}", e),
        }

        match imu.read_gyroscope() {
            Ok(gyro_data) => {
                info!(
                    "Gyro:  X={}°/s Y={}°/s Z={}°/s",
                    gyro_data.x, gyro_data.y, gyro_data.z
                );
            }
            Err(e) => error!("Gyro read error: {:?}", e),
        }

        match imu.read_temperature_celsius() {
            Ok(temp_c) => {
                info!("Temp:  {}°C", temp_c);
            }
            Err(e) => error!("Temp read error: {:?}", e),
        }

        info!("");

        // Read at ~10 Hz
        embassy_time::block_for(embassy_time::Duration::from_millis(1000));
    }
}
