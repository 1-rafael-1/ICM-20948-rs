//! Sensor configuration presets example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates:
//! - Different configuration presets for various use cases
//! - Switching between configurations on the fly
//! - Understanding the trade-offs between settings
//! - Sample rate and DLPF configuration
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

/// Configuration preset for tilt sensing and orientation detection
fn tilt_sensing_config() -> (AccelConfig, GyroConfig) {
    let accel = AccelConfig {
        full_scale: AccelFullScale::G2, // ±2g for precise tilt measurement
        dlpf: AccelDlpf::Hz111,         // Low noise for stable readings
        dlpf_enable: true,
        sample_rate_div: 20, // ~51 Hz (slow update rate is fine)
    };

    let gyro = GyroConfig {
        full_scale: GyroFullScale::Dps250, // ±250°/s for slow rotation
        dlpf: GyroDlpf::Hz152,             // Smooth readings
        dlpf_enable: true,
        sample_rate_div: 20, // ~51 Hz
    };

    (accel, gyro)
}

/// Configuration preset for fast motion tracking (drones, robots)
fn fast_motion_config() -> (AccelConfig, GyroConfig) {
    let accel = AccelConfig {
        full_scale: AccelFullScale::G16, // ±16g for high acceleration
        dlpf: AccelDlpf::Hz246,          // Higher bandwidth for fast response
        dlpf_enable: true,
        sample_rate_div: 4, // ~225 Hz for fast updates
    };

    let gyro = GyroConfig {
        full_scale: GyroFullScale::Dps2000, // ±2000°/s for fast rotation
        dlpf: GyroDlpf::Hz197,              // Fast response
        dlpf_enable: true,
        sample_rate_div: 4, // ~225 Hz
    };

    (accel, gyro)
}

/// Configuration preset for vibration detection
fn vibration_detection_config() -> (AccelConfig, GyroConfig) {
    let accel = AccelConfig {
        full_scale: AccelFullScale::G8, // ±8g for moderate vibrations
        dlpf: AccelDlpf::Disabled,      // Disable DLPF for full bandwidth
        dlpf_enable: false,             // Disable DLPF for full bandwidth
        sample_rate_div: 0,             // 1125 Hz maximum rate
    };

    let gyro = GyroConfig {
        full_scale: GyroFullScale::Dps1000, // ±1000°/s
        dlpf: GyroDlpf::Disabled,           // Disable DLPF for full bandwidth
        dlpf_enable: false,                 // Disable DLPF
        sample_rate_div: 0,                 // 1125 Hz maximum rate
    };

    (accel, gyro)
}

/// Configuration preset for low power / battery operation
fn low_power_config() -> (AccelConfig, GyroConfig) {
    let accel = AccelConfig {
        full_scale: AccelFullScale::G2,
        dlpf: AccelDlpf::Hz50, // Lower frequency = less power
        dlpf_enable: true,
        sample_rate_div: 99, // ~11 Hz very slow updates
    };

    let gyro = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        dlpf: GyroDlpf::Hz120, // Lower frequency
        dlpf_enable: true,
        sample_rate_div: 99, // ~11 Hz
    };

    (accel, gyro)
}

/// Configuration preset for balanced general-purpose use
fn balanced_config() -> (AccelConfig, GyroConfig) {
    let accel = AccelConfig {
        full_scale: AccelFullScale::G4, // ±4g moderate range
        dlpf: AccelDlpf::Hz246,         // Good balance
        dlpf_enable: true,
        sample_rate_div: 10, // ~102 Hz
    };

    let gyro = GyroConfig {
        full_scale: GyroFullScale::Dps500, // ±500°/s moderate range
        dlpf: GyroDlpf::Hz197,             // Good balance
        dlpf_enable: true,
        sample_rate_div: 10, // ~102 Hz
    };

    (accel, gyro)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("ICM-20948 Sensor Configuration Presets Example (Async)");
    info!("================================================");
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

    // Initialize the device
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("Failed to initialize ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    Timer::after_millis(100).await;
    info!("ICM-20948 initialized successfully!");
    info!("");

    // Array of configuration presets with descriptions
    let configs = [
        ("TILT SENSING", tilt_sensing_config()),
        ("FAST MOTION", fast_motion_config()),
        ("VIBRATION DETECTION", vibration_detection_config()),
        ("LOW POWER", low_power_config()),
        ("BALANCED", balanced_config()),
    ];

    // Cycle through each configuration
    for (name, (accel_config, gyro_config)) in configs.iter() {
        info!("=== {} CONFIGURATION ===", name);
        info!("");

        // Display configuration details
        info!("Accelerometer:");
        info!("  Range: {:?}", accel_config.full_scale);
        info!(
            "  DLPF: {:?} (enabled: {})",
            accel_config.dlpf, accel_config.dlpf_enable
        );
        let accel_rate = 1125u16 / (1 + accel_config.sample_rate_div);
        info!("  Sample Rate: ~{} Hz", accel_rate);
        info!("");

        info!("Gyroscope:");
        info!("  Range: {:?}", gyro_config.full_scale);
        info!(
            "  DLPF: {:?} (enabled: {})",
            gyro_config.dlpf, gyro_config.dlpf_enable
        );
        let gyro_rate = 1125u16 / (1 + gyro_config.sample_rate_div as u16);
        info!("  Sample Rate: ~{} Hz", gyro_rate);
        info!("");

        // Apply configuration
        if let Err(e) = imu.configure_accelerometer(*accel_config).await {
            error!("Failed to configure accelerometer: {:?}", e);
        }

        if let Err(e) = imu.configure_gyroscope(*gyro_config).await {
            error!("Failed to configure gyroscope: {:?}", e);
        }

        info!("Configuration applied! Taking 10 sample readings...");
        info!("");

        Timer::after_millis(100).await;

        // Take sample readings with this configuration
        for i in 0..10 {
            // Read accelerometer
            if let Ok(accel_data) = imu.read_accelerometer().await {
                let magnitude = accel_data.magnitude();
                info!(
                    "Sample {}: Accel X={}g Y={}g Z={}g |Mag|={}g",
                    i + 1,
                    accel_data.x,
                    accel_data.y,
                    accel_data.z,
                    magnitude
                );
            }

            // Read gyroscope
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
        info!("--- End of {} configuration ---", name);
        info!("");
        info!("Press reset to stop, or wait 3 seconds for next configuration...");
        info!("");

        Timer::after_millis(3000).await;
    }

    info!("=== ALL CONFIGURATIONS DEMONSTRATED ===");
    info!("");
    info!("Configuration Trade-offs Summary:");
    info!("");
    info!("1. TILT SENSING:");
    info!("   + Low noise, stable readings");
    info!("   + Good for level detection");
    info!("   - Slow response to fast motion");
    info!("");
    info!("2. FAST MOTION:");
    info!("   + Wide range (±16g, ±2000°/s)");
    info!("   + Fast sample rate (~225 Hz)");
    info!("   - More noise in readings");
    info!("   - Higher power consumption");
    info!("");
    info!("3. VIBRATION DETECTION:");
    info!("   + Maximum bandwidth (DLPF disabled)");
    info!("   + Highest sample rate (1125 Hz)");
    info!("   - Very noisy for static measurements");
    info!("   - Highest power consumption");
    info!("");
    info!("4. LOW POWER:");
    info!("   + Minimal power consumption");
    info!("   + Low DLPF frequency reduces noise");
    info!("   - Very slow updates (~11 Hz)");
    info!("   - Not suitable for fast motion");
    info!("");
    info!("5. BALANCED:");
    info!("   + Good general-purpose settings");
    info!("   + Moderate range and sample rate");
    info!("   + Good noise/bandwidth trade-off");
    info!("");

    // Final continuous monitoring with balanced config
    info!("Switching to BALANCED configuration for continuous monitoring...");
    info!("");

    let (accel_config, gyro_config) = balanced_config();

    let _ = imu.configure_accelerometer(accel_config).await;
    let _ = imu.configure_gyroscope(gyro_config).await;

    Timer::after_millis(100).await;

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
        Timer::after_millis(200).await;
    }
}
