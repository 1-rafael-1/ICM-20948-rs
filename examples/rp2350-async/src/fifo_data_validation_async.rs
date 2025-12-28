//! FIFO Data Validation Example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example validates that FIFO data extraction works correctly by:
//! 1. Reading sensor data directly (10 samples)
//! 2. Configuring FIFO and letting it accumulate the same data
//! 3. Reading and parsing FIFO data
//! 4. Comparing direct reads with FIFO-extracted data
//!
//! This confirms the FIFO implementation is working correctly.
//!
//! NOTE: Temperature sensor data is NOT written to FIFO by the ICM-20948 hardware.
//! Temperature readings must be obtained directly from the temperature registers.
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
    fifo::{FifoConfig, FifoMode},
    AccelConfig, AccelData, AccelFullScale, GyroConfig, GyroData, GyroFullScale, I2cInterface,
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

const NUM_SAMPLES: usize = 10;

#[derive(Debug, Clone, Copy)]
struct SensorSample {
    accel: AccelData,
    gyro: GyroData,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("=== ICM-20948 FIFO Data Validation ===");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400kHz
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    // Create and initialize driver
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface).await {
        Ok(imu) => imu,
        Err(e) => {
            error!("Failed to detect ICM-20948: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("Failed to initialize: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    Timer::after_millis(100).await;

    // Configure sensors with known settings
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G2,
        sample_rate_div: 10,
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_accelerometer(accel_config).await {
        error!("Failed to configure accelerometer: {:?}", e);
    }

    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        sample_rate_div: 10,
        dlpf_enable: true,
        ..Default::default()
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config).await {
        error!("Failed to configure gyroscope: {:?}", e);
    }

    Timer::after_millis(100).await;

    info!("Configuration complete");
    info!("Sample rate: ~102 Hz (1125 Hz / (1 + 10))");
    info!("");

    // ========== TEST 1: Accel + Gyro ==========
    info!("TEST 1: Accelerometer + Gyroscope");
    test_fifo_accel_gyro(&mut imu).await;
    Timer::after_millis(2000).await;

    // ========== TEST 2: Accel Only ==========
    info!("TEST 2: Accelerometer Only");
    test_fifo_accel_only(&mut imu).await;
    Timer::after_millis(2000).await;

    // ========== TEST 3: Gyro Only ==========
    info!("TEST 3: Gyroscope Only");
    test_fifo_gyro_only(&mut imu).await;
    Timer::after_millis(2000).await;

    info!("");
    info!("=== All FIFO validation tests complete ===");
    info!("");
    info!("NOTE: Temperature sensor data is NOT supported in FIFO.");
    info!("      The ICM-20948 hardware does not write temperature to FIFO.");
    info!("      Use read_temperature() to get temperature readings directly.");

    loop {
        Timer::after_millis(1000).await;
    }
}

async fn test_fifo_accel_gyro(
    imu: &mut Icm20948Driver<
        I2cInterface<I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>>,
    >,
) {
    info!("  Step 1: Reading {} direct samples...", NUM_SAMPLES);

    let mut direct_samples = [SensorSample {
        accel: AccelData { x: 0, y: 0, z: 0 },
        gyro: GyroData { x: 0, y: 0, z: 0 },
    }; NUM_SAMPLES];

    // Read samples directly
    for sample in &mut direct_samples {
        Timer::after_millis(10).await;

        let accel = match imu.read_accelerometer_raw().await {
            Ok(a) => a,
            Err(e) => {
                error!("  Failed to read accel: {:?}", e);
                return;
            }
        };

        let gyro = match imu.read_gyroscope_raw().await {
            Ok(g) => g,
            Err(e) => {
                error!("  Failed to read gyro: {:?}", e);
                return;
            }
        };

        *sample = SensorSample { accel, gyro };
    }

    info!("  Step 2: Configuring FIFO...");

    let fifo_config = FifoConfig {
        enable_accel: true,
        enable_gyro: true,
        enable_temp: false,
        enable_mag: false,
        mode: FifoMode::Stream,
    };

    if let Err(e) = imu.fifo_configure(&fifo_config).await {
        error!("  Failed to configure FIFO: {:?}", e);
        return;
    }

    if let Err(e) = imu.fifo_reset().await {
        error!("  Failed to reset FIFO: {:?}", e);
        return;
    }

    if let Err(e) = imu.fifo_enable(true).await {
        error!("  Failed to enable FIFO: {:?}", e);
        return;
    }

    info!("  Step 3: Waiting for FIFO to accumulate data...");
    Timer::after_millis(150).await;

    // Check FIFO count
    let count = match imu.fifo_count().await {
        Ok(c) => c,
        Err(e) => {
            error!("  Failed to read FIFO count: {:?}", e);
            return;
        }
    };

    let bytes_per_sample = 12u16; // 6 accel + 6 gyro
    let num_samples = count / bytes_per_sample;

    info!("  FIFO contains {} bytes ({} samples)", count, num_samples);

    if num_samples < NUM_SAMPLES as u16 {
        warn!(
            "  Only {} samples in FIFO, expected at least {}",
            num_samples, NUM_SAMPLES
        );
    }

    // Read FIFO data
    let mut buffer = [0u8; 512];
    let bytes_to_read = (NUM_SAMPLES * 12).min(512);

    let bytes_read = match imu.fifo_read(&mut buffer[..bytes_to_read]).await {
        Ok(n) => n,
        Err(e) => {
            error!("  Failed to read FIFO: {:?}", e);
            return;
        }
    };

    info!("  Step 4: Parsing FIFO data...");

    let records = match imu.fifo_parse(&buffer[..bytes_read], &fifo_config.into()) {
        Ok(r) => r,
        Err(e) => {
            error!("  Failed to parse FIFO: {:?}", e);
            return;
        }
    };

    info!("  Parsed {} records from FIFO", records.len());

    // Compare samples
    info!("  Step 5: Comparing direct vs FIFO data...");

    let compare_count = NUM_SAMPLES.min(records.len());
    let mut max_accel_diff = 0i16;
    let mut max_gyro_diff = 0i16;
    let mut matches = 0usize;

    for i in 0..compare_count {
        if let (Some(fifo_accel), Some(fifo_gyro)) = (records[i].accel, records[i].gyro) {
            let direct = direct_samples[i];

            let accel_diff_x = (direct.accel.x - fifo_accel.0).abs();
            let accel_diff_y = (direct.accel.y - fifo_accel.1).abs();
            let accel_diff_z = (direct.accel.z - fifo_accel.2).abs();

            let gyro_diff_x = (direct.gyro.x - fifo_gyro.0).abs();
            let gyro_diff_y = (direct.gyro.y - fifo_gyro.1).abs();
            let gyro_diff_z = (direct.gyro.z - fifo_gyro.2).abs();

            max_accel_diff = max_accel_diff
                .max(accel_diff_x)
                .max(accel_diff_y)
                .max(accel_diff_z);
            max_gyro_diff = max_gyro_diff
                .max(gyro_diff_x)
                .max(gyro_diff_y)
                .max(gyro_diff_z);

            // Consider match if within reasonable tolerance (sensor noise)
            // Allow differences since samples were taken at different times
            if accel_diff_x < 500
                && accel_diff_y < 500
                && accel_diff_z < 500
                && gyro_diff_x < 500
                && gyro_diff_y < 500
                && gyro_diff_z < 500
            {
                matches += 1;
            }

            if i < 3 {
                info!(
                    "  [{}] Direct: A({},{},{}) G({},{},{}) | FIFO: A({},{},{}) G({},{},{})",
                    i,
                    direct.accel.x,
                    direct.accel.y,
                    direct.accel.z,
                    direct.gyro.x,
                    direct.gyro.y,
                    direct.gyro.z,
                    fifo_accel.0,
                    fifo_accel.1,
                    fifo_accel.2,
                    fifo_gyro.0,
                    fifo_gyro.1,
                    fifo_gyro.2
                );
            }
        }
    }

    info!(
        "  Results: {}/{} samples within tolerance",
        matches, compare_count
    );
    info!("  Max accel difference: {} LSB", max_accel_diff);
    info!("  Max gyro difference: {} LSB", max_gyro_diff);

    // Disable FIFO
    let _ = imu.fifo_enable(false).await;

    if matches >= compare_count * 8 / 10 {
        info!("  ✓ TEST PASSED");
    } else {
        warn!("  ✗ TEST FAILED - too many mismatches");
    }

    info!("");
}

async fn test_fifo_accel_only(
    imu: &mut Icm20948Driver<
        I2cInterface<I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>>,
    >,
) {
    info!("  Configuring FIFO for accelerometer only...");

    let fifo_config = FifoConfig {
        enable_accel: true,
        enable_gyro: false,
        enable_temp: false,
        enable_mag: false,
        mode: FifoMode::Stream,
    };

    if let Err(e) = imu.fifo_configure(&fifo_config).await {
        error!("  Failed to configure FIFO: {:?}", e);
        return;
    }

    if let Err(e) = imu.fifo_reset().await {
        error!("  Failed to reset FIFO: {:?}", e);
        return;
    }

    if let Err(e) = imu.fifo_enable(true).await {
        error!("  Failed to enable FIFO: {:?}", e);
        return;
    }

    Timer::after_millis(100).await;

    let count = match imu.fifo_count().await {
        Ok(c) => c,
        Err(e) => {
            error!("  Failed to read FIFO count: {:?}", e);
            return;
        }
    };

    let bytes_per_sample = 6u16; // 6 accel only
    let num_samples = count / bytes_per_sample;

    info!("  FIFO: {} bytes, {} samples", count, num_samples);

    if num_samples > 0 {
        let mut buffer = [0u8; 128];
        let bytes_to_read = ((num_samples * bytes_per_sample) as usize).min(128);

        if let Ok(bytes_read) = imu.fifo_read(&mut buffer[..bytes_to_read]).await {
            if let Ok(records) = imu.fifo_parse(&buffer[..bytes_read], &fifo_config.into()) {
                info!("  Parsed {} records", records.len());

                let accel_count = records.iter().filter(|r| r.accel.is_some()).count();
                let gyro_count = records.iter().filter(|r| r.gyro.is_some()).count();

                info!(
                    "  Records with accel: {}, with gyro: {}",
                    accel_count, gyro_count
                );

                if accel_count > 0 && gyro_count == 0 {
                    info!("  ✓ TEST PASSED");
                } else {
                    warn!("  ✗ TEST FAILED - unexpected sensor data");
                }
            }
        }
    }

    let _ = imu.fifo_enable(false).await;
    info!("");
}

async fn test_fifo_gyro_only(
    imu: &mut Icm20948Driver<
        I2cInterface<I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>>,
    >,
) {
    info!("  Configuring FIFO for gyroscope only...");

    let fifo_config = FifoConfig {
        enable_accel: false,
        enable_gyro: true,
        enable_temp: false,
        enable_mag: false,
        mode: FifoMode::Stream,
    };

    if let Err(e) = imu.fifo_configure(&fifo_config).await {
        error!("  Failed to configure FIFO: {:?}", e);
        return;
    }

    if let Err(e) = imu.fifo_reset().await {
        error!("  Failed to reset FIFO: {:?}", e);
        return;
    }

    if let Err(e) = imu.fifo_enable(true).await {
        error!("  Failed to enable FIFO: {:?}", e);
        return;
    }

    Timer::after_millis(100).await;

    let count = match imu.fifo_count().await {
        Ok(c) => c,
        Err(e) => {
            error!("  Failed to read FIFO count: {:?}", e);
            return;
        }
    };

    let bytes_per_sample = 6u16; // 6 gyro only
    let num_samples = count / bytes_per_sample;

    info!("  FIFO: {} bytes, {} samples", count, num_samples);

    if num_samples > 0 {
        let mut buffer = [0u8; 128];
        let bytes_to_read = ((num_samples * bytes_per_sample) as usize).min(128);

        if let Ok(bytes_read) = imu.fifo_read(&mut buffer[..bytes_to_read]).await {
            if let Ok(records) = imu.fifo_parse(&buffer[..bytes_read], &fifo_config.into()) {
                info!("  Parsed {} records", records.len());

                let accel_count = records.iter().filter(|r| r.accel.is_some()).count();
                let gyro_count = records.iter().filter(|r| r.gyro.is_some()).count();

                info!(
                    "  Records with accel: {}, with gyro: {}",
                    accel_count, gyro_count
                );

                if gyro_count > 0 && accel_count == 0 {
                    info!("  ✓ TEST PASSED");
                } else {
                    warn!("  ✗ TEST FAILED - unexpected sensor data");
                }
            }
        }
    }

    let _ = imu.fifo_enable(false).await;
    info!("");
}
