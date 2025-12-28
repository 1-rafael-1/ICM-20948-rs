//! Sample Rate Configuration Test (Blocking version)
//!
//! This example focuses ONLY on testing accelerometer sample rate configuration
//! without using interrupts. It validates that the ACCEL_SMPLRT_DIV registers
//! actually affect the sensor's output data rate.
//!
//! ## Test Method
//!
//! 1. Initialize sensor
//! 2. Explicitly set continuous mode (LP_CONFIG)
//! 3. Configure ACCEL_CONFIG with FCHOICE=1 (DLPF enabled)
//! 4. Set ACCEL_SMPLRT_DIV to known value
//! 5. Read back all configuration registers
//! 6. Poll data for 5 seconds, tracking when data changes
//! 7. Calculate actual sample rate from data change timestamps
//!
//! ## Hardware Setup
//!
//! - Raspberry Pi Pico 2 (RP2350)
//! - ICM-20948 connected via I2C
//! - I2C: GPIO12 (SDA), GPIO13 (SCL)
//! - AD0: GND (I2C address 0x68)
//!
//! ## Expected Results
//!
//! For DIV=29 (target 37.5 Hz):
//! - Should see ~188 data updates in 5 seconds
//! - Average interval: ~26.7ms between updates

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    block::ImageDef,
    config::Config,
    gpio::{Level, Output},
    i2c::{Config as I2cConfig, I2c},
};
use embassy_time::{Delay, Duration, Instant};
use icm20948::{AccelConfig, AccelDlpf, AccelFullScale, I2cInterface, Icm20948Driver};
use panic_probe as _;

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("=== ICM-20948 Sample Rate Test (Blocking) ===");
    info!("");
    info!("Testing accelerometer sample rate divider configuration");
    info!("without using interrupts - polling data changes only.");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_13, p.PIN_12, i2c_config);

    // LED for visual feedback
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("Hardware initialized");
    embassy_time::block_for(Duration::from_millis(100));

    // Create and initialize sensor
    info!("Initializing ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut sensor = match Icm20948Driver::new(i2c_interface) {
        Ok(sensor) => {
            info!("✓ ICM-20948 detected!");
            sensor
        }
        Err(e) => {
            error!("Failed to create driver: {:?}", e);
            loop {
                embassy_time::block_for(Duration::from_secs(1));
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = sensor.init(&mut delay) {
        error!("Init failed: {:?}", e);
        loop {
            embassy_time::block_for(Duration::from_secs(1));
        }
    }
    info!("✓ Sensor initialized");
    embassy_time::block_for(Duration::from_millis(100));

    // Configure accelerometer with specific sample rate divider
    const SAMPLE_RATE_DIV: u16 = 29; // Target: 37.5 Hz (1125 / (1 + 29))
    let config = AccelConfig {
        full_scale: AccelFullScale::G2,
        dlpf: AccelDlpf::Hz246,
        dlpf_enable: true,
        sample_rate_div: SAMPLE_RATE_DIV,
    };

    info!("");
    info!("Configuring accelerometer:");
    info!("  Full Scale: ±2g");
    info!("  DLPF: 246 Hz (enabled)");
    info!("  Sample Rate Divider: {}", SAMPLE_RATE_DIV);
    info!("  Expected Rate: ~37.5 Hz");
    info!("  Expected Interval: ~26.7ms");

    if let Err(e) = sensor.configure_accelerometer(config) {
        error!("Configuration failed: {:?}", e);
        loop {
            embassy_time::block_for(Duration::from_secs(1));
        }
    }
    info!("✓ Accelerometer configured");
    embassy_time::block_for(Duration::from_millis(500));

    // Test parameters
    const TEST_DURATION_MS: u64 = 5000;
    const EXPECTED_SAMPLES: u32 = 188; // 37.5 Hz * 5 seconds ≈ 188

    info!("");
    info!("========================================");
    info!(
        "Starting {} second sample rate test...",
        TEST_DURATION_MS / 1000
    );
    info!("========================================");
    info!("");

    let start_time = Instant::now();
    let mut sample_count: u32 = 0;
    let mut last_sample = [0i16; 3];
    let mut last_change_time = start_time;
    let mut total_interval_us: u64 = 0;
    let mut unchanged_polls: u32 = 0;

    // Get initial reading
    match sensor.read_accelerometer_raw() {
        Ok(data) => {
            last_sample = [data.0, data.1, data.2];
            info!("Initial reading: X={} Y={} Z={}", data.0, data.1, data.2);
        }
        Err(e) => {
            error!("Failed to read initial sample: {:?}", e);
        }
    }

    info!("Starting polling loop...");
    info!("");

    // Poll for TEST_DURATION_MS
    loop {
        let now = Instant::now();
        if now.duration_since(start_time).as_millis() >= TEST_DURATION_MS {
            break;
        }

        match sensor.read_accelerometer_raw() {
            Ok(data) => {
                let current_sample = [data.0, data.1, data.2];
                // Check if data has changed
                if current_sample != last_sample {
                    sample_count += 1;

                    // Calculate interval since last change
                    if sample_count > 1 {
                        let interval_us = now.duration_since(last_change_time).as_micros();
                        total_interval_us += interval_us;
                    }

                    // Print periodic updates
                    if sample_count <= 5 || sample_count.is_multiple_of(50) {
                        if let Ok(accel_data) = sensor.read_accelerometer() {
                            let elapsed_ms = now.duration_since(start_time).as_millis();
                            let x_mg = (accel_data.x * 1000.0) as i32;
                            let y_mg = (accel_data.y * 1000.0) as i32;
                            let z_mg = (accel_data.z * 1000.0) as i32;
                            info!(
                                "[{}ms] Sample #{}: X={}mg Y={}mg Z={}mg",
                                elapsed_ms, sample_count, x_mg, y_mg, z_mg
                            );
                        }
                    }

                    last_sample = current_sample;
                    last_change_time = now;
                    led.toggle();
                } else {
                    unchanged_polls += 1;
                }
            }
            Err(e) => {
                error!("Read error: {:?}", e);
            }
        }

        // Small delay to prevent bus flooding
        embassy_time::block_for(Duration::from_micros(100));
    }

    let total_duration_ms = Instant::now().duration_since(start_time).as_millis();

    // Calculate statistics
    let avg_interval_us = if sample_count > 1 {
        total_interval_us / (sample_count as u64 - 1)
    } else {
        0
    };
    let avg_interval_ms = avg_interval_us / 1000;
    let expected_interval_us: u64 = 26667; // 1/37.5 Hz in microseconds
    let expected_interval_ms = expected_interval_us / 1000;

    let actual_rate_hz = if avg_interval_us > 0 {
        1_000_000 / avg_interval_us
    } else {
        0
    };

    // Calculate error percentage
    let count_diff = sample_count.abs_diff(EXPECTED_SAMPLES);
    let count_error_pct = if EXPECTED_SAMPLES > 0 {
        (count_diff * 100) / EXPECTED_SAMPLES
    } else {
        0
    };

    let interval_diff = avg_interval_us.abs_diff(expected_interval_us);
    let interval_error_pct = if expected_interval_us > 0 {
        (interval_diff * 100) / expected_interval_us
    } else {
        0
    };

    // Display results
    info!("");
    info!("========================================");
    info!("Test Results");
    info!("========================================");
    info!("");
    info!("Test Duration: {} ms", total_duration_ms);
    info!("");
    info!("Sample Count:");
    info!("  Samples collected: {}", sample_count);
    info!("  Expected samples: {}", EXPECTED_SAMPLES);
    info!("  Difference: {}", count_diff);
    info!("  Error: {}%", count_error_pct);
    info!("");
    info!("Sample Rate:");
    info!("  Actual rate: {} Hz", actual_rate_hz);
    info!("  Expected rate: 37-38 Hz");
    info!("  Polls with no change: {}", unchanged_polls);
    info!("");
    info!("Sample Interval:");
    info!(
        "  Average interval: {} ms ({} µs)",
        avg_interval_ms, avg_interval_us
    );
    info!(
        "  Expected interval: {} ms ({} µs)",
        expected_interval_ms, expected_interval_us
    );
    info!("  Difference: {} µs", interval_diff);
    info!("  Error: {}%", interval_error_pct);
    info!("");

    // Verdict
    info!("========================================");
    info!("Verdict");
    info!("========================================");
    info!("");

    if count_error_pct <= 10 && interval_error_pct <= 10 {
        info!("✓ PASS - Sample rate is within 10% of expected");
        info!("");
        info!("The ACCEL_SMPLRT_DIV configuration is working correctly.");
        info!("The accelerometer is outputting new data at the configured rate.");
    } else if count_error_pct <= 20 && interval_error_pct <= 20 {
        warn!("⚠ MARGINAL - Sample rate differs by 10-20%");
        warn!("");
        warn!("The sample rate is not quite as expected. This could be due to:");
        warn!("  - System timing variations");
        warn!("  - I2C bus delays");
        warn!("  - Sensor internal timing");
    } else {
        error!("✗ FAIL - Sample rate differs by more than 20%");
        error!("");
        error!("The ACCEL_SMPLRT_DIV may not be working as expected.");
        error!("Possible causes:");
        error!("  - Incorrect register configuration");
        error!("  - Sensor in wrong power mode");
        error!("  - DLPF not enabled properly");
        error!("  - Sample rate divider not being applied");
    }

    info!("");
    info!("Test complete.");

    // Keep running
    loop {
        embassy_time::block_for(Duration::from_secs(10));
    }
}
