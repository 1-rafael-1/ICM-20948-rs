//! Sample Rate Configuration Test
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
    bind_interrupts,
    block::ImageDef,
    config::Config,
    gpio::{Level, Output},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::I2C0,
};
use embassy_time::{Delay, Duration, Instant, Timer};
use icm20948::{AccelConfig, AccelDlpf, AccelFullScale, I2cInterface, Icm20948Driver};
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
    info!("=== ICM-20948 Sample Rate Test ===");
    info!("");
    info!("Testing accelerometer sample rate divider configuration");
    info!("without using interrupts - polling data changes only.");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    // LED for visual feedback
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("Hardware initialized");
    Timer::after(Duration::from_millis(100)).await;

    // Create and initialize sensor
    info!("Initializing ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut sensor = match Icm20948Driver::new(i2c_interface).await {
        Ok(sensor) => {
            info!("✓ ICM-20948 detected!");
            sensor
        }
        Err(e) => {
            error!("Failed to create driver: {:?}", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = sensor.init(&mut delay).await {
        error!("Init failed: {:?}", e);
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    Timer::after(Duration::from_millis(100)).await;
    info!("✓ ICM-20948 initialized successfully!");

    // Configure sample rate to 37.5 Hz
    // Formula: ODR = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    // For 37.5 Hz: DIV = (1125 / 37.5) - 1 = 30 - 1 = 29
    const TARGET_RATE_HZ: u32 = 37;
    const ACCEL_SMPLRT_DIV: u16 = 29;
    const TEST_DURATION_S: u32 = 5;
    const EXPECTED_SAMPLES: u32 = TARGET_RATE_HZ * TEST_DURATION_S; // ~188

    info!("");
    info!("Configuring accelerometer...");
    info!("  Target rate: {} Hz", TARGET_RATE_HZ);
    info!("  Sample rate divider: {}", ACCEL_SMPLRT_DIV);
    info!(
        "  Expected samples in {}s: {}",
        TEST_DURATION_S, EXPECTED_SAMPLES
    );

    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G4,
        dlpf: AccelDlpf::Hz6,
        dlpf_enable: true, // CRITICAL: Must be true for divider to work
        sample_rate_div: ACCEL_SMPLRT_DIV,
    };

    if let Err(e) = sensor.configure_accelerometer(accel_config).await {
        error!("Failed to configure accelerometer: {:?}", e);
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    Timer::after(Duration::from_millis(50)).await;

    // Verify all register configuration
    info!("");
    info!("=== Register Verification ===");

    // Check Bank 0 - LP_CONFIG
    match sensor.read_lp_config().await {
        Ok(lp_cfg) => {
            info!("Bank 0 - LP_CONFIG: 0x{:02X}", lp_cfg);
            let accel_cycle = (lp_cfg >> 5) & 1;
            let gyro_cycle = (lp_cfg >> 4) & 1;
            let i2c_mst_cycle = (lp_cfg >> 6) & 1;
            info!("  ACCEL_CYCLE: {} (0=continuous, 1=cycle)", accel_cycle);
            info!("  GYRO_CYCLE: {} (0=continuous, 1=cycle)", gyro_cycle);
            info!("  I2C_MST_CYCLE: {}", i2c_mst_cycle);

            if accel_cycle == 0 {
                info!("  ✓ Accelerometer in CONTINUOUS mode");
            } else {
                error!("  ✗ ERROR: Accelerometer in CYCLE mode!");
                error!("    Sample rate divider will NOT work in cycle mode");
            }
        }
        Err(e) => {
            error!("Failed to read LP_CONFIG: {:?}", e);
        }
    }

    // Check Bank 2 - ACCEL_CONFIG and SMPLRT_DIV
    match sensor.read_bank2_accel_config().await {
        Ok((config_byte, div1, div2)) => {
            info!("Bank 2 - ACCEL_CONFIG: 0x{:02X}", config_byte);
            let fchoice = config_byte & 0x01;
            let fs_sel = (config_byte >> 1) & 0x03;
            let dlpfcfg = (config_byte >> 3) & 0x07;
            info!("  ACCEL_FCHOICE: {} (0=bypass, 1=enabled)", fchoice);
            info!("  ACCEL_FS_SEL: {} (0=2g, 1=4g, 2=8g, 3=16g)", fs_sel);
            info!("  ACCEL_DLPFCFG: {}", dlpfcfg);

            if fchoice == 1 {
                info!("  ✓ DLPF/divider enabled (FCHOICE=1)");
            } else {
                error!("  ✗ ERROR: DLPF/divider bypassed (FCHOICE=0)!");
            }

            let actual_div = ((div1 as u16) << 8) | (div2 as u16);
            let actual_hz = 1125 / (actual_div + 1);
            info!("Bank 2 - ACCEL_SMPLRT_DIV: {}", actual_div);
            info!("  DIV_1: 0x{:02X} ({})", div1, div1);
            info!("  DIV_2: 0x{:02X} ({})", div2, div2);
            info!("  Calculated ODR: {} Hz", actual_hz);

            if actual_div == ACCEL_SMPLRT_DIV {
                info!("  ✓ Sample rate divider matches expected value");
            } else {
                error!("  ✗ ERROR: Divider mismatch!");
                error!("    Expected: {}", ACCEL_SMPLRT_DIV);
                error!("    Actual: {}", actual_div);
            }
        }
        Err(e) => {
            error!("Failed to read Bank 2 registers: {:?}", e);
        }
    }

    info!("");
    info!("=== Starting {}s Data Polling Test ===", TEST_DURATION_S);
    Timer::after(Duration::from_millis(500)).await;

    // Read initial sample
    let mut last_sample = match sensor.read_accelerometer().await {
        Ok(data) => (data.x, data.y, data.z),
        Err(e) => {
            error!("Failed to read initial sample: {:?}", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let start_time = Instant::now();
    let mut sample_count: u32 = 0;
    let mut last_change_time = start_time;
    let mut min_interval_us: u64 = u64::MAX;
    let mut max_interval_us: u64 = 0;
    let mut total_interval_us: u64 = 0;
    let mut unchanged_polls: u32 = 0;
    let test_end = start_time + Duration::from_secs(TEST_DURATION_S as u64);

    info!("Polling for data changes...");
    led.set_high();

    loop {
        let now = Instant::now();
        if now >= test_end {
            break;
        }

        // Read current sample
        match sensor.read_accelerometer().await {
            Ok(data) => {
                let current_sample = (data.x, data.y, data.z);

                // Check if data has changed
                if current_sample != last_sample {
                    sample_count += 1;

                    // Measure interval
                    if sample_count > 1 {
                        let interval = now.duration_since(last_change_time).as_micros();
                        if interval < min_interval_us {
                            min_interval_us = interval;
                        }
                        if interval > max_interval_us {
                            max_interval_us = interval;
                        }
                        total_interval_us += interval;
                    }

                    // Print periodic updates
                    if sample_count <= 5 || sample_count.is_multiple_of(50) {
                        let elapsed_ms = now.duration_since(start_time).as_millis();
                        let x_mg = (data.x * 1000.0) as i32;
                        let y_mg = (data.y * 1000.0) as i32;
                        let z_mg = (data.z * 1000.0) as i32;
                        info!(
                            "[{}ms] Sample #{}: X={}mg Y={}mg Z={}mg",
                            elapsed_ms, sample_count, x_mg, y_mg, z_mg
                        );
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
        Timer::after(Duration::from_micros(100)).await;
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

    info!("");
    info!("=== Test Complete ===");
    info!("");
    info!("Duration: {} ms", total_duration_ms);
    info!("");
    info!("Sample Statistics:");
    info!("  Data changes detected: {}", sample_count);
    info!("  Expected samples:      {}", EXPECTED_SAMPLES);
    info!("  Error:                 {}%", count_error_pct);
    info!("  Unchanged polls:       {}", unchanged_polls);
    info!("");
    info!("Timing Statistics:");
    info!(
        "  Average interval:      {} µs ({} ms)",
        avg_interval_us, avg_interval_ms
    );
    info!(
        "  Expected interval:     {} µs ({} ms)",
        expected_interval_us, expected_interval_ms
    );
    info!(
        "  Min interval:          {} µs",
        if min_interval_us == u64::MAX {
            0
        } else {
            min_interval_us
        }
    );
    info!("  Max interval:          {} µs", max_interval_us);
    info!("  Measured rate:         {} Hz", actual_rate_hz);
    info!("  Target rate:           {} Hz", TARGET_RATE_HZ);
    info!("");

    // Verdict
    if count_error_pct < 10 && sample_count > 0 {
        info!("✓✓✓ PASS: Sample rate configuration working correctly! ✓✓✓");
        info!("  Sample rate divider is functioning as expected");
    } else if sample_count == 0 {
        error!("✗✗✗ FAIL: No data changes detected! ✗✗✗");
        error!("  Sensor may not be outputting data");
        error!("  Check power configuration and sensor enable state");
    } else if count_error_pct >= 50 {
        error!("✗✗✗ FAIL: Sample rate significantly off target ✗✗✗");
        error!(
            "  Expected: {} samples, Got: {}",
            EXPECTED_SAMPLES, sample_count
        );
        error!("  Sample rate divider may not be working");
        error!("");
        error!("Possible causes:");
        error!("  1. LP_CONFIG.ACCEL_CYCLE != 0 (not in continuous mode)");
        error!("  2. ACCEL_CONFIG.ACCEL_FCHOICE != 1 (divider bypassed)");
        error!("  3. Power management issue (LP_EN or SLEEP mode active)");
    } else {
        warn!(
            "⚠ PARTIAL: Sample rate within {}% of target",
            count_error_pct
        );
        warn!("  This may be acceptable depending on requirements");
    }

    info!("");
    info!("Test finished. System will now idle.");

    // Keep LED on to indicate test complete
    led.set_high();

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
