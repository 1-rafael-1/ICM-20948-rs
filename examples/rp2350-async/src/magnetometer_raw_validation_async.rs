//! Magnetometer raw data validation example for ICM-20948 on Raspberry Pi Pico 2 (Async)
//!
//! This example verifies that `read_magnetometer_raw()` returns correct byte-aligned
//! values, specifically validating the fix for issue #12 where `read_magnetometer_raw`
//! had off-by-one byte indexing in the async path.
//!
//! The library now uses atomic 8-byte reads (one I2C transaction), so raw values
//! are self-consistent. µT is computed directly from the same raw bytes.
//!
//! Validation checks:
//! - Z-axis values vary meaningfully (not stuck near 0 or ±255)
//! - All axes respond to sensor movement
//!
//! No magnetometer calibration is applied, so the raw→µT conversion is purely
//! sensitivity-based (0.15 µT/LSB for the AK09916).
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
use icm20948::{I2cInterface, Icm20948Driver, MagConfig, MagMode};
use panic_probe as _;

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Bind I2C interrupts
bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

/// AK09916 sensitivity: 0.15 µT per LSB
const MAG_SENSITIVITY: f32 = 0.15;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("ICM-20948 Magnetometer Raw Validation (Async)");
    info!("==============================================");
    info!("Validates fix for issue #12: off-by-one byte indexing");
    info!("in async read_magnetometer_raw.");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    info!("I2C configured at 400kHz on pins 12(SDA)/13(SCL)");
    info!("Waiting for ICM-20948 to power up...");
    Timer::after_millis(100).await;

    // Create ICM-20948 driver with I2C interface (uses default address 0x68, AD0 pin LOW)
    info!("Attempting to detect ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::try_new(i2c_interface).await {
        Ok(imu) => {
            info!("✓ ICM-20948 detected successfully!");
            imu
        }
        Err(e) => {
            error!("✗ Failed to detect ICM-20948: {:?}", e);
            error!("This usually means:");
            error!("  1. Device not connected or powered");
            error!("  2. Wrong I2C address (check AD0 pin)");
            error!("  3. I2C pins swapped");
            error!("  4. Previous run left device in bad state - try power cycling");
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    // Initialize the device
    info!("");
    info!("Initializing ICM-20948 (soft reset + configuration)...");
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("✗ Failed to initialize ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    // Wait for device to stabilize after reset
    Timer::after_millis(100).await;
    info!("✓ ICM-20948 initialized successfully!");
    info!("");

    // Configure magnetometer
    info!("Configuring magnetometer...");
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    if let Err(e) = imu.init_magnetometer(mag_config, &mut delay).await {
        error!("Failed to initialize magnetometer: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    // Wait for magnetometer to stabilize
    Timer::after_millis(100).await;
    info!("✓ Magnetometer initialized successfully!");
    info!("");

    info!("╔══════════════════════════════════════════════════════════════════╗");
    info!("║          RAW DATA VALIDATION TEST                                ║");
    info!("╠══════════════════════════════════════════════════════════════════╣");
    info!("║ No calibration — raw→µT is sensitivity only (×0.15).            ║");
    info!("║ Single atomic read per sample — no inter-read drift.            ║");
    info!("║                                                                  ║");
    info!("║ Check: Z-axis values vary (not stuck at 0 or ±255)               ║");
    info!("║        All axes respond to sensor movement                       ║");
    info!("╚══════════════════════════════════════════════════════════════════╝");
    info!("");

    // Wait for first magnetic data to be available
    Timer::after_millis(100).await;

    let mut sample_count: u32 = 0;
    let mut error_count: u32 = 0;
    let mut z_min_raw: i16 = i16::MAX;
    let mut z_max_raw: i16 = i16::MIN;
    let mut z_range_255_only: u32 = 0;
    let mut z_near_zero: u32 = 0;

    info!("Starting validation loop (10 Hz)...");
    info!("Rotate/move the sensor to exercise all axes.");
    info!("");

    loop {
        match imu.read_magnetometer_raw().await {
            Ok((raw_x, raw_y, raw_z)) => {
                sample_count += 1;

                // Compute µT from the same raw bytes
                let ut_x = f32::from(raw_x) * MAG_SENSITIVITY;
                let ut_y = f32::from(raw_y) * MAG_SENSITIVITY;
                let ut_z = f32::from(raw_z) * MAG_SENSITIVITY;
                let magnitude = libm::sqrtf(ut_x * ut_x + ut_y * ut_y + ut_z * ut_z);

                // Track Z-axis range
                if raw_z < z_min_raw {
                    z_min_raw = raw_z;
                }
                if raw_z > z_max_raw {
                    z_max_raw = raw_z;
                }
                if raw_z >= -255 && raw_z <= 255 {
                    z_range_255_only += 1;
                }
                if raw_z >= -10 && raw_z <= 10 {
                    z_near_zero += 1;
                }

                let z_span = z_max_raw - z_min_raw;

                info!(
                    "Sample {}: Raw[X={} Y={} Z={}]  µT[X={}µT Y={}µT Z={}µT]  |Mag|={}µT",
                    sample_count, raw_x, raw_y, raw_z, ut_x, ut_y, ut_z, magnitude
                );

                if sample_count % 50 == 0 {
                    info!("");
                    info!("--- Statistics at {} samples ---", sample_count);
                    info!("  Errors: {}", error_count);
                    info!(
                        "  Z raw range: {} to {} (span: {}, full-scale: ±32768)",
                        z_min_raw, z_max_raw, z_span
                    );
                    info!(
                        "  Z samples in ±255: {}/{} ({}%)",
                        z_range_255_only,
                        sample_count,
                        (z_range_255_only as f32 / sample_count as f32) * 100.0
                    );
                    info!(
                        "  Z samples near zero (±10): {}/{} ({}%)",
                        z_near_zero,
                        sample_count,
                        (z_near_zero as f32 / sample_count as f32) * 100.0
                    );

                    if sample_count >= 100 {
                        let pct_255 = (z_range_255_only as f32 / sample_count as f32) * 100.0;
                        let pct_zero = (z_near_zero as f32 / sample_count as f32) * 100.0;

                        if pct_255 > 90.0 {
                            warn!("");
                            warn!(
                                "⚠ Z-AXIS WARNING: {}% of Z raw values are within ±255!",
                                pct_255
                            );
                            warn!("  Off-by-one bug may still be present.");
                            warn!("");
                        }

                        if pct_zero > 50.0 && z_span < 500 {
                            warn!("");
                            warn!(
                                "⚠ Z-AXIS SUSPICIOUS: {}% near zero, range only {}!",
                                pct_zero, z_span
                            );
                            warn!("  Possible byte alignment issue.");
                            warn!("");
                        }

                        if z_span < 100 {
                            warn!("");
                            warn!("⚠ Z-AXIS RANGE TOO SMALL: span = {}", z_span);
                            warn!("  Corrupted Z-axis data suspected.");
                            warn!("");
                        }
                    }

                    info!("---");
                    info!("");
                }
            }
            Err(e) => {
                error_count += 1;
                warn!("Read error at sample {}: {:?}", sample_count + 1, e);
            }
        }

        Timer::after_millis(100).await;
    }
}
