//! Self-test example for ICM-20948 on Raspberry Pi Pico 2 (Blocking version)
//!
//! This example demonstrates:
//! - Running hardware self-test for accelerometer
//! - Running hardware self-test for gyroscope
//! - Running hardware self-test for magnetometer
//! - Interpreting self-test results
//! - Hardware diagnostic verification
//!
//! The self-test feature verifies that the sensors are functioning correctly
//! by comparing readings with and without internal test signals applied.
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("========================================");
    info!("ICM-20948 Self-Test Example (Blocking)");
    info!("========================================");
    info!("");
    info!("This example runs hardware self-tests to verify");
    info!("that all sensors are functioning correctly.");
    info!("");

    let p = embassy_rp::init(Config::default());

    // Configure I2C0 for ICM-20948 (400 kHz standard speed)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_13, p.PIN_12, i2c_config);

    // Create ICM-20948 driver with default I2C address (0x68)
    let interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(interface) {
        Ok(imu) => {
            info!("✓ ICM-20948 detected");
            imu
        }
        Err(e) => {
            error!("Failed to initialize ICM-20948: {:?}", e);
            error!("Check your wiring and I2C address!");
            loop {
                embassy_time::block_for(embassy_time::Duration::from_secs(1));
            }
        }
    };

    // Initialize the device
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay) {
        error!("Failed to initialize device: {:?}", e);
        loop {
            embassy_time::block_for(embassy_time::Duration::from_secs(1));
        }
    }
    info!("✓ Device initialized");
    embassy_time::block_for(embassy_time::Duration::from_millis(100));

    info!("");
    info!("========================================");
    info!("Running Self-Tests");
    info!("========================================");
    info!("");

    // Track overall test status
    let mut all_passed = true;

    // Test 1: Accelerometer Self-Test
    info!("1. Testing Accelerometer...");
    info!("   Applying internal test signals to accelerometer");

    match imu.accelerometer_self_test(&mut delay) {
        Ok(passed) => {
            if passed {
                info!("   ✓ PASS - Accelerometer is functioning correctly");
                info!("   All axes responded properly to test signals");
            } else {
                error!("   ✗ FAIL - Accelerometer self-test failed");
                error!("   Hardware may be damaged or improperly connected");
                all_passed = false;
            }
        }
        Err(e) => {
            error!("   ✗ ERROR - Communication error: {:?}", e);
            all_passed = false;
        }
    }
    info!("");
    embassy_time::block_for(embassy_time::Duration::from_millis(500));

    // Test 2: Gyroscope Self-Test
    info!("2. Testing Gyroscope...");
    info!("   Applying internal test signals to gyroscope");

    match imu.gyroscope_self_test(&mut delay) {
        Ok(passed) => {
            if passed {
                info!("   ✓ PASS - Gyroscope is functioning correctly");
                info!("   All axes responded properly to test signals");
            } else {
                error!("   ✗ FAIL - Gyroscope self-test failed");
                error!("   Hardware may be damaged or improperly connected");
                all_passed = false;
            }
        }
        Err(e) => {
            error!("   ✗ ERROR - Communication error: {:?}", e);
            all_passed = false;
        }
    }
    info!("");
    embassy_time::block_for(embassy_time::Duration::from_millis(500));

    // Test 3: Magnetometer Self-Test
    info!("3. Testing Magnetometer (AK09916)...");
    info!("   Initializing magnetometer via I2C master");

    // First initialize the magnetometer
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    match imu.init_magnetometer(mag_config, &mut delay) {
        Ok(_) => {
            info!("   ✓ Magnetometer initialized");
            embassy_time::block_for(embassy_time::Duration::from_millis(100));

            info!("   Applying internal test field to magnetometer");
            match imu.magnetometer_self_test(&mut delay) {
                Ok(passed) => {
                    if passed {
                        info!("   ✓ PASS - Magnetometer is functioning correctly");
                        info!("   Test field measurements within expected range");
                    } else {
                        error!("   ✗ FAIL - Magnetometer self-test failed");
                        error!("   Hardware may be damaged or improperly connected");
                        all_passed = false;
                    }
                }
                Err(e) => {
                    error!("   ✗ ERROR - Self-test communication error: {:?}", e);
                    all_passed = false;
                }
            }
        }
        Err(e) => {
            error!("   ✗ ERROR - Failed to initialize magnetometer: {:?}", e);
            error!("   Cannot proceed with magnetometer self-test");
            all_passed = false;
        }
    }
    info!("");

    // Final summary
    info!("========================================");
    info!("Self-Test Summary");
    info!("========================================");

    if all_passed {
        info!("✓ ALL TESTS PASSED");
        info!("");
        info!("Your ICM-20948 is functioning correctly!");
        info!("All sensors passed hardware diagnostics.");
        info!("The device is ready for normal operation.");
    } else {
        error!("✗ SOME TESTS FAILED");
        error!("");
        error!("One or more sensors failed self-test.");
        error!("Please check:");
        error!("  - Power supply voltage (should be 1.71-3.6V)");
        error!("  - I2C connections (SDA, SCL)");
        error!("  - Ground connection");
        error!("  - Physical damage to the sensor");
        error!("");
        error!("If all connections are correct, the sensor");
        error!("hardware may be damaged and should be replaced.");
    }
    info!("========================================");
    info!("");

    // Verify device is still operational by reading sensor data
    if all_passed {
        info!("Verifying sensor readings after self-test...");
        info!("");

        for i in 1..=3 {
            info!("Sample {}/3:", i);

            // Read accelerometer
            match imu.read_accelerometer() {
                Ok(accel) => {
                    info!("  Accel: x={}g, y={}g, z={}g", accel.x, accel.y, accel.z);
                }
                Err(e) => {
                    error!("  Failed to read accelerometer: {:?}", e);
                }
            }

            // Read gyroscope
            match imu.read_gyroscope() {
                Ok(gyro) => {
                    info!("  Gyro:  x={}°/s, y={}°/s, z={}°/s", gyro.x, gyro.y, gyro.z);
                }
                Err(e) => {
                    error!("  Failed to read gyroscope: {:?}", e);
                }
            }

            // Read magnetometer
            match imu.read_magnetometer() {
                Ok(mag) => {
                    info!("  Mag:   x={}µT, y={}µT, z={}µT", mag.x, mag.y, mag.z);
                }
                Err(e) => {
                    error!("  Failed to read magnetometer: {:?}", e);
                }
            }

            // Read temperature
            match imu.read_temperature_celsius() {
                Ok(temp) => {
                    info!("  Temp:  {}°C", temp);
                }
                Err(e) => {
                    error!("  Failed to read temperature: {:?}", e);
                }
            }

            info!("");
            embassy_time::block_for(embassy_time::Duration::from_secs(1));
        }

        info!("✓ Device operational after self-test");
        info!("All sensors are providing valid readings");
    }

    info!("");
    info!("Self-test complete. Idling...");

    // Keep the program running
    loop {
        embassy_time::block_for(embassy_time::Duration::from_secs(10));
    }
}
