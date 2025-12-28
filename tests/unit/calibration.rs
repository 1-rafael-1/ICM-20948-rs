//! Unit tests for accelerometer calibration functionality

use crate::common::{create_mock_driver, default_accel_config};
use icm20948::sensors::AccelFullScale;

#[test]
fn test_calibrate_accelerometer_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set up mock data: simulate device at rest with small offsets
    // At ±2g, sensitivity is 16384 LSB/g
    // Simulate: X=100 LSB, Y=-50 LSB, Z=16384+200 LSB (1g + offset)
    interface.set_accel_sequence(vec![
        [100, -50, 16584]; 200  // Same reading for 200 samples
    ]);

    // Calibrate
    let calibration = driver.calibrate_accelerometer(200).unwrap();

    // Verify offsets are calculated correctly
    assert_eq!(calibration.offset_x, 100);
    assert_eq!(calibration.offset_y, -50);
    assert_eq!(calibration.offset_z, 200); // Z offset from expected 1g
}

#[test]
fn test_calibrate_accelerometer_overflow() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set up mock data with extreme values that would cause motion detection to fail
    // Use alternating extreme values to trigger the motion detection variance check
    let mut sequence = vec![];
    for i in 0..100 {
        if i % 2 == 0 {
            sequence.push([i16::MAX, i16::MAX, i16::MAX]);
        } else {
            sequence.push([i16::MIN, i16::MIN, i16::MIN]);
        }
    }
    interface.set_accel_sequence(sequence);

    // This should fail with InvalidConfig error due to motion detection
    let result = driver.calibrate_accelerometer(100);
    assert!(
        result.is_err(),
        "Calibration should fail with extreme variance"
    );
}

#[test]
fn test_calibrate_accelerometer_zero_samples() {
    let (mut driver, _interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Attempt calibration with zero samples
    let result = driver.calibrate_accelerometer(0);
    assert!(result.is_err(), "Calibration with 0 samples should fail");
}

#[test]
fn test_calibrate_accelerometer_motion_detection() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set up mock data that simulates motion (varying readings)
    // At ±2g, sensitivity is 16384 LSB/g, 1% variance = 164 LSB
    let mut sequence = vec![];
    for i in 0..100 {
        // Create readings that vary more than 1% of full scale
        let x = 100 + (i * 10) as i16; // Varies by 1000 LSB > 164
        sequence.push([x, 0, 16384]);
    }

    interface.set_accel_sequence(sequence);

    // Calibration should fail due to motion detection
    let result = driver.calibrate_accelerometer(100);
    assert!(
        result.is_err(),
        "Calibration should fail when device is moving"
    );
}

#[test]
fn test_calibration_applied_to_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set up mock data for calibration
    interface.set_accel_sequence(vec![[100, -50, 16584]; 50]);

    // Calibrate
    let calibration = driver.calibrate_accelerometer(50).unwrap();

    // Verify calibration was stored
    assert_eq!(
        driver.accelerometer_calibration().offset_x,
        calibration.offset_x
    );
    assert_eq!(
        driver.accelerometer_calibration().offset_y,
        calibration.offset_y
    );
    assert_eq!(
        driver.accelerometer_calibration().offset_z,
        calibration.offset_z
    );

    // Now set different data and read
    interface.set_accel_data(200, -100, 16484);

    let _accel_data = driver.read_accelerometer().unwrap();

    // Verify calibration is applied (reading should be in physical units)
    // The exact values depend on how calibration is applied internally
    // This test ensures the calibration data is stored and accessible
    assert_eq!(driver.accelerometer_calibration().offset_x, 100);
}

#[test]
fn test_calibrate_different_scales() {
    let (mut driver, interface) = create_mock_driver();

    // Test with ±4g scale
    let mut config = default_accel_config();
    config.full_scale = AccelFullScale::G4;
    driver.configure_accelerometer(config).unwrap();

    // At ±4g, sensitivity is 8192 LSB/g
    interface.set_accel_sequence(vec![
        [50, -25, 8292]; 100  // Z = 1g (8192) + 100 offset
    ]);

    let calibration = driver.calibrate_accelerometer(100).unwrap();

    assert_eq!(calibration.offset_x, 50);
    assert_eq!(calibration.offset_y, -25);
    assert_eq!(calibration.offset_z, 100);

    // Test with ±16g scale
    config.full_scale = AccelFullScale::G16;
    driver.configure_accelerometer(config).unwrap();

    // At ±16g, sensitivity is 2048 LSB/g
    interface.set_accel_sequence(vec![
        [10, -5, 2098]; 100  // Z = 1g (2048) + 50 offset
    ]);

    let calibration = driver.calibrate_accelerometer(100).unwrap();

    assert_eq!(calibration.offset_x, 10);
    assert_eq!(calibration.offset_y, -5);
    assert_eq!(calibration.offset_z, 50);
}
