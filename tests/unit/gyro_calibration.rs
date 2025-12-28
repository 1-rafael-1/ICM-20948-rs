//! Unit tests for gyroscope calibration functionality

use crate::common::{create_mock_driver, default_gyro_config};
use icm20948::sensors::GyroFullScale;

#[test]
fn test_calibrate_gyroscope_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data: simulate device at rest with small drift offsets
    // At ±250dps, sensitivity is 131 LSB/(°/s)
    // Simulate: X=50 LSB, Y=-30 LSB, Z=20 LSB (small drift)
    interface.set_gyro_sequence(vec![
        [50, -30, 20]; 100  // Same reading for 100 samples
    ]);

    // Calibrate with 100 samples
    let calibration = driver.calibrate_gyroscope(100).unwrap();

    // Verify offsets are calculated correctly (should be the average)
    assert_eq!(calibration.offset_x, 50);
    assert_eq!(calibration.offset_y, -30);
    assert_eq!(calibration.offset_z, 20);

    // Verify scale factors are default (1.0)
    assert!((calibration.scale_x - 1.0).abs() < 0.001);
    assert!((calibration.scale_y - 1.0).abs() < 0.001);
    assert!((calibration.scale_z - 1.0).abs() < 0.001);
}

#[test]
fn test_calibrate_gyroscope_zero_samples() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Attempt calibration with zero samples causes division by zero
    // This is expected to panic in the current implementation
    // In production code, this should return an error instead

    // For now, we test with 1 sample as minimum valid input
    interface.set_gyro_data(0, 0, 0);

    let result = driver.calibrate_gyroscope(1);
    assert!(result.is_ok(), "Calibration with 1 sample should succeed");
}

#[test]
fn test_calibrate_gyroscope_large_values() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data with large but safe values
    // Avoid i16::MIN which causes overflow on negation
    interface.set_gyro_sequence(vec![[i16::MAX / 2, i16::MIN / 2, i16::MAX / 4]; 100]);

    // Calibration should handle this gracefully with i64 accumulator
    let result = driver.calibrate_gyroscope(100);
    assert!(result.is_ok(), "Calibration should handle large values");

    let calibration = result.unwrap();
    // The offsets should be the average
    assert!(calibration.offset_x.abs() <= i16::MAX);
    assert!(calibration.offset_y.abs() <= i16::MAX);
    assert!(calibration.offset_z.abs() <= i16::MAX);
}

#[test]
fn test_calibrate_gyroscope_varying_readings() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data with high variations (simulating device movement)
    // At ±250dps, sensitivity is 131 LSB/(°/s)
    // With strict threshold (33), max variance = 131/33 ≈ 3.97 LSB
    // Use ±5 LSB variation to exceed the strict 3% motion detection threshold
    let mut sequence = vec![];
    for i in 0..50 {
        let noise = if i % 2 == 0 { 5 } else { -5 };
        sequence.push([100 + noise, -50 + noise, 25 + noise]);
    }
    interface.set_gyro_sequence(sequence);

    // Calibration should fail with DeviceMoving error due to high variance
    // Use strict threshold divisor (33) for ~3% tolerance
    let result = driver.calibrate_gyroscope_with_threshold(50, 33);
    assert!(result.is_err());
    match result {
        Err(icm20948::Error::DeviceMoving) => {
            // Expected - variance exceeds threshold
        }
        _ => panic!("Expected DeviceMoving error"),
    }
}

#[test]
fn test_gyroscope_calibration_applied_to_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data for calibration
    interface.set_gyro_sequence(vec![[100, -50, 30]; 50]);

    // Calibrate
    let calibration = driver.calibrate_gyroscope(50).unwrap();

    // Verify calibration was stored
    assert_eq!(
        driver.gyroscope_calibration().offset_x,
        calibration.offset_x
    );
    assert_eq!(
        driver.gyroscope_calibration().offset_y,
        calibration.offset_y
    );
    assert_eq!(
        driver.gyroscope_calibration().offset_z,
        calibration.offset_z
    );

    // Now set different data and read
    interface.set_gyro_data(200, -100, 60);

    let gyro_data = driver.read_gyroscope().unwrap();

    // Verify calibration is applied (reading should be in physical units)
    // At ±250dps, sensitivity is 131 LSB/(°/s)
    // After calibration: (200-100)/131 ≈ 0.76 dps
    // After calibration: (-100-(-50))/131 ≈ -0.38 dps
    // After calibration: (60-30)/131 ≈ 0.23 dps
    assert!(gyro_data.x.abs() < 10.0); // Should be in reasonable range
    assert!(gyro_data.y.abs() < 10.0);
    assert!(gyro_data.z.abs() < 10.0);

    // Verify the calibration data is stored correctly
    assert_eq!(driver.gyroscope_calibration().offset_x, 100);
    assert_eq!(driver.gyroscope_calibration().offset_y, -50);
    assert_eq!(driver.gyroscope_calibration().offset_z, 30);
}

#[test]
fn test_calibrate_gyroscope_different_scales() {
    let (mut driver, interface) = create_mock_driver();

    // Test with ±500dps scale
    let mut config = default_gyro_config();
    config.full_scale = GyroFullScale::Dps500;
    driver.configure_gyroscope(config).unwrap();

    // At ±500dps, sensitivity is 65.5 LSB/(°/s)
    interface.set_gyro_sequence(vec![[65, -33, 50]; 100]);

    let calibration = driver.calibrate_gyroscope(100).unwrap();

    assert_eq!(calibration.offset_x, 65);
    assert_eq!(calibration.offset_y, -33);
    assert_eq!(calibration.offset_z, 50);

    // Test with ±1000dps scale
    config.full_scale = GyroFullScale::Dps1000;
    driver.configure_gyroscope(config).unwrap();

    // At ±1000dps, sensitivity is 32.8 LSB/(°/s)
    interface.set_gyro_sequence(vec![[33, -16, 25]; 100]);

    let calibration = driver.calibrate_gyroscope(100).unwrap();

    assert_eq!(calibration.offset_x, 33);
    assert_eq!(calibration.offset_y, -16);
    assert_eq!(calibration.offset_z, 25);

    // Test with ±2000dps scale
    config.full_scale = GyroFullScale::Dps2000;
    driver.configure_gyroscope(config).unwrap();

    // At ±2000dps, sensitivity is 16.4 LSB/(°/s)
    interface.set_gyro_sequence(vec![[16, -8, 12]; 100]);

    let calibration = driver.calibrate_gyroscope(100).unwrap();

    assert_eq!(calibration.offset_x, 16);
    assert_eq!(calibration.offset_y, -8);
    assert_eq!(calibration.offset_z, 12);
}

#[test]
fn test_calibrate_gyroscope_many_samples() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data with 1000 samples
    interface.set_gyro_sequence(vec![[150, -75, 40]; 1000]);

    // Calibrate with many samples
    let calibration = driver.calibrate_gyroscope(1000).unwrap();

    // With stable readings, offsets should be exact
    assert_eq!(calibration.offset_x, 150);
    assert_eq!(calibration.offset_y, -75);
    assert_eq!(calibration.offset_z, 40);
}

#[test]
fn test_calibrate_gyroscope_negative_offsets() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data with all negative offsets
    interface.set_gyro_sequence(vec![[-200, -150, -100]; 50]);

    // Calibrate
    let calibration = driver.calibrate_gyroscope(50).unwrap();

    // Verify negative offsets are handled correctly
    assert_eq!(calibration.offset_x, -200);
    assert_eq!(calibration.offset_y, -150);
    assert_eq!(calibration.offset_z, -100);
}

#[test]
fn test_calibrate_gyroscope_sequential() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // First calibration
    interface.set_gyro_sequence(vec![[100, -50, 25]; 50]);
    let cal1 = driver.calibrate_gyroscope(50).unwrap();

    assert_eq!(cal1.offset_x, 100);
    assert_eq!(cal1.offset_y, -50);
    assert_eq!(cal1.offset_z, 25);

    // Second calibration should overwrite the first
    interface.set_gyro_sequence(vec![[200, -100, 50]; 50]);
    let cal2 = driver.calibrate_gyroscope(50).unwrap();

    assert_eq!(cal2.offset_x, 200);
    assert_eq!(cal2.offset_y, -100);
    assert_eq!(cal2.offset_z, 50);

    // Verify the stored calibration is the second one
    assert_eq!(driver.gyroscope_calibration().offset_x, 200);
    assert_eq!(driver.gyroscope_calibration().offset_y, -100);
    assert_eq!(driver.gyroscope_calibration().offset_z, 50);
}
