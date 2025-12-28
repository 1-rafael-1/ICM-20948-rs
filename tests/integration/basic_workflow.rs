//! Integration tests for basic workflow scenarios

use crate::common::{create_mock_driver, default_accel_config, default_gyro_config, test_utils};

#[test]
fn test_complete_initialization_workflow() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize the device
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Configure accelerometer
    let accel_config = default_accel_config();
    driver.configure_accelerometer(accel_config).unwrap();

    // Configure gyroscope
    let gyro_config = default_gyro_config();
    driver.configure_gyroscope(gyro_config).unwrap();

    // Set up mock sensor data
    interface.set_accel_data(100, -50, 16384);
    interface.set_gyro_data(10, -20, 30);

    // Read accelerometer
    let accel_data = driver.read_accelerometer().unwrap();
    assert!(accel_data.x > 0.0);

    // Read gyroscope
    let gyro_data = driver.read_gyroscope().unwrap();
    assert!(gyro_data.x > 0.0);

    // Read temperature
    let temp = driver.read_temperature_celsius().unwrap();
    assert!(temp > -40.0 && temp < 85.0); // Valid temperature range for ICM-20948
}

#[test]
fn test_error_recovery() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Inject a read failure
    interface.fail_next_read();

    // This read should fail
    let result = driver.read_accel();
    assert!(result.is_err());

    // But subsequent reads should work (error was only for one operation)
    interface.set_accel_data(100, 200, 300);

    let result = driver.read_accel();
    assert!(result.is_ok());
}

#[test]
fn test_bank_switching_during_workflow() {
    let (mut driver, interface) = create_mock_driver();

    // Clear operations log
    interface.clear_operations();

    // Initialize (should involve bank switches)
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Configure accelerometer (Bank 2)
    let accel_config = default_accel_config();
    driver.configure_accelerometer(accel_config).unwrap();

    // Read accelerometer (Bank 0)
    interface.set_accel_data(0, 0, 16384);
    driver.read_accel().unwrap();

    // Verify bank switches occurred
    let switch_count = interface.bank_switch_count();

    assert!(
        switch_count > 0,
        "Bank switches should occur during normal operations"
    );
}

#[test]
fn test_calibration_workflow() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize and configure
    driver.init(&mut test_utils::MockDelay).unwrap();
    let accel_config = default_accel_config();
    driver.configure_accelerometer(accel_config).unwrap();

    // Set stable readings for calibration
    interface.set_accel_sequence(vec![
        [50, -30, 16434]; 100  // Stable readings with small offsets
    ]);

    // Perform calibration
    let calibration = driver.calibrate_accelerometer(100).unwrap();

    // Verify calibration is reasonable
    assert!(calibration.offset_x.abs() < 1000); // Offsets should be small
    assert!(calibration.offset_y.abs() < 1000);
    assert!(calibration.offset_z.abs() < 1000);

    // Verify calibration is stored
    assert_eq!(
        driver.accelerometer_calibration().offset_x,
        calibration.offset_x
    );

    // Read accelerometer data (calibration should be applied automatically)
    interface.set_accel_data(100, -50, 16484);

    let accel_data = driver.read_accelerometer().unwrap();
    // Data should be in physical units (g)
    assert!(accel_data.x.abs() < 10.0); // Reasonable range
    assert!(accel_data.y.abs() < 10.0);
    assert!(accel_data.z.abs() < 10.0);
}

#[test]
fn test_torn_read_protection() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set accelerometer data
    interface.set_accel_data(1000, 2000, 3000);

    // Read accelerometer - should use burst read to prevent torn reads
    let accel_raw = driver.read_accel().unwrap();

    // Verify data consistency
    assert_eq!(accel_raw.x, 1000);
    assert_eq!(accel_raw.y, 2000);
    assert_eq!(accel_raw.z, 3000);

    // Check that the read was done properly
    // The mock interface should have recorded the operations
    let ops = interface.operations();

    // Should have 6 consecutive read operations starting at ACCEL_XOUT_H (0x2D)
    // The mock interface logs each byte read separately, but they should be consecutive
    let accel_reads: Vec<_> = ops
        .iter()
        .filter_map(|op| {
            if let crate::common::Operation::ReadRegister { address, .. } = op {
                if *address >= 0x2D && *address <= 0x32 {
                    Some(*address)
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect();

    assert_eq!(
        accel_reads.len(),
        6,
        "Should have read 6 consecutive bytes for accelerometer data"
    );

    // Verify they are consecutive addresses (0x2D through 0x32)
    for (i, &addr) in accel_reads.iter().enumerate() {
        assert_eq!(
            addr,
            0x2D + i as u8,
            "Address should be consecutive starting from ACCEL_XOUT_H (0x2D)"
        );
    }
}
