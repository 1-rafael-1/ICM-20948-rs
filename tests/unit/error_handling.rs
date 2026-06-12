//! Unit tests for error handling and recovery

use crate::common::{create_mock_driver, default_accel_config, default_gyro_config, test_utils};

#[test]
fn test_read_failure_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Inject a read failure
    interface.fail_next_read();

    // This read should fail
    let result = driver.read_accel();
    assert!(result.is_err(), "Read should fail when error is injected");
}

#[test]
fn test_read_failure_recovery() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Inject a read failure
    interface.fail_next_read();

    // This read should fail
    let result = driver.read_accel();
    assert!(result.is_err(), "First read should fail");

    // Set valid data for next read
    interface.set_accel_data(100, 200, 300);

    // Subsequent read should succeed (error was only for one operation)
    let result = driver.read_accel();
    assert!(
        result.is_ok(),
        "Subsequent read should succeed after single failure"
    );
}

#[test]
fn test_write_failure_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Inject a write failure
    interface.fail_next_write();

    // This configuration write should fail
    let config = default_accel_config();
    let result = driver.configure_accelerometer(config);
    assert!(result.is_err(), "Write should fail when error is injected");
}

#[test]
fn test_multiple_read_failures() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test multiple failures in sequence
    for i in 0..3 {
        interface.fail_next_read();
        let result = driver.read_accel();
        assert!(
            result.is_err(),
            "Read {} should fail when error is injected",
            i
        );
    }

    // Recovery should still work
    interface.set_accel_data(100, 200, 300);
    let result = driver.read_accel();
    assert!(result.is_ok(), "Should recover after multiple failures");
}

#[test]
fn test_bank_switch_failure_propagation() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Enable bank switch failure
    interface.fail_bank_switch(true);

    // Configure accelerometer requires bank switch to Bank 2
    let config = default_accel_config();
    let result = driver.configure_accelerometer(config);

    // Should fail due to bank switch error
    assert!(
        result.is_err(),
        "Configuration should fail when bank switch fails"
    );

    // Disable bank switch failure
    interface.fail_bank_switch(false);

    // Now it should work
    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "Should succeed after bank switch is fixed");
}

#[test]
fn test_error_during_calibration() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set up initial data
    interface.set_accel_sequence(vec![[100, 200, 300]; 10]);

    // Start calibration but inject failure midway
    // This will fail on one of the reads during calibration
    interface.fail_next_read();

    let result = driver.calibrate_accelerometer(50);
    assert!(
        result.is_err(),
        "Calibration should fail if read fails during sampling"
    );
}

#[test]
fn test_gyro_read_failure() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize and configure
    driver.init(&mut test_utils::MockDelay).unwrap();
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Inject read failure
    interface.fail_next_read();

    // Gyro read should fail
    let result = driver.read_gyroscope();
    assert!(result.is_err(), "Gyroscope read should fail");

    // Recovery
    interface.set_gyro_data(10, 20, 30);
    let result = driver.read_gyroscope();
    assert!(result.is_ok(), "Should recover from gyroscope read failure");
}

#[test]
fn test_temperature_read_failure() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Inject read failure
    interface.fail_next_read();

    // Temperature read should fail
    let result = driver.read_temperature_celsius();
    assert!(result.is_err(), "Temperature read should fail");

    // Recovery
    interface.set_temperature_data(1000);
    let result = driver.read_temperature_celsius();
    assert!(
        result.is_ok(),
        "Should recover from temperature read failure"
    );
}

#[test]
fn test_consecutive_write_failures() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    let config = default_accel_config();

    // Test multiple write failures
    for _ in 0..3 {
        interface.fail_next_write();
        let result = driver.configure_accelerometer(config);
        assert!(result.is_err(), "Each write should fail");
    }

    // Should eventually succeed
    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "Should succeed after failures are cleared");
}

#[test]
fn test_alternating_failures() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Alternating read and write failures
    interface.fail_next_read();
    let result = driver.read_accel();
    assert!(result.is_err(), "Read should fail");

    interface.fail_next_write();
    let config = default_accel_config();
    let result = driver.configure_accelerometer(config);
    assert!(result.is_err(), "Write should fail");

    // Both should work now
    interface.set_accel_data(100, 200, 300);
    let result = driver.read_accel();
    assert!(result.is_ok(), "Read should succeed");

    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "Write should succeed");
}

#[test]
fn test_error_during_initialization() {
    use crate::common::mock_interface::MockInterface;
    use icm20948::Icm20948Driver;

    // Create an interface and inject a failure before WHO_AM_I is read.
    // try_new() reads WHO_AM_I as its very first I2C operation, so this
    // failure must cause driver creation to fail.
    let interface = MockInterface::new();
    interface.fail_next_read();

    let result = Icm20948Driver::try_new(interface);
    assert!(
        result.is_err(),
        "Driver creation must fail when the WHO_AM_I read fails"
    );
}

#[test]
fn test_bank_switch_failure_during_read() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Enable bank switch failure
    interface.fail_bank_switch(true);

    // Reading accelerometer requires staying in Bank 0, so this might still work
    // But any operation requiring bank switch will fail
    interface.set_accel_data(100, 200, 300);

    // This should succeed since accel data is in Bank 0
    let result = driver.read_accel();
    assert!(
        result.is_ok(),
        "Read in same bank should succeed even with bank switch failure enabled"
    );

    // But configuration that requires bank switch should fail
    let config = default_accel_config();
    let result = driver.configure_accelerometer(config);
    assert!(
        result.is_err(),
        "Configuration requiring bank switch should fail"
    );
}

#[test]
fn test_error_clears_operations_log() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Clear operations log, then inject a read failure.
    interface.clear_operations();
    interface.fail_next_read();
    let _result = driver.read_accel();

    // The mock checks fail_next_read *before* appending to the operations log,
    // so a failed read contributes zero entries. The log must still be empty.
    let ops = interface.operations();
    assert!(
        ops.is_empty(),
        "A failed read must not be logged: mock returns before the logging path"
    );
}

#[test]
fn test_partial_calibration_failure() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    interface.set_accel_sequence(vec![[100, 200, 16584]; 100]);

    // fail_next_read fires on the very next I2C read, which will be the first
    // accel sample read inside calibrate_accelerometer. The error must propagate
    // out of the calibration routine.
    interface.fail_next_read();

    let result = driver.calibrate_accelerometer(50);
    assert!(
        result.is_err(),
        "Calibration must fail when the first sample read fails"
    );
}

#[test]
fn test_gyro_calibration_with_error() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    interface.set_gyro_sequence(vec![[50, -30, 20]; 100]);

    // fail_next_read fires on the very next I2C read, which will be the first
    // gyro sample read inside calibrate_gyroscope. The error must propagate out.
    interface.fail_next_read();

    let result = driver.calibrate_gyroscope(50);
    assert!(
        result.is_err(),
        "Calibration must fail when the first sample read fails"
    );
}

#[test]
fn test_error_state_isolation() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Create error condition
    interface.fail_next_read();
    let _result = driver.read_accel();

    // Error should not affect unrelated operations
    interface.set_temperature_data(1000);
    let temp_result = driver.read_temperature_celsius();
    assert!(
        temp_result.is_ok(),
        "Error in accel read should not affect temperature read"
    );

    // And subsequent accel reads should work
    interface.set_accel_data(100, 200, 300);
    let accel_result = driver.read_accel();
    assert!(accel_result.is_ok(), "Accel read should recover");
}
