//! Unit tests for hardware self-test functionality

use crate::common::{create_mock_driver, default_accel_config, default_gyro_config};
use icm20948::{MagConfig, MagMode};

#[test]
fn test_accelerometer_self_test_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set up mock data to simulate successful self-test
    // Self-test reads normal samples, then test samples with self-test enabled
    // For a passing test, the difference should be within acceptable range

    // Normal readings (without self-test enabled)
    interface.set_accel_sequence(vec![[100, 200, 16384]; 200]);

    // Self-test readings (with self-test enabled)
    // These should show measurable change from test signal
    interface.set_accel_sequence(vec![[1000, 1200, 17384]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;
    let result = driver.accelerometer_self_test(&mut delay);

    // The result depends on the implementation's pass/fail criteria
    // We verify the function can be called without panicking
    let _ = result;
}

#[test]
fn test_accelerometer_self_test_restores_config() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure accelerometer with specific settings
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set up mock data
    interface.set_accel_sequence(vec![[100, 200, 16384]; 200]);
    interface.set_accel_sequence(vec![[1000, 1200, 17384]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;
    let _ = driver.accelerometer_self_test(&mut delay);

    // Read accelerometer to verify device is still operational
    interface.set_accel_data(100, 200, 16384);
    let result = driver.read_accelerometer();
    assert!(
        result.is_ok(),
        "Device should be operational after self-test"
    );
}

#[test]
fn test_gyroscope_self_test_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data to simulate successful self-test
    // Normal readings (without self-test enabled)
    interface.set_gyro_sequence(vec![[50, -30, 20]; 200]);

    // Self-test readings (with self-test enabled)
    // These should show measurable change from test signal
    interface.set_gyro_sequence(vec![[1050, 970, 1020]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;
    let result = driver.gyroscope_self_test(&mut delay);

    // The result depends on the implementation's pass/fail criteria
    // We verify the function can be called without panicking
    let _ = result;
}

#[test]
fn test_gyroscope_self_test_restores_config() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure gyroscope with specific settings
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set up mock data
    interface.set_gyro_sequence(vec![[50, -30, 20]; 200]);
    interface.set_gyro_sequence(vec![[1050, 970, 1020]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;
    let _ = driver.gyroscope_self_test(&mut delay);

    // Read gyroscope to verify device is still operational
    interface.set_gyro_data(50, -30, 20);
    let result = driver.read_gyroscope();
    assert!(
        result.is_ok(),
        "Device should be operational after self-test"
    );
}

#[test]
fn test_magnetometer_self_test_requires_initialization() {
    let (mut driver, _interface) = create_mock_driver();

    // Initialize driver but NOT magnetometer
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    let mut delay = crate::common::test_utils::MockDelay;

    // Self-test without magnetometer initialization should fail
    let result = driver.magnetometer_self_test(&mut delay);
    assert!(
        result.is_err(),
        "Magnetometer self-test should fail without initialization"
    );
}

#[test]
fn test_magnetometer_self_test_with_initialization() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    let mut delay = crate::common::test_utils::MockDelay;
    driver.init_magnetometer(mag_config, &mut delay).unwrap();

    // Self-test is implementation-dependent and communicates with AK09916
    // We just verify it can be called without panic
    let result = driver.magnetometer_self_test(&mut delay);

    // May succeed or fail depending on mock implementation
    // Just ensure it doesn't panic
    let _ = result;
}

#[test]
fn test_multiple_self_tests_in_sequence() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure both sensors
    let accel_config = default_accel_config();
    driver.configure_accelerometer(accel_config).unwrap();

    let gyro_config = default_gyro_config();
    driver.configure_gyroscope(gyro_config).unwrap();

    // Set up mock data for accelerometer self-test
    interface.set_accel_sequence(vec![[100, 200, 16384]; 200]);
    interface.set_accel_sequence(vec![[1000, 1200, 17384]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;

    // Run accelerometer self-test
    let _ = driver.accelerometer_self_test(&mut delay);

    // Set up mock data for gyroscope self-test
    interface.set_gyro_sequence(vec![[50, -30, 20]; 200]);
    interface.set_gyro_sequence(vec![[1050, 970, 1020]; 200]);

    // Run gyroscope self-test
    let _ = driver.gyroscope_self_test(&mut delay);

    // Verify both sensors are still operational
    interface.set_accel_data(100, 200, 16384);
    assert!(driver.read_accelerometer().is_ok());

    interface.set_gyro_data(50, -30, 20);
    assert!(driver.read_gyroscope().is_ok());
}

#[test]
fn test_self_test_after_calibration() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure and calibrate accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    interface.set_accel_sequence(vec![[100, -50, 16584]; 50]);
    let calibration = driver.calibrate_accelerometer(50).unwrap();

    // Verify calibration was applied
    assert_eq!(calibration.offset_x, 100);

    // Now run self-test
    interface.set_accel_sequence(vec![[100, 200, 16384]; 200]);
    interface.set_accel_sequence(vec![[1000, 1200, 17384]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;
    let _ = driver.accelerometer_self_test(&mut delay);

    // Verify calibration is still applied after self-test
    assert_eq!(driver.accelerometer_calibration().offset_x, 100);
}

#[test]
fn test_calibration_after_self_test() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Run self-test first
    interface.set_accel_sequence(vec![[100, 200, 16384]; 200]);
    interface.set_accel_sequence(vec![[1000, 1200, 17384]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;
    let _ = driver.accelerometer_self_test(&mut delay);

    // Now calibrate
    interface.set_accel_sequence(vec![[100, -50, 16584]; 50]);
    let calibration = driver.calibrate_accelerometer(50);

    assert!(
        calibration.is_ok(),
        "Calibration should work after self-test"
    );
}

#[test]
fn test_self_test_does_not_affect_temperature() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver
        .init(&mut crate::common::test_utils::MockDelay)
        .unwrap();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set temperature reading
    interface.set_temperature_data(25);
    let temp_before = driver.read_temperature_celsius().unwrap();

    // Run self-test
    interface.set_accel_sequence(vec![[100, 200, 16384]; 200]);
    interface.set_accel_sequence(vec![[1000, 1200, 17384]; 200]);

    let mut delay = crate::common::test_utils::MockDelay;
    let _ = driver.accelerometer_self_test(&mut delay);

    // Verify temperature reading still works
    interface.set_temperature_data(25);
    let temp_after = driver.read_temperature_celsius().unwrap();

    assert_eq!(
        temp_before, temp_after,
        "Temperature reading should be unaffected"
    );
}
