//! Unit tests for temperature sensor functionality

use crate::common::{create_mock_driver, test_utils};

#[test]
fn test_temperature_read_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set temperature to room temperature (21°C corresponds to raw value 0)
    // Formula: Temp_degC = (TEMP_OUT / 333.87) + 21
    // For 25°C: TEMP_OUT = (25 - 21) * 333.87 = 1335.48 ≈ 1335
    interface.set_temperature_data(1335);

    let temp = driver.read_temperature_celsius().unwrap();

    // Should be approximately 25°C (with some tolerance for float math)
    assert!((temp - 25.0).abs() < 0.1, "Expected ~25°C, got {}", temp);
}

#[test]
fn test_temperature_read_range() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test minimum operating temperature (-40°C)
    // TEMP_OUT = (-40 - 21) * 333.87 = -20366
    interface.set_temperature_data(-20366);
    let temp_min = driver.read_temperature_celsius().unwrap();
    assert!(
        (temp_min - (-40.0)).abs() < 1.0,
        "Expected ~-40°C, got {}",
        temp_min
    );

    // Test maximum operating temperature (85°C)
    // TEMP_OUT = (85 - 21) * 333.87 = 21368
    interface.set_temperature_data(21368);
    let temp_max = driver.read_temperature_celsius().unwrap();
    assert!(
        (temp_max - 85.0).abs() < 1.0,
        "Expected ~85°C, got {}",
        temp_max
    );

    // Verify within valid range
    assert!(
        temp_min > -45.0 && temp_min < -35.0,
        "Min temp out of range"
    );
    assert!(temp_max > 80.0 && temp_max < 90.0, "Max temp out of range");
}

#[test]
fn test_temperature_conversion() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test conversion formula: Temp_degC = (TEMP_OUT / 333.87) + 21
    // Test with raw value 0 (should give 21°C)
    interface.set_temperature_data(0);
    let temp = driver.read_temperature_celsius().unwrap();
    assert!(
        (temp - 21.0).abs() < 0.1,
        "Expected 21°C for raw=0, got {}",
        temp
    );

    // Test with positive offset
    interface.set_temperature_data(3339); // (3339 / 333.87) + 21 ≈ 31°C
    let temp = driver.read_temperature_celsius().unwrap();
    assert!((temp - 31.0).abs() < 0.5, "Expected ~31°C, got {}", temp);

    // Test with negative offset
    interface.set_temperature_data(-3339); // (-3339 / 333.87) + 21 ≈ 11°C
    let temp = driver.read_temperature_celsius().unwrap();
    assert!((temp - 11.0).abs() < 0.5, "Expected ~11°C, got {}", temp);
}

#[test]
fn test_temperature_read_raw() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set specific raw value
    let expected_raw: i16 = 5000;
    interface.set_temperature_data(expected_raw);

    // Read raw temperature
    let raw = driver.read_temperature().unwrap();
    assert_eq!(raw, expected_raw, "Raw temperature should match");
}

#[test]
fn test_temperature_data_sanity() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set reasonable room temperature
    interface.set_temperature_data(668); // ~23°C

    let temp = driver.read_temperature_celsius().unwrap();

    // Verify temperature is physically reasonable for room temperature
    assert!(temp > 15.0, "Temperature too low for room temp");
    assert!(temp < 35.0, "Temperature too high for room temp");
}

#[test]
fn test_temperature_sequential_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set initial temperature
    interface.set_temperature_data(1000);
    let temp1 = driver.read_temperature_celsius().unwrap();

    // Change temperature (simulating warming)
    interface.set_temperature_data(2000);
    let temp2 = driver.read_temperature_celsius().unwrap();

    // Second reading should be higher
    assert!(
        temp2 > temp1,
        "Temperature should increase: {} -> {}",
        temp1,
        temp2
    );
}

#[test]
fn test_temperature_negative_values() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test with various negative raw values
    interface.set_temperature_data(-1000);
    let temp = driver.read_temperature_celsius().unwrap();
    assert!(
        temp < 21.0,
        "Negative raw value should give temp below 21°C"
    );

    interface.set_temperature_data(-5000);
    let temp = driver.read_temperature_celsius().unwrap();
    assert!(temp < 10.0, "Large negative raw should give cold temp");
}

#[test]
fn test_temperature_extreme_values() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test with i16::MAX
    interface.set_temperature_data(i16::MAX);
    let temp = driver.read_temperature_celsius().unwrap();
    assert!(
        temp < 150.0,
        "Even with max raw value, temp should be reasonable"
    );

    // Test with i16::MIN
    interface.set_temperature_data(i16::MIN);
    let temp = driver.read_temperature_celsius().unwrap();
    assert!(
        temp > -80.0,
        "Even with min raw value, temp should be reasonable"
    );
}

#[test]
fn test_temperature_precision() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test sensitivity: 333.87 LSB/°C means resolution is ~0.003°C
    // Set two values that differ by 1 LSB
    interface.set_temperature_data(1000);
    let temp1 = driver.read_temperature_celsius().unwrap();

    interface.set_temperature_data(1001);
    let temp2 = driver.read_temperature_celsius().unwrap();

    let diff = (temp2 - temp1).abs();
    // 1 LSB = 1/333.87 ≈ 0.003°C
    assert!(
        diff < 0.01,
        "Single LSB difference should be < 0.01°C, got {}",
        diff
    );
}

#[test]
fn test_temperature_read_without_init() {
    let (mut driver, interface) = create_mock_driver();

    // Don't initialize driver

    // Set temperature data
    interface.set_temperature_data(1000);

    // Read should still work (no initialization required for temperature)
    let result = driver.read_temperature_celsius();
    assert!(
        result.is_ok(),
        "Temperature read should work without full initialization"
    );
}

#[test]
fn test_temperature_multiple_reads_consistency() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set stable temperature
    interface.set_temperature_data(2000);

    // Read multiple times
    let temp1 = driver.read_temperature_celsius().unwrap();
    let temp2 = driver.read_temperature_celsius().unwrap();
    let temp3 = driver.read_temperature_celsius().unwrap();

    // All readings should be identical for stable temperature
    assert_eq!(temp1, temp2, "Sequential reads should be consistent");
    assert_eq!(temp2, temp3, "Sequential reads should be consistent");
}

#[test]
fn test_temperature_byte_order() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test big-endian byte order with specific pattern
    // Raw value: 0x1234 (4660 decimal)
    interface.set_temperature_data(0x1234);

    let raw = driver.read_temperature().unwrap();
    assert_eq!(raw, 0x1234, "Big-endian byte order should be preserved");
}
