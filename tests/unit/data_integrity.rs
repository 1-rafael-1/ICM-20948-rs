//! Unit tests for data integrity validation

use crate::common::{create_mock_driver, default_accel_config, default_gyro_config, test_utils};
use icm20948::sensors::{AccelFullScale, GyroFullScale};

#[test]
fn test_sensor_data_range_validation_accel() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer with ±2g scale
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // At ±2g, maximum raw value should be ±32768
    // Test values within valid range
    interface.set_accel_data(16384, -16384, 32767);
    let result = driver.read_accel();
    assert!(result.is_ok(), "Valid data should be readable");

    let data = result.unwrap();
    assert!(data.x.abs() <= i16::MAX, "X should be within i16 range");
    assert!(data.y.abs() <= i16::MAX, "Y should be within i16 range");
    assert!(data.z.abs() <= i16::MAX, "Z should be within i16 range");
}

#[test]
fn test_sensor_data_range_validation_gyro() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope with ±250dps scale
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Test values within valid range
    interface.set_gyro_data(16000, -16000, 0);
    let result = driver.read_gyroscope();
    assert!(result.is_ok(), "Valid data should be readable");

    let data = result.unwrap();
    // At ±250dps with sensitivity 131 LSB/(°/s), max is ±32768
    // Physical range: ±250 dps
    assert!(
        data.x.abs() <= 300.0,
        "X rotation rate should be reasonable"
    );
    assert!(
        data.y.abs() <= 300.0,
        "Y rotation rate should be reasonable"
    );
    assert!(
        data.z.abs() <= 300.0,
        "Z rotation rate should be reasonable"
    );
}

#[test]
fn test_temperature_data_sanity() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Test various temperatures within operating range
    let test_temps = vec![
        (-20000, -40.0), // Cold: -40°C
        (0, 21.0),       // Room temp offset: 21°C
        (1335, 25.0),    // Room temp: ~25°C
        (21368, 85.0),   // Hot: ~85°C
    ];

    for (raw, expected) in test_temps {
        interface.set_temperature_data(raw);
        let temp = driver.read_temperature_celsius().unwrap();

        // Allow for floating-point tolerance
        assert!(
            (temp - expected).abs() < 5.0,
            "Temperature {} should be near {} but got {}",
            raw,
            expected,
            temp
        );

        // Verify within physical operating range
        assert!(
            temp >= -45.0 && temp <= 90.0,
            "Temperature should be within operating range"
        );
    }
}

#[test]
fn test_sequential_read_consistency_accel() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set stable data
    interface.set_accel_data(1000, 2000, 16384);

    // Read multiple times
    let read1 = driver.read_accel().unwrap();
    let read2 = driver.read_accel().unwrap();
    let read3 = driver.read_accel().unwrap();

    // All reads should be identical for stable data
    assert_eq!(read1.x, read2.x, "Sequential reads should be consistent");
    assert_eq!(read2.x, read3.x, "Sequential reads should be consistent");
    assert_eq!(read1.y, read2.y, "Sequential reads should be consistent");
    assert_eq!(read2.y, read3.y, "Sequential reads should be consistent");
    assert_eq!(read1.z, read2.z, "Sequential reads should be consistent");
    assert_eq!(read2.z, read3.z, "Sequential reads should be consistent");
}

#[test]
fn test_sequential_read_consistency_gyro() {
    let (mut driver, interface) = create_mock_driver();

    // Configure gyroscope
    let config = default_gyro_config();
    driver.configure_gyroscope(config).unwrap();

    // Set stable data
    interface.set_gyro_data(100, -200, 50);

    // Read multiple times
    let read1 = driver.read_gyroscope().unwrap();
    let read2 = driver.read_gyroscope().unwrap();
    let read3 = driver.read_gyroscope().unwrap();

    // All reads should be identical for stable data
    assert!(
        (read1.x - read2.x).abs() < 0.001,
        "Sequential reads should be consistent"
    );
    assert!(
        (read2.x - read3.x).abs() < 0.001,
        "Sequential reads should be consistent"
    );
    assert!(
        (read1.y - read2.y).abs() < 0.001,
        "Sequential reads should be consistent"
    );
    assert!(
        (read2.y - read3.y).abs() < 0.001,
        "Sequential reads should be consistent"
    );
}

#[test]
fn test_simultaneous_sensor_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize and configure
    driver.init(&mut test_utils::MockDelay).unwrap();
    let accel_config = default_accel_config();
    let gyro_config = default_gyro_config();
    driver.configure_accelerometer(accel_config).unwrap();
    driver.configure_gyroscope(gyro_config).unwrap();

    // Set all sensor data
    interface.set_accel_data(1000, 2000, 16384);
    interface.set_gyro_data(100, -200, 50);
    interface.set_temperature_data(1335);

    // Read all sensors in quick succession
    let accel = driver.read_accelerometer().unwrap();
    let gyro = driver.read_gyroscope().unwrap();
    let temp = driver.read_temperature_celsius().unwrap();

    // Verify all readings are valid
    assert!(accel.x.abs() < 10.0, "Accel X should be reasonable");
    assert!(accel.y.abs() < 10.0, "Accel Y should be reasonable");
    assert!(accel.z.abs() < 10.0, "Accel Z should be reasonable");

    assert!(gyro.x.abs() < 10.0, "Gyro X should be reasonable");
    assert!(gyro.y.abs() < 10.0, "Gyro Y should be reasonable");
    assert!(gyro.z.abs() < 10.0, "Gyro Z should be reasonable");

    assert!(
        temp > 15.0 && temp < 35.0,
        "Temperature should be room temp"
    );
}

#[test]
fn test_data_after_configuration_change() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer with ±2g
    let mut config = default_accel_config();
    config.full_scale = AccelFullScale::G2;
    driver.configure_accelerometer(config).unwrap();

    // Set and read data
    interface.set_accel_data(16384, 0, 16384);
    let data1 = driver.read_accelerometer().unwrap();

    // Reconfigure with ±16g
    config.full_scale = AccelFullScale::G16;
    driver.configure_accelerometer(config).unwrap();

    // Read same raw values with different scale
    interface.set_accel_data(16384, 0, 16384);
    let data2 = driver.read_accelerometer().unwrap();

    // Physical values should differ due to different sensitivity
    // At ±2g: 16384 LSB/g, so 16384 raw = 1g
    // At ±16g: 2048 LSB/g, so 16384 raw = 8g
    assert!(
        (data2.x - data1.x * 8.0).abs() < 0.5,
        "Scale change should affect physical readings"
    );
}

#[test]
fn test_gyro_data_after_scale_change() {
    let (mut driver, interface) = create_mock_driver();

    // Configure with ±250dps
    let mut config = default_gyro_config();
    config.full_scale = GyroFullScale::Dps250;
    driver.configure_gyroscope(config).unwrap();

    interface.set_gyro_data(131, 0, 0); // 1 dps at ±250 scale
    let data1 = driver.read_gyroscope().unwrap();

    // Reconfigure with ±2000dps
    config.full_scale = GyroFullScale::Dps2000;
    driver.configure_gyroscope(config).unwrap();

    interface.set_gyro_data(131, 0, 0); // 8 dps at ±2000 scale
    let data2 = driver.read_gyroscope().unwrap();

    // Physical values should differ
    assert!(
        data2.x > data1.x,
        "Same raw value at different scale should give different physical value"
    );
}

#[test]
fn test_zero_values_handling() {
    let (mut driver, interface) = create_mock_driver();

    // Configure sensors
    driver.init(&mut test_utils::MockDelay).unwrap();
    let accel_config = default_accel_config();
    let gyro_config = default_gyro_config();
    driver.configure_accelerometer(accel_config).unwrap();
    driver.configure_gyroscope(gyro_config).unwrap();

    // Set all zeros
    interface.set_accel_data(0, 0, 0);
    interface.set_gyro_data(0, 0, 0);
    interface.set_temperature_data(0);

    // All should read successfully
    let accel = driver.read_accelerometer().unwrap();
    let gyro = driver.read_gyroscope().unwrap();
    let temp = driver.read_temperature_celsius().unwrap();

    // Verify zero handling
    assert!(accel.x.abs() < 0.1, "Zero accel should be near zero");
    assert!(gyro.x.abs() < 0.1, "Zero gyro should be near zero");
    assert!((temp - 21.0).abs() < 0.5, "Zero temp raw = 21°C");
}

#[test]
fn test_maximum_values_handling() {
    let (mut driver, interface) = create_mock_driver();

    // Configure sensors
    driver.init(&mut test_utils::MockDelay).unwrap();
    let accel_config = default_accel_config();
    let gyro_config = default_gyro_config();
    driver.configure_accelerometer(accel_config).unwrap();
    driver.configure_gyroscope(gyro_config).unwrap();

    // Set maximum values
    interface.set_accel_data(i16::MAX, i16::MAX, i16::MAX);
    interface.set_gyro_data(i16::MAX, i16::MAX, i16::MAX);
    interface.set_temperature_data(i16::MAX);

    // All should read successfully without overflow
    let accel = driver.read_accelerometer().unwrap();
    let gyro = driver.read_gyroscope().unwrap();
    let temp = driver.read_temperature_celsius().unwrap();

    // Values should be large but not infinite
    assert!(accel.x.is_finite(), "Accel should be finite");
    assert!(gyro.x.is_finite(), "Gyro should be finite");
    assert!(temp.is_finite(), "Temperature should be finite");
}

#[test]
fn test_minimum_values_handling() {
    let (mut driver, interface) = create_mock_driver();

    // Configure sensors
    driver.init(&mut test_utils::MockDelay).unwrap();
    let accel_config = default_accel_config();
    let gyro_config = default_gyro_config();
    driver.configure_accelerometer(accel_config).unwrap();
    driver.configure_gyroscope(gyro_config).unwrap();

    // Set minimum values
    interface.set_accel_data(i16::MIN, i16::MIN, i16::MIN);
    interface.set_gyro_data(i16::MIN, i16::MIN, i16::MIN);
    interface.set_temperature_data(i16::MIN);

    // All should read successfully without underflow
    let accel = driver.read_accelerometer().unwrap();
    let gyro = driver.read_gyroscope().unwrap();
    let temp = driver.read_temperature_celsius().unwrap();

    // Values should be large negative but not infinite
    assert!(accel.x.is_finite(), "Accel should be finite");
    assert!(gyro.x.is_finite(), "Gyro should be finite");
    assert!(temp.is_finite(), "Temperature should be finite");
}

#[test]
fn test_data_consistency_after_calibration() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Calibrate
    interface.set_accel_sequence(vec![[100, -50, 16584]; 50]);
    driver.calibrate_accelerometer(50).unwrap();

    // Read data multiple times
    interface.set_accel_data(200, 50, 16684);

    let read1 = driver.read_accelerometer().unwrap();
    let read2 = driver.read_accelerometer().unwrap();
    let read3 = driver.read_accelerometer().unwrap();

    // Calibrated reads should be consistent
    assert!(
        (read1.x - read2.x).abs() < 0.001,
        "Calibrated reads should be consistent"
    );
    assert!(
        (read2.x - read3.x).abs() < 0.001,
        "Calibrated reads should be consistent"
    );
}

#[test]
fn test_alternating_sensor_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize and configure
    driver.init(&mut test_utils::MockDelay).unwrap();
    let accel_config = default_accel_config();
    let gyro_config = default_gyro_config();
    driver.configure_accelerometer(accel_config).unwrap();
    driver.configure_gyroscope(gyro_config).unwrap();

    interface.set_accel_data(1000, 2000, 16384);
    interface.set_gyro_data(100, -200, 50);

    // Alternate between sensor reads
    for _ in 0..5 {
        let accel = driver.read_accelerometer().unwrap();
        assert!(accel.x.abs() < 10.0, "Accel should be stable");

        let gyro = driver.read_gyroscope().unwrap();
        assert!(gyro.x.abs() < 10.0, "Gyro should be stable");
    }
}

#[test]
fn test_rapid_sequential_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    interface.set_accel_data(1000, 2000, 16384);

    // Rapid reads should all succeed
    for _ in 0..100 {
        let result = driver.read_accelerometer();
        assert!(result.is_ok(), "Rapid reads should not fail");
    }
}

#[test]
fn test_data_integrity_across_bank_switches() {
    let (mut driver, interface) = create_mock_driver();

    // Configure accelerometer (requires bank switch to Bank 2)
    let config = default_accel_config();
    driver.configure_accelerometer(config).unwrap();

    // Set data
    interface.set_accel_data(1000, 2000, 16384);

    // Read (Bank 0)
    let read1 = driver.read_accelerometer().unwrap();

    // Reconfigure (Bank 2 switch)
    driver.configure_accelerometer(config).unwrap();

    // Read again (Bank 0)
    interface.set_accel_data(1000, 2000, 16384);
    let read2 = driver.read_accelerometer().unwrap();

    // Data should be consistent across bank switches
    assert!(
        (read1.x - read2.x).abs() < 0.001,
        "Data should be consistent across bank switches"
    );
}
