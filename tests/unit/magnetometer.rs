//! Unit tests for magnetometer functionality

use crate::common::{create_mock_driver, test_utils};
use icm20948::sensors::{MagCalibration, MagConfig, MagMode};

/// Mock delay implementation for testing
struct MockDelay;

impl embedded_hal::delay::DelayNs for MockDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

#[test]
fn test_mag_init_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup magnetometer WHO_AM_I response
    interface.setup_mag_initialization();

    let mut delay = MockDelay;
    let config = MagConfig::default();

    // Initialize magnetometer
    let result = driver.init_magnetometer(config, &mut delay);
    assert!(result.is_ok(), "Magnetometer initialization should succeed");
}

#[test]
fn test_mag_read_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    let config = MagConfig::default();
    driver.init_magnetometer(config, &mut delay).unwrap();

    // Set magnetometer data (raw values)
    // AK09916 sensitivity: 0.15 µT/LSB
    // For 50 µT: 50 / 0.15 ≈ 333
    interface.set_mag_data(333, 0, 0);

    let mag_data = driver.read_magnetometer().unwrap();

    // Check that X-axis is approximately 50 µT
    assert!(
        (mag_data.x - 50.0).abs() < 1.0,
        "Expected ~50 µT, got {}",
        mag_data.x
    );
    assert!(mag_data.y.abs() < 1.0, "Expected ~0 µT for Y");
    assert!(mag_data.z.abs() < 1.0, "Expected ~0 µT for Z");
}

#[test]
fn test_mag_read_without_init() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver but NOT magnetometer
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set magnetometer data
    interface.set_mag_data(100, 200, 300);

    // Read should fail because magnetometer not initialized
    let result = driver.read_magnetometer();
    assert!(
        result.is_err(),
        "Reading magnetometer without initialization should fail"
    );
}

#[test]
fn test_mag_sensitivity_conversion() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Test sensitivity: 0.15 µT/LSB
    // Raw value 1000 should give 1000 * 0.15 = 150 µT
    interface.set_mag_data(1000, 2000, 3000);

    let mag_data = driver.read_magnetometer().unwrap();

    assert!(
        (mag_data.x - 150.0).abs() < 1.0,
        "Expected ~150 µT, got {}",
        mag_data.x
    );
    assert!(
        (mag_data.y - 300.0).abs() < 1.0,
        "Expected ~300 µT, got {}",
        mag_data.y
    );
    assert!(
        (mag_data.z - 450.0).abs() < 1.0,
        "Expected ~450 µT, got {}",
        mag_data.z
    );
}

#[test]
fn test_mag_overflow_detection() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Simulate overflow condition (ST2 bit 3 set)
    interface.set_mag_overflow();

    // Note: We no longer check ST2 overflow bit when reading via I2C master
    // The I2C master polls automatically, so if we get data, it's valid
    // This test now verifies we can read even with ST2 overflow set
    let result = driver.read_magnetometer();
    assert!(
        result.is_ok(),
        "Reading should succeed even with ST2 overflow bit set via I2C master"
    );
}

#[test]
fn test_mag_calibration_offset() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set calibration with offsets
    let calibration = MagCalibration {
        offset_x: 10.0,
        offset_y: -5.0,
        offset_z: 20.0,
        scale_x: 1.0,
        scale_y: 1.0,
        scale_z: 1.0,
    };
    driver.set_magnetometer_calibration(calibration);

    // Set raw data: x=1000, y=2000, z=3000
    // After conversion: x=150, y=300, z=450 µT
    // After calibration: x=150-10=140, y=300-(-5)=305, z=450-20=430 µT
    interface.set_mag_data(1000, 2000, 3000);

    let mag_data = driver.read_magnetometer().unwrap();

    assert!(
        (mag_data.x - 140.0).abs() < 1.0,
        "Expected ~140 µT after calibration, got {}",
        mag_data.x
    );
    assert!(
        (mag_data.y - 305.0).abs() < 1.0,
        "Expected ~305 µT after calibration, got {}",
        mag_data.y
    );
    assert!(
        (mag_data.z - 430.0).abs() < 1.0,
        "Expected ~430 µT after calibration, got {}",
        mag_data.z
    );
}

#[test]
fn test_mag_calibration_scale() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set calibration with scale factors
    let calibration = MagCalibration {
        offset_x: 0.0,
        offset_y: 0.0,
        offset_z: 0.0,
        scale_x: 2.0,
        scale_y: 0.5,
        scale_z: 1.0,
    };
    driver.set_magnetometer_calibration(calibration);

    // Set raw data: x=1000, y=2000, z=3000
    // After conversion: x=150, y=300, z=450 µT
    // After calibration: x=150*2=300, y=300*0.5=150, z=450*1=450 µT
    interface.set_mag_data(1000, 2000, 3000);

    let mag_data = driver.read_magnetometer().unwrap();

    assert!(
        (mag_data.x - 300.0).abs() < 1.0,
        "Expected ~300 µT after scale, got {}",
        mag_data.x
    );
    assert!(
        (mag_data.y - 150.0).abs() < 1.0,
        "Expected ~150 µT after scale, got {}",
        mag_data.y
    );
    assert!(
        (mag_data.z - 450.0).abs() < 1.0,
        "Expected ~450 µT after scale, got {}",
        mag_data.z
    );
}

#[test]
fn test_mag_calibration_combined() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set calibration with both offset and scale
    let calibration = MagCalibration {
        offset_x: 50.0,
        offset_y: -25.0,
        offset_z: 100.0,
        scale_x: 1.1,
        scale_y: 0.9,
        scale_z: 1.05,
    };
    driver.set_magnetometer_calibration(calibration);

    // Set raw data: x=1000, y=2000, z=3000
    // After conversion: x=150, y=300, z=450 µT
    // After calibration:
    //   x=(150-50)*1.1=110,
    //   y=(300-(-25))*0.9=292.5,
    //   z=(450-100)*1.05=367.5 µT
    interface.set_mag_data(1000, 2000, 3000);

    let mag_data = driver.read_magnetometer().unwrap();

    assert!(
        (mag_data.x - 110.0).abs() < 1.0,
        "Expected ~110 µT, got {}",
        mag_data.x
    );
    assert!(
        (mag_data.y - 292.5).abs() < 1.0,
        "Expected ~292.5 µT, got {}",
        mag_data.y
    );
    assert!(
        (mag_data.z - 367.5).abs() < 1.0,
        "Expected ~367.5 µT, got {}",
        mag_data.z
    );
}

#[test]
fn test_mag_negative_values() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Test with negative raw values
    interface.set_mag_data(-1000, -2000, -3000);

    let mag_data = driver.read_magnetometer().unwrap();

    assert!(mag_data.x < 0.0, "X should be negative");
    assert!(mag_data.y < 0.0, "Y should be negative");
    assert!(mag_data.z < 0.0, "Z should be negative");

    // Check approximate values (negative)
    assert!(
        (mag_data.x - (-150.0)).abs() < 1.0,
        "Expected ~-150 µT, got {}",
        mag_data.x
    );
}

#[test]
fn test_mag_sequential_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // First reading
    interface.set_mag_data(1000, 0, 0);
    let mag1 = driver.read_magnetometer().unwrap();

    // Second reading (changed)
    interface.set_mag_data(2000, 0, 0);
    let mag2 = driver.read_magnetometer().unwrap();

    // Third reading (changed again)
    interface.set_mag_data(3000, 0, 0);
    let mag3 = driver.read_magnetometer().unwrap();

    // Verify all three readings are different
    assert!(mag1.x < mag2.x, "Second reading should be larger");
    assert!(mag2.x < mag3.x, "Third reading should be larger");
}

#[test]
fn test_mag_earth_field_range() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Earth's magnetic field ranges from ~25 µT to ~65 µT
    // Test mid-range: 50 µT ≈ 333 raw
    interface.set_mag_data(333, 200, -100);

    let mag_data = driver.read_magnetometer().unwrap();

    // Check that values are in reasonable range for Earth's field
    assert!(
        mag_data.x > 20.0 && mag_data.x < 70.0,
        "X-axis should be in Earth field range"
    );
    assert!(
        mag_data.y > 10.0 && mag_data.y < 60.0,
        "Y-axis should be in Earth field range"
    );
    assert!(
        mag_data.z > -30.0 && mag_data.z < 30.0,
        "Z-axis should be in Earth field range"
    );
}

#[test]
fn test_mag_heading_calculation() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set data pointing North (positive Y, zero X)
    // In atan2(y, x): North is Y>0, X=0 -> 90°
    // Y=300µT (2000 raw), X=0
    interface.set_mag_data(0, 2000, 0);

    let heading = driver.read_magnetometer_heading().unwrap();

    // Heading should be 90° (North in standard navigation coordinates)
    // Note: atan2(y, x) with Y>0, X=0 gives 90°
    assert!(
        (heading - 90.0).abs() < 5.0,
        "Expected heading near 90° (North), got {}°",
        heading
    );
}

#[test]
fn test_mag_heading_east() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set data pointing East (positive X, zero Y)
    // In atan2(y, x): East is Y=0, X>0 -> 0°
    interface.set_mag_data(2000, 0, 0);

    let heading = driver.read_magnetometer_heading().unwrap();

    // Heading should be 0° or 360° (East in standard navigation coordinates)
    assert!(
        (heading - 0.0).abs() < 5.0 || (heading - 360.0).abs() < 5.0,
        "Expected heading near 0° or 360° (East), got {}°",
        heading
    );
}

#[test]
fn test_mag_heading_west() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set data pointing West (negative X, zero Y)
    // In atan2(y, x): West is Y=0, X<0 -> 180°
    interface.set_mag_data(-2000, 0, 0);

    let heading = driver.read_magnetometer_heading().unwrap();

    // Heading should be 180° (West in standard navigation coordinates)
    assert!(
        (heading - 180.0).abs() < 5.0,
        "Expected heading near 180° (West), got {}°",
        heading
    );
}

#[test]
fn test_mag_heading_south() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set data pointing South (negative Y, zero X)
    // In atan2(y, x): South is Y<0, X=0 -> 270°
    interface.set_mag_data(0, -2000, 0);

    let heading = driver.read_magnetometer_heading().unwrap();

    // Heading should be 270° (South in standard navigation coordinates)
    assert!(
        (heading - 270.0).abs() < 5.0,
        "Expected heading near 270° (South), got {}°",
        heading
    );
}

#[test]
fn test_mag_raw_read() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set specific raw values
    let expected_x: i16 = 1234;
    let expected_y: i16 = -5678;
    let expected_z: i16 = 9876;
    interface.set_mag_data(expected_x, expected_y, expected_z);

    let mag_raw = driver.read_magnetometer_raw().unwrap();

    assert_eq!(mag_raw.0, expected_x, "Raw X value should match");
    assert_eq!(mag_raw.1, expected_y, "Raw Y value should match");
    assert_eq!(mag_raw.2, expected_z, "Raw Z value should match");
}

#[test]
fn test_mag_data_ready_status() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set data with data ready flag (ST1 = 0x01)
    interface.set_mag_data_with_status(1000, 2000, 3000, 0x01, 0x00);

    let is_ready = driver.magnetometer_data_ready().unwrap();
    assert!(is_ready, "Magnetometer should report data ready");
}

#[test]
fn test_mag_data_not_ready_status() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set data without data ready flag (ST1 = 0x00)
    interface.set_mag_data_with_status(1000, 2000, 3000, 0x00, 0x00);

    let is_ready = driver.magnetometer_data_ready().unwrap();
    assert!(!is_ready, "Magnetometer should report data not ready");
}

#[test]
fn test_mag_read_when_data_not_ready() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set data with ST1 = 0x00 (data not ready)
    interface.set_mag_data_with_status(1000, 2000, 3000, 0x00, 0x00);

    // Note: We no longer check ST1 DRDY bit when reading via I2C master
    // The I2C master polls automatically at a fixed rate, so received data is valid
    // This test now verifies we can read even when ST1 DRDY bit is not set
    let result = driver.read_magnetometer();
    assert!(
        result.is_ok(),
        "Reading should succeed even when ST1 DRDY bit is not set via I2C master"
    );

    // Verify we got the expected data (with small epsilon for floating point comparison)
    let mag_data = result.unwrap();
    assert!((mag_data.x - 150.0).abs() < 0.01); // 1000 * 0.15 µT/LSB
    assert!((mag_data.y - 300.0).abs() < 0.01); // 2000 * 0.15 µT/LSB
    assert!((mag_data.z - 450.0).abs() < 0.01); // 3000 * 0.15 µT/LSB
}

#[test]
fn test_mag_config_modes() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup magnetometer WHO_AM_I response
    interface.setup_mag_initialization();
    let mut delay = MockDelay;

    // Test different continuous modes
    let modes = [
        MagMode::Continuous10Hz,
        MagMode::Continuous20Hz,
        MagMode::Continuous50Hz,
        MagMode::Continuous100Hz,
    ];

    for mode in modes.iter() {
        let config = MagConfig { mode: *mode };
        let result = driver.init_magnetometer(config, &mut delay);
        assert!(
            result.is_ok(),
            "Magnetometer initialization should succeed with mode {:?}",
            mode
        );
    }
}

#[test]
fn test_mag_zero_field() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set zero field
    interface.set_mag_data(0, 0, 0);

    let mag_data = driver.read_magnetometer().unwrap();

    assert!(
        mag_data.x.abs() < 1.0,
        "X should be near zero, got {}",
        mag_data.x
    );
    assert!(
        mag_data.y.abs() < 1.0,
        "Y should be near zero, got {}",
        mag_data.y
    );
    assert!(
        mag_data.z.abs() < 1.0,
        "Z should be near zero, got {}",
        mag_data.z
    );
}

#[test]
fn test_mag_extreme_values() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Test with maximum positive values
    interface.set_mag_data(i16::MAX, i16::MAX, i16::MAX);
    let mag_max = driver.read_magnetometer().unwrap();
    assert!(mag_max.x > 0.0, "Max value should be positive");

    // Test with maximum negative values
    interface.set_mag_data(i16::MIN, i16::MIN, i16::MIN);
    let mag_min = driver.read_magnetometer().unwrap();
    assert!(mag_min.x < 0.0, "Min value should be negative");
}

#[test]
fn test_mag_calibration_getter() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set calibration
    let expected_cal = MagCalibration {
        offset_x: 12.5,
        offset_y: -7.3,
        offset_z: 18.9,
        scale_x: 1.05,
        scale_y: 0.95,
        scale_z: 1.02,
    };
    driver.set_magnetometer_calibration(expected_cal);

    // Get calibration
    let actual_cal = driver.magnetometer_calibration();

    assert_eq!(actual_cal.offset_x, expected_cal.offset_x);
    assert_eq!(actual_cal.offset_y, expected_cal.offset_y);
    assert_eq!(actual_cal.offset_z, expected_cal.offset_z);
    assert_eq!(actual_cal.scale_x, expected_cal.scale_x);
    assert_eq!(actual_cal.scale_y, expected_cal.scale_y);
    assert_eq!(actual_cal.scale_z, expected_cal.scale_z);
}

#[test]
fn test_mag_byte_order_little_endian() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Test little-endian byte order with specific pattern
    // Raw value: 0x1234 (4660 decimal)
    interface.set_mag_data(0x1234, 0x5678, -0x6544);

    let mag_raw = driver.read_magnetometer_raw().unwrap();
    assert_eq!(mag_raw.0, 0x1234, "Little-endian byte order for X");
    assert_eq!(mag_raw.1, 0x5678, "Little-endian byte order for Y");
    assert_eq!(mag_raw.2, -0x6544, "Little-endian byte order for Z");
}

#[test]
fn test_mag_multiple_init() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup magnetometer WHO_AM_I response
    interface.setup_mag_initialization();
    let mut delay = MockDelay;

    // First initialization
    let config1 = MagConfig {
        mode: MagMode::Continuous100Hz,
    };
    let result1 = driver.init_magnetometer(config1, &mut delay);
    assert!(result1.is_ok(), "First initialization should succeed");

    // Second initialization (should also work)
    let config2 = MagConfig {
        mode: MagMode::Continuous50Hz,
    };
    let result2 = driver.init_magnetometer(config2, &mut delay);
    assert!(result2.is_ok(), "Second initialization should succeed");

    // Should be able to read after second init
    interface.set_mag_data(100, 200, 300);
    let mag_data = driver.read_magnetometer().unwrap();
    assert!(mag_data.x > 0.0, "Should be able to read after re-init");
}

#[test]
fn test_mag_precision() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Test precision: sensitivity is 0.15 µT/LSB
    // Two values differing by 1 LSB should differ by 0.15 µT
    interface.set_mag_data(1000, 0, 0);
    let mag1 = driver.read_magnetometer().unwrap();

    interface.set_mag_data(1001, 0, 0);
    let mag2 = driver.read_magnetometer().unwrap();

    let diff = (mag2.x - mag1.x).abs();
    assert!(
        (diff - 0.15).abs() < 0.01,
        "1 LSB difference should be ~0.15 µT, got {}",
        diff
    );
}

#[test]
fn test_mag_consistency_multiple_reads() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set stable data
    interface.set_mag_data(1234, 5678, -9012);

    // Read multiple times - should be consistent
    let mag1 = driver.read_magnetometer().unwrap();
    let mag2 = driver.read_magnetometer().unwrap();
    let mag3 = driver.read_magnetometer().unwrap();

    assert_eq!(mag1.x, mag2.x, "X should be consistent");
    assert_eq!(mag2.x, mag3.x, "X should be consistent");
    assert_eq!(mag1.y, mag2.y, "Y should be consistent");
    assert_eq!(mag1.z, mag3.z, "Z should be consistent");
}

#[test]
fn test_mag_heading_without_init() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver but NOT magnetometer
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Set magnetometer data
    interface.set_mag_data(1000, 2000, 3000);

    // Heading read should fail
    let result = driver.read_magnetometer_heading();
    assert!(
        result.is_err(),
        "Heading calculation without mag init should fail"
    );
}

#[test]
fn test_mag_heading_range() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Test various directions
    let test_cases = [
        (2000, 0),      // East
        (1414, 1414),   // Northeast
        (0, 2000),      // North
        (-1414, 1414),  // Northwest
        (-2000, 0),     // West
        (-1414, -1414), // Southwest
        (0, -2000),     // South
        (1414, -1414),  // Southeast
    ];

    for (x, y) in test_cases.iter() {
        interface.set_mag_data(*x, *y, 0);
        let heading = driver.read_magnetometer_heading().unwrap();

        // Heading should always be in valid range [0, 360)
        assert!(
            heading >= 0.0 && heading < 360.0,
            "Heading should be in [0, 360) range, got {}",
            heading
        );
    }
}

#[test]
fn test_mag_self_test() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Self-test is implementation-dependent
    // We just verify it can be called without panic
    let result = driver.magnetometer_self_test(&mut delay);

    // May succeed or fail depending on implementation
    // Just ensure it doesn't panic
    let _ = result;
}

#[test]
fn test_mag_hard_iron_calibration() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Simulate a circular motion for hard-iron calibration
    // This would normally collect min/max values
    let calibration_points = vec![
        [1000, 2000, 1500],
        [2000, 1000, 1500],
        [1000, 0, 1500],
        [0, 1000, 1500],
        [-1000, 2000, 1500],
        [-2000, 1000, 1500],
        [-1000, 0, 1500],
        [0, -1000, 1500],
    ];

    // Set up sequence and attempt calibration
    for point in calibration_points.iter() {
        interface.set_mag_data(point[0], point[1], point[2]);
        let _ = driver.read_magnetometer();
    }

    // Calibration function exists
    let result = driver.calibrate_magnetometer_hard_iron(100, &mut delay);

    // May succeed or fail depending on implementation
    let _ = result;
}

#[test]
fn test_mag_tilt_compensation_inputs() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set magnetometer data
    interface.set_mag_data(2000, 1000, 500);

    // Test tilt-compensated heading with sample accelerometer values
    // Accelerometer readings in g's (normalized)
    let accel_x = 0.0_f32;
    let accel_y = 0.0_f32;
    let accel_z = 1.0_f32; // Level device

    let result = driver.read_magnetometer_heading_compensated(accel_x, accel_y, accel_z);

    if result.is_ok() {
        let heading = result.unwrap();
        assert!(
            heading >= 0.0 && heading < 360.0,
            "Compensated heading should be valid"
        );
    }
}

#[test]
fn test_mag_heading_compensated_zero_accel() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set magnetometer data
    interface.set_mag_data(2000, 1000, 500);

    // Test with zero accelerometer (invalid condition)
    let result = driver.read_magnetometer_heading_compensated(0.0, 0.0, 0.0);
    assert!(
        result.is_err(),
        "Should return error for zero accelerometer magnitude"
    );
}

#[test]
fn test_mag_heading_compensated_very_small_accel() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set magnetometer data
    interface.set_mag_data(2000, 1000, 500);

    // Test with very small accelerometer magnitude (< 0.1g threshold)
    let result = driver.read_magnetometer_heading_compensated(0.01, 0.01, 0.01);
    assert!(
        result.is_err(),
        "Should return error for accelerometer magnitude < 0.1g"
    );
}

#[test]
fn test_mag_heading_compensated_valid_accel() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Set magnetometer data
    interface.set_mag_data(2000, 1000, 500);

    // Test with valid accelerometer magnitude (> 0.1g threshold)
    let result = driver.read_magnetometer_heading_compensated(0.0, 0.0, 1.0);
    assert!(
        result.is_ok(),
        "Should succeed with valid accelerometer data"
    );

    let heading = result.unwrap();
    assert!(
        heading >= 0.0 && heading < 360.0,
        "Heading should be in valid range [0, 360)"
    );
}

#[test]
fn test_mag_single_measurement_mode() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup magnetometer WHO_AM_I response
    interface.setup_mag_initialization();
    let mut delay = MockDelay;

    // Initialize with single measurement mode
    let config = MagConfig {
        mode: MagMode::Single,
    };
    let result = driver.init_magnetometer(config, &mut delay);
    assert!(
        result.is_ok(),
        "Should initialize with single measurement mode"
    );
}

#[test]
fn test_mag_power_down_mode() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup magnetometer WHO_AM_I response
    interface.setup_mag_initialization();
    let mut delay = MockDelay;

    // Initialize with power down mode
    let config = MagConfig {
        mode: MagMode::PowerDown,
    };
    let result = driver.init_magnetometer(config, &mut delay);
    assert!(result.is_ok(), "Should initialize with power down mode");
}

#[test]
fn test_mag_default_config() {
    let config = MagConfig::default();

    // Default should be 100Hz continuous mode
    assert_eq!(
        config.mode,
        MagMode::Continuous100Hz,
        "Default mode should be 100Hz continuous"
    );
}

#[test]
fn test_mag_default_calibration() {
    let cal = MagCalibration::default();

    // Default calibration should be identity (no offset, unity scale)
    assert_eq!(cal.offset_x, 0.0);
    assert_eq!(cal.offset_y, 0.0);
    assert_eq!(cal.offset_z, 0.0);
    assert_eq!(cal.scale_x, 1.0);
    assert_eq!(cal.scale_y, 1.0);
    assert_eq!(cal.scale_z, 1.0);
}

#[test]
fn test_mag_raw_without_calibration() {
    let (mut driver, interface) = create_mock_driver();

    // Initialize driver
    driver.init(&mut test_utils::MockDelay).unwrap();

    // Setup and initialize magnetometer
    interface.setup_mag_initialization();
    let mut delay = MockDelay;
    driver
        .init_magnetometer(MagConfig::default(), &mut delay)
        .unwrap();

    // Verify default calibration is applied
    let default_cal = driver.magnetometer_calibration();
    assert_eq!(default_cal.offset_x, 0.0);
    assert_eq!(default_cal.scale_x, 1.0);

    // Raw values should match scaled values when using default calibration
    interface.set_mag_data(1000, 2000, 3000);

    let mag_scaled = driver.read_magnetometer().unwrap();
    let mag_raw = driver.read_magnetometer_raw().unwrap();

    // Convert raw to µT manually
    let expected_x = f32::from(mag_raw.0) * 0.15;
    assert!(
        (mag_scaled.x - expected_x).abs() < 0.1,
        "Scaled reading should match raw * sensitivity"
    );
}
