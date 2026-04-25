//! DMP guardrails and FIFO overflow behavior tests (blocking driver)

use crate::common::test_utils::create_mock_driver;
use icm20948::{Bank, Error};

#[cfg(feature = "dmp")]
use icm20948::dmp::DmpConfig;

#[test]
fn test_fifo_read_overflow_returns_error() {
    let (mut driver, interface) = create_mock_driver();

    // Simulate FIFO overflow in INT_STATUS_2 (Bank 0, 0x1B)
    interface.set_register(Bank::Bank0, 0x1B, 0x01);

    let mut buffer = [0u8; 8];
    let result = driver.fifo_read(&mut buffer);

    assert!(matches!(result, Err(Error::FifoOverflow)));
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_read_fifo_overflow_returns_error() {
    let (mut driver, interface) = create_mock_driver();

    // Simulate FIFO overflow in INT_STATUS_2 (Bank 0, 0x1B)
    interface.set_register(Bank::Bank0, 0x1B, 0x01);

    let result = driver.dmp_read_fifo();
    assert!(matches!(result, Err(Error::FifoOverflow)));
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_configure_requires_firmware() {
    let (mut driver, _interface) = create_mock_driver();

    let config = DmpConfig::new()
        .with_quaternion_6axis(true)
        .with_sample_rate(100);

    let result = driver.dmp_configure(&config);
    assert!(matches!(result, Err(Error::DmpFirmwareNotLoaded)));
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_configure_requires_mag_init_for_mag_outputs() {
    let (mut driver, _interface) = create_mock_driver();

    // Load firmware first
    driver.dmp_load_firmware().expect("Firmware load failed");

    // 9-axis requires magnetometer init
    let config = DmpConfig::new()
        .with_quaternion_9axis(true)
        .with_sample_rate(100);

    let result = driver.dmp_configure(&config);
    assert!(matches!(result, Err(Error::MagnetometerNotInitialized)));
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_enable_requires_firmware_and_config() {
    let (mut driver, _interface) = create_mock_driver();

    // No firmware loaded
    let result = driver.dmp_enable(true);
    assert!(matches!(result, Err(Error::DmpFirmwareNotLoaded)));

    // Load firmware
    driver.dmp_load_firmware().expect("Firmware load failed");

    // Not configured yet
    let result = driver.dmp_enable(true);
    assert!(matches!(result, Err(Error::DmpNotConfigured)));

    // Configure (6-axis, no magnetometer needed)
    let config = DmpConfig::new()
        .with_quaternion_6axis(true)
        .with_sample_rate(100);

    driver.dmp_configure(&config).expect("DMP configure failed");

    // Now enable should succeed
    assert!(driver.dmp_enable(true).is_ok());
    assert!(driver.dmp_enable(false).is_ok());
}
