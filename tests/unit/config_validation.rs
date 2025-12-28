//! Unit tests for configuration validation

use crate::common::{create_mock_driver, default_accel_config, default_gyro_config};
use icm20948::sensors::{AccelDlpf, AccelFullScale, GyroDlpf, GyroFullScale};

#[test]
fn test_valid_accel_config() {
    let (mut driver, _interface) = create_mock_driver();

    // Test all valid accelerometer full-scale ranges
    let scales = [
        AccelFullScale::G2,
        AccelFullScale::G4,
        AccelFullScale::G8,
        AccelFullScale::G16,
    ];

    for scale in &scales {
        let mut config = default_accel_config();
        config.full_scale = *scale;

        let result = driver.configure_accelerometer(config);
        assert!(
            result.is_ok(),
            "Valid accel scale {:?} should be accepted",
            scale
        );
    }
}

#[test]
fn test_valid_gyro_config() {
    let (mut driver, _interface) = create_mock_driver();

    // Test all valid gyroscope full-scale ranges
    let scales = [
        GyroFullScale::Dps250,
        GyroFullScale::Dps500,
        GyroFullScale::Dps1000,
        GyroFullScale::Dps2000,
    ];

    for scale in &scales {
        let mut config = default_gyro_config();
        config.full_scale = *scale;

        let result = driver.configure_gyroscope(config);
        assert!(
            result.is_ok(),
            "Valid gyro scale {:?} should be accepted",
            scale
        );
    }
}

#[test]
fn test_accel_dlpf_settings() {
    let (mut driver, _interface) = create_mock_driver();

    // Test all DLPF configurations
    let dlpf_configs = [
        AccelDlpf::Hz246,
        AccelDlpf::Hz111,
        AccelDlpf::Hz50,
        AccelDlpf::Hz24,
        AccelDlpf::Hz12,
        AccelDlpf::Hz6,
        AccelDlpf::Hz473,
    ];

    for dlpf in &dlpf_configs {
        let mut config = default_accel_config();
        config.dlpf = *dlpf;
        config.dlpf_enable = true;

        let result = driver.configure_accelerometer(config);
        assert!(
            result.is_ok(),
            "Valid accel DLPF {:?} should be accepted",
            dlpf
        );
    }
}

#[test]
fn test_gyro_dlpf_settings() {
    let (mut driver, _interface) = create_mock_driver();

    // Test all DLPF configurations
    let dlpf_configs = [
        GyroDlpf::Disabled,
        GyroDlpf::Hz197,
        GyroDlpf::Hz152,
        GyroDlpf::Hz120,
        GyroDlpf::Hz51,
        GyroDlpf::Hz24,
        GyroDlpf::Hz12,
        GyroDlpf::Hz6,
    ];

    for dlpf in &dlpf_configs {
        let mut config = default_gyro_config();
        config.dlpf = *dlpf;
        config.dlpf_enable = true;

        let result = driver.configure_gyroscope(config);
        assert!(
            result.is_ok(),
            "Valid gyro DLPF {:?} should be accepted",
            dlpf
        );
    }
}

#[test]
fn test_dlpf_enable_disable() {
    let (mut driver, _interface) = create_mock_driver();

    // Test with DLPF enabled
    let mut config = default_accel_config();
    config.dlpf_enable = true;
    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "DLPF enabled should work");

    // Test with DLPF disabled
    config.dlpf_enable = false;
    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "DLPF disabled should work");
}

#[test]
fn test_sample_rate_divider_range() {
    let (mut driver, _interface) = create_mock_driver();

    // Test various sample rate dividers
    let dividers = [0, 1, 10, 50, 100, 200, 255];

    for &div in &dividers {
        let mut config = default_accel_config();
        config.sample_rate_div = div;

        let result = driver.configure_accelerometer(config);
        assert!(
            result.is_ok(),
            "Sample rate divider {} should be valid",
            div
        );
    }
}

#[test]
fn test_gyro_sample_rate_divider() {
    let (mut driver, _interface) = create_mock_driver();

    // Test various sample rate dividers
    let dividers = [0, 1, 10, 50, 100, 200, 255];

    for &div in &dividers {
        let mut config = default_gyro_config();
        config.sample_rate_div = div;

        let result = driver.configure_gyroscope(config);
        assert!(
            result.is_ok(),
            "Gyro sample rate divider {} should be valid",
            div
        );
    }
}

#[test]
fn test_reconfiguration() {
    let (mut driver, _interface) = create_mock_driver();

    // Initial configuration
    let config1 = default_accel_config();
    driver.configure_accelerometer(config1).unwrap();

    // Reconfigure with different settings
    let mut config2 = default_accel_config();
    config2.full_scale = AccelFullScale::G16;
    config2.sample_rate_div = 50;

    let result = driver.configure_accelerometer(config2);
    assert!(result.is_ok(), "Reconfiguration should succeed");
}

#[test]
fn test_multiple_reconfigurations() {
    let (mut driver, _interface) = create_mock_driver();

    // Configure multiple times
    for i in 0..5 {
        let mut config = default_accel_config();
        config.sample_rate_div = i * 10;

        let result = driver.configure_accelerometer(config);
        assert!(result.is_ok(), "Multiple reconfigurations should succeed");
    }
}

#[test]
fn test_accel_and_gyro_independent_config() {
    let (mut driver, _interface) = create_mock_driver();

    // Configure accelerometer
    let accel_config = default_accel_config();
    driver.configure_accelerometer(accel_config).unwrap();

    // Configure gyroscope independently
    let gyro_config = default_gyro_config();
    let result = driver.configure_gyroscope(gyro_config);

    assert!(result.is_ok(), "Gyro config should not affect accel config");

    // Reconfigure accelerometer
    let result = driver.configure_accelerometer(accel_config);
    assert!(
        result.is_ok(),
        "Accel reconfig should not affect gyro config"
    );
}

#[test]
fn test_configuration_before_init() {
    let (mut driver, _interface) = create_mock_driver();

    // The driver is already initialized by create_mock_driver
    // But we can test configuration works after init
    let config = default_accel_config();
    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "Configuration after init should succeed");
}

#[test]
fn test_default_configurations() {
    let (mut driver, _interface) = create_mock_driver();

    // Use default configurations
    let accel_config = default_accel_config();
    let gyro_config = default_gyro_config();

    let result1 = driver.configure_accelerometer(accel_config);
    let result2 = driver.configure_gyroscope(gyro_config);

    assert!(result1.is_ok(), "Default accel config should be valid");
    assert!(result2.is_ok(), "Default gyro config should be valid");
}

#[test]
fn test_extreme_sample_rate_dividers() {
    let (mut driver, _interface) = create_mock_driver();

    // Test minimum divider (0)
    let mut config = default_accel_config();
    config.sample_rate_div = 0;
    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "Minimum sample rate divider should work");

    // Test maximum divider (255)
    config.sample_rate_div = 255;
    let result = driver.configure_accelerometer(config);
    assert!(result.is_ok(), "Maximum sample rate divider should work");
}

#[test]
fn test_configuration_combinations() {
    let (mut driver, _interface) = create_mock_driver();

    // Test various combinations of settings
    let combinations = [
        (AccelFullScale::G2, AccelDlpf::Hz246, true, 0),
        (AccelFullScale::G4, AccelDlpf::Hz111, true, 10),
        (AccelFullScale::G8, AccelDlpf::Hz50, false, 50),
        (AccelFullScale::G16, AccelDlpf::Hz24, true, 100),
    ];

    for (scale, dlpf, dlpf_enable, sample_rate_div) in &combinations {
        let config = icm20948::sensors::AccelConfig {
            full_scale: *scale,
            dlpf: *dlpf,
            dlpf_enable: *dlpf_enable,
            sample_rate_div: *sample_rate_div,
        };

        let result = driver.configure_accelerometer(config);
        assert!(
            result.is_ok(),
            "Valid configuration combination should work"
        );
    }
}

#[test]
fn test_gyro_configuration_combinations() {
    let (mut driver, _interface) = create_mock_driver();

    // Test various combinations of settings
    let combinations = [
        (GyroFullScale::Dps250, GyroDlpf::Hz197, true, 0),
        (GyroFullScale::Dps500, GyroDlpf::Hz152, true, 10),
        (GyroFullScale::Dps1000, GyroDlpf::Hz51, false, 50),
        (GyroFullScale::Dps2000, GyroDlpf::Hz24, true, 100),
    ];

    for (scale, dlpf, dlpf_enable, sample_rate_div) in &combinations {
        let config = icm20948::sensors::GyroConfig {
            full_scale: *scale,
            dlpf: *dlpf,
            dlpf_enable: *dlpf_enable,
            sample_rate_div: *sample_rate_div,
        };

        let result = driver.configure_gyroscope(config);
        assert!(
            result.is_ok(),
            "Valid gyro configuration combination should work"
        );
    }
}

#[test]
fn test_configuration_persistence() {
    let (mut driver, interface) = create_mock_driver();

    // Configure with specific settings
    let mut config = default_accel_config();
    config.full_scale = AccelFullScale::G16;
    driver.configure_accelerometer(config).unwrap();

    // Read data to ensure configuration is active
    interface.set_accel_data(2048, 0, 2048);
    let data = driver.read_accelerometer().unwrap();

    // At ±16g scale, sensitivity is 2048 LSB/g
    // So 2048 raw should give 1g
    assert!(
        (data.x - 1.0).abs() < 0.1,
        "Configuration should persist and be applied"
    );
}

#[test]
fn test_all_scales_produce_different_sensitivities() {
    let (mut driver, interface) = create_mock_driver();

    let scales = [
        AccelFullScale::G2,
        AccelFullScale::G4,
        AccelFullScale::G8,
        AccelFullScale::G16,
    ];

    let mut results = Vec::new();

    for scale in &scales {
        let mut config = default_accel_config();
        config.full_scale = *scale;
        driver.configure_accelerometer(config).unwrap();

        // Use same raw value for all scales
        interface.set_accel_data(16384, 0, 0);
        let data = driver.read_accelerometer().unwrap();
        results.push(data.x);
    }

    // Each scale should produce a different physical value
    for i in 0..results.len() - 1 {
        assert!(
            (results[i] - results[i + 1]).abs() > 0.1,
            "Different scales should produce different physical values"
        );
    }
}

#[test]
fn test_gyro_scales_produce_different_sensitivities() {
    let (mut driver, interface) = create_mock_driver();

    let scales = [
        GyroFullScale::Dps250,
        GyroFullScale::Dps500,
        GyroFullScale::Dps1000,
        GyroFullScale::Dps2000,
    ];

    let mut results = Vec::new();

    for scale in &scales {
        let mut config = default_gyro_config();
        config.full_scale = *scale;
        driver.configure_gyroscope(config).unwrap();

        // Use same raw value for all scales
        interface.set_gyro_data(1000, 0, 0);
        let data = driver.read_gyroscope().unwrap();
        results.push(data.x);
    }

    // Each scale should produce a different physical value
    for i in 0..results.len() - 1 {
        assert!(
            (results[i] - results[i + 1]).abs() > 0.1,
            "Different gyro scales should produce different physical values"
        );
    }
}

#[test]
fn test_configuration_after_error() {
    let (mut driver, interface) = create_mock_driver();

    // Inject an error
    interface.fail_next_write();

    // Try to configure (will fail)
    let config = default_accel_config();
    let result = driver.configure_accelerometer(config);
    assert!(
        result.is_err(),
        "Configuration should fail with injected error"
    );

    // Try again (should succeed)
    let result = driver.configure_accelerometer(config);
    assert!(
        result.is_ok(),
        "Configuration should succeed after error recovery"
    );
}
