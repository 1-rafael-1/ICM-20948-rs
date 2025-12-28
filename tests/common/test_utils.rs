//! Test utilities and helper functions

use crate::common::mock_interface::MockInterface;
use icm20948::Icm20948Driver;
use icm20948::sensors::{
    AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale,
};

/// Mock delay implementation for testing
///
/// This is a no-op delay that implements the embedded-hal DelayNs trait
/// for use in tests where actual delays are not needed.
#[derive(Debug, Clone, Copy)]
pub struct MockDelay;

impl embedded_hal::delay::DelayNs for MockDelay {
    fn delay_ns(&mut self, _ns: u32) {
        // No-op for testing
    }

    fn delay_us(&mut self, _us: u32) {
        // No-op for testing
    }

    fn delay_ms(&mut self, _ms: u32) {
        // No-op for testing
    }
}

#[cfg(feature = "async")]
impl embedded_hal_async::delay::DelayNs for MockDelay {
    async fn delay_ns(&mut self, _ns: u32) {
        // No-op for testing
    }

    async fn delay_us(&mut self, _us: u32) {
        // No-op for testing
    }

    async fn delay_ms(&mut self, _ms: u32) {
        // No-op for testing
    }
}

/// Create a mock driver for testing
/// Returns (driver, interface) where interface is a clone that shares state with the driver
pub fn create_mock_driver() -> (Icm20948Driver<MockInterface>, MockInterface) {
    let interface = MockInterface::new();
    let interface_clone = interface.clone();
    let driver = Icm20948Driver::new(interface).expect("Failed to create mock driver");
    (driver, interface_clone)
}

/// Assert that two floating point values are approximately equal
pub fn assert_float_eq(a: f32, b: f32, epsilon: f32) {
    let diff = (a - b).abs();
    assert!(
        diff < epsilon,
        "Values not equal within epsilon: {} vs {} (diff: {}, epsilon: {})",
        a,
        b,
        diff,
        epsilon
    );
}

/// Create a default accelerometer configuration for testing
pub fn default_accel_config() -> AccelConfig {
    AccelConfig {
        full_scale: AccelFullScale::G2,
        dlpf: AccelDlpf::Hz246,
        dlpf_enable: true,
        sample_rate_div: 10,
    }
}

/// Create a default gyroscope configuration for testing
pub fn default_gyro_config() -> GyroConfig {
    GyroConfig {
        full_scale: GyroFullScale::Dps250,
        dlpf: GyroDlpf::Hz197,
        dlpf_enable: true,
        sample_rate_div: 10,
    }
}
