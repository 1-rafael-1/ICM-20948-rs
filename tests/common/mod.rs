//! Common test utilities and mock implementations

pub mod mock_interface;
pub mod test_utils;

pub use mock_interface::Operation;
pub use test_utils::{create_mock_driver, default_accel_config, default_gyro_config};
