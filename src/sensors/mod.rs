//! Sensor modules for the ICM-20948
//!
//! This module provides types, enums, and configuration structures for each sensor
//! in the ICM-20948:
//! - Accelerometer (3-axis)
//! - Gyroscope (3-axis)
//! - Magnetometer (3-axis, AK09916)
//!
//! All sensor operations are performed through methods on `Icm20948Driver`.

pub mod accelerometer;
pub mod gyroscope;
pub mod magnetometer;

// Re-export main types
pub use accelerometer::{AccelCalibration, AccelConfig, AccelDataG, AccelDlpf, AccelFullScale};
pub use gyroscope::{
    GyroCalibration, GyroConfig, GyroDataDps, GyroDataRps, GyroDlpf, GyroFullScale,
};
pub use magnetometer::{MagCalibration, MagConfig, MagDataUT, MagMode};
