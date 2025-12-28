//! Digital Motion Processor (DMP) Support
//!
//! This module provides support for the ICM-20948's Digital Motion Processor (DMP),
//! which can offload motion processing algorithms from the host processor.
//!
//! ## Important: Firmware Loading Required
//!
//! The DMP **does not have pre-programmed firmware**. The firmware must be loaded
//! by the host MCU on every power-up. This module includes the firmware binary
//! and provides functions to load it into the DMP's processor memory.
//!
//! ## Features
//!
//! When enabled, the DMP can provide:
//! - **6-axis quaternion**: Fusion of accelerometer and gyroscope
//! - **9-axis quaternion**: Fusion of accelerometer, gyroscope, and magnetometer
//! - **Calibrated sensor data**: Runtime-calibrated accelerometer and gyroscope
//! - **Game Rotation Vector**: 6-axis orientation without magnetometer drift
//! - **Geomagnetic Rotation Vector**: 9-axis orientation with heading accuracy
//! - **Activity recognition**: Step detection, tap detection (if configured)
//!
//! ## Usage Example
//!
//! ```ignore
//! # use icm20948::{Icm20948Driver, dmp::DmpConfig};
//! # let mut driver: Icm20948Driver<_> = todo!();
//! // Initialize and load DMP firmware (must be done after device init)
//! driver.dmp_init()?;
//!
//! // Configure which DMP features to enable
//! let config = DmpConfig::default()
//!     .with_quaternion_9axis(true)
//!     .with_calibrated_gyro(true)
//!     .with_sample_rate(100); // 100 Hz
//!
//! driver.dmp_configure(&config)?;
//!
//! // Enable DMP
//! driver.dmp_enable(true)?;
//!
//! // Read DMP data from FIFO
//! loop {
//!     if let Some(data) = driver.dmp_read_fifo()? {
//!         if let Some(quat) = data.quaternion_9axis {
//!             println!("Quaternion: w={}, x={}, y={}, z={}",
//!                      quat.w, quat.x, quat.y, quat.z);
//!         }
//!     }
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ## Power Consumption
//!
//! Using the DMP typically provides **lower power consumption** compared to
//! reading raw sensor data and performing sensor fusion on the host processor,
//! especially when using batch modes and interrupts.
//!
//! ## Licensing
//!
//! The DMP firmware is distributed under the MIT License. See the `firmware`
//! module for full licensing details and attribution.

pub mod config;
pub mod firmware;
pub mod loader;
pub mod parser;

// Re-export common items
pub use config::{
    ArbitrarySampleRate, ConfigSequence, DmpFeatures, DmpMemoryAddresses, DmpPacketHeader,
    DmpSampleRate, calculate_gyro_sf,
};
pub use firmware::{
    DMP_FIRMWARE, DMP_FIRMWARE_SIZE, DMP_MEM_BANK_SEL, DMP_MEM_R_W, DMP_MEM_START_ADDR,
    DMP_START_ADDRESS,
};
pub use loader::{DMP_LOAD_DELAY_US, DmpInitializer, FirmwareLoader};
pub use parser::DmpParser;

/// DMP configuration options
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpConfig {
    /// Enable 6-axis quaternion (accel + gyro)
    pub quaternion_6axis: bool,

    /// Enable 9-axis quaternion (accel + gyro + mag)
    pub quaternion_9axis: bool,

    /// Enable game rotation vector (6-axis without mag drift)
    pub game_rotation_vector: bool,

    /// Enable geomagnetic rotation vector (9-axis with heading accuracy)
    pub geomag_rotation_vector: bool,

    /// Enable calibrated accelerometer output
    pub calibrated_accel: bool,

    /// Enable calibrated gyroscope output
    pub calibrated_gyro: bool,

    /// Enable calibrated magnetometer output
    pub calibrated_mag: bool,

    /// Enable raw accelerometer output from DMP
    pub raw_accel: bool,

    /// Enable raw gyroscope output from DMP
    pub raw_gyro: bool,

    /// Enable raw magnetometer output from DMP
    pub raw_mag: bool,

    /// DMP sample rate (Hz)
    /// Valid range depends on sensor configuration, typically 1-225 Hz
    pub sample_rate: u16,
}

impl Default for DmpConfig {
    fn default() -> Self {
        Self {
            quaternion_6axis: false,
            quaternion_9axis: true, // Most common use case
            game_rotation_vector: false,
            geomag_rotation_vector: false,
            calibrated_accel: false,
            calibrated_gyro: false,
            calibrated_mag: false,
            raw_accel: false,
            raw_gyro: false,
            raw_mag: false,
            sample_rate: 100, // 100 Hz is a good default
        }
    }
}

impl DmpConfig {
    /// Create a new DMP configuration with all features disabled
    pub fn new() -> Self {
        Self {
            quaternion_6axis: false,
            quaternion_9axis: false,
            game_rotation_vector: false,
            geomag_rotation_vector: false,
            calibrated_accel: false,
            calibrated_gyro: false,
            calibrated_mag: false,
            raw_accel: false,
            raw_gyro: false,
            raw_mag: false,
            sample_rate: 100,
        }
    }

    /// Enable 6-axis quaternion output (accel + gyro)
    pub fn with_quaternion_6axis(mut self, enable: bool) -> Self {
        self.quaternion_6axis = enable;
        self
    }

    /// Enable 9-axis quaternion output (accel + gyro + mag)
    pub fn with_quaternion_9axis(mut self, enable: bool) -> Self {
        self.quaternion_9axis = enable;
        self
    }

    /// Enable game rotation vector (6-axis without magnetometer drift)
    pub fn with_game_rotation_vector(mut self, enable: bool) -> Self {
        self.game_rotation_vector = enable;
        self
    }

    /// Enable geomagnetic rotation vector (9-axis with heading accuracy)
    pub fn with_geomag_rotation_vector(mut self, enable: bool) -> Self {
        self.geomag_rotation_vector = enable;
        self
    }

    /// Enable calibrated gyroscope output
    pub fn with_calibrated_gyro(mut self, enable: bool) -> Self {
        self.calibrated_gyro = enable;
        self
    }

    /// Enable calibrated accelerometer output
    pub fn with_calibrated_accel(mut self, enable: bool) -> Self {
        self.calibrated_accel = enable;
        self
    }

    /// Enable calibrated magnetometer output
    pub fn with_calibrated_mag(mut self, enable: bool) -> Self {
        self.calibrated_mag = enable;
        self
    }

    /// Set DMP sample rate in Hz
    pub fn with_sample_rate(mut self, rate: u16) -> Self {
        self.sample_rate = rate;
        self
    }
}

/// Quaternion data from DMP
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Quaternion {
    /// W component (scalar part)
    pub w: f32,
    /// X component (i)
    pub x: f32,
    /// Y component (j)
    pub y: f32,
    /// Z component (k)
    pub z: f32,
}

impl Quaternion {
    /// Create a new quaternion
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Create identity quaternion (no rotation)
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Calculate quaternion magnitude (norm)
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Normalize the quaternion to unit length
    pub fn normalize(&mut self) {
        let mag = self.magnitude();
        if mag > 0.0 {
            self.w /= mag;
            self.x /= mag;
            self.y /= mag;
            self.z /= mag;
        }
    }

    /// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
    #[allow(clippy::similar_names)]
    pub fn to_euler_angles(&self) -> EulerAngles {
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = libm::atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if libm::fabsf(sinp) >= 1.0 {
            libm::copysignf(core::f32::consts::FRAC_PI_2, sinp) // Use 90 degrees if out of range
        } else {
            libm::asinf(sinp)
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = libm::atan2f(siny_cosp, cosy_cosp);

        EulerAngles { roll, pitch, yaw }
    }
}

/// Euler angles (roll, pitch, yaw) in radians
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EulerAngles {
    /// Roll angle (rotation around x-axis) in radians
    pub roll: f32,
    /// Pitch angle (rotation around y-axis) in radians
    pub pitch: f32,
    /// Yaw angle (rotation around z-axis) in radians
    pub yaw: f32,
}

impl EulerAngles {
    /// Convert radians to degrees
    pub fn to_degrees(&self) -> (f32, f32, f32) {
        (
            self.roll * 180.0 / core::f32::consts::PI,
            self.pitch * 180.0 / core::f32::consts::PI,
            self.yaw * 180.0 / core::f32::consts::PI,
        )
    }
}

/// DMP data read from FIFO
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpData {
    /// 6-axis quaternion (accel + gyro)
    pub quaternion_6axis: Option<Quaternion>,

    /// 9-axis quaternion (accel + gyro + mag)
    pub quaternion_9axis: Option<Quaternion>,

    /// Game rotation vector
    pub game_rotation_vector: Option<Quaternion>,

    /// Geomagnetic rotation vector
    pub geomag_rotation_vector: Option<Quaternion>,

    /// Heading accuracy (for 9-axis quaternion)
    pub heading_accuracy: Option<f32>,

    /// Calibrated accelerometer data
    pub calibrated_accel: Option<(i16, i16, i16)>,

    /// Calibrated gyroscope data
    pub calibrated_gyro: Option<(i32, i32, i32)>,

    /// Calibrated magnetometer data
    pub calibrated_mag: Option<(i32, i32, i32)>,

    /// Raw accelerometer data from DMP
    pub raw_accel: Option<(i16, i16, i16)>,

    /// Raw gyroscope data from DMP
    pub raw_gyro: Option<(i16, i16, i16)>,

    /// Raw magnetometer data from DMP
    pub raw_mag: Option<(i16, i16, i16)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dmp_config_default() {
        let config = DmpConfig::default();
        assert!(config.quaternion_9axis); // Should be enabled by default
        assert_eq!(config.sample_rate, 100);
    }

    #[test]
    fn test_dmp_config_builder() {
        let config = DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_calibrated_gyro(true)
            .with_sample_rate(200);

        assert!(config.quaternion_6axis);
        assert!(config.calibrated_gyro);
        assert_eq!(config.sample_rate, 200);
        assert!(!config.quaternion_9axis); // Should not be enabled
    }

    #[test]
    fn test_quaternion_identity() {
        let q = Quaternion::identity();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
    }

    #[test]

    fn test_quaternion_magnitude() {
        let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        assert!((q.magnitude() - 1.0).abs() < 0.001);

        let q2 = Quaternion::new(0.5, 0.5, 0.5, 0.5);
        assert!((q2.magnitude() - 1.0).abs() < 0.001);
    }
}
