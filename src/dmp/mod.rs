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
//! **Note:** Activity recognition, pickup/tilt, and significant motion flags are exposed but not implemented yet.
//! Missing pieces: DMP memory configuration writes and packet parsing for event payloads. PRs welcome.
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
//! let config = DmpConfig::nine_axis()
//!     .with_calibrated_gyro()
//!     .with_sample_rate(100);
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
    DmpPacketHeader2, DmpSampleRate, calculate_gyro_sf,
};
pub use firmware::{
    DMP_FIRMWARE, DMP_MEM_BANK_SEL, DMP_MEM_R_W, DMP_MEM_START_ADDR, DMP_START_ADDRESS,
};
pub use loader::{DMP_LOAD_DELAY_US, DmpInitializer, FirmwareLoader};
pub use parser::DmpParser;

/// The quaternion fusion algorithm the DMP should run.
///
/// These are mutually exclusive: the DMP can only run one fusion algorithm at a time.
/// The choice is made by calling the corresponding named constructor on [`DmpConfig`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmpFusionMode {
    /// No quaternion fusion output. Use this when you only need raw or calibrated sensor data.
    #[default]
    None,
    /// 6-axis fusion (accelerometer + gyroscope). Also known as Game Rotation Vector.
    SixAxis,
    /// 9-axis fusion (accelerometer + gyroscope + magnetometer).
    /// Requires magnetometer initialisation via `dmp_init_magnetometer()` before calling
    /// `dmp_configure()`.
    NineAxis,
    /// Geomagnetic rotation vector (9-axis with heading accuracy estimate).
    /// Requires magnetometer initialisation via `dmp_init_magnetometer()` before calling
    /// `dmp_configure()`.
    GeomagRotationVector,
    /// Pedometer-fused 6-axis quaternion (`PQuat6`).
    PedometerSixAxis,
}

/// DMP configuration options
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
#[must_use]
pub struct DmpConfig {
    /// Which quaternion fusion algorithm the DMP should run.
    pub fusion_mode: DmpFusionMode,

    /// Enable host-calibrated accelerometer output
    pub host_calibrated_accel: bool,

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

    /// Enable pedometer step detector (triggers an event on step)
    pub step_detector: bool,

    /// Enable pedometer step counter (tracks total steps)
    pub step_counter: bool,

    // /// Enable significant motion detection (not implemented yet)
    // pub significant_motion: bool,

    // /// Enable tilt detector (not implemented yet)
    // pub tilt_detector: bool,

    // /// Enable pickup/flip detector (not implemented yet)
    // pub pickup_detector: bool,

    // /// Enable activity classification (not implemented yet)
    // pub activity_classification: bool,
    /// DMP sample rate (Hz)
    /// Valid range depends on sensor configuration, typically 1-225 Hz
    pub sample_rate: u16,
}

impl Default for DmpConfig {
    fn default() -> Self {
        // 9-axis quaternion at 100 Hz is the most common use case
        Self::nine_axis()
    }
}

impl DmpConfig {
    /// Create a `DmpConfig` with no fusion and all outputs disabled.
    pub const fn new() -> Self {
        Self {
            fusion_mode: DmpFusionMode::None,
            host_calibrated_accel: false,
            calibrated_gyro: false,
            calibrated_mag: false,
            raw_accel: false,
            raw_gyro: false,
            raw_mag: false,
            step_detector: false,
            step_counter: false,
            sample_rate: 100,
        }
    }

    /// Start with 9-axis quaternion fusion (accelerometer + gyroscope + magnetometer).
    ///
    /// Requires magnetometer initialisation via `dmp_init_magnetometer()` before calling
    /// `dmp_configure()`.
    pub const fn nine_axis() -> Self {
        Self {
            fusion_mode: DmpFusionMode::NineAxis,
            ..Self::new()
        }
    }

    /// Start with 6-axis quaternion fusion (accelerometer + gyroscope).
    ///
    /// Also known as Game Rotation Vector. No magnetometer required.
    pub const fn six_axis() -> Self {
        Self {
            fusion_mode: DmpFusionMode::SixAxis,
            ..Self::new()
        }
    }

    /// Start with geomagnetic rotation vector fusion (9-axis with heading accuracy).
    ///
    /// Requires magnetometer initialisation via `dmp_init_magnetometer()` before calling
    /// `dmp_configure()`.
    pub const fn geomag() -> Self {
        Self {
            fusion_mode: DmpFusionMode::GeomagRotationVector,
            ..Self::new()
        }
    }

    /// Start with pedometer-fused 6-axis quaternion (`PQuat6`).
    pub const fn pedometer_six_axis() -> Self {
        Self {
            fusion_mode: DmpFusionMode::PedometerSixAxis,
            ..Self::new()
        }
    }

    /// Enable host-calibrated accelerometer output.
    pub const fn with_host_calibrated_accel(mut self) -> Self {
        self.host_calibrated_accel = true;
        self
    }

    /// Enable calibrated gyroscope output.
    pub const fn with_calibrated_gyro(mut self) -> Self {
        self.calibrated_gyro = true;
        self
    }

    /// Enable calibrated magnetometer output.
    pub const fn with_calibrated_mag(mut self) -> Self {
        self.calibrated_mag = true;
        self
    }

    /// Enable raw accelerometer output from DMP.
    pub const fn with_raw_accel(mut self) -> Self {
        self.raw_accel = true;
        self
    }

    /// Enable raw gyroscope output from DMP.
    pub const fn with_raw_gyro(mut self) -> Self {
        self.raw_gyro = true;
        self
    }

    /// Enable raw magnetometer output from DMP.
    pub const fn with_raw_mag(mut self) -> Self {
        self.raw_mag = true;
        self
    }

    /// Enable pedometer step detector.
    pub const fn with_step_detector(mut self) -> Self {
        self.step_detector = true;
        self
    }

    /// Enable pedometer step counter.
    pub const fn with_step_counter(mut self) -> Self {
        self.step_counter = true;
        self
    }

    /// Set the DMP output sample rate in Hz.
    ///
    /// Valid range is 1–225 Hz. Values of 0 are clamped to 1; values above 225 are
    /// clamped to 225. This ensures that both the sample-rate divider and the
    /// calibration-parameter lookup always see the same effective rate.
    pub const fn with_sample_rate(mut self, rate: u16) -> Self {
        self.sample_rate = if rate == 0 {
            1
        } else if rate > 225 {
            225
        } else {
            rate
        };
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
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Create identity quaternion (no rotation)
    pub const fn identity() -> Self {
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
    #[allow(clippy::missing_const_for_fn)]
    pub fn to_degrees(&self) -> (f32, f32, f32) {
        (
            self.roll.to_degrees(),
            self.pitch.to_degrees(),
            self.yaw.to_degrees(),
        )
    }
}

/// DMP data read from FIFO
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpData {
    /// 6-axis quaternion (accel + gyro)/ Game rotation vector / GRAVITY / `LINEAR_ACCEL`
    pub quaternion_6axis: Option<Quaternion>,

    /// `PQuat6` (Pedometer Quaternion)
    pub pedometer_quaternion: Option<Quaternion>,

    /// 9-axis quaternion (accel + gyro + mag)
    pub quaternion_9axis: Option<Quaternion>,

    /// Geomagnetic rotation vector
    pub geomag_rotation_vector: Option<Quaternion>,

    /// Heading accuracy (for 9-axis quaternion)
    pub heading_accuracy: Option<f32>,

    /// Host-calibrated accelerometer data
    pub host_calibrated_accel: Option<(i16, i16, i16)>,

    /// Calibrated gyroscope data (raw - bias); i32 because i16 - i16 can overflow
    pub calibrated_gyro: Option<(i32, i32, i32)>,

    /// Calibrated gyroscope data from DMP (32-bit values)
    pub dmp_calibrated_gyro: Option<(i32, i32, i32)>,

    /// Calibrated magnetometer data
    pub calibrated_mag: Option<(i32, i32, i32)>,

    /// Raw accelerometer data from DMP
    pub raw_accel: Option<(i16, i16, i16)>,

    /// Raw gyroscope data from DMP
    pub raw_gyro: Option<(i16, i16, i16)>,

    /// Raw magnetometer data from DMP
    pub raw_mag: Option<(i16, i16, i16)>,

    /// Gyroscope bias data from DMP (dynamically calculated zero-offset)
    /// Only available when `raw_gyro` output is enabled.
    pub gyro_bias: Option<(i16, i16, i16)>,

    /// Accelerometer accuracy status from DMP (0=unreliable, 3=high accuracy)
    pub accel_accuracy: Option<u16>,

    /// Gyroscope accuracy status from DMP (0=unreliable, 3=high accuracy)
    pub gyro_accuracy: Option<u16>,

    /// Compass/Magnetometer accuracy status from DMP (0=unreliable, 3=high accuracy)
    pub compass_accuracy: Option<u16>,

    /// Pedometer step detector timestamp/count
    pub pedometer_timestamp: Option<u32>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dmp_config_default() {
        let config = DmpConfig::default();
        assert_eq!(config.fusion_mode, DmpFusionMode::NineAxis);
        assert_eq!(config.sample_rate, 100);
    }

    #[test]
    fn test_dmp_config_builder() {
        let config = DmpConfig::six_axis()
            .with_host_calibrated_accel()
            .with_calibrated_gyro()
            .with_sample_rate(200);

        assert_eq!(config.fusion_mode, DmpFusionMode::SixAxis);
        assert!(config.host_calibrated_accel);
        assert!(config.calibrated_gyro);
        assert_eq!(config.sample_rate, 200);
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
