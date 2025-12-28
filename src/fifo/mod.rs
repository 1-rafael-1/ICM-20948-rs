//! FIFO (First In First Out) buffer management
//!
//! The ICM-20948 has a 512-byte FIFO buffer that can store sensor data for batch processing.
//! This is useful for:
//! - Reducing interrupt frequency
//! - Enabling low-power operation
//! - Buffering data during host MCU sleep
//! - Synchronizing multi-sensor data
//!
//! # Example
//!
//! ```ignore
//! # use icm20948::{Icm20948Driver, fifo::{FifoConfig, FifoMode}};
//! # let mut imu: Icm20948Driver<_> = todo!();
//! // Configure FIFO to store accelerometer and gyroscope data
//! let config = FifoConfig {
//!     enable_accel: true,
//!     enable_gyro: true,
//!     enable_temp: false,
//!     enable_mag: false,
//!     mode: FifoMode::Stream,
//! };
//!
//! imu.fifo_configure(&config)?;
//! imu.fifo_enable(true)?;
//!
//! // Later, read data in batches
//! let count = imu.fifo_count()?;
//! if count >= 12 {  // One complete accel + gyro sample
//!     let mut buffer = [0u8; 512];
//!     let bytes_read = imu.fifo_read(&mut buffer[..count as usize])?;
//!
//!     // Parse the data
//!     let records = imu.fifo_parse(&buffer[..bytes_read])?;
//! }
//! # Ok::<(), icm20948::Error<()>>(())
//! ```

pub mod parser;

use crate::Error;

/// FIFO size in bytes
pub const FIFO_SIZE: u16 = 512;

/// FIFO configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct FifoConfig {
    /// Enable accelerometer data in FIFO
    pub enable_accel: bool,
    /// Enable gyroscope data in FIFO (per-axis control available via advanced config)
    pub enable_gyro: bool,
    /// Enable temperature data in FIFO
    pub enable_temp: bool,
    /// Enable magnetometer data in FIFO (via I2C slave)
    pub enable_mag: bool,
    /// FIFO operating mode
    pub mode: FifoMode,
}

impl Default for FifoConfig {
    fn default() -> Self {
        Self {
            enable_accel: false,
            enable_gyro: false,
            enable_temp: false,
            enable_mag: false,
            mode: FifoMode::Stream,
        }
    }
}

/// Advanced FIFO configuration with per-axis control
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct FifoConfigAdvanced {
    /// Enable accelerometer data in FIFO
    pub enable_accel: bool,
    /// Enable gyroscope X-axis data in FIFO
    pub enable_gyro_x: bool,
    /// Enable gyroscope Y-axis data in FIFO
    pub enable_gyro_y: bool,
    /// Enable gyroscope Z-axis data in FIFO
    pub enable_gyro_z: bool,
    /// Enable temperature data in FIFO
    pub enable_temp: bool,
    /// Enable I2C slave 0 data in FIFO (typically magnetometer)
    pub enable_slv0: bool,
    /// Enable I2C slave 1 data in FIFO
    pub enable_slv1: bool,
    /// Enable I2C slave 2 data in FIFO
    pub enable_slv2: bool,
    /// Enable I2C slave 3 data in FIFO
    pub enable_slv3: bool,
    /// FIFO operating mode
    pub mode: FifoMode,
}

impl Default for FifoConfigAdvanced {
    fn default() -> Self {
        Self {
            enable_accel: false,
            enable_gyro_x: false,
            enable_gyro_y: false,
            enable_gyro_z: false,
            enable_temp: false,
            enable_slv0: false,
            enable_slv1: false,
            enable_slv2: false,
            enable_slv3: false,
            mode: FifoMode::Stream,
        }
    }
}

impl From<FifoConfig> for FifoConfigAdvanced {
    fn from(config: FifoConfig) -> Self {
        Self {
            enable_accel: config.enable_accel,
            enable_gyro_x: config.enable_gyro,
            enable_gyro_y: config.enable_gyro,
            enable_gyro_z: config.enable_gyro,
            enable_temp: config.enable_temp,
            enable_slv0: config.enable_mag,
            enable_slv1: false,
            enable_slv2: false,
            enable_slv3: false,
            mode: config.mode,
        }
    }
}

/// FIFO operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// Stream mode - oldest data is dropped when FIFO is full
    Stream = 0,
    /// Snapshot mode - FIFO stops accepting new data when full
    Snapshot = 1,
}

/// Watermark configuration for FIFO interrupt
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoWatermark {
    /// Watermark threshold in bytes (0-511)
    /// Interrupt fires when FIFO count reaches this value
    pub threshold: u16,
}

impl FifoWatermark {
    /// Create a new watermark configuration
    ///
    /// # Arguments
    /// * `threshold` - Number of bytes that trigger watermark interrupt (0-511)
    ///
    /// # Errors
    /// Returns `InvalidConfig` if threshold exceeds FIFO size
    pub const fn new(threshold: u16) -> Result<Self, Error<()>> {
        if threshold >= FIFO_SIZE {
            return Err(Error::InvalidConfig);
        }
        Ok(Self { threshold })
    }

    /// Create a watermark for a specific number of complete samples
    ///
    /// # Arguments
    /// * `samples` - Number of complete sensor samples
    /// * `bytes_per_sample` - Bytes per sample (e.g., 12 for accel+gyro)
    ///
    /// # Errors
    /// Returns `InvalidConfig` if the calculated threshold exceeds FIFO size
    pub const fn from_samples(samples: u16, bytes_per_sample: u16) -> Result<Self, Error<()>> {
        let threshold = samples.saturating_mul(bytes_per_sample);
        Self::new(threshold)
    }
}

/// FIFO overflow status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct FifoOverflowStatus {
    /// FIFO 0 overflow
    pub fifo0: bool,
    /// FIFO 1 overflow
    pub fifo1: bool,
    /// FIFO 2 overflow
    pub fifo2: bool,
    /// FIFO 3 overflow
    pub fifo3: bool,
    /// FIFO 4 overflow
    pub fifo4: bool,
}

impl FifoOverflowStatus {
    /// Check if any FIFO has overflowed
    pub const fn any_overflow(&self) -> bool {
        self.fifo0 || self.fifo1 || self.fifo2 || self.fifo3 || self.fifo4
    }
}

/// FIFO data record containing one or more sensor readings
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoRecord {
    /// Accelerometer data (if enabled)
    pub accel: Option<(i16, i16, i16)>,
    /// Gyroscope data (if enabled)
    pub gyro: Option<(i16, i16, i16)>,
    /// Temperature data (if enabled)
    pub temp: Option<i16>,
    /// Magnetometer data (if enabled via I2C slave)
    pub mag: Option<(i16, i16, i16)>,
}

impl FifoRecord {
    /// Create an empty FIFO record
    pub const fn new() -> Self {
        Self {
            accel: None,
            gyro: None,
            temp: None,
            mag: None,
        }
    }

    /// Check if the record is empty (no data)
    pub const fn is_empty(&self) -> bool {
        self.accel.is_none() && self.gyro.is_none() && self.temp.is_none() && self.mag.is_none()
    }

    /// Get the size in bytes this record would occupy in the FIFO
    pub const fn size_bytes(&self) -> usize {
        let mut size = 0;
        if self.accel.is_some() {
            size += 6; // 3 axes * 2 bytes
        }
        if self.gyro.is_some() {
            size += 6; // 3 axes * 2 bytes
        }
        if self.temp.is_some() {
            size += 2;
        }
        if self.mag.is_some() {
            size += 6; // 3 axes * 2 bytes
        }
        size
    }
}

impl Default for FifoRecord {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculate expected bytes per FIFO record based on configuration
pub const fn calculate_record_size(config: &FifoConfigAdvanced) -> usize {
    let mut size = 0;

    if config.enable_accel {
        size += 6; // 3 axes * 2 bytes
    }

    // Gyro axes are stored as a group if any are enabled
    if config.enable_gyro_x || config.enable_gyro_y || config.enable_gyro_z {
        size += 6; // 3 axes * 2 bytes (all axes are always present)
    }

    if config.enable_temp {
        size += 2;
    }

    // Each slave can contribute variable bytes, but typically 6 for mag
    if config.enable_slv0 {
        size += 6; // Typical magnetometer size
    }
    if config.enable_slv1 {
        size += 6;
    }
    if config.enable_slv2 {
        size += 6;
    }
    if config.enable_slv3 {
        size += 6;
    }

    size
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn test_fifo_config_default() {
        let config = FifoConfig::default();
        assert!(!config.enable_accel);
        assert!(!config.enable_gyro);
        assert!(!config.enable_temp);
        assert!(!config.enable_mag);
        assert_eq!(config.mode, FifoMode::Stream);
    }

    #[test]
    fn test_fifo_watermark_valid() {
        let wm = FifoWatermark::new(256);
        assert!(wm.is_ok());
        assert_eq!(wm.unwrap().threshold, 256);
    }

    #[test]
    fn test_fifo_watermark_invalid() {
        let wm = FifoWatermark::new(512);
        assert!(wm.is_err());

        let wm = FifoWatermark::new(1000);
        assert!(wm.is_err());
    }

    #[test]
    fn test_fifo_watermark_from_samples() {
        // Accel (6 bytes) + Gyro (6 bytes) = 12 bytes per sample
        let wm = FifoWatermark::from_samples(10, 12);
        assert!(wm.is_ok());
        assert_eq!(wm.unwrap().threshold, 120);
    }

    #[test]
    fn test_fifo_record_size() {
        let mut record = FifoRecord::new();
        assert_eq!(record.size_bytes(), 0);

        record.accel = Some((0, 0, 0));
        assert_eq!(record.size_bytes(), 6);

        record.gyro = Some((0, 0, 0));
        assert_eq!(record.size_bytes(), 12);

        record.temp = Some(0);
        assert_eq!(record.size_bytes(), 14);

        record.mag = Some((0, 0, 0));
        assert_eq!(record.size_bytes(), 20);
    }

    #[test]
    fn test_calculate_record_size() {
        let mut config = FifoConfigAdvanced::default();
        assert_eq!(calculate_record_size(&config), 0);

        config.enable_accel = true;
        assert_eq!(calculate_record_size(&config), 6);

        config.enable_gyro_x = true;
        assert_eq!(calculate_record_size(&config), 12);

        config.enable_temp = true;
        assert_eq!(calculate_record_size(&config), 14);

        config.enable_slv0 = true;
        assert_eq!(calculate_record_size(&config), 20);
    }

    #[test]
    fn test_overflow_status() {
        let mut status = FifoOverflowStatus {
            fifo0: false,
            fifo1: false,
            fifo2: false,
            fifo3: false,
            fifo4: false,
        };
        assert!(!status.any_overflow());

        status.fifo2 = true;
        assert!(status.any_overflow());
    }

    #[test]
    fn test_fifo_config_conversion() {
        let basic = FifoConfig {
            enable_accel: true,
            enable_gyro: true,
            enable_temp: false,
            enable_mag: true,
            mode: FifoMode::Snapshot,
        };

        let advanced: FifoConfigAdvanced = basic.into();
        assert!(advanced.enable_accel);
        assert!(advanced.enable_gyro_x);
        assert!(advanced.enable_gyro_y);
        assert!(advanced.enable_gyro_z);
        assert!(!advanced.enable_temp);
        assert!(advanced.enable_slv0);
        assert_eq!(advanced.mode, FifoMode::Snapshot);
    }
}
