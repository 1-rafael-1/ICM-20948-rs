//! Interrupt configuration and management
//!
//! The ICM-20948 has a single interrupt pin (INT1) that can be configured to trigger on various events:
//! - Raw data ready
//! - FIFO watermark reached
//! - FIFO overflow
//! - Wake-on-motion (`WoM`)
//! - DMP interrupt
//! - I2C master operations
//!
//! # Example
//!
//! ```ignore
//! # use icm20948::{Icm20948Driver, interrupt::{InterruptConfig, InterruptPinConfig}};
//! # let mut imu: Icm20948Driver<_> = todo!();
//! // Configure interrupt pin as active-low, open-drain
//! let pin_config = InterruptPinConfig {
//!     active_low: true,
//!     open_drain: true,
//!     latch_enabled: true,
//!     clear_on_any_read: false,
//! };
//! imu.configure_interrupt_pin(&pin_config)?;
//!
//! // Enable data ready interrupt
//! let int_config = InterruptConfig {
//!     raw_data_ready: true,
//!     ..Default::default()
//! };
//! imu.configure_interrupts(&int_config)?;
//! # Ok::<(), icm20948::Error<()>>(())
//! ```
/// Interrupt pin electrical configuration
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct InterruptPinConfig {
    /// Active low (true) or active high (false)
    pub active_low: bool,
    /// Open-drain (true) or push-pull (false)
    pub open_drain: bool,
    /// Latch interrupt until cleared
    pub latch_enabled: bool,
    /// Clear interrupt status on any register read (true) or only on status read (false)
    pub clear_on_any_read: bool,
}

impl InterruptPinConfig {
    /// Create configuration for active-low, open-drain interrupt (common for I2C)
    pub const fn i2c_default() -> Self {
        Self {
            active_low: true,
            open_drain: true,
            latch_enabled: true,
            clear_on_any_read: false,
        }
    }

    /// Create configuration for active-high, push-pull interrupt (common for SPI)
    pub const fn spi_default() -> Self {
        Self {
            active_low: false,
            open_drain: false,
            latch_enabled: true,
            clear_on_any_read: false,
        }
    }
}

/// Interrupt source configuration
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct InterruptConfig {
    /// Enable raw data ready interrupt (fires when new sensor data is available)
    pub raw_data_ready: bool,
    /// Enable FIFO overflow interrupt
    pub fifo_overflow: bool,
    /// Enable FIFO watermark interrupt
    pub fifo_watermark: bool,
    /// Enable wake-on-motion interrupt
    pub wake_on_motion: bool,
    /// Enable DMP interrupt
    pub dmp: bool,
    /// Enable I2C master interrupt
    pub i2c_master: bool,
    /// Enable PLL ready interrupt
    pub pll_ready: bool,
}

impl InterruptConfig {
    /// Create configuration with only data ready interrupt enabled
    pub const fn data_ready_only() -> Self {
        Self {
            raw_data_ready: true,
            fifo_overflow: false,
            fifo_watermark: false,
            wake_on_motion: false,
            dmp: false,
            i2c_master: false,
            pll_ready: false,
        }
    }

    /// Create configuration for FIFO batch reading
    pub const fn fifo_batch() -> Self {
        Self {
            raw_data_ready: false,
            fifo_overflow: true,
            fifo_watermark: true,
            wake_on_motion: false,
            dmp: false,
            i2c_master: false,
            pll_ready: false,
        }
    }

    /// Create configuration for wake-on-motion
    pub const fn wake_on_motion_only() -> Self {
        Self {
            raw_data_ready: false,
            fifo_overflow: false,
            fifo_watermark: false,
            wake_on_motion: true,
            dmp: false,
            i2c_master: false,
            pll_ready: false,
        }
    }

    /// Check if any interrupt is enabled
    pub const fn any_enabled(&self) -> bool {
        self.raw_data_ready
            || self.fifo_overflow
            || self.fifo_watermark
            || self.wake_on_motion
            || self.dmp
            || self.i2c_master
            || self.pll_ready
    }
}

/// Interrupt status flags
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct InterruptStatus {
    /// Raw data ready interrupt flag
    pub raw_data_ready: bool,
    /// FIFO overflow interrupt flag
    pub fifo_overflow: bool,
    /// FIFO watermark interrupt flag
    pub fifo_watermark: bool,
    /// Wake-on-motion interrupt flag
    pub wake_on_motion: bool,
    /// DMP interrupt flag
    pub dmp: bool,
    /// I2C master interrupt flag
    pub i2c_master: bool,
    /// PLL ready interrupt flag
    pub pll_ready: bool,
}

impl InterruptStatus {
    /// Create empty interrupt status
    pub const fn new() -> Self {
        Self {
            raw_data_ready: false,
            fifo_overflow: false,
            fifo_watermark: false,
            wake_on_motion: false,
            dmp: false,
            i2c_master: false,
            pll_ready: false,
        }
    }

    /// Check if any interrupt flag is set
    pub const fn any_set(&self) -> bool {
        self.raw_data_ready
            || self.fifo_overflow
            || self.fifo_watermark
            || self.wake_on_motion
            || self.dmp
            || self.i2c_master
            || self.pll_ready
    }

    /// Clear all interrupt flags
    pub const fn clear() -> Self {
        Self::new()
    }

    /// Convert interrupt status to raw register value
    pub const fn to_raw(&self) -> u8 {
        let mut value = 0u8;
        if self.raw_data_ready {
            value |= 0x01;
        }
        if self.fifo_overflow {
            value |= 0x02;
        }
        if self.fifo_watermark {
            value |= 0x04;
        }
        if self.i2c_master {
            value |= 0x08;
        }
        if self.dmp {
            value |= 0x10;
        }
        if self.pll_ready {
            value |= 0x20;
        }
        if self.wake_on_motion {
            value |= 0x40;
        }
        value
    }
}

/// FSYNC (Frame Synchronization) configuration
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FsyncConfig {
    /// Enable FSYNC interrupt mode
    pub enable: bool,
    /// FSYNC pin active low
    pub active_low: bool,
}

/// Data ready status for all sensors
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct DataReadyStatus {
    /// Raw data ready for sensor register 0
    pub sensor0_ready: bool,
    /// Raw data ready for sensor register 1
    pub sensor1_ready: bool,
    /// Raw data ready for sensor register 2
    pub sensor2_ready: bool,
    /// Raw data ready for sensor register 3
    pub sensor3_ready: bool,
}

impl DataReadyStatus {
    /// Create empty data ready status
    pub const fn new() -> Self {
        Self {
            sensor0_ready: false,
            sensor1_ready: false,
            sensor2_ready: false,
            sensor3_ready: false,
        }
    }

    /// Check if any sensor has data ready
    pub const fn any_ready(&self) -> bool {
        self.sensor0_ready || self.sensor1_ready || self.sensor2_ready || self.sensor3_ready
    }
}

/// Wake-on-motion status for each axis
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WomStatus {
    /// Motion detected on X-axis
    pub x_motion: bool,
    /// Motion detected on Y-axis
    pub y_motion: bool,
    /// Motion detected on Z-axis
    pub z_motion: bool,
}

impl WomStatus {
    /// Create empty `WoM` status
    pub const fn new() -> Self {
        Self {
            x_motion: false,
            y_motion: false,
            z_motion: false,
        }
    }

    /// Check if motion was detected on any axis
    pub const fn any_motion(&self) -> bool {
        self.x_motion || self.y_motion || self.z_motion
    }

    /// Get number of axes with motion detected
    pub const fn motion_count(&self) -> u8 {
        (self.x_motion as u8) + (self.y_motion as u8) + (self.z_motion as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_interrupt_pin_config_default() {
        let config = InterruptPinConfig::default();
        assert!(!config.active_low);
        assert!(!config.open_drain);
        assert!(!config.latch_enabled);
    }

    #[test]
    fn test_interrupt_pin_config_i2c() {
        let config = InterruptPinConfig::i2c_default();
        assert!(config.active_low);
        assert!(config.open_drain);
        assert!(config.latch_enabled);
    }

    #[test]
    fn test_interrupt_pin_config_spi() {
        let config = InterruptPinConfig::spi_default();
        assert!(!config.active_low);
        assert!(!config.open_drain);
        assert!(config.latch_enabled);
    }

    #[test]
    fn test_interrupt_config_default() {
        let config = InterruptConfig::default();
        assert!(!config.any_enabled());
    }

    #[test]
    fn test_interrupt_config_data_ready() {
        let config = InterruptConfig::data_ready_only();
        assert!(config.raw_data_ready);
        assert!(config.any_enabled());
        assert!(!config.fifo_overflow);
    }

    #[test]
    fn test_interrupt_config_fifo() {
        let config = InterruptConfig::fifo_batch();
        assert!(!config.raw_data_ready);
        assert!(config.fifo_overflow);
        assert!(config.fifo_watermark);
        assert!(config.any_enabled());
    }

    #[test]
    fn test_interrupt_status() {
        let mut status = InterruptStatus::new();
        assert!(!status.any_set());

        status.raw_data_ready = true;
        assert!(status.any_set());

        let status = InterruptStatus::clear();
        assert!(!status.any_set());
    }

    #[test]
    fn test_data_ready_status() {
        let mut status = DataReadyStatus::new();
        assert!(!status.any_ready());

        status.sensor0_ready = true;
        assert!(status.any_ready());
    }

    #[test]
    fn test_wom_status() {
        let mut status = WomStatus::new();
        assert!(!status.any_motion());
        assert_eq!(status.motion_count(), 0);

        status.x_motion = true;
        assert!(status.any_motion());
        assert_eq!(status.motion_count(), 1);

        status.y_motion = true;
        assert_eq!(status.motion_count(), 2);

        status.z_motion = true;
        assert_eq!(status.motion_count(), 3);
    }

    #[test]
    fn test_fsync_config_default() {
        let config = FsyncConfig::default();
        assert!(!config.enable);
        assert!(!config.active_low);
    }
}
