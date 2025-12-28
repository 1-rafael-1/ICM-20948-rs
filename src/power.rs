//! Power management and low-power modes
//!
//! The ICM-20948 supports various power modes to reduce power consumption:
//! - **Sleep mode**: Minimum power, all sensors off
//! - **Low-power mode**: Accel-only at reduced sample rates (0.24 Hz - 500 Hz)
//! - **Cycle mode**: Periodic wake-up for sensor readings
//! - **Wake-on-motion (`WoM`)**: Interrupt on motion detection
//!
//! # Power Consumption (typical)
//! - Full operation (all sensors): ~3.4 mA
//! - Low-power accel (31.25 Hz): ~15 μA
//! - Sleep mode: ~7 μA
//!
//! # Example
//!
//! ```ignore
//! # use icm20948::{Icm20948Driver, power::{LowPowerConfig, LowPowerRate, WomMode}};
//! # let mut imu: Icm20948Driver<_> = todo!();
//! // Configure low-power mode with wake-on-motion
//! let config = LowPowerConfig {
//!     accel_rate: LowPowerRate::Hz31_25,
//!     enable_wake_on_motion: true,
//!     wom_threshold: 20, // mg
//!     wom_mode: WomMode::CompareCurrentSample,
//! };
//!
//! imu.enter_low_power_mode(&config)?;
//! # Ok::<(), icm20948::Error<()>>(())
//! ```

use crate::Error;

/// Power mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerMode {
    /// Normal operation mode - all sensors available
    Normal,
    /// Low-power mode - accelerometer only at reduced rate
    LowPower,
    /// Sleep mode - minimum power consumption
    Sleep,
}

/// Clock source selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// Internal 20 MHz oscillator
    Internal20MHz = 0,
    /// Auto-select best available clock
    AutoSelect = 1,
    /// Stop clock (lowest power)
    Stop = 7,
}

/// Low-power accelerometer sample rates
///
/// These rates are available in low-power mode. Lower rates consume less power.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LowPowerRate {
    /// 0.24 Hz (4.17 second period) - lowest power
    Hz0_24 = 0,
    /// 0.49 Hz (2.04 second period)
    Hz0_49 = 1,
    /// 0.98 Hz (1.02 second period)
    Hz0_98 = 2,
    /// 1.95 Hz (512 ms period)
    Hz1_95 = 3,
    /// 3.91 Hz (256 ms period)
    Hz3_91 = 4,
    /// 7.81 Hz (128 ms period)
    Hz7_81 = 5,
    /// 15.63 Hz (64 ms period)
    Hz15_63 = 6,
    /// 31.25 Hz (32 ms period)
    Hz31_25 = 7,
    /// 62.50 Hz (16 ms period)
    Hz62_50 = 8,
    /// 125 Hz (8 ms period)
    Hz125 = 9,
    /// 250 Hz (4 ms period)
    Hz250 = 10,
    /// 500 Hz (2 ms period)
    Hz500 = 11,
}

impl LowPowerRate {
    /// Get the sample rate in Hz
    pub const fn rate_hz(&self) -> f32 {
        match self {
            Self::Hz0_24 => 0.24,
            Self::Hz0_49 => 0.49,
            Self::Hz0_98 => 0.98,
            Self::Hz1_95 => 1.95,
            Self::Hz3_91 => 3.91,
            Self::Hz7_81 => 7.81,
            Self::Hz15_63 => 15.63,
            Self::Hz31_25 => 31.25,
            Self::Hz62_50 => 62.50,
            Self::Hz125 => 125.0,
            Self::Hz250 => 250.0,
            Self::Hz500 => 500.0,
        }
    }

    /// Get the sample period in milliseconds
    pub const fn period_ms(&self) -> f32 {
        1000.0 / self.rate_hz()
    }

    /// Get register value for ODR configuration
    pub const fn odr_value(&self) -> u8 {
        *self as u8
    }
}

/// Wake-on-Motion (`WoM`) comparison mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WomMode {
    /// Compare to initial sample (more sensitive to slow changes)
    CompareInitialSample = 0,
    /// Compare to current sample (detects sudden motion)
    CompareCurrentSample = 1,
}

/// Low-power mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LowPowerConfig {
    /// Accelerometer sample rate in low-power mode
    pub accel_rate: LowPowerRate,
    /// Enable wake-on-motion interrupt
    pub enable_wake_on_motion: bool,
    /// Wake-on-motion threshold in milligravity (mg)
    /// Range: 0-1020 mg in 4 mg steps
    pub wom_threshold: u16,
    /// Wake-on-motion comparison mode
    pub wom_mode: WomMode,
}

impl Default for LowPowerConfig {
    fn default() -> Self {
        Self {
            accel_rate: LowPowerRate::Hz31_25,
            enable_wake_on_motion: false,
            wom_threshold: 20, // 20 mg default
            wom_mode: WomMode::CompareCurrentSample,
        }
    }
}

impl LowPowerConfig {
    /// Create a new low-power configuration
    ///
    /// # Arguments
    /// * `rate` - Accelerometer sample rate
    /// * `wom_threshold_mg` - Wake-on-motion threshold in mg (0-1020)
    ///
    /// # Errors
    /// Returns `InvalidConfig` if threshold is out of range
    pub const fn new(rate: LowPowerRate, wom_threshold_mg: u16) -> Result<Self, Error<()>> {
        if wom_threshold_mg > 1020 {
            return Err(Error::InvalidConfig);
        }
        Ok(Self {
            accel_rate: rate,
            enable_wake_on_motion: true,
            wom_threshold: wom_threshold_mg,
            wom_mode: WomMode::CompareCurrentSample,
        })
    }

    /// Get the `WoM` threshold register value
    ///
    /// The register uses 4 mg steps, so we divide by 4.
    /// If the threshold exceeds the valid range, it saturates to 255.
    pub fn wom_threshold_register_value(&self) -> u8 {
        u8::try_from(self.wom_threshold / 4).unwrap_or(255)
    }

    /// Create a low-power config without wake-on-motion
    pub const fn without_wom(rate: LowPowerRate) -> Self {
        Self {
            accel_rate: rate,
            enable_wake_on_motion: false,
            wom_threshold: 0,
            wom_mode: WomMode::CompareCurrentSample,
        }
    }
}

/// Cycle mode configuration for periodic sensor readings
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CycleConfig {
    /// Enable accelerometer cycle mode
    pub enable_accel_cycle: bool,
    /// Enable gyroscope cycle mode
    pub enable_gyro_cycle: bool,
    /// Enable I2C master cycle mode (for magnetometer)
    pub enable_i2c_master_cycle: bool,
}

/// Power management status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct PowerStatus {
    /// Current power mode
    pub mode: PowerMode,
    /// Sleep enabled
    pub sleep: bool,
    /// Low-power mode enabled
    pub low_power: bool,
    /// Temperature sensor disabled
    pub temp_disabled: bool,
    /// Accelerometer cycle mode
    pub accel_cycle: bool,
    /// Gyroscope cycle mode
    pub gyro_cycle: bool,
    /// I2C master cycle mode
    pub i2c_master_cycle: bool,
}

impl Default for PowerStatus {
    fn default() -> Self {
        Self {
            mode: PowerMode::Normal,
            sleep: false,
            low_power: false,
            temp_disabled: false,
            accel_cycle: false,
            gyro_cycle: false,
            i2c_master_cycle: false,
        }
    }
}

/// Individual sensor power control
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct SensorPowerConfig {
    /// Disable accelerometer X-axis
    pub disable_accel_x: bool,
    /// Disable accelerometer Y-axis
    pub disable_accel_y: bool,
    /// Disable accelerometer Z-axis
    pub disable_accel_z: bool,
    /// Disable gyroscope X-axis
    pub disable_gyro_x: bool,
    /// Disable gyroscope Y-axis
    pub disable_gyro_y: bool,
    /// Disable gyroscope Z-axis
    pub disable_gyro_z: bool,
}

impl SensorPowerConfig {
    /// Enable all sensors
    pub const fn all_enabled() -> Self {
        Self {
            disable_accel_x: false,
            disable_accel_y: false,
            disable_accel_z: false,
            disable_gyro_x: false,
            disable_gyro_y: false,
            disable_gyro_z: false,
        }
    }

    /// Disable all accelerometer axes
    pub const fn accel_disabled() -> Self {
        Self {
            disable_accel_x: true,
            disable_accel_y: true,
            disable_accel_z: true,
            disable_gyro_x: false,
            disable_gyro_y: false,
            disable_gyro_z: false,
        }
    }

    /// Disable all gyroscope axes
    pub const fn gyro_disabled() -> Self {
        Self {
            disable_accel_x: false,
            disable_accel_y: false,
            disable_accel_z: false,
            disable_gyro_x: true,
            disable_gyro_y: true,
            disable_gyro_z: true,
        }
    }

    /// Check if all accelerometer axes are enabled
    pub const fn is_accel_enabled(&self) -> bool {
        !self.disable_accel_x && !self.disable_accel_y && !self.disable_accel_z
    }

    /// Check if all gyroscope axes are enabled
    pub const fn is_gyro_enabled(&self) -> bool {
        !self.disable_gyro_x && !self.disable_gyro_y && !self.disable_gyro_z
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::float_cmp)]
    fn test_low_power_rate_values() {
        assert_eq!(LowPowerRate::Hz31_25.rate_hz(), 31.25);
        assert_eq!(LowPowerRate::Hz125.rate_hz(), 125.0);
        assert!(LowPowerRate::Hz31_25.period_ms() > 30.0);
        assert!(LowPowerRate::Hz31_25.period_ms() < 33.0);
    }

    #[test]
    fn test_low_power_config_default() {
        let config = LowPowerConfig::default();
        assert_eq!(config.accel_rate, LowPowerRate::Hz31_25);
        assert!(!config.enable_wake_on_motion);
    }

    #[test]
    fn test_low_power_config_new() {
        let config = LowPowerConfig::new(LowPowerRate::Hz62_50, 100);
        assert!(config.is_ok());
        let config = config.unwrap();
        assert_eq!(config.accel_rate, LowPowerRate::Hz62_50);
        assert_eq!(config.wom_threshold, 100);
        assert!(config.enable_wake_on_motion);
    }

    #[test]
    fn test_low_power_config_invalid_threshold() {
        let config = LowPowerConfig::new(LowPowerRate::Hz31_25, 1100);
        assert!(config.is_err());
    }

    #[test]
    fn test_wom_threshold_register_value() {
        let config = LowPowerConfig {
            wom_threshold: 100,
            ..Default::default()
        };
        assert_eq!(config.wom_threshold_register_value(), 25);

        let config = LowPowerConfig {
            wom_threshold: 20,
            ..Default::default()
        };
        assert_eq!(config.wom_threshold_register_value(), 5);
    }

    #[test]
    fn test_sensor_power_config() {
        let config = SensorPowerConfig::all_enabled();
        assert!(config.is_accel_enabled());
        assert!(config.is_gyro_enabled());

        let config = SensorPowerConfig::accel_disabled();
        assert!(!config.is_accel_enabled());
        assert!(config.is_gyro_enabled());

        let config = SensorPowerConfig::gyro_disabled();
        assert!(config.is_accel_enabled());
        assert!(!config.is_gyro_enabled());
    }

    #[test]
    fn test_cycle_config_default() {
        let config = CycleConfig::default();
        assert!(!config.enable_accel_cycle);
        assert!(!config.enable_gyro_cycle);
        assert!(!config.enable_i2c_master_cycle);
    }

    #[test]
    fn test_power_status_default() {
        let status = PowerStatus::default();
        assert_eq!(status.mode, PowerMode::Normal);
        assert!(!status.sleep);
        assert!(!status.low_power);
    }
}
