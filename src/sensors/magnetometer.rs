//! Magnetometer sensor types and configuration
//!
//! Provides types, enums, constants, and utility functions for the ICM-20948's
//! AK09916 3-axis magnetometer.

/// AK09916 magnetometer I2C address
pub const AK09916_I2C_ADDRESS: u8 = 0x0C;

/// AK09916 `WHO_AM_I` register address
pub const AK09916_REG_WIA2: u8 = 0x01;

/// Expected `WHO_AM_I` value for AK09916
pub const AK09916_WIA2_VALUE: u8 = 0x09;

/// AK09916 Reserved 2 register address (used for DMP magnetometer reads)
pub const AK09916_REG_RSV2: u8 = 0x03;

/// AK09916 Status 1 register address
pub const AK09916_REG_ST1: u8 = 0x10;

/// AK09916 measurement data start register (HXL)
pub const AK09916_REG_HXL: u8 = 0x11;

/// AK09916 Status 2 register address
pub const AK09916_REG_ST2: u8 = 0x18;

/// AK09916 Control 2 register address (mode control)
pub const AK09916_REG_CNTL2: u8 = 0x31;

/// AK09916 Control 3 register address (reset)
pub const AK09916_REG_CNTL3: u8 = 0x32;

/// Magnetometer operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MagMode {
    /// Power-down mode
    PowerDown = 0x00,
    /// Single measurement mode
    Single = 0x01,
    /// Continuous measurement mode 1 (10 Hz)
    Continuous10Hz = 0x02,
    /// Continuous measurement mode 2 (20 Hz)
    Continuous20Hz = 0x04,
    /// Continuous measurement mode 3 (50 Hz)
    Continuous50Hz = 0x06,
    /// Continuous measurement mode 4 (100 Hz)
    Continuous100Hz = 0x08,
    /// Self-test mode
    SelfTest = 0x10,
}

impl MagMode {
    /// Get the sample rate in Hz for continuous modes
    #[must_use]
    pub const fn sample_rate_hz(self) -> Option<u8> {
        match self {
            Self::Continuous10Hz => Some(10),
            Self::Continuous20Hz => Some(20),
            Self::Continuous50Hz => Some(50),
            Self::Continuous100Hz => Some(100),
            _ => None,
        }
    }

    /// Check if this is a continuous measurement mode
    #[must_use]
    pub const fn is_continuous(self) -> bool {
        matches!(
            self,
            Self::Continuous10Hz
                | Self::Continuous20Hz
                | Self::Continuous50Hz
                | Self::Continuous100Hz
        )
    }
}

/// Magnetometer configuration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagConfig {
    /// Operating mode
    pub mode: MagMode,
}

impl Default for MagConfig {
    fn default() -> Self {
        Self {
            mode: MagMode::Continuous100Hz,
        }
    }
}

/// Magnetometer data in microteslas (µT)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagDataUT {
    /// X-axis magnetic field in µT
    pub x: f32,
    /// Y-axis magnetic field in µT
    pub y: f32,
    /// Z-axis magnetic field in µT
    pub z: f32,
}

impl MagDataUT {
    /// Calculate the magnitude of the magnetic field vector
    ///
    /// Returns the magnitude in µT.
    #[must_use]
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Normalize the magnetic field vector to unit length
    ///
    /// If the magnitude is near zero, returns (0, 0, 0).
    #[must_use]
    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag < 1e-6 {
            Self {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }
        } else {
            Self {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        }
    }
}

/// Magnetometer calibration data
///
/// Compensates for hard-iron (offset) and soft-iron (scale) effects.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagCalibration {
    /// Offset for X-axis (hard-iron bias)
    pub offset_x: f32,
    /// Offset for Y-axis (hard-iron bias)
    pub offset_y: f32,
    /// Offset for Z-axis (hard-iron bias)
    pub offset_z: f32,
    /// Scale factor for X-axis (soft-iron correction)
    pub scale_x: f32,
    /// Scale factor for Y-axis (soft-iron correction)
    pub scale_y: f32,
    /// Scale factor for Z-axis (soft-iron correction)
    pub scale_z: f32,
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self {
            offset_x: 0.0,
            offset_y: 0.0,
            offset_z: 0.0,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        }
    }
}

impl MagCalibration {
    /// Apply calibration to magnetometer data
    ///
    /// Corrects for hard-iron (offset) and soft-iron (scale) effects.
    #[must_use]
    pub fn apply(&self, data: &MagDataUT) -> MagDataUT {
        MagDataUT {
            x: (data.x - self.offset_x) * self.scale_x,
            y: (data.y - self.offset_y) * self.scale_y,
            z: (data.z - self.offset_z) * self.scale_z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mag_mode_sample_rate() {
        assert_eq!(MagMode::Continuous10Hz.sample_rate_hz(), Some(10));
        assert_eq!(MagMode::Continuous100Hz.sample_rate_hz(), Some(100));
        assert_eq!(MagMode::PowerDown.sample_rate_hz(), None);
    }

    #[test]
    fn test_mag_mode_is_continuous() {
        assert!(MagMode::Continuous10Hz.is_continuous());
        assert!(!MagMode::PowerDown.is_continuous());
    }

    #[test]
    fn test_mag_data_magnitude() {
        let data = MagDataUT {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };
        assert!((data.magnitude() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_mag_data_normalize() {
        let data = MagDataUT {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };
        let norm = data.normalize();
        assert!((norm.magnitude() - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_mag_calibration_apply() {
        let cal = MagCalibration {
            offset_x: 10.0,
            offset_y: -5.0,
            offset_z: 20.0,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        let data = MagDataUT {
            x: 110.0,
            y: 95.0,
            z: 220.0,
        };

        let corrected = cal.apply(&data);
        assert!((corrected.x - 100.0).abs() < 0.001);
        assert!((corrected.y - 100.0).abs() < 0.001);
        assert!((corrected.z - 200.0).abs() < 0.001);
    }

    #[test]
    fn test_default_calibration() {
        let cal = MagCalibration::default();
        assert!((cal.offset_x - 0.0).abs() < f32::EPSILON);
        assert!((cal.scale_x - 1.0).abs() < f32::EPSILON);

        let data = MagDataUT {
            x: 100.0,
            y: 200.0,
            z: 300.0,
        };

        let corrected = cal.apply(&data);
        assert!((corrected.x - 100.0).abs() < 0.001);
        assert!((corrected.y - 200.0).abs() < 0.001);
        assert!((corrected.z - 300.0).abs() < 0.001);
    }
}
