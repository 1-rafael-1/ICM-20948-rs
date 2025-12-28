//! Gyroscope sensor types and configuration
//!
//! Provides types, enums, and utility functions for the ICM-20948's 3-axis gyroscope.

/// Gyroscope full-scale range
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroFullScale {
    /// ±250°/s range
    Dps250 = 0,
    /// ±500°/s range
    Dps500 = 1,
    /// ±1000°/s range
    Dps1000 = 2,
    /// ±2000°/s range
    Dps2000 = 3,
}

impl GyroFullScale {
    /// Get the sensitivity in LSB/(°/s)
    ///
    /// This is used to convert raw sensor values to physical units.
    #[must_use]
    pub const fn sensitivity(self) -> f32 {
        match self {
            Self::Dps250 => 131.0, // LSB/(°/s)
            Self::Dps500 => 65.5,  // LSB/(°/s)
            Self::Dps1000 => 32.8, // LSB/(°/s)
            Self::Dps2000 => 16.4, // LSB/(°/s)
        }
    }

    /// Get the maximum value in °/s
    #[must_use]
    pub const fn max_value(self) -> u16 {
        match self {
            Self::Dps250 => 250,
            Self::Dps500 => 500,
            Self::Dps1000 => 1000,
            Self::Dps2000 => 2000,
        }
    }
}

/// Gyroscope Digital Low Pass Filter (DLPF) configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroDlpf {
    /// DLPF disabled, 3281 Hz bandwidth
    Disabled = 0,
    /// 197 Hz bandwidth
    Hz197 = 1,
    /// 152 Hz bandwidth
    Hz152 = 2,
    /// 120 Hz bandwidth
    Hz120 = 3,
    /// 51 Hz bandwidth
    Hz51 = 4,
    /// 24 Hz bandwidth
    Hz24 = 5,
    /// 12 Hz bandwidth
    Hz12 = 6,
    /// 6 Hz bandwidth
    Hz6 = 7,
}

impl GyroDlpf {
    /// Get the 3dB bandwidth in Hz
    #[must_use]
    pub const fn bandwidth_hz(self) -> u16 {
        match self {
            Self::Disabled => 3281,
            Self::Hz197 => 197,
            Self::Hz152 => 152,
            Self::Hz120 => 120,
            Self::Hz51 => 51,
            Self::Hz24 => 24,
            Self::Hz12 => 12,
            Self::Hz6 => 6,
        }
    }
}

/// Gyroscope configuration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroConfig {
    /// Full-scale range
    pub full_scale: GyroFullScale,
    /// Digital Low Pass Filter configuration
    pub dlpf: GyroDlpf,
    /// Enable DLPF (if false, uses `Disabled` mode regardless of dlpf setting)
    pub dlpf_enable: bool,
    /// Sample rate divider (0-255)
    /// Actual sample rate = 1.1 kHz / (1 + `sample_rate_div`)
    pub sample_rate_div: u8,
}

impl Default for GyroConfig {
    fn default() -> Self {
        Self {
            full_scale: GyroFullScale::Dps250,
            dlpf: GyroDlpf::Hz197,
            dlpf_enable: true,
            sample_rate_div: 0,
        }
    }
}

impl GyroConfig {
    /// Calculate the effective sample rate in Hz
    #[must_use]
    pub fn sample_rate_hz(&self) -> f32 {
        1100.0 / (1.0 + f32::from(self.sample_rate_div))
    }
}

/// Gyroscope data in degrees per second
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroDataDps {
    /// X-axis rotation rate in °/s
    pub x: f32,
    /// Y-axis rotation rate in °/s
    pub y: f32,
    /// Z-axis rotation rate in °/s
    pub z: f32,
}

impl GyroDataDps {
    /// Create from raw sensor values
    ///
    /// # Arguments
    ///
    /// * `raw_x` - Raw X-axis value
    /// * `raw_y` - Raw Y-axis value
    /// * `raw_z` - Raw Z-axis value
    /// * `sensitivity` - Sensitivity in LSB/(°/s) (from `GyroFullScale::sensitivity()`)
    #[must_use]
    pub fn from_raw(raw_x: i16, raw_y: i16, raw_z: i16, sensitivity: f32) -> Self {
        Self {
            x: f32::from(raw_x) / sensitivity,
            y: f32::from(raw_y) / sensitivity,
            z: f32::from(raw_z) / sensitivity,
        }
    }

    /// Convert to radians per second
    #[must_use]
    pub fn to_radians_per_sec(&self) -> GyroDataRps {
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        GyroDataRps {
            x: self.x * DEG_TO_RAD,
            y: self.y * DEG_TO_RAD,
            z: self.z * DEG_TO_RAD,
        }
    }

    /// Get the magnitude of the rotation rate vector
    #[must_use]
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

/// Gyroscope data in radians per second
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroDataRps {
    /// X-axis rotation rate in rad/s
    pub x: f32,
    /// Y-axis rotation rate in rad/s
    pub y: f32,
    /// Z-axis rotation rate in rad/s
    pub z: f32,
}

impl GyroDataRps {
    /// Convert to degrees per second
    #[must_use]
    pub fn to_degrees_per_sec(&self) -> GyroDataDps {
        const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;
        GyroDataDps {
            x: self.x * RAD_TO_DEG,
            y: self.y * RAD_TO_DEG,
            z: self.z * RAD_TO_DEG,
        }
    }

    /// Get the magnitude of the rotation rate vector
    #[must_use]
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

/// Gyroscope calibration data
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroCalibration {
    /// Offset for X-axis (subtracted from raw value)
    pub offset_x: i16,
    /// Offset for Y-axis (subtracted from raw value)
    pub offset_y: i16,
    /// Offset for Z-axis (subtracted from raw value)
    pub offset_z: i16,
    /// Scale factor for X-axis (multiplied after offset)
    pub scale_x: f32,
    /// Scale factor for Y-axis (multiplied after offset)
    pub scale_y: f32,
    /// Scale factor for Z-axis (multiplied after offset)
    pub scale_z: f32,
}

impl Default for GyroCalibration {
    fn default() -> Self {
        Self {
            offset_x: 0,
            offset_y: 0,
            offset_z: 0,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        }
    }
}

impl GyroCalibration {
    /// Apply calibration to raw gyroscope data
    #[must_use]
    pub fn apply(&self, raw_x: i16, raw_y: i16, raw_z: i16) -> (i16, i16, i16) {
        let x = f32::from(raw_x - self.offset_x) * self.scale_x;
        let y = f32::from(raw_y - self.offset_y) * self.scale_y;
        let z = f32::from(raw_z - self.offset_z) * self.scale_z;

        // Clamp to i16 range - clamped value is guaranteed to fit in i16
        #[allow(clippy::cast_possible_truncation)]
        let x_clamped = x.clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16;
        #[allow(clippy::cast_possible_truncation)]
        let y_clamped = y.clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16;
        #[allow(clippy::cast_possible_truncation)]
        let z_clamped = z.clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16;

        (x_clamped, y_clamped, z_clamped)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 1e-6;

    #[test]
    fn test_sensitivity() {
        assert!((GyroFullScale::Dps250.sensitivity() - 131.0).abs() < EPSILON);
        assert!((GyroFullScale::Dps500.sensitivity() - 65.5).abs() < EPSILON);
        assert!((GyroFullScale::Dps1000.sensitivity() - 32.8).abs() < EPSILON);
        assert!((GyroFullScale::Dps2000.sensitivity() - 16.4).abs() < EPSILON);
    }

    #[test]
    fn test_sample_rate() {
        let config = GyroConfig {
            sample_rate_div: 0,
            ..Default::default()
        };
        assert!((config.sample_rate_hz() - 1100.0).abs() < EPSILON);

        let config = GyroConfig {
            sample_rate_div: 10,
            ..Default::default()
        };
        assert!((config.sample_rate_hz() - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_gyro_data_conversion() {
        let data = GyroDataDps::from_raw(131, 0, -131, 131.0);
        assert!((data.x - 1.0).abs() < 0.001);
        assert!((data.y - 0.0).abs() < 0.001);
        assert!((data.z - (-1.0)).abs() < 0.001);
    }

    #[test]
    fn test_magnitude() {
        let data = GyroDataDps {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        };
        assert!((data.magnitude() - 1.0).abs() < 0.001);

        let data = GyroDataDps {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        assert!((data.magnitude() - 1.732).abs() < 0.001);
    }

    #[test]
    fn test_calibration_apply() {
        let cal = GyroCalibration {
            offset_x: 100,
            offset_y: -50,
            offset_z: 200,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        let (x, y, z) = cal.apply(1100, 950, 2200);
        assert_eq!(x, 1000);
        assert_eq!(y, 1000);
        assert_eq!(z, 2000);
    }

    #[test]
    fn test_deg_rad_conversion() {
        let dps = GyroDataDps {
            x: 180.0,
            y: 90.0,
            z: 45.0,
        };

        let rps = dps.to_radians_per_sec();
        assert!((rps.x - core::f32::consts::PI).abs() < 0.001);
        assert!((rps.y - core::f32::consts::PI / 2.0).abs() < 0.001);
        assert!((rps.z - core::f32::consts::PI / 4.0).abs() < 0.001);

        let dps_back = rps.to_degrees_per_sec();
        assert!((dps_back.x - 180.0).abs() < 0.001);
        assert!((dps_back.y - 90.0).abs() < 0.001);
        assert!((dps_back.z - 45.0).abs() < 0.001);
    }
}
