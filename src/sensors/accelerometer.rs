//! Accelerometer sensor types and configuration
//!
//! Provides types, enums, and utility functions for the ICM-20948's 3-axis accelerometer.

/// Accelerometer full-scale range
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelFullScale {
    /// ±2g range (most sensitive, least range)
    G2 = 0,
    /// ±4g range
    G4 = 1,
    /// ±8g range
    G8 = 2,
    /// ±16g range (least sensitive, most range)
    G16 = 3,
}

impl AccelFullScale {
    /// Get the sensitivity in LSB/g (Least Significant Bit per g)
    ///
    /// This is used to convert raw sensor values to physical units.
    #[must_use]
    pub const fn sensitivity(self) -> f32 {
        match self {
            Self::G2 => 16384.0, // LSB/g
            Self::G4 => 8192.0,  // LSB/g
            Self::G8 => 4096.0,  // LSB/g
            Self::G16 => 2048.0, // LSB/g
        }
    }

    /// Get the maximum value in g
    #[must_use]
    pub const fn max_value(self) -> u8 {
        match self {
            Self::G2 => 2,
            Self::G4 => 4,
            Self::G8 => 8,
            Self::G16 => 16,
        }
    }
}

/// Accelerometer Digital Low Pass Filter (DLPF) configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelDlpf {
    /// DLPF disabled, 1209 Hz bandwidth
    Disabled = 0,
    /// 246 Hz bandwidth, 1.13 kHz sample rate
    Hz246 = 1,
    /// 111 Hz bandwidth, 1.13 kHz sample rate
    Hz111 = 2,
    /// 50 Hz bandwidth, 1.13 kHz sample rate
    Hz50 = 3,
    /// 24 Hz bandwidth, 1.13 kHz sample rate
    Hz24 = 4,
    /// 12 Hz bandwidth, 1.13 kHz sample rate
    Hz12 = 5,
    /// 6 Hz bandwidth, 1.13 kHz sample rate
    Hz6 = 6,
    /// 473 Hz bandwidth, 1.13 kHz sample rate
    Hz473 = 7,
}

impl AccelDlpf {
    /// Get the 3dB bandwidth in Hz
    #[must_use]
    pub const fn bandwidth_hz(self) -> u16 {
        match self {
            Self::Disabled => 1209,
            Self::Hz246 => 246,
            Self::Hz111 => 111,
            Self::Hz50 => 50,
            Self::Hz24 => 24,
            Self::Hz12 => 12,
            Self::Hz6 => 6,
            Self::Hz473 => 473,
        }
    }
}

/// Accelerometer configuration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelConfig {
    /// Full-scale range
    pub full_scale: AccelFullScale,
    /// Digital Low Pass Filter configuration
    pub dlpf: AccelDlpf,
    /// Enable DLPF (if false, uses `Disabled` mode regardless of dlpf setting)
    pub dlpf_enable: bool,
    /// Sample rate divider (0-4095)
    /// Actual sample rate = 1.125 kHz / (1 + `sample_rate_div`)
    pub sample_rate_div: u16,
}

impl Default for AccelConfig {
    fn default() -> Self {
        Self {
            full_scale: AccelFullScale::G2,
            dlpf: AccelDlpf::Hz246,
            dlpf_enable: true,
            sample_rate_div: 0,
        }
    }
}

impl AccelConfig {
    /// Calculate the effective sample rate in Hz
    #[must_use]
    pub fn sample_rate_hz(&self) -> f32 {
        1125.0 / (1.0 + f32::from(self.sample_rate_div))
    }
}

/// Accelerometer data in physical units (g-force)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelDataG {
    /// X-axis acceleration in g
    pub x: f32,
    /// Y-axis acceleration in g
    pub y: f32,
    /// Z-axis acceleration in g
    pub z: f32,
}

impl AccelDataG {
    /// Create from raw sensor values
    ///
    /// # Arguments
    ///
    /// * `raw_x` - Raw X-axis value
    /// * `raw_y` - Raw Y-axis value
    /// * `raw_z` - Raw Z-axis value
    /// * `sensitivity` - Sensitivity in LSB/g (from `AccelFullScale::sensitivity()`)
    #[must_use]
    pub fn from_raw(raw_x: i16, raw_y: i16, raw_z: i16, sensitivity: f32) -> Self {
        Self {
            x: f32::from(raw_x) / sensitivity,
            y: f32::from(raw_y) / sensitivity,
            z: f32::from(raw_z) / sensitivity,
        }
    }

    /// Get the magnitude of the acceleration vector
    #[must_use]
    pub fn magnitude(&self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Normalize the acceleration vector (make magnitude = 1.0)
    #[must_use]
    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0 {
            Self {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        } else {
            *self
        }
    }
}

/// Accelerometer calibration data
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelCalibration {
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

impl Default for AccelCalibration {
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

impl AccelCalibration {
    /// Apply calibration to raw accelerometer data
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
        assert!((AccelFullScale::G2.sensitivity() - 16384.0).abs() < EPSILON);
        assert!((AccelFullScale::G4.sensitivity() - 8192.0).abs() < EPSILON);
        assert!((AccelFullScale::G8.sensitivity() - 4096.0).abs() < EPSILON);
        assert!((AccelFullScale::G16.sensitivity() - 2048.0).abs() < EPSILON);
    }

    #[test]
    fn test_sample_rate() {
        let config = AccelConfig {
            sample_rate_div: 0,
            ..Default::default()
        };
        assert!((config.sample_rate_hz() - 1125.0).abs() < EPSILON);

        let config = AccelConfig {
            sample_rate_div: 10,
            ..Default::default()
        };
        assert!((config.sample_rate_hz() - 102.27).abs() < 0.01);
    }

    #[test]
    fn test_accel_data_conversion() {
        let data = AccelDataG::from_raw(16384, 0, -16384, 16384.0);
        assert!((data.x - 1.0).abs() < 0.001);
        assert!((data.y - 0.0).abs() < 0.001);
        assert!((data.z - (-1.0)).abs() < 0.001);
    }

    #[test]
    fn test_magnitude() {
        let data = AccelDataG {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        };
        assert!((data.magnitude() - 1.0).abs() < 0.001);

        let data = AccelDataG {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        assert!((data.magnitude() - 1.732).abs() < 0.001);
    }

    #[test]
    fn test_calibration_apply() {
        let cal = AccelCalibration {
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
}
