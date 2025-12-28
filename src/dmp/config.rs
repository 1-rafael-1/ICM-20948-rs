//! DMP Configuration
//!
//! This module provides configuration functionality for the ICM-20948's Digital Motion Processor.
//! After loading the firmware, the DMP must be configured to enable specific features like
//! quaternion output, calibrated sensor data, and sample rates.
//!
//! ## Configuration Process
//!
//! 1. Load DMP firmware using `dmp_init()`
//! 2. Create a `DmpConfig` with desired features
//! 3. Apply configuration using `dmp_configure()`
//! 4. Enable DMP using `dmp_enable(true)`
//!
//! ## Important Implementation Notes
//!
//! The DMP requires extensive initialization beyond just loading firmware. This includes:
//!
//! - **Sensor Configuration**: Accel (4g), Gyro (2000dps), sample rates (~56Hz)
//! - **Scaling Factors**: Internal DMP scaling (2^25 = 1g) and output scaling
//! - **Mount Matrices**: Magnetometer alignment (converts AK09916 to DMP format)
//! - **Calibration Parameters**: Rate-dependent gain and variance values
//! - **Feature Enablement**: Which outputs (quaternion, calibrated data, etc.)
//!
//! The complete initialization writes 28 configuration values to DMP memory at specific
//! addresses. These values are based on the InvenSense reference implementation and
//! SparkFun's validated Arduino library.
//!
//! ## Feature Bits
//!
//! The DMP uses a 16-bit feature mask to control which outputs are enabled.
//! These are written to specific DMP memory addresses after firmware loading.

use crate::dmp::DmpConfig;

/// Calculate GYRO_SF (Gyro Scaling Factor) for DMP
///
/// This value depends on the gyro sample rate divider and the PLL correction value.
/// The PLL value should be read from Bank 1, Register 0x28 (TIMEBASE_CORRECTION_PLL).
///
/// # Arguments
///
/// * `gyro_sample_rate_div` - Value from GYRO_SMPLRT_DIV register (0-255)
///   - 0 = 1125 Hz, 1 = 562.5 Hz, 4 = 225 Hz, 9 = 112 Hz, 19 = 55 Hz
/// * `pll_correction` - Value from TIMEBASE_CORRECTION_PLL register (typically 0x18)
///
/// # Formula
///
/// ```text
/// MagicConstant = 264446880937391
/// gyro_level = 4 (always 4 regardless of FSR)
///
/// if (pll & 0x80):
///     result = MagicConstant * (1 << gyro_level) * (1 + div) / (1270 - (pll & 0x7F)) / 100000
/// else:
///     result = MagicConstant * (1 << gyro_level) * (1 + div) / (1270 + pll) / 100000
/// ```
///
/// # Example
///
/// ```
/// # use icm20948::dmp::config::calculate_gyro_sf;
/// // For 56Hz (div=19) with PLL=0x09
/// let gyro_sf = calculate_gyro_sf(19, 0x09);
/// assert_eq!(gyro_sf, 0x276FBC37);
/// ```
pub fn calculate_gyro_sf(gyro_sample_rate_div: u8, pll_correction: i8) -> u32 {
    const MAGIC_CONSTANT: u64 = 264446880937391;
    const MAGIC_CONSTANT_SCALE: u64 = 100000;
    const GYRO_LEVEL: u8 = 4; // Always 4 regardless of FSR

    let div = gyro_sample_rate_div as u64;
    let pll = pll_correction as i16; // Sign-extend to i16

    let result: u64 = if pll < 0 {
        // PLL has bit 7 set (negative when treated as signed)
        // Convert to positive value by masking off sign bit
        #[allow(clippy::cast_sign_loss)]
        let pll_abs = u64::from((pll & 0x7F) as u16);
        MAGIC_CONSTANT * (1u64 << GYRO_LEVEL) * (1 + div) / (1270 - pll_abs) / MAGIC_CONSTANT_SCALE
    } else {
        // PLL is positive, safe to convert
        #[allow(clippy::cast_sign_loss)]
        let pll_val = u64::from(pll as u16);
        MAGIC_CONSTANT * (1u64 << GYRO_LEVEL) * (1 + div) / (1270 + pll_val) / MAGIC_CONSTANT_SCALE
    };

    // Clamp to i32::MAX if overflow
    if result > 0x7FFF_FFFF {
        0x7FFF_FFFF
    } else {
        u32::try_from(result).unwrap_or(0x7FFF_FFFF)
    }
}

/// Predefined DMP sample rates with calibration parameters
///
/// These are the three validated sample rates.
/// Each rate has specific calibration parameters for optimal performance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmpSampleRate {
    /// 56 Hz - Most common, lowest power
    Hz56,
    /// 112 Hz - Medium rate
    Hz112,
    /// 225 Hz - Highest rate
    Hz225,
}

impl DmpSampleRate {
    /// Get the gyro sample rate divider for this rate
    ///
    /// Formula: ODR = 1100 Hz / (1 + divider)
    pub const fn gyro_sample_rate_div(&self) -> u8 {
        match self {
            Self::Hz56 => 19, // 1100 / 20 = 55 Hz
            Self::Hz112 => 9, // 1100 / 10 = 110 Hz
            Self::Hz225 => 4, // 1100 / 5 = 220 Hz
        }
    }

    /// Get ACCEL_ONLY_GAIN parameter for this rate
    ///
    /// Validated values:
    /// - 56Hz: 0x03A49249
    /// - 112Hz: 0x01D1745D
    /// - 225Hz: 0x00E8BA2E
    pub const fn accel_only_gain(&self) -> u32 {
        match self {
            Self::Hz56 => 0x03A49249,
            Self::Hz112 => 0x01D1745D,
            Self::Hz225 => 0x00E8BA2E,
        }
    }

    /// Get ACCEL_ALPHA_VAR parameter for this rate
    ///
    /// Validated values:
    /// - 56Hz: 0x34924925
    /// - 112Hz: 0x3A492492
    /// - 225Hz: 0x3D27D27D
    pub const fn accel_alpha_var(&self) -> u32 {
        match self {
            Self::Hz56 => 0x34924925,
            Self::Hz112 => 0x3A492492,
            Self::Hz225 => 0x3D27D27D,
        }
    }

    /// Get ACCEL_A_VAR parameter for this rate
    ///
    /// Validated values:
    /// - 56Hz: 0x0B6DB6DB
    /// - 112Hz: 0x05B6DB6E
    /// - 225Hz: 0x02D82D83
    pub const fn accel_a_var(&self) -> u32 {
        match self {
            Self::Hz56 => 0x0B6DB6DB,
            Self::Hz112 => 0x05B6DB6E,
            Self::Hz225 => 0x02D82D83,
        }
    }

    /// Select the closest supported rate for a given frequency
    ///
    /// # Example
    ///
    /// ```
    /// # use icm20948::dmp::config::DmpSampleRate;
    /// assert_eq!(DmpSampleRate::from_hz(50), DmpSampleRate::Hz56);
    /// assert_eq!(DmpSampleRate::from_hz(100), DmpSampleRate::Hz112);
    /// assert_eq!(DmpSampleRate::from_hz(200), DmpSampleRate::Hz225);
    /// ```
    pub const fn from_hz(hz: u16) -> Self {
        if hz <= 84 {
            Self::Hz56
        } else if hz <= 168 {
            Self::Hz112
        } else {
            Self::Hz225
        }
    }
}

/// Configuration for arbitrary DMP sample rates with interpolated calibration parameters
///
/// For rates not in the validated set (56, 112, 225 Hz), this struct provides
/// linearly interpolated calibration parameters based on the nearest validated rates.
///
/// # Warning
///
/// Interpolated parameters are not fully validated. Use at your own risk.
/// For production use, stick to the validated rates in `DmpSampleRate`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ArbitrarySampleRate {
    /// Gyro sample rate divider (ODR = 1100 Hz / (1 + div))
    pub gyro_div: u8,
    /// Accelerometer only gain parameter
    pub accel_only_gain: u32,
    /// Accelerometer alpha variance parameter
    pub accel_alpha_var: u32,
    /// Accelerometer A variance parameter
    pub accel_a_var: u32,
}

impl ArbitrarySampleRate {
    /// Calculate interpolated parameters for an arbitrary sample rate
    ///
    /// This function linearly interpolates calibration parameters between the
    /// nearest validated rates. The interpolation assumes the parameters scale
    /// linearly with sample rate, which is an approximation.
    ///
    /// # Arguments
    ///
    /// * `hz` - Target sample rate in Hz (clamped to 4-550 Hz range)
    ///
    /// # Returns
    ///
    /// `ArbitrarySampleRate` with interpolated parameters, or `None` if the rate
    /// matches a validated rate (use `DmpSampleRate` instead).
    ///
    /// # Example
    ///
    /// ```
    /// # use icm20948::dmp::config::ArbitrarySampleRate;
    /// // Get interpolated parameters for 75 Hz
    /// let rate_config = ArbitrarySampleRate::interpolate(75).unwrap();
    /// assert_eq!(rate_config.gyro_div, 13); // 1100 / 14 ≈ 78.57 Hz
    /// ```
    pub fn interpolate(hz: u16) -> Option<Self> {
        // Clamp to reasonable range: 4 Hz (div=274) to 550 Hz (div=1)
        let hz = hz.clamp(4, 550);

        // Calculate gyro divider: ODR = 1100 / (1 + div)
        let gyro_div = if hz >= 1100 {
            0
        } else {
            ((1100 / hz as u32).saturating_sub(1)).min(255) as u8
        };

        // If this exactly matches a validated rate, return None
        // (caller should use DmpSampleRate enum instead)
        if gyro_div == 19 || gyro_div == 9 || gyro_div == 4 {
            return None;
        }

        // Linear interpolation between validated rates
        let (lower_rate, upper_rate) = if hz <= 56 {
            // Extrapolate below 56 Hz using 56-112 Hz slope
            (DmpSampleRate::Hz56, DmpSampleRate::Hz112)
        } else if hz <= 112 {
            // Interpolate between 56-112 Hz
            (DmpSampleRate::Hz56, DmpSampleRate::Hz112)
        } else if hz <= 225 {
            // Interpolate between 112-225 Hz
            (DmpSampleRate::Hz112, DmpSampleRate::Hz225)
        } else {
            // Extrapolate above 225 Hz using 112-225 Hz slope
            (DmpSampleRate::Hz112, DmpSampleRate::Hz225)
        };

        let lower_hz = 1100.0 / (1.0 + lower_rate.gyro_sample_rate_div() as f32);
        let upper_hz = 1100.0 / (1.0 + upper_rate.gyro_sample_rate_div() as f32);
        let hz_f = hz as f32;

        // Interpolation factor
        let t = (hz_f - lower_hz) / (upper_hz - lower_hz);
        let t = t.clamp(0.0, 1.0);

        // Interpolate each parameter
        let accel_only_gain = Self::lerp_u32(
            lower_rate.accel_only_gain(),
            upper_rate.accel_only_gain(),
            t,
        );
        let accel_alpha_var = Self::lerp_u32(
            lower_rate.accel_alpha_var(),
            upper_rate.accel_alpha_var(),
            t,
        );
        let accel_a_var = Self::lerp_u32(lower_rate.accel_a_var(), upper_rate.accel_a_var(), t);

        Some(Self {
            gyro_div,
            accel_only_gain,
            accel_alpha_var,
            accel_a_var,
        })
    }

    /// Linear interpolation for u32 values
    #[inline]
    fn lerp_u32(a: u32, b: u32, t: f32) -> u32 {
        let a_f = a as f32;
        let b_f = b as f32;
        let result = a_f + (b_f - a_f) * t;
        let rounded = libm::roundf(result);
        // Clamp to valid u32 range
        if rounded < 0.0 {
            0
        } else if rounded > u32::MAX as f32 {
            u32::MAX
        } else {
            #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
            {
                rounded as u32
            }
        }
    }
}

/// DMP feature control bits
///
/// These bits are used in the DMP feature mask to enable/disable specific
/// DMP processing features.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpFeatures;

impl DmpFeatures {
    /// 16-bit accelerometer (bit 15)
    pub const ACCEL: u16 = 0x8000;

    /// 16-bit gyroscope (bit 14)
    pub const GYRO: u16 = 0x4000;

    /// 16-bit compass (bit 13)
    pub const COMPASS: u16 = 0x2000;

    /// 6-axis quaternion output (gyro + accel) (bit 11)
    pub const QUAT6: u16 = 0x0800;

    /// 9-axis quaternion output (gyro + accel + mag) (bit 10)
    pub const QUAT9: u16 = 0x0400;

    /// Geomagnetic rotation vector (9-axis with heading) (bit 8)
    pub const GEOMAG_ROTATION_VECTOR: u16 = 0x0100;

    /// 32-bit calibrated gyroscope (bit 6)
    pub const SEND_CAL_GYRO: u16 = 0x0040;

    /// 32-bit calibrated compass (bit 5)
    pub const SEND_CAL_COMPASS: u16 = 0x0020;

    /// Pedometer/step detector (bit 4)
    pub const PEDOMETER: u16 = 0x0010;

    /// Header 2 (bit 3)
    pub const HEADER2: u16 = 0x0008;

    /// Game rotation vector (6-axis, no magnetometer) - uses QUAT6
    pub const GAME_ROTATION_VECTOR: u16 = Self::QUAT6;

    /// Send raw accelerometer data - uses ACCEL
    pub const SEND_RAW_ACCEL: u16 = Self::ACCEL;

    /// Send raw gyroscope data - uses GYRO
    pub const SEND_RAW_GYRO: u16 = Self::GYRO;

    /// Send calibrated accelerometer data - not directly supported, use ACCEL
    pub const SEND_CAL_ACCEL: u16 = Self::ACCEL;

    /// Gyroscope calibration - enabled via MOTION_EVENT_CTL, not feature mask
    pub const GYRO_CALIBRATION: u16 = 0x0000;
}

/// DMP memory addresses for configuration
///
/// These addresses are used to write configuration data to the DMP's
/// internal memory after firmware loading.
#[derive(Debug, Clone, Copy)]
pub struct DmpMemoryAddresses;

impl DmpMemoryAddresses {
    /// Data output control register 1
    pub const DATA_OUT_CTL1: u16 = 0x0040;

    /// Data output control register 2
    pub const DATA_OUT_CTL2: u16 = 0x0042;

    /// Data interrupt control
    pub const DATA_INTR_CTL: u16 = 0x004C;

    /// FIFO watermark
    pub const FIFO_WATERMARK: u16 = 0x01FE;

    /// Motion event control
    pub const MOTION_EVENT_CTL: u16 = 0x004E;

    /// Data ready status
    pub const DATA_RDY_STATUS: u16 = 0x008A;

    /// Sample rate divider
    pub const ODR_RATE: u16 = 0x029C;

    /// Gyroscope scaling factor (depends on sample rate and FSR)
    /// Formula: GYRO_SF = (sample_rate_div + 1) * gyro_fsr_scale
    pub const GYRO_SF: u16 = 0x0130;

    /// Accelerometer scaling factor 1 (for DMP internal alignment)
    /// Write 0x04000000 for 4g FSR (2^25 = 1g internally)
    pub const ACC_SCALE: u16 = 0x01E0;

    /// Accelerometer scaling factor 2 (for output)
    /// Write 0x00040000 for 4g FSR (outputs in hardware units)
    pub const ACC_SCALE2: u16 = 0x04F4;

    /// Compass (magnetometer) mount matrix element (0,0)
    pub const CPASS_MTX_00: u16 = 0x04B4;
    /// Compass (magnetometer) mount matrix element (0,1)
    pub const CPASS_MTX_01: u16 = 0x04B8;
    /// Compass (magnetometer) mount matrix element (0,2)
    pub const CPASS_MTX_02: u16 = 0x04BC;
    /// Compass (magnetometer) mount matrix element (1,0)
    pub const CPASS_MTX_10: u16 = 0x04C0;
    /// Compass (magnetometer) mount matrix element (1,1)
    pub const CPASS_MTX_11: u16 = 0x04C4;
    /// Compass (magnetometer) mount matrix element (1,2)
    pub const CPASS_MTX_12: u16 = 0x04C8;
    /// Compass (magnetometer) mount matrix element (2,0)
    pub const CPASS_MTX_20: u16 = 0x04CC;
    /// Compass (magnetometer) mount matrix element (2,1)
    pub const CPASS_MTX_21: u16 = 0x04D0;
    /// Compass (magnetometer) mount matrix element (2,2)
    pub const CPASS_MTX_22: u16 = 0x04D4;

    /// Body to Sensor mount matrix element (0,0)
    pub const B2S_MTX_00: u16 = 0x04D8;
    /// Body to Sensor mount matrix element (0,1)
    pub const B2S_MTX_01: u16 = 0x04DC;
    /// Body to Sensor mount matrix element (0,2)
    pub const B2S_MTX_02: u16 = 0x04E0;
    /// Body to Sensor mount matrix element (1,0)
    pub const B2S_MTX_10: u16 = 0x04E4;
    /// Body to Sensor mount matrix element (1,1)
    pub const B2S_MTX_11: u16 = 0x04E8;
    /// Body to Sensor mount matrix element (1,2)
    pub const B2S_MTX_12: u16 = 0x04EC;
    /// Body to Sensor mount matrix element (2,0)
    pub const B2S_MTX_20: u16 = 0x04F0;
    /// Body to Sensor mount matrix element (2,1)
    pub const B2S_MTX_21: u16 = 0x04F4;
    /// Body to Sensor mount matrix element (2,2)
    pub const B2S_MTX_22: u16 = 0x04F8;

    /// Gyroscope full scale setting (2^28 for 2000dps)
    pub const GYRO_FULLSCALE: u16 = 0x048C;

    /// Accel only gain
    pub const ACCEL_ONLY_GAIN: u16 = 0x0500;

    /// Accel alpha variance
    pub const ACCEL_ALPHA_VAR: u16 = 0x0504;

    /// Accel A variance
    pub const ACCEL_A_VAR: u16 = 0x0508;

    /// Accel calibration rate
    pub const ACCEL_CAL_RATE: u16 = 0x050C;

    /// Compass time buffer (magnetometer sample rate)
    pub const CPASS_TIME_BUFFER: u16 = 0x050E;
}

/// DMP Data Output Control 2 (DATA_OUT_CTL2) bit masks
///
/// These bits control accuracy reporting in the DMP output packets (header2)
pub struct DmpDataOutputControl2;

impl DmpDataOutputControl2 {
    /// Accelerometer accuracy bit
    pub const ACCEL_ACCURACY: u16 = 0x4000;

    /// Gyroscope accuracy bit
    pub const GYRO_ACCURACY: u16 = 0x2000;

    /// Compass (magnetometer) accuracy bit
    pub const COMPASS_ACCURACY: u16 = 0x1000;
}

/// DMP Data Ready Status (DATA_RDY_STATUS) bit masks
///
/// These bits control which sensors trigger data ready events
pub struct DmpDataReadyStatus;

impl DmpDataReadyStatus {
    /// Gyroscope data ready
    pub const GYRO: u16 = 0x0001;

    /// Accelerometer data ready
    pub const ACCEL: u16 = 0x0002;

    /// Compass (magnetometer) data ready
    pub const COMPASS: u16 = 0x0008;
}

/// DMP Motion Event Control (MOTION_EVENT_CTL) bit masks
///
/// These bits control calibration and sensor fusion features
pub struct DmpMotionEventControl;

impl DmpMotionEventControl {
    /// Geomagnetic rotation vector enable
    pub const GEOMAG: u16 = 0x0008;

    /// 9-axis sensor fusion enable
    pub const NINE_AXIS: u16 = 0x0040;

    /// Compass (magnetometer) calibration enable
    pub const COMPASS_CALIBR: u16 = 0x0080;

    /// Gyroscope calibration enable
    pub const GYRO_CALIBR: u16 = 0x0100;

    /// Accelerometer calibration enable
    pub const ACCEL_CALIBR: u16 = 0x0200;

    /// Pedometer interrupt enable
    pub const PEDOMETER_INTERRUPT: u16 = 0x2000;
}

/// DMP Output Data Rate (ODR) register addresses
///
/// These registers control the output rate for each DMP feature
pub struct DmpOdrRegisters;

impl DmpOdrRegisters {
    /// ODR for 9-axis quaternion
    pub const QUAT9: u16 = 0x00A8;

    /// ODR for 6-axis quaternion
    pub const QUAT6: u16 = 0x00AC;

    /// ODR for accelerometer
    pub const ACCEL: u16 = 0x00BE;

    /// ODR for gyroscope
    pub const GYRO: u16 = 0x00BA;

    /// ODR for compass (magnetometer)
    pub const CPASS: u16 = 0x00B6;

    /// ODR for calibrated gyroscope
    pub const GYRO_CALIBR: u16 = 0x00B8;

    /// ODR for calibrated compass
    pub const CPASS_CALIBR: u16 = 0x00B4;
}

/// DMP Output Data Rate Counter (ODR_CNTR) register addresses
///
/// These counters must be reset (set to 0) when changing ODR values
pub struct DmpOdrCounterRegisters;

impl DmpOdrCounterRegisters {
    /// ODR counter for 9-axis quaternion
    pub const QUAT9: u16 = 0x0088;

    /// ODR counter for 6-axis quaternion
    pub const QUAT6: u16 = 0x008C;

    /// ODR counter for accelerometer
    pub const ACCEL: u16 = 0x009E;

    /// ODR counter for gyroscope
    pub const GYRO: u16 = 0x009A;

    /// ODR counter for compass (magnetometer)
    pub const CPASS: u16 = 0x0096;

    /// ODR counter for calibrated gyroscope
    pub const GYRO_CALIBR: u16 = 0x0098;

    /// ODR counter for calibrated compass
    pub const CPASS_CALIBR: u16 = 0x0094;
}

/// DMP packet header bits
///
/// The DMP writes packets to the FIFO with a 2-byte header indicating
/// what data is present in the packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpPacketHeader;

impl DmpPacketHeader {
    /// Header bit for 6-axis quaternion
    pub const QUAT6_BIT: u16 = 0x0001;

    /// Header bit for 9-axis quaternion
    pub const QUAT9_BIT: u16 = 0x0002;

    /// Header bit for accelerometer data
    pub const ACCEL_BIT: u16 = 0x0004;

    /// Header bit for gyroscope data
    pub const GYRO_BIT: u16 = 0x0008;

    /// Header bit for calibrated gyroscope
    pub const CAL_GYRO_BIT: u16 = 0x0010;

    /// Header bit for calibrated accelerometer
    pub const CAL_ACCEL_BIT: u16 = 0x0020;

    /// Header bit for step counter
    pub const STEP_BIT: u16 = 0x0040;
}

/// DMP packet sizes (in bytes)
#[derive(Debug, Clone, Copy)]
pub struct DmpPacketSize;

impl DmpPacketSize {
    /// Size of quaternion data (4 × i32 = 16 bytes)
    pub const QUATERNION: usize = 16;

    /// Size of 3-axis data (3 × i16 = 6 bytes)
    pub const ACCEL_GYRO: usize = 6;

    /// Size of calibrated gyro data (3 × i32 = 12 bytes)
    pub const CAL_GYRO: usize = 12;

    /// Size of packet header
    pub const HEADER: usize = 2;
}

impl DmpConfig {
    /// Convert configuration to DMP feature mask
    ///
    /// This creates a 16-bit feature mask that can be written to the DMP
    /// to enable the configured features.
    ///
    /// When any feature requires accuracy reporting (quaternions, calibrated sensors),
    /// the HEADER2 bit (0x0008) is automatically added to enable accuracy data in packets.
    pub fn to_feature_mask(&self) -> u16 {
        let mut mask = 0u16;

        if self.quaternion_6axis {
            mask |= DmpFeatures::QUAT6;
        }

        if self.quaternion_9axis {
            mask |= DmpFeatures::QUAT9;
        }

        if self.game_rotation_vector {
            mask |= DmpFeatures::GAME_ROTATION_VECTOR;
        }

        if self.geomag_rotation_vector {
            mask |= DmpFeatures::GEOMAG_ROTATION_VECTOR;
        }

        if self.calibrated_accel {
            mask |= DmpFeatures::SEND_CAL_ACCEL;
        }

        if self.calibrated_gyro {
            mask |= DmpFeatures::SEND_CAL_GYRO | DmpFeatures::GYRO_CALIBRATION;
        }

        if self.raw_accel {
            mask |= DmpFeatures::SEND_RAW_ACCEL;
        }

        if self.raw_gyro {
            mask |= DmpFeatures::SEND_RAW_GYRO;
        }

        // Add HEADER2 bit when accuracy data is needed
        // This enables the accuracy reporting in DATA_OUT_CTL2
        if self.quaternion_6axis
            || self.quaternion_9axis
            || self.game_rotation_vector
            || self.geomag_rotation_vector
            || self.calibrated_accel
            || self.calibrated_gyro
            || self.calibrated_mag
        {
            mask |= DmpFeatures::HEADER2;
        }

        mask
    }

    /// Calculate expected FIFO packet size based on configuration
    ///
    /// This calculates how many bytes each DMP packet will contain based on
    /// which features are enabled. This is useful for reading the correct
    /// amount of data from the FIFO.
    pub fn packet_size(&self) -> usize {
        let mut size = DmpPacketSize::HEADER;

        // Quaternion data (only one type can be active)
        if self.quaternion_6axis
            || self.quaternion_9axis
            || self.game_rotation_vector
            || self.geomag_rotation_vector
        {
            size += DmpPacketSize::QUATERNION;
        }

        // Raw sensor data
        if self.raw_accel {
            size += DmpPacketSize::ACCEL_GYRO;
        }

        if self.raw_gyro {
            size += DmpPacketSize::ACCEL_GYRO;
        }

        // Calibrated sensor data
        if self.calibrated_accel {
            size += DmpPacketSize::ACCEL_GYRO;
        }

        if self.calibrated_gyro {
            size += DmpPacketSize::CAL_GYRO;
        }

        size
    }

    /// Calculate DMP sample rate divider
    ///
    /// The DMP sample rate is derived from the gyroscope sample rate divided
    /// by (1 + divider). For a gyro rate of 1125 Hz, the divider is:
    ///
    /// divider = (1125 / `desired_rate`) - 1
    ///
    /// Returns the divider value to write to the DMP.
    pub fn sample_rate_divider(&self) -> u16 {
        const GYRO_RATE: u16 = 1125; // Hz

        if self.sample_rate == 0 || self.sample_rate > GYRO_RATE {
            return 0; // Invalid, default to maximum rate
        }

        (GYRO_RATE / self.sample_rate).saturating_sub(1)
    }
}

/// Configuration sequence builder
///
/// This struct helps build the sequence of register/memory writes needed
/// to configure the DMP. Based on InvenSense reference implementation.
pub struct ConfigSequence {
    /// Feature mask to enable
    pub feature_mask: u16,

    /// Sample rate divider
    pub rate_divider: u16,

    /// FIFO watermark (bytes before interrupt)
    pub fifo_watermark: u16,

    /// Data output control 1 (what to output to FIFO)
    pub data_out_ctl1: u16,

    /// Data output control 2 (accuracy bits)
    pub data_out_ctl2: u16,

    /// Data ready status (which sensors trigger data ready)
    pub data_rdy_status: u16,

    /// Motion event control (calibration and fusion features)
    pub motion_event_ctl: u16,

    /// Cached byte arrays for dynamic values
    feature_bytes: [u8; 2],
    data_out_ctl1_bytes: [u8; 2],
    rate_bytes: [u8; 2],
    data_out_ctl2_bytes: [u8; 2],
    data_rdy_status_bytes: [u8; 2],
    motion_event_ctl_bytes: [u8; 2],

    /// Cached byte arrays for rate-dependent parameters
    gyro_sf_bytes: [u8; 4],
    accel_only_gain_bytes: [u8; 4],
    accel_alpha_var_bytes: [u8; 4],
    accel_a_var_bytes: [u8; 4],
}

impl ConfigSequence {
    /// Create configuration sequence from `DmpConfig`
    ///
    /// The calibration parameters are automatically selected based on the sample rate.
    pub fn from_config(config: &DmpConfig) -> Self {
        let feature_mask = config.to_feature_mask();
        let rate_divider = config.sample_rate_divider();
        #[allow(clippy::cast_possible_truncation)]
        let fifo_watermark = config.packet_size() as u16;

        // Calculate DATA_OUT_CTL1 - tells DMP what to output to FIFO
        // Must include ACCEL (0x8000) and GYRO_CALIBR (0x0008) bits when those sensors are needed
        let mut data_out_ctl1 = feature_mask;

        // Add ACCEL bit (0x8000) if any feature needs accelerometer
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_accel
            || config.calibrated_accel
        {
            data_out_ctl1 |= 0x8000; // ACCEL bit
        }

        // Add GYRO_CALIBR bit (0x0008) if any feature needs gyroscope
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_gyro
            || config.calibrated_gyro
        {
            data_out_ctl1 |= 0x0008; // GYRO_CALIBR bit
        }

        // Calculate DATA_OUT_CTL2 (header2) bits based on enabled features
        let mut data_out_ctl2 = 0u16;
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_accel
            || config.calibrated_accel
        {
            data_out_ctl2 |= DmpDataOutputControl2::ACCEL_ACCURACY;
        }
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_gyro
            || config.calibrated_gyro
        {
            data_out_ctl2 |= DmpDataOutputControl2::GYRO_ACCURACY;
        }
        if config.quaternion_9axis
            || config.geomag_rotation_vector
            || config.raw_mag
            || config.calibrated_mag
        {
            data_out_ctl2 |= DmpDataOutputControl2::COMPASS_ACCURACY;
        }

        // Calculate DATA_RDY_STATUS bits - which sensors should trigger data ready
        let mut data_rdy_status = 0u16;
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_accel
            || config.calibrated_accel
        {
            data_rdy_status |= DmpDataReadyStatus::ACCEL;
        }
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_gyro
            || config.calibrated_gyro
        {
            data_rdy_status |= DmpDataReadyStatus::GYRO;
        }
        if config.quaternion_9axis
            || config.geomag_rotation_vector
            || config.raw_mag
            || config.calibrated_mag
        {
            data_rdy_status |= DmpDataReadyStatus::COMPASS;
        }

        // Calculate MOTION_EVENT_CTL bits - which calibration and fusion features to enable
        let mut motion_event_ctl = 0u16;
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_accel
            || config.calibrated_accel
        {
            motion_event_ctl |= DmpMotionEventControl::ACCEL_CALIBR;
        }
        if config.quaternion_6axis
            || config.quaternion_9axis
            || config.game_rotation_vector
            || config.geomag_rotation_vector
            || config.raw_gyro
            || config.calibrated_gyro
        {
            motion_event_ctl |= DmpMotionEventControl::GYRO_CALIBR;
        }
        if config.quaternion_9axis
            || config.geomag_rotation_vector
            || config.raw_mag
            || config.calibrated_mag
        {
            motion_event_ctl |= DmpMotionEventControl::COMPASS_CALIBR;
        }
        if config.quaternion_9axis {
            motion_event_ctl |= DmpMotionEventControl::NINE_AXIS;
        }
        if config.geomag_rotation_vector {
            motion_event_ctl |= DmpMotionEventControl::GEOMAG;
        }

        // Calculate sample-rate-dependent parameters
        let sample_rate_config = DmpSampleRate::from_hz(config.sample_rate);
        // Use PLL=0x09 as default
        // For best accuracy, read from Bank 1, reg 0x28 (TIMEBASE_CORRECTION_PLL)
        let gyro_sf = calculate_gyro_sf(sample_rate_config.gyro_sample_rate_div(), 0x09);

        Self {
            feature_mask,
            rate_divider,
            fifo_watermark,
            data_out_ctl1,
            data_out_ctl2,
            data_rdy_status,
            motion_event_ctl,
            feature_bytes: feature_mask.to_be_bytes(),
            data_out_ctl1_bytes: data_out_ctl1.to_be_bytes(),
            rate_bytes: rate_divider.to_be_bytes(),
            data_out_ctl2_bytes: data_out_ctl2.to_be_bytes(),
            data_rdy_status_bytes: data_rdy_status.to_be_bytes(),
            motion_event_ctl_bytes: motion_event_ctl.to_be_bytes(),
            gyro_sf_bytes: gyro_sf.to_be_bytes(),
            accel_only_gain_bytes: sample_rate_config.accel_only_gain().to_be_bytes(),
            accel_alpha_var_bytes: sample_rate_config.accel_alpha_var().to_be_bytes(),
            accel_a_var_bytes: sample_rate_config.accel_a_var().to_be_bytes(),
        }
    }

    /// Get all memory writes needed to fully configure the DMP
    ///
    /// This returns the complete initialization sequence. Each entry is (address, data_bytes).
    ///
    /// Without these writes, the DMP will load and enable successfully but will not
    /// generate any data in the FIFO.
    ///
    /// The configuration includes:
    /// - **Gyroscope scaling factor**: Sample rate dependent
    /// - **Accelerometer scaling factors**: Internal (2^25=1g) and output scaling
    /// - **Magnetometer mount matrix**: Aligns AK09916 axes and converts to DMP units
    /// - **Body-to-Sensor mount matrix**: Coordinate frame alignment
    /// - **Gyroscope full scale**: 2000dps (2^28)
    /// - **Accel calibration parameters**: Rate-dependent gain/variance
    /// - **Magnetometer sample rate**: Compass time buffer (69Hz)
    /// - **Feature mask and output control**: Enable selected features
    /// - **Sample rate divider**: Set DMP output rate
    /// - **Data output control 2**: Accuracy reporting (header2)
    /// - **Data ready status**: Enable sensor data ready events
    /// - **Motion event control**: Enable calibration and fusion
    ///
    /// Total: 37 memory writes
    pub fn get_init_sequence(&self) -> [InitWrite<'_>; 37] {
        [
            // Gyroscope Scaling Factor
            // Calculated from sample rate div and PLL correction (Bank 1, reg 0x28)
            InitWrite::new(DmpMemoryAddresses::GYRO_SF, &self.gyro_sf_bytes),
            // Accelerometer Scaling
            // DMP uses 2^25 = 1g internally, 4g full scale
            InitWrite::new(DmpMemoryAddresses::ACC_SCALE, &[0x04, 0x00, 0x00, 0x00]),
            // Accelerometer output scaling: 4g
            InitWrite::new(DmpMemoryAddresses::ACC_SCALE2, &[0x00, 0x04, 0x00, 0x00]),
            // Magnetometer Mount Matrix
            // Converts AK09916 data to DMP format and aligns axes
            // Multiplier: 2^30 / 6.66 = 0x09999999
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_00, &[0x09, 0x99, 0x99, 0x99]),
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_01, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_02, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_10, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_11, &[0xF6, 0x66, 0x66, 0x67]), // -0x09999999
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_12, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_20, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_21, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::CPASS_MTX_22, &[0xF6, 0x66, 0x66, 0x67]), // -0x09999999
            // Body to Sensor Mount Matrix
            // Identity matrix scaled by 2^30
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_00, &[0x40, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_01, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_02, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_10, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_11, &[0x40, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_12, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_20, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_21, &[0x00, 0x00, 0x00, 0x00]),
            InitWrite::new(DmpMemoryAddresses::B2S_MTX_22, &[0x40, 0x00, 0x00, 0x00]),
            // Gyroscope Full Scale
            // 2^28 for 2000dps
            InitWrite::new(
                DmpMemoryAddresses::GYRO_FULLSCALE,
                &[0x10, 0x00, 0x00, 0x00],
            ),
            // Accelerometer Calibration Parameters
            // Sample rate dependent values
            InitWrite::new(
                DmpMemoryAddresses::ACCEL_ONLY_GAIN,
                &self.accel_only_gain_bytes,
            ),
            InitWrite::new(
                DmpMemoryAddresses::ACCEL_ALPHA_VAR,
                &self.accel_alpha_var_bytes,
            ),
            InitWrite::new(DmpMemoryAddresses::ACCEL_A_VAR, &self.accel_a_var_bytes),
            InitWrite::new(DmpMemoryAddresses::ACCEL_CAL_RATE, &[0x00, 0x00]),
            // Magnetometer Sample Rate
            // Compass time buffer: 69Hz
            InitWrite::new(DmpMemoryAddresses::CPASS_TIME_BUFFER, &[0x00, 0x45]),
            // Feature Control
            // Data output control 1 - what to output and which sensors to enable
            InitWrite::new(DmpMemoryAddresses::DATA_OUT_CTL1, &self.data_out_ctl1_bytes),
            // Data output control 2 - accuracy reporting
            InitWrite::new(DmpMemoryAddresses::DATA_OUT_CTL2, &self.data_out_ctl2_bytes),
            // Data interrupt control - must match DATA_OUT_CTL1
            InitWrite::new(DmpMemoryAddresses::DATA_INTR_CTL, &self.data_out_ctl1_bytes),
            // Motion event control - calibration and fusion features
            InitWrite::new(
                DmpMemoryAddresses::MOTION_EVENT_CTL,
                &self.motion_event_ctl_bytes,
            ),
            // Data ready status - which sensors trigger data ready
            InitWrite::new(
                DmpMemoryAddresses::DATA_RDY_STATUS,
                &self.data_rdy_status_bytes,
            ),
            // Sample rate divider
            InitWrite::new(DmpMemoryAddresses::ODR_RATE, &self.rate_bytes),
            // ODR Registers
            // Output data rate for 9-axis quaternion (0 = use sensor rate)
            InitWrite::new(DmpOdrRegisters::QUAT9, &[0x00, 0x00]),
            // Reset ODR counter for 9-axis quaternion
            InitWrite::new(DmpOdrCounterRegisters::QUAT9, &[0x00, 0x00]),
            // Set output data rate for accelerometer (0 = use sensor rate)
            InitWrite::new(DmpOdrRegisters::ACCEL, &[0x00, 0x00]),
            // Reset ODR counter for accelerometer
            InitWrite::new(DmpOdrCounterRegisters::ACCEL, &[0x00, 0x00]),
        ]
    }
}

/// A single DMP memory write operation
#[derive(Debug, Clone, Copy)]
pub struct InitWrite<'a> {
    /// Memory address to write to
    pub address: u16,
    /// Data bytes to write
    pub data: &'a [u8],
}

impl<'a> InitWrite<'a> {
    const fn new(address: u16, data: &'a [u8]) -> Self {
        Self { address, data }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_gyro_sf() {
        // Test with validated values: 56Hz (div=19), PLL=0x09
        let gyro_sf = calculate_gyro_sf(19, 0x09);
        assert_eq!(gyro_sf, 0x276FBC37);

        // Test with PLL=0x18 (typical value, ~24 decimal)
        let gyro_sf_pll18 = calculate_gyro_sf(19, 0x18);
        // Should be close but not exact
        assert!(gyro_sf_pll18 > 0x26000000 && gyro_sf_pll18 < 0x28000000);
    }

    #[test]
    fn test_dmp_sample_rate_params() {
        // Test 56Hz values match Cybergear
        let rate = DmpSampleRate::Hz56;
        assert_eq!(rate.gyro_sample_rate_div(), 19);
        assert_eq!(rate.accel_only_gain(), 0x03A49249);
        assert_eq!(rate.accel_alpha_var(), 0x34924925);
        assert_eq!(rate.accel_a_var(), 0x0B6DB6DB);

        // Test 112Hz values
        let rate = DmpSampleRate::Hz112;
        assert_eq!(rate.gyro_sample_rate_div(), 9);
        assert_eq!(rate.accel_only_gain(), 0x01D1745D);

        // Test 225Hz values
        let rate = DmpSampleRate::Hz225;
        assert_eq!(rate.gyro_sample_rate_div(), 4);
        assert_eq!(rate.accel_only_gain(), 0x00E8BA2E);
    }

    #[test]
    fn test_dmp_sample_rate_from_hz() {
        assert_eq!(DmpSampleRate::from_hz(50), DmpSampleRate::Hz56);
        assert_eq!(DmpSampleRate::from_hz(56), DmpSampleRate::Hz56);
        assert_eq!(DmpSampleRate::from_hz(100), DmpSampleRate::Hz112);
        assert_eq!(DmpSampleRate::from_hz(112), DmpSampleRate::Hz112);
        assert_eq!(DmpSampleRate::from_hz(200), DmpSampleRate::Hz225);
        assert_eq!(DmpSampleRate::from_hz(225), DmpSampleRate::Hz225);
    }

    #[test]
    fn test_feature_mask_6axis() {
        let config = DmpConfig::new().with_quaternion_6axis(true);
        let mask = config.to_feature_mask();
        // HEADER2 bit (0x0008) is automatically added for accuracy reporting
        assert_eq!(mask, DmpFeatures::QUAT6 | DmpFeatures::HEADER2);
    }

    #[test]
    fn test_feature_mask_9axis() {
        let config = DmpConfig::new().with_quaternion_9axis(true);
        let mask = config.to_feature_mask();
        // HEADER2 bit (0x0008) is automatically added for accuracy reporting
        assert_eq!(mask, DmpFeatures::QUAT9 | DmpFeatures::HEADER2);
    }

    #[test]
    fn test_feature_mask_multiple() {
        let config = DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_calibrated_gyro(true);
        let mask = config.to_feature_mask();
        assert!(mask & DmpFeatures::QUAT6 != 0);
        assert!(mask & DmpFeatures::SEND_CAL_GYRO != 0);
        assert!(mask & DmpFeatures::HEADER2 != 0);
        // GYRO_CALIBRATION is 0x0000 - it's enabled via MOTION_EVENT_CTL, not feature mask
    }

    #[test]
    fn test_packet_size_quat_only() {
        let config = DmpConfig::new().with_quaternion_6axis(true);
        let size = config.packet_size();
        assert_eq!(size, DmpPacketSize::HEADER + DmpPacketSize::QUATERNION);
    }

    #[test]
    fn test_packet_size_with_sensors() {
        let config = DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_calibrated_accel(true)
            .with_calibrated_gyro(true);
        let size = config.packet_size();
        let expected = DmpPacketSize::HEADER
            + DmpPacketSize::QUATERNION
            + DmpPacketSize::ACCEL_GYRO  // calibrated_accel
            + DmpPacketSize::CAL_GYRO; // calibrated_gyro (12 bytes)
        assert_eq!(size, expected);
    }

    #[test]
    fn test_sample_rate_divider_100hz() {
        let config = DmpConfig::new().with_sample_rate(100);
        let divider = config.sample_rate_divider();
        // 1125 / 100 - 1 = 11.25 - 1 = 10.25 → 10
        assert_eq!(divider, 10);
    }

    #[test]
    fn test_sample_rate_divider_225hz() {
        let config = DmpConfig::new().with_sample_rate(225);
        let divider = config.sample_rate_divider();
        // 1125 / 225 - 1 = 5 - 1 = 4
        assert_eq!(divider, 4);
    }

    #[test]
    fn test_sample_rate_divider_invalid() {
        let config = DmpConfig::new().with_sample_rate(0);
        let divider = config.sample_rate_divider();
        assert_eq!(divider, 0); // Invalid rate returns 0
    }

    #[test]
    fn test_config_sequence() {
        let config = DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_sample_rate(100);

        let seq = ConfigSequence::from_config(&config);

        // HEADER2 bit (0x0008) is automatically added for accuracy reporting
        assert_eq!(seq.feature_mask, DmpFeatures::QUAT6 | DmpFeatures::HEADER2);
        assert_eq!(seq.rate_divider, 10);
        assert!(seq.fifo_watermark > 0);
    }

    #[test]
    fn test_arbitrary_sample_rate_interpolate_75hz() {
        // Test interpolation for 75 Hz (between 56 and 112 Hz)
        let rate = ArbitrarySampleRate::interpolate(75).unwrap();

        // 1100 / 75 ≈ 14.67, so div should be 13 or 14
        assert!(rate.gyro_div == 13 || rate.gyro_div == 14);

        // Parameters should be between 56Hz and 112Hz values
        assert!(rate.accel_only_gain > 0x01D1745D); // > 112Hz value
        assert!(rate.accel_only_gain < 0x03A49249); // < 56Hz value

        assert!(rate.accel_alpha_var > 0x34924925); // > 56Hz value
        assert!(rate.accel_alpha_var < 0x3A492492); // < 112Hz value
    }

    #[test]
    fn test_arbitrary_sample_rate_interpolate_150hz() {
        // Test interpolation for 150 Hz (between 112 and 225 Hz)
        let rate = ArbitrarySampleRate::interpolate(150).unwrap();

        // 1100 / 150 ≈ 7.33, so div should be 6 or 7
        assert!(rate.gyro_div >= 6 && rate.gyro_div <= 7);

        // Parameters should be between 112Hz and 225Hz values
        assert!(rate.accel_only_gain > 0x00E8BA2E); // > 225Hz value
        assert!(rate.accel_only_gain < 0x01D1745D); // < 112Hz value
    }

    #[test]
    fn test_arbitrary_sample_rate_exact_match_returns_none() {
        // Exact matches should return None (use DmpSampleRate enum instead)
        assert!(ArbitrarySampleRate::interpolate(55).is_none()); // 56Hz rounded
        assert!(ArbitrarySampleRate::interpolate(110).is_none()); // 112Hz rounded
        assert!(ArbitrarySampleRate::interpolate(220).is_none()); // 225Hz rounded
    }

    #[test]
    fn test_arbitrary_sample_rate_clamping() {
        // Test that very low rates are clamped and interpolated
        let rate_low = ArbitrarySampleRate::interpolate(1).unwrap();
        assert!(rate_low.gyro_div > 0); // Should be clamped to minimum 4 Hz

        // Test that very high rates are clamped
        let rate_high = ArbitrarySampleRate::interpolate(1000).unwrap();
        assert!(rate_high.gyro_div < 255); // Should be clamped to max ~550 Hz
    }

    #[test]
    fn test_arbitrary_sample_rate_extrapolation_low() {
        // Test extrapolation below 56 Hz
        let rate = ArbitrarySampleRate::interpolate(30).unwrap();

        // Should extrapolate using 56-112 slope
        // Lower rates generally have larger calibration values
        // The exact value depends on the extrapolation, so just verify it's reasonable
        assert!(rate.accel_only_gain > 0); // Should have a valid value
        assert!(rate.gyro_div > 19); // Should be larger divider for lower rate
    }

    #[test]
    fn test_arbitrary_sample_rate_extrapolation_high() {
        // Test extrapolation above 225 Hz
        let rate = ArbitrarySampleRate::interpolate(300).unwrap();

        // Should extrapolate using 112-225 slope
        // Gyro div should be small for high rate
        assert!(rate.gyro_div <= 3); // 1100 / 300 ≈ 3.67
    }
}
