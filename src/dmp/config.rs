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
//! addresses. These values are based on the `InvenSense` reference implementation and
//! `SparkFun`'s validated Arduino library.
//!
//! ## Feature Bits
//!
//! The DMP uses a 16-bit feature mask to control which outputs are enabled.
//! These are written to specific DMP memory addresses after firmware loading.

use crate::dmp::DmpConfig;

/// Calculate `GYRO_SF` (Gyro Scaling Factor) for DMP
///
/// This value depends on the gyro sample rate divider and the PLL correction value.
/// The PLL value should be read from Bank 1, Register 0x28 (`TIMEBASE_CORRECTION_PLL`).
///
/// # Arguments
///
/// * `gyro_sample_rate_div` - Value from `GYRO_SMPLRT_DIV` register (0-255)
///   - 0 = 1125 Hz, 1 = 562.5 Hz, 4 = 225 Hz, 9 = 112 Hz, 19 = 55 Hz
/// * `pll_correction` - Value from `TIMEBASE_CORRECTION_PLL` register (typically 0x18)
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
    const MAGIC_CONSTANT: u64 = 264_446_880_937_391;
    const MAGIC_CONSTANT_SCALE: u64 = 100_000;
    const GYRO_LEVEL: u8 = 4; // Always 4 regardless of FSR

    let div = u64::from(gyro_sample_rate_div);
    let pll = i16::from(pll_correction); // Sign-extend to i16

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

    /// Get the accel sample rate divider for this rate
    ///
    /// Formula: ODR = 1125 Hz / (1 + divider)
    /// Note: Returns a 16-bit value because the Accel divider is 12-bit
    /// split across `ACCEL_SMPLRT_DIV_1` and `ACCEL_SMPLRT_DIV_2`.
    pub const fn accel_sample_rate_div(&self) -> u16 {
        match self {
            Self::Hz56 => 19, // 1125 / 20 = 56.25 Hz
            Self::Hz112 => 9, // 1125 / 10 = 112.5 Hz
            Self::Hz225 => 4, // 1125 / 5 = 225 Hz
        }
    }

    /// Get `ACCEL_ONLY_GAIN` parameter for this rate
    ///
    /// Validated values:
    /// - 56Hz: 0x03A49249
    /// - 112Hz: 0x01D1745D
    /// - 225Hz: 0x00E8BA2E
    pub const fn accel_only_gain(&self) -> u32 {
        match self {
            Self::Hz56 => 0x03A4_9249,
            Self::Hz112 => 0x01D1_745D,
            Self::Hz225 => 0x00E8_BA2E,
        }
    }

    /// Get `ACCEL_ALPHA_VAR` parameter for this rate
    ///
    /// Validated values:
    /// - 56Hz: 0x34924925
    /// - 112Hz: 0x3A492492
    /// - 225Hz: 0x3D27D27D
    pub const fn accel_alpha_var(&self) -> u32 {
        match self {
            Self::Hz56 => 0x3492_4925,
            Self::Hz112 => 0x3A49_2492,
            Self::Hz225 => 0x3D27_D27D,
        }
    }

    /// Get `ACCEL_A_VAR` parameter for this rate
    ///
    /// Validated values:
    /// - 56Hz: 0x0B6DB6DB
    /// - 112Hz: 0x05B6DB6E
    /// - 225Hz: 0x02D82D83
    pub const fn accel_a_var(&self) -> u32 {
        match self {
            Self::Hz56 => 0x0B6D_B6DB,
            Self::Hz112 => 0x05B6_DB6E,
            Self::Hz225 => 0x02D8_2D83,
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
            ((1100 / u32::from(hz)).saturating_sub(1)).min(255) as u8
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

        let lower_hz = 1100.0 / (1.0 + f32::from(lower_rate.gyro_sample_rate_div()));
        let upper_hz = 1100.0 / (1.0 + f32::from(upper_rate.gyro_sample_rate_div()));
        let hz_f = f32::from(hz);

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
    #[allow(clippy::cast_precision_loss)]
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

bitflags::bitflags! {
    /// DMP Data Output Control 1 (DATA_OUT_CTL1) bit masks
    ///
    /// These bits control what sensor data and fusion outputs are directly written into the FIFO.
    /// Do NOT enable base sensor bits here (like ACCEL) just because a fusion algorithm (like QUAT6)
    /// needs them; only enable them if you explicitly want the raw data in the FIFO packet.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DmpControl1Flags: u16 {
        /// 16-bit accelerometer output (bit 15)
        const ACCEL              = 0x8000;
        /// 16-bit gyroscope output (bit 14)
        const GYRO               = 0x4000;
        /// 16-bit compass (magnetometer) output (bit 13)
        const COMPASS            = 0x2000;
        /// 16-bit ALS (Ambient Light Sensor) (bit 12)
        const ALS                = 0x1000;
        /// 32-bit 6-axis quaternion (accel + gyro) (bit 11)
        const QUAT6              = 0x0800;
        /// 32-bit 9-axis quaternion + heading accuracy (bit 10)
        const QUAT9              = 0x0400;
        /// 6-axis pedometer quaternion output (bit 9)
        const PQUAT6             = 0x0200;
        /// 32-bit Geomag rotation vector + heading accuracy (bit 8)
        const GEOMAG             = 0x0100;
        /// 16-bit Pressure (bit 7)
        const PRESSURE           = 0x0080;
        /// 32-bit calibrated gyroscope (bit 6)
        const CALIBRATED_GYRO    = 0x0040;
        /// 32-bit calibrated compass (magnetometer) (bit 5)
        const CALIBRATED_COMPASS = 0x0020;
        /// Pedometer Step Detector (bit 4)
        const STEP_DETECTOR      = 0x0010;
        /// Header 2 enable (required for accuracy reporting and pedometer) (bit 3)
        const HEADER2            = 0x0008;
    }

    /// DMP Data Output Control 2 (DATA_OUT_CTL2) bit masks
    ///
    /// These bits control accuracy reporting in the DMP output packets (header2).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DmpControl2Flags: u16 {
        /// Accelerometer accuracy bit
        const ACCEL_ACCURACY   = 0x4000;
        /// Gyroscope accuracy bit
        const GYRO_ACCURACY    = 0x2000;
        /// Compass (magnetometer) accuracy bit
        const COMPASS_ACCURACY = 0x1000;
    }

    /// DMP Data Ready Status (DATA_RDY_STATUS) bit masks
    ///
    /// These bits control which physical sensors trigger data ready events for the DMP to process.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DmpDataReadyStatus: u16 {
        /// Gyroscope data ready
        const GYRO    = 0x0001;
        /// Accelerometer data ready
        const ACCEL   = 0x0002;
        /// Compass (magnetometer) data ready
        const COMPASS = 0x0008;
    }

    /// DMP Motion Event Control (MOTION_EVENT_CTL) bit masks
    ///
    /// These bits control which calibration and sensor fusion engines are enabled internally.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DmpMotionEventControl: u16 {
        /// Geomagnetic rotation vector enable
        const GEOMAG              = 0x0008;
        /// 9-axis sensor fusion enable
        const NINE_AXIS           = 0x0040;
        /// Compass (magnetometer) calibration enable
        const COMPASS_CALIBR      = 0x0080;
        /// Gyroscope calibration enable
        const GYRO_CALIBR         = 0x0100;
        /// Accelerometer calibration enable
        const ACCEL_CALIBR        = 0x0200;
        /// Pedometer interrupt enable
        const PEDOMETER_INTERRUPT = 0x2000;
    }


    /// Logical DMP Features
    ///
    /// This strictly aligns with the boolean configuration fields in `DmpConfig`.
    /// It acts as the single source of truth for the user's intended configuration,
    /// providing a bridge between high-level functional requests and low-level
    /// hardware register settings.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DmpFeatures: u32 {
        /// Enable 6-axis quaternion output (accel + gyro)
        const QUATERNION_6AXIS       = 1 << 0;
        /// Enable PQuat6 Pedometer Quaternion (accel + gyro)
        const QUATERNION_P6AXIS       = 1 << 1;
        /// Enable 9-axis quaternion output (accel + gyro + mag)
        const QUATERNION_9AXIS       = 1 << 2;
        /// Enable geomagnetic rotation vector (9-axis with heading accuracy)
        const GEOMAG_ROTATION_VECTOR = 1 << 3;
        /// Enable host-calibrated accelerometer output
        const HOST_CALIBRATED_ACCEL  = 1 << 4;
        /// Enable calibrated gyroscope output
        const CALIBRATED_GYRO        = 1 << 5;
        /// Enable calibrated magnetometer output
        const CALIBRATED_MAG         = 1 << 6;
        /// Enable raw accelerometer output from DMP
        const RAW_ACCEL              = 1 << 7;
        /// Enable raw gyroscope output from DMP
        const RAW_GYRO               = 1 << 8;
        /// Enable raw magnetometer output from DMP
        const RAW_MAG                = 1 << 9;
        /// Enable pedometer step detector (triggers an event on step)
        const STEP_DETECTOR          = 1 << 10;
        /// Enable pedometer step counter (tracks total steps)
        const STEP_COUNTER           = 1 << 11;

        // /// Enable significant motion detection (not implemented yet; missing DMP memory config + packet parsing. PRs welcome.)
        // const SIGNIFICANT_MOTION       = 1 << 12;
        // /// Enable tilt detector (not implemented yet; missing DMP memory config + packet parsing. PRs welcome.)
        // const TILT_DETECTOR            = 1 << 13;
        // /// Enable pickup/flip detector (not implemented yet; missing DMP memory config + packet parsing. PRs welcome.)
        // const PICKUP_DETECTOR          = 1 << 14;
        // /// Enable activity classification (not implemented yet; missing DMP memory config + packet parsing. PRs welcome.)
        // const ACTIVITY_CLASSIFICATION  = 1 << 15;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DmpControl1Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "DmpControl1Flags({:x})", self.bits());
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DmpControl2Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "DmpControl2Flags({:x})", self.bits());
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DmpDataReadyStatus {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "DmpDataReadyStatus({:x})", self.bits());
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DmpMotionEventControl {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "DmpMotionEventControl({:x})", self.bits());
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DmpFeatures {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "DmpFeatures({:x})", self.bits());
    }
}

impl DmpFeatures {
    /// Convert logical features to `DATA_OUT_CTL2` (Accuracy Reporting)
    pub fn as_control2(&self) -> DmpControl2Flags {
        let mut c2 = DmpControl2Flags::empty();
        if self.intersects(Self::RAW_ACCEL | Self::HOST_CALIBRATED_ACCEL) {
            c2.insert(DmpControl2Flags::ACCEL_ACCURACY);
        }

        if self.intersects(Self::RAW_GYRO | Self::CALIBRATED_GYRO) {
            c2.insert(DmpControl2Flags::GYRO_ACCURACY);
        }

        if self.intersects(
            Self::RAW_MAG
                | Self::CALIBRATED_MAG
                | Self::QUATERNION_9AXIS
                | Self::GEOMAG_ROTATION_VECTOR,
        ) {
            c2.insert(DmpControl2Flags::COMPASS_ACCURACY);
        }
        c2
    }

    /// Convert logical features to `DATA_OUT_CTL1` mask (What goes into FIFO)
    pub fn as_control1(&self) -> DmpControl1Flags {
        let mut c1 = DmpControl1Flags::empty();

        // Map logical fusion outputs
        if self.intersects(Self::QUATERNION_6AXIS) {
            c1.insert(DmpControl1Flags::QUAT6);
        }
        if self.intersects(Self::QUATERNION_P6AXIS) {
            c1.insert(DmpControl1Flags::PQUAT6);
        }
        if self.intersects(Self::QUATERNION_9AXIS) {
            c1.insert(DmpControl1Flags::QUAT9);
        }
        if self.intersects(Self::GEOMAG_ROTATION_VECTOR) {
            c1.insert(DmpControl1Flags::GEOMAG);
        }

        // Map base physical outputs
        // Note: Linear acceleration is derived from raw accel and gravity in the driver/host,
        // so we must put RAW_ACCEL into the FIFO if linear_acceleration is requested.
        if self.intersects(Self::RAW_ACCEL | Self::HOST_CALIBRATED_ACCEL) {
            c1.insert(DmpControl1Flags::ACCEL);
        }
        if self.intersects(Self::RAW_GYRO) {
            c1.insert(DmpControl1Flags::GYRO);
        }
        if self.intersects(Self::RAW_MAG | Self::CALIBRATED_MAG) {
            c1.insert(DmpControl1Flags::COMPASS);
        }

        // Calibrated outputs
        if self.intersects(Self::CALIBRATED_GYRO) {
            c1.insert(DmpControl1Flags::CALIBRATED_GYRO | DmpControl1Flags::GYRO);
        }
        if self.intersects(Self::CALIBRATED_MAG) {
            c1.insert(DmpControl1Flags::CALIBRATED_COMPASS);
        }
        // Pedometer
        if self.intersects(Self::STEP_DETECTOR | Self::STEP_COUNTER) {
            c1.insert(DmpControl1Flags::STEP_DETECTOR);
        }
        // Must include HEADER2 (0x0008) ONLY IF CTL2 is used OR pedometer is active
        if !self.as_control2().is_empty()
            || self.intersects(Self::STEP_DETECTOR | Self::STEP_COUNTER)
            || self.intersects(
                Self::QUATERNION_6AXIS | Self::QUATERNION_9AXIS | Self::GEOMAG_ROTATION_VECTOR,
            )
            || self.intersects(
                Self::RAW_ACCEL
                    | Self::RAW_GYRO
                    | Self::RAW_MAG
                    | Self::HOST_CALIBRATED_ACCEL
                    | Self::CALIBRATED_GYRO
                    | Self::CALIBRATED_MAG,
            )
        {
            c1.insert(DmpControl1Flags::HEADER2);
        }
        c1
    }

    /// Convert logical features to `DATA_RDY_STATUS` mask (Trigger sources)
    pub fn as_data_ready(&self) -> DmpDataReadyStatus {
        let mut rdy = DmpDataReadyStatus::empty();

        if self.intersects(
            Self::QUATERNION_6AXIS
                | Self::QUATERNION_9AXIS
                | Self::GEOMAG_ROTATION_VECTOR
                | Self::RAW_ACCEL
                | Self::HOST_CALIBRATED_ACCEL
                | Self::STEP_DETECTOR
                | Self::STEP_COUNTER,
        ) {
            rdy.insert(DmpDataReadyStatus::ACCEL);
        }
        if self.intersects(
            Self::QUATERNION_6AXIS
                | Self::QUATERNION_9AXIS
                | Self::GEOMAG_ROTATION_VECTOR
                | Self::RAW_GYRO
                | Self::CALIBRATED_GYRO,
        ) {
            rdy.insert(DmpDataReadyStatus::GYRO);
        }
        if self.intersects(
            Self::QUATERNION_9AXIS
                | Self::GEOMAG_ROTATION_VECTOR
                | Self::RAW_MAG
                | Self::CALIBRATED_MAG,
        ) {
            rdy.insert(DmpDataReadyStatus::COMPASS);
        }
        rdy
    }

    /// Convert logical features to `MOTION_EVENT_CTL` mask (Fusion & Calibration Engines)
    pub fn as_motion_event(&self) -> DmpMotionEventControl {
        let mut me = DmpMotionEventControl::empty();

        if self.intersects(
            Self::QUATERNION_6AXIS
                | Self::QUATERNION_9AXIS
                | Self::GEOMAG_ROTATION_VECTOR
                | Self::RAW_ACCEL
                | Self::HOST_CALIBRATED_ACCEL,
        ) {
            me.insert(DmpMotionEventControl::ACCEL_CALIBR);
        }
        if self.intersects(
            Self::QUATERNION_6AXIS
                | Self::QUATERNION_9AXIS
                | Self::GEOMAG_ROTATION_VECTOR
                | Self::RAW_GYRO
                | Self::CALIBRATED_GYRO,
        ) {
            me.insert(DmpMotionEventControl::GYRO_CALIBR);
        }
        if self.intersects(
            Self::QUATERNION_9AXIS
                | Self::GEOMAG_ROTATION_VECTOR
                | Self::RAW_MAG
                | Self::CALIBRATED_MAG,
        ) {
            me.insert(DmpMotionEventControl::COMPASS_CALIBR);
        }

        if self.intersects(Self::QUATERNION_9AXIS) {
            me.insert(DmpMotionEventControl::NINE_AXIS);
        }
        if self.intersects(Self::GEOMAG_ROTATION_VECTOR) {
            me.insert(DmpMotionEventControl::GEOMAG);
        }
        if self.intersects(Self::STEP_DETECTOR | Self::STEP_COUNTER) {
            me.insert(DmpMotionEventControl::PEDOMETER_INTERRUPT);
        }

        me
    }
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

    /// Gyroscope scaling factor (depends on sample rate and FSR)
    /// Formula: `GYRO_SF` = (`sample_rate_div` + 1) * `gyro_fsr_scale`
    pub const GYRO_SF: u16 = 0x0130;

    /// Accelerometer scaling factor 1 (for DMP internal alignment)
    /// Write 0x04000000 for 4g FSR (2^25 = 1g internally)
    pub const ACC_SCALE: u16 = 0x01E0;

    /// Accelerometer scaling factor 2 (for output)
    /// Write 0x00040000 for 4g FSR (outputs in hardware units)
    pub const ACC_SCALE2: u16 = 0x04F4;

    /// Compass (magnetometer) mount matrix element (0,0)
    pub const CPASS_MTX_00: u16 = 0x0170;
    /// Compass (magnetometer) mount matrix element (0,1)
    pub const CPASS_MTX_01: u16 = 0x0174;
    /// Compass (magnetometer) mount matrix element (0,2)
    pub const CPASS_MTX_02: u16 = 0x0178;
    /// Compass (magnetometer) mount matrix element (1,0)
    pub const CPASS_MTX_10: u16 = 0x017C;
    /// Compass (magnetometer) mount matrix element (1,1)
    pub const CPASS_MTX_11: u16 = 0x0180;
    /// Compass (magnetometer) mount matrix element (1,2)
    pub const CPASS_MTX_12: u16 = 0x0184;
    /// Compass (magnetometer) mount matrix element (2,0)
    pub const CPASS_MTX_20: u16 = 0x0188;
    /// Compass (magnetometer) mount matrix element (2,1)
    pub const CPASS_MTX_21: u16 = 0x018C;
    /// Compass (magnetometer) mount matrix element (2,2)
    pub const CPASS_MTX_22: u16 = 0x0190;

    /// Body to Sensor mount matrix element (0,0)
    pub const B2S_MTX_00: u16 = 0x0D00;
    /// Body to Sensor mount matrix element (0,1)
    pub const B2S_MTX_01: u16 = 0x0D04;
    /// Body to Sensor mount matrix element (0,2)
    pub const B2S_MTX_02: u16 = 0x0D08;
    /// Body to Sensor mount matrix element (1,0)
    pub const B2S_MTX_10: u16 = 0x0D0C;
    /// Body to Sensor mount matrix element (1,1)
    pub const B2S_MTX_11: u16 = 0x0D10;
    /// Body to Sensor mount matrix element (1,2)
    pub const B2S_MTX_12: u16 = 0x0D14;
    /// Body to Sensor mount matrix element (2,0)
    pub const B2S_MTX_20: u16 = 0x0D18;
    /// Body to Sensor mount matrix element (2,1)
    pub const B2S_MTX_21: u16 = 0x0D1C;
    /// Body to Sensor mount matrix element (2,2)
    pub const B2S_MTX_22: u16 = 0x0D20;

    /// Gyroscope full scale setting (2^28 for 2000dps)
    pub const GYRO_FULLSCALE: u16 = 0x048C;

    /// Accel only gain
    pub const ACCEL_ONLY_GAIN: u16 = 0x010C;

    /// Accel alpha variance
    pub const ACCEL_ALPHA_VAR: u16 = 0x05B0;

    /// Accel A variance
    pub const ACCEL_A_VAR: u16 = 0x05C0;

    /// Accel calibration rate
    pub const ACCEL_CAL_RATE: u16 = 0x05E4;

    /// Compass time buffer (magnetometer sample rate)
    pub const CPASS_TIME_BUFFER: u16 = 0x070E;
}

/// DMP Output Data Rate (ODR) register addresses
///
/// These registers control the output rate for each DMP feature
/// The value to write is calculated as: Value = (DMP running rate (225Hz) / desired ODR) - 1.
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

    /// ODR for Ambient Light Sensor (ALS)
    pub const ALS: u16 = 0x00B2;

    /// ODR for 6-axis pedometer quaternion
    pub const PQUAT6: u16 = 0x00A4;

    /// ODR for Geomagnetic Rotation Vector
    pub const GEOMAG: u16 = 0x00A0;

    /// ODR for pressure sensor
    pub const PRESSURE: u16 = 0x00BC;
}

/// DMP Output Data Rate Counter (`ODR_CNTR`) register addresses
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

    /// ODR counter for Ambient Light Sensor (ALS)
    pub const ALS: u16 = 0x0092;

    /// ODR counter for 6-axis pedometer quaternion
    pub const PQUAT6: u16 = 0x0084;

    /// ODR counter for Geomagnetic Rotation Vector
    pub const GEOMAG: u16 = 0x0080;

    /// ODR counter for pressure sensor
    pub const PRESSURE: u16 = 0x009C;
}

/// DMP packet header bits
///
/// The DMP writes packets to the FIFO with a 2-byte header indicating
/// what data is present in the packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpPacketHeader;

impl DmpPacketHeader {
    /// Header bit for secondary header (Header 2)
    pub const HEADER2_BIT: u16 = 0x0008;
    /// Header bit for pedometer step detector
    pub const STEP_BIT: u16 = 0x0010;
    /// Header bit for calibrated compass (magnetometer)
    pub const COMPASS_CAL_BIT: u16 = 0x0020;
    /// Header bit for calibrated gyroscope
    pub const GYRO_CAL_BIT: u16 = 0x0040;
    /// Header bit for pressure sensor data
    pub const PRESSURE_BIT: u16 = 0x0080;
    /// Header bit for geomagnetic rotation vector
    pub const GEOMAG_BIT: u16 = 0x0100;
    /// Header bit for 6-axis pedometer quaternion
    pub const PQUAT6_BIT: u16 = 0x0200;
    /// Header bit for 9-axis quaternion
    pub const QUAT9_BIT: u16 = 0x0400;
    /// Header bit for 6-axis quaternion
    pub const QUAT6_BIT: u16 = 0x0800;
    /// Header bit for ALS (Ambient Light Sensor)
    pub const ALS_BIT: u16 = 0x1000;
    /// Header bit for raw compass (magnetometer) data
    pub const COMPASS_BIT: u16 = 0x2000;
    /// Header bit for raw gyroscope data
    pub const GYRO_BIT: u16 = 0x4000;
    /// Header bit for raw accelerometer data
    pub const ACCEL_BIT: u16 = 0x8000;
}

/// DMP packet header 2 bits
///
/// The secondary header indicates accuracy and additional event data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpPacketHeader2;

impl DmpPacketHeader2 {
    /// Header 2 bit for secondary on/off
    pub const SECONDARY_ON_OFF_BIT: u16 = 0x0040;
    /// Header 2 bit for activity recognition
    pub const ACTIVITY_RECOG_BIT: u16 = 0x0080;
    /// Header 2 bit for pickup detection
    pub const PICKUP_BIT: u16 = 0x0400;
    /// Header 2 bit for FSYNC detection
    pub const FSYNC_BIT: u16 = 0x0800;
    /// Header 2 bit for compass accuracy
    pub const COMPASS_ACCURACY_BIT: u16 = 0x1000;
    /// Header 2 bit for gyroscope accuracy
    pub const GYRO_ACCURACY_BIT: u16 = 0x2000;
    /// Header 2 bit for accelerometer accuracy
    pub const ACCEL_ACCURACY_BIT: u16 = 0x4000;
}

/// DMP packet sizes (in bytes)
#[derive(Debug, Clone, Copy)]
pub struct DmpPacketSize;

impl DmpPacketSize {
    /// Size of packet header
    pub const HEADER: usize = 2;

    /// Size of packet secondary header
    pub const HEADER2: usize = 2;

    /// Every packet ends with a 2-byte footer
    pub const FOOTER: usize = 2;

    /// Size of 3-axis data (3 × i16 = 6 bytes)
    pub const ACCEL_COMPASS: usize = 6;

    /// Size of raw gyro (3 × i16 data + 3 × i16 bias = 12 bytes)
    pub const RAW_GYRO: usize = 12;

    /// Size of 6-axis quaternion data (3 × i32 = 12 bytes)
    pub const QUAT6: usize = 12;

    /// Size of 9-axis quaternion data (3 × i32 + 1 × u16 accuracy = 14 bytes)
    pub const QUAT9: usize = 14;

    /// Size of calibrated gyro data (3 × i32 = 12 bytes)
    pub const CAL_GYRO: usize = 12;

    /// Size of calibrated compass data (3 × i32 = 12 bytes)
    pub const CAL_COMPASS: usize = 12;

    /// Size of pedometer data (4 bytes)
    pub const PEDOMETER: usize = 4;

    /// Size of accelerometer accuracy data (1 × u16 = 2 bytes)
    pub const ACCEL_ACCURACY: usize = 2;

    /// Size of gyroscope accuracy data (1 × u16 = 2 bytes)
    pub const GYRO_ACCURACY: usize = 2;

    /// Size of compass (magnetometer) accuracy data (1 × u16 = 2 bytes)
    pub const COMPASS_ACCURACY: usize = 2;

    /// Size of Ambient Light Sensor data (8 bytes)
    pub const ALS: usize = 8;

    /// Size of 6-axis Pedometer Quaternion data (6 bytes)
    pub const PQUAT6: usize = 6;

    /// Size of Pressure sensor data (6 bytes)
    pub const PRESSURE: usize = 6;

    /// Absolute maximum possible packet size for buffer allocation
    pub const MAX_PACKET_SIZE: usize = 128;
}

impl DmpConfig {
    /// Consolidate user configuration into logical DMP features
    pub fn get_active_features(&self) -> DmpFeatures {
        let mut f = DmpFeatures::empty();

        let quat9_enabled = self.quaternion_9axis;
        let mut geomag_enabled = self.geomag_rotation_vector;
        if quat9_enabled && geomag_enabled {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "DmpConfig Conflict: Both QUATERNION_9AXIS and GEOMAG_ROTATION_VECTOR enabled. They are mutually exclusive. GEOMAG will be ignored."
            );
            geomag_enabled = false;
        }

        if self.host_calibrated_accel {
            #[cfg(feature = "defmt")]
            defmt::info!(
                "DmpConfig Conflict: DMP only outputs Raw Accel + Accuracy. Host handles calibration."
            );
        }

        if quat9_enabled {
            f.insert(DmpFeatures::QUATERNION_9AXIS);
        } else if geomag_enabled {
            f.insert(DmpFeatures::GEOMAG_ROTATION_VECTOR);
        }

        if self.quaternion_6axis {
            f.insert(DmpFeatures::QUATERNION_6AXIS);
        }
        if self.quaternion_p6axis {
            f.insert(DmpFeatures::QUATERNION_P6AXIS);
        }
        if self.host_calibrated_accel {
            f.insert(DmpFeatures::HOST_CALIBRATED_ACCEL);
        }
        if self.calibrated_gyro {
            f.insert(DmpFeatures::CALIBRATED_GYRO);
        }
        if self.calibrated_mag {
            f.insert(DmpFeatures::CALIBRATED_MAG);
        }
        if self.raw_accel {
            f.insert(DmpFeatures::RAW_ACCEL);
        }
        if self.raw_gyro {
            f.insert(DmpFeatures::RAW_GYRO);
        }
        if self.raw_mag {
            f.insert(DmpFeatures::RAW_MAG);
        }
        if self.step_detector {
            f.insert(DmpFeatures::STEP_DETECTOR);
        }
        if self.step_counter {
            f.insert(DmpFeatures::STEP_COUNTER);
        }

        // if self.significant_motion {
        //     f.insert(DmpFeatures::SIGNIFICANT_MOTION);
        // }
        // if self.tilt_detector {
        //     f.insert(DmpFeatures::TILT_DETECTOR);
        // }
        // if self.pickup_detector {
        //     f.insert(DmpFeatures::PICKUP_DETECTOR);
        // }
        // if self.activity_classification {
        //     f.insert(DmpFeatures::ACTIVITY_CLASSIFICATION);
        // }

        f
    }

    /// Calculate expected FIFO packet size based on configuration
    ///
    /// This calculates how many bytes each DMP packet will contain by strictly parsing
    /// the generated `DATA_OUT_CTL1` mask, ensuring total alignment with the hardware.
    pub fn packet_size(&self) -> usize {
        // Base packet size: Header + Footer
        let mut size = DmpPacketSize::HEADER + DmpPacketSize::FOOTER;

        let features = self.get_active_features();
        let c1 = features.as_control1();
        let c2 = features.as_control2();

        // Count HEADER2 if specifically enabled in CTL1
        if c1.contains(DmpControl1Flags::HEADER2) {
            size += DmpPacketSize::HEADER2;

            if c2.contains(DmpControl2Flags::ACCEL_ACCURACY) {
                size += DmpPacketSize::ACCEL_ACCURACY;
            }
            if c2.contains(DmpControl2Flags::GYRO_ACCURACY) {
                size += DmpPacketSize::GYRO_ACCURACY;
            }
            if c2.contains(DmpControl2Flags::COMPASS_ACCURACY) {
                size += DmpPacketSize::COMPASS_ACCURACY;
            }
        }

        if c1.contains(DmpControl1Flags::ACCEL) {
            size += DmpPacketSize::ACCEL_COMPASS;
        }
        if c1.contains(DmpControl1Flags::GYRO) {
            size += DmpPacketSize::RAW_GYRO;
        }
        if c1.contains(DmpControl1Flags::COMPASS) {
            size += DmpPacketSize::ACCEL_COMPASS;
        }
        if c1.contains(DmpControl1Flags::QUAT6) {
            size += DmpPacketSize::QUAT6;
        }
        if c1.contains(DmpControl1Flags::QUAT9) || c1.contains(DmpControl1Flags::GEOMAG) {
            size += DmpPacketSize::QUAT9;
        }
        if c1.contains(DmpControl1Flags::CALIBRATED_GYRO) {
            size += DmpPacketSize::CAL_GYRO;
        }
        if c1.contains(DmpControl1Flags::CALIBRATED_COMPASS) {
            size += DmpPacketSize::CAL_COMPASS;
        }
        if c1.contains(DmpControl1Flags::STEP_DETECTOR) {
            size += DmpPacketSize::PEDOMETER;
        }
        if c1.contains(DmpControl1Flags::ALS) {
            size += DmpPacketSize::ALS;
        }
        if c1.contains(DmpControl1Flags::PQUAT6) {
            size += DmpPacketSize::PQUAT6;
        }
        if c1.contains(DmpControl1Flags::PRESSURE) {
            size += DmpPacketSize::PRESSURE;
        }
        size
    }

    /// Calculate DMP sample rate divider
    ///
    /// The DMP sample rate is derived from the gyroscope sample rate divided
    /// by (1 + divider). For a gyro rate of 225 Hz, the divider is:
    ///
    /// divider = (225 / `desired_rate`) - 1
    ///
    /// Returns the divider value to write to the DMP.
    pub const fn sample_rate_divider(&self) -> u16 {
        const GYRO_RATE: u16 = 225; // Hz

        if self.sample_rate == 0 || self.sample_rate > GYRO_RATE {
            return 0; // Invalid, default to maximum rate
        }

        (GYRO_RATE / self.sample_rate).saturating_sub(1)
    }
}

/// Configuration sequence builder
///
/// This struct helps build the sequence of register/memory writes needed
/// to configure the DMP. Based on `InvenSense` reference implementation.
pub struct ConfigSequence {
    /// Feature mask to enable
    pub features: DmpFeatures,

    /// Sample rate
    pub sample_rate: DmpSampleRate,

    /// Sample rate divider
    pub rate_divider: u16,

    /// FIFO watermark (bytes before interrupt)
    pub fifo_watermark: u16,

    /// Data output control 1 (what to output to FIFO)
    pub data_out_ctl1: DmpControl1Flags,

    /// Data output control 2 (accuracy bits)
    pub data_out_ctl2: DmpControl2Flags,

    /// Data ready status (which sensors trigger data ready)
    pub data_rdy_status: DmpDataReadyStatus,

    /// Motion event control (calibration and fusion features)
    pub motion_event_ctl: DmpMotionEventControl,

    /// Cached byte arrays for dynamic values
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
        Self::from_config_and_pll(config, 0x09)
    }

    /// Create configuration sequence from `DmpConfig`
    ///
    /// The calibration parameters are automatically selected based on the sample rate.
    /// - Uses `pll_correction` for optimal timing accuracy
    /// - Reads correction value from Bank 1, register 0x28 (`TIMEBASE_CORSE_CORRECTION_PLL`)
    pub fn from_config_and_pll(config: &DmpConfig, pll_correction: i8) -> Self {
        let features = config.get_active_features();
        // Calculate DATA_OUT_CTL1 - tells DMP what to output to FIFO
        // Must include ACCEL (0x8000) and HEADER2 (0x0008) bits when those sensors are needed
        let data_out_ctl1 = features.as_control1();
        // Calculate DATA_OUT_CTL2 - tells DMP what to output to FIFO
        let data_out_ctl2 = features.as_control2();
        let data_rdy_status = features.as_data_ready();
        let motion_event_ctl = features.as_motion_event();

        let rate_divider = config.sample_rate_divider();
        #[allow(clippy::cast_possible_truncation)]
        let fifo_watermark = config.packet_size() as u16;

        // Calculate sample-rate-dependent parameters
        let sample_rate_config = DmpSampleRate::from_hz(config.sample_rate);

        // Use PLL=0x09 as default
        // For best accuracy, read from Bank 1, reg 0x28 (TIMEBASE_CORRECTION_PLL)
        let gyro_sf = calculate_gyro_sf(sample_rate_config.gyro_sample_rate_div(), pll_correction);

        Self {
            features,
            sample_rate: sample_rate_config,
            rate_divider,
            fifo_watermark,
            data_out_ctl1,
            data_out_ctl2,
            data_rdy_status,
            motion_event_ctl,
            data_out_ctl1_bytes: data_out_ctl1.bits().to_be_bytes(),
            rate_bytes: rate_divider.to_be_bytes(),
            data_out_ctl2_bytes: data_out_ctl2.bits().to_be_bytes(),
            data_rdy_status_bytes: data_rdy_status.bits().to_be_bytes(),
            motion_event_ctl_bytes: motion_event_ctl.bits().to_be_bytes(),
            gyro_sf_bytes: gyro_sf.to_be_bytes(),
            accel_only_gain_bytes: sample_rate_config.accel_only_gain().to_be_bytes(),
            accel_alpha_var_bytes: sample_rate_config.accel_alpha_var().to_be_bytes(),
            accel_a_var_bytes: sample_rate_config.accel_a_var().to_be_bytes(),
        }
    }

    /// Get all memory writes needed to fully configure the DMP
    ///
    /// This returns the complete initialization sequence. Each entry is (address, `data_bytes`).
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
    #[allow(clippy::missing_const_for_fn)]
    pub fn get_init_sequence(&self) -> [InitWrite<'_>; 50] {
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
            // ODR Registers and Counters
            // Output data rate for 9-axis quaternion
            InitWrite::new(DmpOdrRegisters::QUAT9, &self.rate_bytes),
            // Reset ODR counter for 9-axis quaternion to apply the new rate
            InitWrite::new(DmpOdrCounterRegisters::QUAT9, &[0x00, 0x00]),
            // Output data rate for 6-axis quaternion
            InitWrite::new(DmpOdrRegisters::QUAT6, &self.rate_bytes),
            // Reset ODR counter for 6-axis quaternion to apply the new rate
            InitWrite::new(DmpOdrCounterRegisters::QUAT6, &[0x00, 0x00]),
            // Output data rate for accelerometer
            InitWrite::new(DmpOdrRegisters::ACCEL, &self.rate_bytes),
            // Reset ODR counter for accelerometer to apply the new rate
            InitWrite::new(DmpOdrCounterRegisters::ACCEL, &[0x00, 0x00]),
            // Output data rate for gyroscope
            InitWrite::new(DmpOdrRegisters::GYRO, &self.rate_bytes),
            // Reset ODR counter for gyroscope to apply the new rate
            InitWrite::new(DmpOdrCounterRegisters::GYRO, &[0x00, 0x00]),
            // Output data rate for compass (magnetometer)
            InitWrite::new(DmpOdrRegisters::CPASS, &self.rate_bytes),
            // Reset ODR counter for compass (magnetometer) to apply the new rate
            InitWrite::new(DmpOdrCounterRegisters::CPASS, &[0x00, 0x00]),
            // Output data rate for Calibrated Gyro
            InitWrite::new(DmpOdrRegisters::GYRO_CALIBR, &self.rate_bytes),
            InitWrite::new(DmpOdrCounterRegisters::GYRO_CALIBR, &[0x00, 0x00]),
            // Output data rate for Calibrated Compass (Mag)
            InitWrite::new(DmpOdrRegisters::CPASS_CALIBR, &self.rate_bytes),
            InitWrite::new(DmpOdrCounterRegisters::CPASS_CALIBR, &[0x00, 0x00]),
            // Output data rate for Geomagnetic Rotation Vector
            InitWrite::new(DmpOdrRegisters::GEOMAG, &self.rate_bytes),
            InitWrite::new(DmpOdrCounterRegisters::GEOMAG, &[0x00, 0x00]),
            // Output data rate for Game Rotation Vector (PQUAT6)
            InitWrite::new(DmpOdrRegisters::PQUAT6, &self.rate_bytes),
            InitWrite::new(DmpOdrCounterRegisters::PQUAT6, &[0x00, 0x00]),
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
        let features = config.get_active_features();
        // HEADER2 bit is not part of DmpFeatures; it is added later in as_control1()
        assert!(features.contains(DmpFeatures::QUATERNION_6AXIS));
    }

    #[test]
    fn test_feature_mask_9axis() {
        let config = DmpConfig::new().with_quaternion_9axis(true);
        let features = config.get_active_features();
        assert!(features.contains(DmpFeatures::QUATERNION_9AXIS));
    }

    #[test]
    fn test_feature_mask_multiple() {
        let config = DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_calibrated_gyro(true);
        let features = config.get_active_features();
        assert!(features.contains(DmpFeatures::QUATERNION_6AXIS));
        assert!(features.contains(DmpFeatures::CALIBRATED_GYRO));
        // GYRO_CALIBRATION is enabled via MOTION_EVENT_CTL, not feature mask
    }

    #[test]
    fn test_packet_size_quat_only() {
        let config = DmpConfig::new().with_quaternion_6axis(true);
        let size = config.packet_size();
        // Header (2) + QUAT6 (12) + Footer (2) = 16
        let expected = DmpPacketSize::HEADER + DmpPacketSize::QUAT6 + DmpPacketSize::FOOTER;
        assert_eq!(size, expected);
    }

    #[test]
    fn test_packet_size_with_sensors() {
        let config = DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_host_calibrated_accel(true)
            .with_calibrated_gyro(true);
        let size = config.packet_size();
        let expected = DmpPacketSize::HEADER
            + DmpPacketSize::QUAT6
            + DmpPacketSize::ACCEL_COMPASS  // raw accel (6 bytes) for host_calibrated_accel
            + DmpPacketSize::CAL_GYRO
            + DmpPacketSize::FOOTER;
        assert_eq!(size, expected);
    }

    #[test]
    fn test_sample_rate_divider_100hz() {
        let config = DmpConfig::new().with_sample_rate(100);
        let divider = config.sample_rate_divider();
        // 225 / 100 - 1 = 2 - 1 = 1 (integer division)
        assert_eq!(divider, 1);
    }

    #[test]
    fn test_sample_rate_divider_225hz() {
        let config = DmpConfig::new().with_sample_rate(225);
        let divider = config.sample_rate_divider();
        assert_eq!(divider, 0);
    }

    #[test]
    fn test_sample_rate_divider_invalid() {
        let config = DmpConfig::new().with_sample_rate(0);
        let divider = config.sample_rate_divider();
        assert_eq!(divider, 0);
    }

    #[test]
    fn test_config_sequence() {
        let config = DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_sample_rate(100);
        let seq = ConfigSequence::from_config(&config);

        assert!(seq.features.contains(DmpFeatures::QUATERNION_6AXIS));
        assert_eq!(seq.rate_divider, 1);
        assert!(seq.fifo_watermark > 0);
    }

    #[test]
    fn test_arbitrary_sample_rate_interpolate_75hz() {
        // Test interpolation for 75 Hz (between 56 and 112 Hz)
        let rate = ArbitrarySampleRate::interpolate(75).unwrap();
        // 1100 / 75 ≈ 14.67, so gyro_div should be 13 or 14
        assert!(rate.gyro_div == 13 || rate.gyro_div == 14);

        // Parameters should be between 56Hz and 112Hz values
        assert!(rate.accel_only_gain > 0x01D1745D);
        assert!(rate.accel_only_gain < 0x03A49249);
        assert!(rate.accel_alpha_var > 0x34924925);
        assert!(rate.accel_alpha_var < 0x3A492492);
    }

    #[test]
    fn test_arbitrary_sample_rate_interpolate_150hz() {
        // Test interpolation for 150 Hz (between 112 and 225 Hz)
        let rate = ArbitrarySampleRate::interpolate(150).unwrap();
        // 1100 / 150 ≈ 7.33, so gyro_div should be 6 or 7
        assert!(rate.gyro_div >= 6 && rate.gyro_div <= 7);

        // Parameters should be between 112Hz and 225Hz values
        assert!(rate.accel_only_gain > 0x00E8BA2E);
        assert!(rate.accel_only_gain < 0x01D1745D);
    }

    #[test]
    fn test_arbitrary_sample_rate_exact_match_returns_none() {
        // Exact matches should return None (use DmpSampleRate enum instead)
        assert!(ArbitrarySampleRate::interpolate(56).is_none());
        assert!(ArbitrarySampleRate::interpolate(112).is_none());
        assert!(ArbitrarySampleRate::interpolate(225).is_none());
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
        assert!(rate.accel_only_gain > 0);
        assert!(rate.gyro_div > 19); // Larger divider for lower rate
    }

    #[test]
    fn test_arbitrary_sample_rate_extrapolation_high() {
        // Test extrapolation above 225 Hz
        let rate = ArbitrarySampleRate::interpolate(300).unwrap();

        // Should extrapolate using 112-225 slope
        // Gyro div should be small for high rate
        assert!(rate.gyro_div <= 3); // 1100/300 ≈ 3.67
    }
}
