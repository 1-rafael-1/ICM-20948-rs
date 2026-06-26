//! DMP FIFO Packet Parser
//!
//! This module provides functionality for parsing DMP FIFO packets and extracting
//! quaternion and sensor data. The DMP writes data to the FIFO in a specific packet
//! format with headers indicating what data is present.
//!
//! ## Packet Format
//!
//! Each DMP packet consists of:
//! - 2-byte header (indicates which data fields are present)
//! - Variable data payload (depends on enabled features)
//!
//! ## Data Formats
//!
//! - **Quaternion**: 4 × 32-bit signed integers in Q30 fixed-point format
//! - **Raw Accel/Gyro**: 3 × 16-bit signed integers
//! - **Calibrated Gyro**: 3 × 32-bit signed integers
//!
//! ## Usage
//!
//! ```no_run
//! # use icm20948::dmp::parser::DmpParser;
//! # let fifo_data = [0u8; 32];
//! let parser = DmpParser::new();
//! if let Some((data, consumed)) = parser.parse_packet(&fifo_data) {
//!     if let Some(quat) = data.quaternion_6axis {
//!         println!("Quaternion: w={}, x={}, y={}, z={}", quat.w, quat.x, quat.y, quat.z);
//!     }
//! }
//! ```

use crate::dmp::config::{DmpPacketHeader, DmpPacketHeader2, DmpPacketSize};
use crate::dmp::{DmpData, Quaternion};

/// DMP FIFO packet parser
pub struct DmpParser;

impl DmpParser {
    /// Create a new DMP parser
    pub const fn new() -> Self {
        Self
    }

    /// Calculate the exact total packet size based on the parsed headers
    ///
    /// This allows the driver to know exactly how many bytes to read from the FIFO
    /// without relying on external configuration states, preventing over-reads.
    ///
    /// # Arguments
    ///
    /// * `header` - Primary 16-bit packet header
    /// * `header2` - Optional secondary 16-bit packet header (if `HEADER2_BIT` is set)
    ///
    /// # Returns
    ///
    /// Returns the exact size of the packet in bytes.
    pub const fn calculate_packet_size(header: u16, header2: Option<u16>) -> usize {
        // Base packet size: Header + Footer
        let mut size = DmpPacketSize::HEADER + DmpPacketSize::FOOTER;

        // Add Header 2 and its payload if present
        if header & DmpPacketHeader::HEADER2_BIT != 0 {
            size += DmpPacketSize::HEADER2;

            if let Some(h2) = header2 {
                if h2 & DmpPacketHeader2::ACCEL_ACCURACY_BIT != 0 {
                    size += DmpPacketSize::ACCEL_ACCURACY;
                }
                if h2 & DmpPacketHeader2::GYRO_ACCURACY_BIT != 0 {
                    size += DmpPacketSize::GYRO_ACCURACY;
                }
                if h2 & DmpPacketHeader2::COMPASS_ACCURACY_BIT != 0 {
                    size += DmpPacketSize::COMPASS_ACCURACY;
                }
                // Future expansion: Activity Recognition, Fsync, Pickup sizes
            }
        }

        // Add payload sizes based on primary header bits in EXACT hardware order
        if header & DmpPacketHeader::ACCEL_BIT != 0 {
            size += DmpPacketSize::ACCEL_COMPASS; // 6 bytes
        }
        if header & DmpPacketHeader::GYRO_BIT != 0 {
            size += DmpPacketSize::RAW_GYRO; // 12 bytes (Raw + Bias)
        }
        if header & DmpPacketHeader::COMPASS_BIT != 0 {
            size += DmpPacketSize::ACCEL_COMPASS; // 6 bytes
        }
        if header & DmpPacketHeader::ALS_BIT != 0 {
            size += DmpPacketSize::ALS;
        }
        if header & DmpPacketHeader::QUAT6_BIT != 0 {
            size += DmpPacketSize::QUAT6; // 12 bytes
        }
        if header & DmpPacketHeader::QUAT9_BIT != 0 {
            size += DmpPacketSize::QUAT9; // 14 bytes
        }
        if header & DmpPacketHeader::PQUAT6_BIT != 0 {
            size += DmpPacketSize::PQUAT6;
        }
        if header & DmpPacketHeader::GEOMAG_BIT != 0 {
            size += DmpPacketSize::QUAT9; // 14 bytes
        }
        if header & DmpPacketHeader::PRESSURE_BIT != 0 {
            size += DmpPacketSize::PRESSURE;
        }
        // GYRO_CAL_BIT: the DMP sets this bit as a status flag but never writes bytes to FIFO
        if header & DmpPacketHeader::COMPASS_CAL_BIT != 0 {
            size += DmpPacketSize::CAL_COMPASS; // 12 bytes
        }
        if header & DmpPacketHeader::STEP_BIT != 0 {
            size += DmpPacketSize::PEDOMETER; // 4 bytes
        }

        size
    }

    /// Parse a DMP packet from FIFO data
    ///
    /// Extracts the header and data fields from a DMP FIFO packet. Returns `None`
    /// if the packet is too short or malformed.
    ///
    /// # Arguments
    ///
    /// * `data` - Raw bytes from FIFO (must be at least `packet_size` bytes)
    ///
    /// # Returns
    ///
    /// Returns `Some((DmpData, usize))` with the parsed data and the number of
    /// bytes consumed, or `None` if parsing failed.
    /// Parse a DMP packet from FIFO data
    #[allow(clippy::too_many_lines)]
    pub fn parse_packet(&self, data: &[u8]) -> Option<(DmpData, usize)> {
        if data.len() < DmpPacketSize::HEADER {
            return None;
        }

        // Parse header (big-endian 16-bit)
        let header = u16::from_be_bytes([data[0], data[1]]);
        let mut dmp_data = DmpData::default();
        let mut offset = DmpPacketSize::HEADER;

        // Parse Header 2 if present
        let mut header2 = 0;
        if header & DmpPacketHeader::HEADER2_BIT != 0 {
            if data.len() < offset + DmpPacketSize::HEADER2 {
                return None;
            }
            header2 = u16::from_be_bytes([data[offset], data[offset + 1]]);
            offset += DmpPacketSize::HEADER2;
        }

        // 1. Parse raw accelerometer (6 bytes)
        if header & DmpPacketHeader::ACCEL_BIT != 0 {
            if data.len() < offset + DmpPacketSize::ACCEL_COMPASS {
                return None;
            }
            dmp_data.raw_accel = self.parse_accel_gyro(&data[offset..]);
            offset += DmpPacketSize::ACCEL_COMPASS;
        }

        // 2. Parse raw gyroscope & bias (12 bytes)
        if header & DmpPacketHeader::GYRO_BIT != 0 {
            if data.len() < offset + DmpPacketSize::RAW_GYRO {
                return None;
            }

            // First 6 bytes are Raw Gyro
            dmp_data.raw_gyro = self.parse_accel_gyro(&data[offset..]);

            // Next 6 bytes are Gyro Bias
            dmp_data.gyro_bias = self.parse_accel_gyro(&data[offset + 6..]);

            if let (Some(raw_gyro), Some(gyro_bias)) = (dmp_data.raw_gyro, dmp_data.gyro_bias) {
                dmp_data.calibrated_gyro = Some((
                    i32::from(raw_gyro.0) - i32::from(gyro_bias.0),
                    i32::from(raw_gyro.1) - i32::from(gyro_bias.1),
                    i32::from(raw_gyro.2) - i32::from(gyro_bias.2),
                ));
            }

            offset += DmpPacketSize::RAW_GYRO;
        }

        // 3. Parse raw compass / magnetometer (6 bytes)
        if header & DmpPacketHeader::COMPASS_BIT != 0 {
            if data.len() < offset + DmpPacketSize::ACCEL_COMPASS {
                return None;
            }
            dmp_data.raw_mag = self.parse_accel_gyro(&data[offset..]);
            offset += DmpPacketSize::ACCEL_COMPASS;
        }

        // 4. ALS (Ambient Light Sensor) - Skip 8 bytes
        if header & DmpPacketHeader::ALS_BIT != 0 {
            offset += DmpPacketSize::ALS;
        }

        // 5. Parse 6-axis quaternion (12 bytes)
        if header & DmpPacketHeader::QUAT6_BIT != 0 {
            if data.len() < offset + DmpPacketSize::QUAT6 {
                return None;
            }
            let q = self.parse_quaternion6(&data[offset..]);
            // Map to generic 6-axis quat (serves QUATERNION_6AXIS, GAME_RV, GRAVITY, LINEAR_ACCEL)
            dmp_data.quaternion_6axis = q;
            offset += DmpPacketSize::QUAT6;
        }

        // 6. Parse 9-axis quaternion (14 bytes)
        if header & DmpPacketHeader::QUAT9_BIT != 0 {
            if data.len() < offset + DmpPacketSize::QUAT9 {
                return None;
            }
            if let Some((quat, accuracy)) = self.parse_quaternion9(&data[offset..]) {
                dmp_data.quaternion_9axis = Some(quat);
                dmp_data.heading_accuracy = Some(accuracy);
            }
            offset += DmpPacketSize::QUAT9;
        }

        // 7. PQuat6 (Pedometer Quaternion)
        if header & DmpPacketHeader::PQUAT6_BIT != 0 {
            if data.len() < offset + DmpPacketSize::PQUAT6 {
                return None;
            }
            dmp_data.pedometer_quaternion = self.parse_pquat6(&data[offset..]);
            offset += DmpPacketSize::PQUAT6;
        }

        // 8. Geomagnetic Rotation Vector (14 bytes)
        if header & DmpPacketHeader::GEOMAG_BIT != 0 {
            if data.len() < offset + DmpPacketSize::QUAT9 {
                return None;
            }
            if let Some((quat, accuracy)) = self.parse_quaternion9(&data[offset..]) {
                dmp_data.geomag_rotation_vector = Some(quat);
                dmp_data.heading_accuracy = Some(accuracy);
            }
            offset += DmpPacketSize::QUAT9;
        }

        // 9. Pressure - Skip 6 bytes
        if header & DmpPacketHeader::PRESSURE_BIT != 0 {
            offset += DmpPacketSize::PRESSURE;
        }

        // 10. GYRO_CAL_BIT is a status flag only — the DMP never writes bytes for it

        // 11. Calibrated Compass (12 bytes)
        if header & DmpPacketHeader::COMPASS_CAL_BIT != 0 {
            if data.len() < offset + DmpPacketSize::CAL_COMPASS {
                return None;
            }
            dmp_data.calibrated_mag = self.parse_calibrated_gyro(&data[offset..]);
            offset += DmpPacketSize::CAL_COMPASS;
        }

        // 12. Pedometer Step Detector (4 bytes)
        if header & DmpPacketHeader::STEP_BIT != 0 {
            if data.len() < offset + DmpPacketSize::PEDOMETER {
                return None;
            }
            let timestamp = u32::from_be_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
            ]);
            dmp_data.pedometer_timestamp = Some(timestamp);
            offset += DmpPacketSize::PEDOMETER;
        }

        // --- Process Header 2 Fields (Accuracies & Events) ---
        if header2 != 0 {
            if header2 & DmpPacketHeader2::ACCEL_ACCURACY_BIT != 0 {
                if data.len() < offset + DmpPacketSize::ACCEL_ACCURACY {
                    return None;
                }
                dmp_data.accel_accuracy =
                    Some(u16::from_be_bytes([data[offset], data[offset + 1]]));
                offset += DmpPacketSize::ACCEL_ACCURACY;
            }
            if header2 & DmpPacketHeader2::GYRO_ACCURACY_BIT != 0 {
                if data.len() < offset + DmpPacketSize::GYRO_ACCURACY {
                    return None;
                }
                dmp_data.gyro_accuracy = Some(u16::from_be_bytes([data[offset], data[offset + 1]]));
                offset += DmpPacketSize::GYRO_ACCURACY;
            }
            if header2 & DmpPacketHeader2::COMPASS_ACCURACY_BIT != 0 {
                if data.len() < offset + DmpPacketSize::COMPASS_ACCURACY {
                    return None;
                }
                dmp_data.compass_accuracy =
                    Some(u16::from_be_bytes([data[offset], data[offset + 1]]));
                offset += DmpPacketSize::COMPASS_ACCURACY;
            }
        }

        // Final packet footer (Every DMP packet ends with a 2-byte Gyro count footer)
        if data.len() < offset + DmpPacketSize::FOOTER {
            return None;
        }
        offset += DmpPacketSize::FOOTER;

        Some((dmp_data, offset))
    }

    /// Parse quaternion from Q30 fixed-point format
    ///
    /// The DMP outputs quaternions as 3 × 32-bit signed integers (X, Y, Z) in Q30 format.
    /// The W (scalar) component is omitted to save FIFO space and must be calculated by the host.
    /// Q30 means the value has 30 fractional bits, so to convert to float:
    /// `float_value` = `int_value` / 2^30
    ///
    /// The W component is calculated using the property that a unit quaternion's
    /// magnitude is exactly 1.0: W = sqrt(1.0 - (X^2 + Y^2 + Z^2)).
    ///
    /// # Arguments
    ///
    /// * `data` - At least 12 bytes containing quaternion data (x, y, z)
    ///
    /// # Returns
    ///
    /// Returns `Some(Quaternion)` if parsing succeeded, `None` if data too short.
    #[allow(clippy::unused_self)]
    #[allow(clippy::cast_precision_loss)]
    pub(crate) fn parse_quaternion6(&self, data: &[u8]) -> Option<Quaternion> {
        // Convert from Q30 to float
        // Q30: 1 bit sign, 1 bit integer, 30 bits fractional
        const Q30_DIVISOR: f32 = 1_073_741_824.0; // 2^30

        if data.len() < DmpPacketSize::QUAT6 {
            return None;
        }

        // Extract 3 × 32-bit values (big-endian)
        let qx = i32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let qy = i32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        let qz = i32::from_be_bytes([data[8], data[9], data[10], data[11]]);

        let x = (qx as f32) / Q30_DIVISOR;
        let y = (qy as f32) / Q30_DIVISOR;
        let z = (qz as f32) / Q30_DIVISOR;

        let w_sq = 1.0 - (x * x + y * y + z * z);
        let w = if w_sq > 0.0 { libm::sqrtf(w_sq) } else { 0.0 };

        Some(Quaternion { w, x, y, z })
    }

    /// Parse quaternion and accuracy from Q30 fixed-point format (9-byte variant)
    ///
    /// This function parses a quaternion from the first 12 bytes of data using the same Q30
    /// conversion as [`parse_quaternion6`], then extracts an additional 2-byte accuracy value.
    /// The accuracy value is a raw unsigned 16-bit integer provided by the DMP, which may
    /// represent a confidence metric or sensor fusion quality.
    ///
    /// # Arguments
    ///
    /// * `data` - At least 14 bytes containing quaternion data (x, y, z) plus accuracy word
    ///
    /// # Returns
    ///
    /// Returns `Some((quaternion, accuracy))` if parsing succeeded, where `accuracy` is the
    /// raw 16-bit value converted to `f32`. Returns `None` if the data slice is too short.
    pub(crate) fn parse_quaternion9(&self, data: &[u8]) -> Option<(Quaternion, f32)> {
        if data.len() < DmpPacketSize::QUAT9 {
            return None;
        }
        let quat = self.parse_quaternion6(data)?;
        let accuracy_raw = u16::from_be_bytes([data[12], data[13]]);
        Some((quat, f32::from(accuracy_raw)))
    }

    /// Parse 16-bit compressed quaternion (PQUAT6)
    #[allow(clippy::unused_self)]
    pub(crate) fn parse_pquat6(&self, data: &[u8]) -> Option<Quaternion> {
        const Q14_DIVISOR: f32 = 16384.0; // 2^14

        if data.len() < 6 {
            return None;
        }

        // PQUAT6: extract 3 × 16-bit signed integers (big-endian, Q14 fixed-point format)
        let qx = i16::from_be_bytes([data[0], data[1]]);
        let qy = i16::from_be_bytes([data[2], data[3]]);
        let qz = i16::from_be_bytes([data[4], data[5]]);

        let x = f32::from(qx) / Q14_DIVISOR;
        let y = f32::from(qy) / Q14_DIVISOR;
        let z = f32::from(qz) / Q14_DIVISOR;

        let w_sq = 1.0 - (x * x + y * y + z * z);
        let w = if w_sq > 0.0 { libm::sqrtf(w_sq) } else { 0.0 };

        Some(Quaternion { w, x, y, z })
    }

    /// Parse 3-axis accelerometer or gyroscope data (16-bit)
    ///
    /// Parses 6 bytes as 3 × 16-bit signed integers (big-endian).
    ///
    /// # Arguments
    ///
    /// * `data` - At least 6 bytes containing 3-axis data (x, y, z)
    ///
    /// # Returns
    ///
    /// Returns `Some((x, y, z))` if parsing succeeded, `None` if data too short.
    #[allow(clippy::unused_self)]
    fn parse_accel_gyro(&self, data: &[u8]) -> Option<(i16, i16, i16)> {
        if data.len() < DmpPacketSize::ACCEL_COMPASS {
            return None;
        }

        let x = i16::from_be_bytes([data[0], data[1]]);
        let y = i16::from_be_bytes([data[2], data[3]]);
        let z = i16::from_be_bytes([data[4], data[5]]);

        Some((x, y, z))
    }

    /// Parse calibrated gyroscope data (32-bit)
    ///
    /// Calibrated gyroscope values are stored as 3 × 32-bit signed integers.
    ///
    /// # Arguments
    ///
    /// * `data` - At least 12 bytes containing calibrated gyro data (x, y, z)
    ///
    /// # Returns
    ///
    /// Returns `Some((x, y, z))` if parsing succeeded, `None` if data too short.
    #[allow(clippy::unused_self)]
    fn parse_calibrated_gyro(&self, data: &[u8]) -> Option<(i32, i32, i32)> {
        if data.len() < DmpPacketSize::CAL_GYRO {
            return None;
        }

        let x = i32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let y = i32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        let z = i32::from_be_bytes([data[8], data[9], data[10], data[11]]);

        Some((x, y, z))
    }

    /// Validate packet header
    ///
    /// Checks if a packet header is valid by verifying that at least one
    /// data field bit is set.
    ///
    /// # Arguments
    ///
    /// * `header` - 16-bit packet header value
    ///
    /// # Returns
    ///
    /// Returns `true` if header is valid, `false` otherwise.
    #[allow(clippy::unused_self)]
    pub const fn validate_header(&self, header: u16) -> bool {
        // Header should have at least one data bit set and no unknown bits
        let known_bits = DmpPacketHeader::HEADER2_BIT
            | DmpPacketHeader::STEP_BIT
            | DmpPacketHeader::COMPASS_CAL_BIT
            | DmpPacketHeader::GYRO_CAL_BIT
            | DmpPacketHeader::PRESSURE_BIT
            | DmpPacketHeader::GEOMAG_BIT
            | DmpPacketHeader::PQUAT6_BIT
            | DmpPacketHeader::QUAT9_BIT
            | DmpPacketHeader::QUAT6_BIT
            | DmpPacketHeader::ALS_BIT
            | DmpPacketHeader::COMPASS_BIT
            | DmpPacketHeader::GYRO_BIT
            | DmpPacketHeader::ACCEL_BIT;
        header != 0 && (header & !known_bits) == 0
    }

    /// Extract packet header from FIFO data
    ///
    /// Reads the first 2 bytes as a big-endian 16-bit header.
    ///
    /// # Arguments
    ///
    /// * `data` - At least 2 bytes from FIFO
    ///
    /// # Returns
    ///
    /// Returns `Some(header)` if data is long enough, `None` otherwise.
    pub fn extract_header(&self, data: &[u8]) -> Option<u16> {
        if data.len() < 2 {
            return None;
        }
        Some(u16::from_be_bytes([data[0], data[1]]))
    }
}

impl Default for DmpParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_quaternion6_identity() {
        let parser = DmpParser::new();

        // Q30 format: 0 = 0x00000000
        // Identity quaternion: w=1, x=0, y=0, z=0
        // parse_quaternion6 only reads X,Y,Z (12 bytes) and computes W.
        let data = [
            0x00, 0x00, 0x00, 0x00, // x = 0.0
            0x00, 0x00, 0x00, 0x00, // y = 0.0
            0x00, 0x00, 0x00, 0x00, // z = 0.0
        ];

        let quat = parser.parse_quaternion6(&data).unwrap();

        assert!((quat.w - 1.0).abs() < 0.001);
        assert!((quat.x - 0.0).abs() < 0.001);
        assert!((quat.y - 0.0).abs() < 0.001);
        assert!((quat.z - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_parse_quaternion6_half() {
        let parser = DmpParser::new();

        // Q30: 0.5 = 2^29 = 0x20000000
        let data = [
            0x20, 0x00, 0x00, 0x00, // x = 0.5
            0x20, 0x00, 0x00, 0x00, // y = 0.5
            0x20, 0x00, 0x00, 0x00, // z = 0.5
        ];

        let quat = parser.parse_quaternion6(&data).unwrap();

        // w = sqrt(1 - 3*(0.5^2)) = sqrt(1 - 0.75) = sqrt(0.25) = 0.5
        assert!((quat.w - 0.5).abs() < 0.001);
        assert!((quat.x - 0.5).abs() < 0.001);
        assert!((quat.y - 0.5).abs() < 0.001);
        assert!((quat.z - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_parse_quaternion9() {
        let parser = DmpParser::new();

        // X=0, Y=0, Z=0, accuracy = 1234
        let data = [
            0x00, 0x00, 0x00, 0x00, // x = 0
            0x00, 0x00, 0x00, 0x00, // y = 0
            0x00, 0x00, 0x00, 0x00, // z = 0
            0x04, 0xD2, // accuracy = 1234
        ];

        let (quat, acc) = parser.parse_quaternion9(&data).unwrap();
        assert!((quat.w - 1.0).abs() < 0.001);
        assert_eq!(acc, 1234.0);
    }

    #[test]
    fn test_parse_accel_gyro() {
        let parser = DmpParser::new();

        // Example: x=100, y=-200, z=300
        let data = [
            0x00, 0x64, // x = 100
            0xFF, 0x38, // y = -200
            0x01, 0x2C, // z = 300
        ];

        let (x, y, z) = parser.parse_accel_gyro(&data).unwrap();

        assert_eq!(x, 100);
        assert_eq!(y, -200);
        assert_eq!(z, 300);
    }

    #[test]
    fn test_parse_calibrated_gyro() {
        let parser = DmpParser::new();

        // Example: x=10000, y=-20000, z=30000
        let data = [
            0x00, 0x00, 0x27, 0x10, // x = 10000
            0xFF, 0xFF, 0xB1, 0xE0, // y = -20000
            0x00, 0x00, 0x75, 0x30, // z = 30000
        ];

        let (x, y, z) = parser.parse_calibrated_gyro(&data).unwrap();

        assert_eq!(x, 10000);
        assert_eq!(y, -20000);
        assert_eq!(z, 30000);
    }

    #[test]
    fn test_parse_packet_quat6() {
        let parser = DmpParser::new();

        // Packet: header (QUAT6_BIT = 0x0800) + 12-byte identity quaternion + 2-byte footer
        let data = [
            0x08, 0x00, // Header: QUAT6_BIT
            // X, Y, Z (all zero)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, // Footer (any value)
        ];

        let (dmp_data, consumed) = parser.parse_packet(&data).unwrap();
        assert_eq!(consumed, data.len());

        assert!(dmp_data.quaternion_6axis.is_some());
        let quat = dmp_data.quaternion_6axis.unwrap();
        assert!((quat.w - 1.0).abs() < 0.001);
        assert!((quat.x - 0.0).abs() < 0.001);
        assert!((quat.y - 0.0).abs() < 0.001);
        assert!((quat.z - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_parse_packet_with_accel_gyro() {
        let parser = DmpParser::new();

        // Header: QUAT6_BIT (0x0800) | ACCEL_BIT (0x8000) | GYRO_BIT (0x4000) = 0xC800
        let header =
            DmpPacketHeader::QUAT6_BIT | DmpPacketHeader::ACCEL_BIT | DmpPacketHeader::GYRO_BIT;
        let header_bytes = header.to_be_bytes();

        // Hardware packet order: ACCEL, GYRO (raw + bias), QUAT6, Footer
        // Accel: (100, 200, 300) — 6 bytes
        let accel_data = [
            0x00, 0x64, // x = 100
            0x00, 0xC8, // y = 200
            0x01, 0x2C, // z = 300
        ];
        // Gyro raw: (10, 20, 30) — 6 bytes; bias: (0, 0, 0) — 6 bytes
        // GYRO_BIT = RAW_GYRO = 12 bytes total (raw data + bias)
        let gyro_raw = [
            0x00, 0x0A, // x = 10
            0x00, 0x14, // y = 20
            0x00, 0x1E, // z = 30
        ];
        let gyro_bias = [0x00u8; 6]; // bias = (0, 0, 0)
        // Quaternion (X,Y,Z all zero) — 12 bytes
        let quat_data = [0x00u8; 12];
        // Footer — 2 bytes
        let footer = [0x00u8; 2];

        // header(2) + accel(6) + gyro_raw(6) + gyro_bias(6) + quat6(12) + footer(2) = 34 bytes
        let mut data = [0u8; 34];
        data[0..2].copy_from_slice(&header_bytes);
        data[2..8].copy_from_slice(&accel_data);
        data[8..14].copy_from_slice(&gyro_raw);
        data[14..20].copy_from_slice(&gyro_bias);
        data[20..32].copy_from_slice(&quat_data);
        data[32..34].copy_from_slice(&footer);

        let (dmp_data, consumed) = parser.parse_packet(&data).unwrap();
        assert_eq!(consumed, data.len());

        assert!(dmp_data.quaternion_6axis.is_some());
        let quat = dmp_data.quaternion_6axis.unwrap();
        assert!((quat.w - 1.0).abs() < 0.001);

        assert!(dmp_data.raw_accel.is_some());
        let (ax, ay, az) = dmp_data.raw_accel.unwrap();
        assert_eq!((ax, ay, az), (100, 200, 300));

        assert!(dmp_data.raw_gyro.is_some());
        let (gx, gy, gz) = dmp_data.raw_gyro.unwrap();
        assert_eq!((gx, gy, gz), (10, 20, 30));
    }

    #[test]
    fn test_extract_header() {
        let parser = DmpParser::new();

        let data = [0x08, 0x00, 0xFF, 0xFF];
        let header = parser.extract_header(&data).unwrap();

        assert_eq!(header, 0x0800);
    }

    #[test]
    fn test_validate_header() {
        let parser = DmpParser::new();

        assert!(parser.validate_header(0x0800)); // QUAT6_BIT alone
        assert!(parser.validate_header(0x8000)); // ACCEL_BIT alone
        assert!(!parser.validate_header(0x0000)); // empty — no data bits
        // 0xFFF8 = all 13 known bits set; bits 0-2 are not defined and must be 0
        assert!(parser.validate_header(0xFFF8));
        // 0x7FFF sets bits 0-2 (unknown) so must be rejected
        assert!(!parser.validate_header(0x7FFF));
    }

    #[test]
    fn test_parse_packet_too_short() {
        let parser = DmpParser::new();

        let data = [0x00]; // Only 1 byte
        let result = parser.parse_packet(&data);

        assert!(result.is_none());
    }

    // Nine-axis packet without COMPASS_ACCURACY in header2: consumed should be 20, not 22
    // This is the common case — compass accuracy only changes occasionally
    #[test]
    fn test_nine_axis_without_compass_accuracy_consumed() {
        use super::super::config::{DmpPacketHeader, DmpPacketSize};

        let parser = DmpParser::new();

        // header: QUAT9_BIT | HEADER2_BIT
        let header: u16 = DmpPacketHeader::QUAT9_BIT | DmpPacketHeader::HEADER2_BIT;
        // header2: no accuracy bits set this packet
        let header2: u16 = 0x0000;
        // QUAT9: 12 bytes xyz (all zero = identity) + 2 bytes accuracy
        let quat9 = [0u8; DmpPacketSize::QUAT9];
        let footer = [0u8; DmpPacketSize::FOOTER];
        // 2 "next packet" bytes that should NOT be consumed
        let next_packet_start = [0xAB, 0xCD];

        let mut data = [0u8; 22];
        data[0..2].copy_from_slice(&header.to_be_bytes());
        data[2..4].copy_from_slice(&header2.to_be_bytes());
        data[4..18].copy_from_slice(&quat9);
        data[18..20].copy_from_slice(&footer);
        data[20..22].copy_from_slice(&next_packet_start);

        let (_, consumed) = parser.parse_packet(&data).unwrap();
        assert_eq!(consumed, 20);
        // over_read = 2, within MAX_OVER_READ = 6
        assert!(22 - consumed <= DmpPacketSize::MAX_OVER_READ);
    }

    // Nine-axis packet WITH COMPASS_ACCURACY in header2: consumed should be 22
    #[test]
    fn test_nine_axis_with_compass_accuracy_consumed() {
        use super::super::config::{DmpPacketHeader, DmpPacketHeader2, DmpPacketSize};

        let parser = DmpParser::new();

        let header: u16 = DmpPacketHeader::QUAT9_BIT | DmpPacketHeader::HEADER2_BIT;
        let header2: u16 = DmpPacketHeader2::COMPASS_ACCURACY_BIT;
        let quat9 = [0u8; DmpPacketSize::QUAT9];
        let compass_acc: u16 = 3;
        let footer = [0u8; DmpPacketSize::FOOTER];

        let mut data = [0u8; 22];
        data[0..2].copy_from_slice(&header.to_be_bytes());
        data[2..4].copy_from_slice(&header2.to_be_bytes());
        data[4..18].copy_from_slice(&quat9);
        data[18..20].copy_from_slice(&compass_acc.to_be_bytes());
        data[20..22].copy_from_slice(&footer);

        let (_, consumed) = parser.parse_packet(&data).unwrap();
        assert_eq!(consumed, 22);
    }
}
