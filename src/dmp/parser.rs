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
//! if let Some(data) = parser.parse_packet(&fifo_data) {
//!     if let Some(quat) = data.quaternion_6axis {
//!         println!("Quaternion: w={}, x={}, y={}, z={}", quat.w, quat.x, quat.y, quat.z);
//!     }
//! }
//! ```

use crate::dmp::config::{DmpPacketHeader, DmpPacketSize};
use crate::dmp::{DmpData, Quaternion};

/// DMP FIFO packet parser
pub struct DmpParser;

impl DmpParser {
    /// Create a new DMP parser
    pub const fn new() -> Self {
        Self
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
    /// Returns `Some(DmpData)` with the parsed data, or `None` if parsing failed.
    pub fn parse_packet(&self, data: &[u8]) -> Option<DmpData> {
        if data.len() < DmpPacketSize::HEADER {
            return None;
        }

        // Parse header (big-endian 16-bit)
        let header = u16::from_be_bytes([data[0], data[1]]);

        let mut dmp_data = DmpData::default();
        let mut offset = DmpPacketSize::HEADER;

        // Parse quaternion data if present
        if header & DmpPacketHeader::QUAT6_BIT != 0 {
            if let Some(quat) = self.parse_quaternion(&data[offset..]) {
                dmp_data.quaternion_6axis = Some(quat);
                offset += DmpPacketSize::QUATERNION;
            }
        } else if header & DmpPacketHeader::QUAT9_BIT != 0 {
            if let Some(quat) = self.parse_quaternion(&data[offset..]) {
                dmp_data.quaternion_9axis = Some(quat);
                offset += DmpPacketSize::QUATERNION;
            }
        }

        // Parse raw accelerometer if present
        if header & DmpPacketHeader::ACCEL_BIT != 0 {
            if let Some(accel) = self.parse_accel_gyro(&data[offset..]) {
                dmp_data.raw_accel = Some(accel);
                offset += DmpPacketSize::ACCEL_GYRO;
            }
        }

        // Parse raw gyroscope if present
        if header & DmpPacketHeader::GYRO_BIT != 0 {
            if let Some(gyro) = self.parse_accel_gyro(&data[offset..]) {
                dmp_data.raw_gyro = Some(gyro);
                offset += DmpPacketSize::ACCEL_GYRO;
            }
        }

        // Parse calibrated accelerometer if present
        if header & DmpPacketHeader::CAL_ACCEL_BIT != 0 {
            if let Some(accel) = self.parse_accel_gyro(&data[offset..]) {
                dmp_data.calibrated_accel = Some(accel);
                offset += DmpPacketSize::ACCEL_GYRO;
            }
        }

        // Parse calibrated gyroscope if present (32-bit values)
        if header & DmpPacketHeader::CAL_GYRO_BIT != 0 {
            if let Some(gyro) = self.parse_calibrated_gyro(&data[offset..]) {
                dmp_data.calibrated_gyro = Some(gyro);
                // offset not used after this point
            }
        }

        Some(dmp_data)
    }

    /// Parse quaternion from Q30 fixed-point format
    ///
    /// The DMP outputs quaternions as 4 × 32-bit signed integers in Q30 format.
    /// Q30 means the value has 30 fractional bits, so to convert to float:
    /// `float_value` = `int_value` / 2^30
    ///
    /// # Arguments
    ///
    /// * `data` - At least 16 bytes containing quaternion data (w, x, y, z)
    ///
    /// # Returns
    ///
    /// Returns `Some(Quaternion)` if parsing succeeded, `None` if data too short.
    fn parse_quaternion(&self, data: &[u8]) -> Option<Quaternion> {
        if data.len() < DmpPacketSize::QUATERNION {
            return None;
        }

        // Extract 4 × 32-bit values (big-endian)
        let qw = i32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let qx = i32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        let qy = i32::from_be_bytes([data[8], data[9], data[10], data[11]]);
        let qz = i32::from_be_bytes([data[12], data[13], data[14], data[15]]);

        // Convert from Q30 to float
        // Q30: 1 bit sign, 1 bit integer, 30 bits fractional
        const Q30_DIVISOR: f32 = 1_073_741_824.0; // 2^30

        Some(Quaternion {
            w: qw as f32 / Q30_DIVISOR,
            x: qx as f32 / Q30_DIVISOR,
            y: qy as f32 / Q30_DIVISOR,
            z: qz as f32 / Q30_DIVISOR,
        })
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
    fn parse_accel_gyro(&self, data: &[u8]) -> Option<(i16, i16, i16)> {
        if data.len() < DmpPacketSize::ACCEL_GYRO {
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
    pub fn validate_header(&self, header: u16) -> bool {
        // Header should have at least one data bit set
        header != 0 && header < 0x8000 // Reasonable upper bound
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
    fn test_parse_quaternion_identity() {
        let parser = DmpParser::new();

        // Q30 format: 1.0 = 2^30 = 0x40000000
        // Identity quaternion: w=1, x=0, y=0, z=0
        let data = [
            0x40, 0x00, 0x00, 0x00, // w = 1.0
            0x00, 0x00, 0x00, 0x00, // x = 0.0
            0x00, 0x00, 0x00, 0x00, // y = 0.0
            0x00, 0x00, 0x00, 0x00, // z = 0.0
        ];

        let quat = parser.parse_quaternion(&data).unwrap();

        assert!((quat.w - 1.0).abs() < 0.001);
        assert!((quat.x - 0.0).abs() < 0.001);
        assert!((quat.y - 0.0).abs() < 0.001);
        assert!((quat.z - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_parse_quaternion_half() {
        let parser = DmpParser::new();

        // Q30 format: 0.5 = 2^29 = 0x20000000
        let data = [
            0x20, 0x00, 0x00, 0x00, // w = 0.5
            0x20, 0x00, 0x00, 0x00, // x = 0.5
            0x20, 0x00, 0x00, 0x00, // y = 0.5
            0x20, 0x00, 0x00, 0x00, // z = 0.5
        ];

        let quat = parser.parse_quaternion(&data).unwrap();

        assert!((quat.w - 0.5).abs() < 0.001);
        assert!((quat.x - 0.5).abs() < 0.001);
        assert!((quat.y - 0.5).abs() < 0.001);
        assert!((quat.z - 0.5).abs() < 0.001);
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

        // Packet: header (QUAT6_BIT) + identity quaternion
        let data = [
            0x00, 0x01, // Header: QUAT6_BIT
            0x40, 0x00, 0x00, 0x00, // w = 1.0
            0x00, 0x00, 0x00, 0x00, // x = 0.0
            0x00, 0x00, 0x00, 0x00, // y = 0.0
            0x00, 0x00, 0x00, 0x00, // z = 0.0
        ];

        let dmp_data = parser.parse_packet(&data).unwrap();

        assert!(dmp_data.quaternion_6axis.is_some());
        let quat = dmp_data.quaternion_6axis.unwrap();
        assert!((quat.w - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_parse_packet_with_accel_gyro() {
        let parser = DmpParser::new();

        // Packet: header + quaternion + accel + gyro
        let data = [
            0x00, 0x0D, // Header: QUAT6_BIT | ACCEL_BIT | GYRO_BIT
            // Quaternion (16 bytes)
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, // Accel (6 bytes)
            0x00, 0x64, 0x00, 0xC8, 0x01, 0x2C, // Gyro (6 bytes)
            0x00, 0x0A, 0x00, 0x14, 0x00, 0x1E,
        ];

        let dmp_data = parser.parse_packet(&data).unwrap();

        assert!(dmp_data.quaternion_6axis.is_some());
        assert!(dmp_data.raw_accel.is_some());
        assert!(dmp_data.raw_gyro.is_some());

        let (ax, ay, az) = dmp_data.raw_accel.unwrap();
        assert_eq!((ax, ay, az), (100, 200, 300));

        let (gx, gy, gz) = dmp_data.raw_gyro.unwrap();
        assert_eq!((gx, gy, gz), (10, 20, 30));
    }

    #[test]
    fn test_extract_header() {
        let parser = DmpParser::new();

        let data = [0x00, 0x01, 0xFF, 0xFF];
        let header = parser.extract_header(&data).unwrap();

        assert_eq!(header, 0x0001);
    }

    #[test]
    fn test_validate_header() {
        let parser = DmpParser::new();

        assert!(parser.validate_header(0x0001));
        assert!(parser.validate_header(0x00FF));
        assert!(!parser.validate_header(0x0000));
        assert!(parser.validate_header(0x7FFF));
    }

    #[test]
    fn test_parse_packet_too_short() {
        let parser = DmpParser::new();

        let data = [0x00]; // Only 1 byte
        let result = parser.parse_packet(&data);

        assert!(result.is_none());
    }
}
