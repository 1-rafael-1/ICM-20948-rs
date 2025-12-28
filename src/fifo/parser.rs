//! FIFO data parsing
//!
//! This module provides functionality to parse raw FIFO data into structured sensor readings.
//! The FIFO stores data in a specific order based on what's enabled:
//! 1. Accelerometer (6 bytes) - if enabled
//! 2. Gyroscope (6 bytes) - if any axis enabled
//! 3. Temperature (2 bytes) - if enabled
//! 4. External sensors (variable) - if I2C slaves enabled
//!
//! # Example
//!
//! ```ignore
//! # use icm20948::fifo::{parser::FifoParser, FifoConfigAdvanced, FifoRecord};
//! # let buffer = [0u8; 512];
//! # let config = FifoConfigAdvanced::default();
//! let parser = FifoParser::new(&config);
//! let records: Vec<FifoRecord> = parser.parse(&buffer)?;
//! # Ok::<(), icm20948::Error<()>>(())
//! ```

use super::{FifoConfigAdvanced, FifoRecord, calculate_record_size};
use crate::Error;

/// FIFO data parser
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoParser {
    /// Expected bytes per record
    bytes_per_record: usize,
    /// Configuration used for parsing
    config: FifoConfigAdvanced,
}

impl FifoParser {
    /// Create a new FIFO parser with the given configuration
    ///
    /// # Arguments
    /// * `config` - FIFO configuration that determines data layout
    pub const fn new(config: &FifoConfigAdvanced) -> Self {
        Self {
            bytes_per_record: calculate_record_size(config),
            config: *config,
        }
    }

    /// Get the expected bytes per record
    pub const fn bytes_per_record(&self) -> usize {
        self.bytes_per_record
    }

    /// Parse raw FIFO data into records
    ///
    /// # Arguments
    /// * `data` - Raw FIFO data bytes
    ///
    /// # Returns
    /// Vector of parsed FIFO records
    ///
    /// # Errors
    /// Returns `InvalidConfig` if data length doesn't match expected record size.
    /// Returns `FifoOverflow` if there are more than 64 records to parse.
    pub fn parse(&self, data: &[u8]) -> Result<heapless::Vec<FifoRecord, 64>, Error<()>> {
        if self.bytes_per_record == 0 {
            return Ok(heapless::Vec::new());
        }

        // Calculate number of complete records
        let num_records = data.len() / self.bytes_per_record;
        let mut records = heapless::Vec::new();

        for i in 0..num_records {
            let start = i * self.bytes_per_record;
            let end = start + self.bytes_per_record;

            if end > data.len() {
                break;
            }

            let record_data = &data[start..end];
            let record = self.parse_record(record_data)?;

            // Return error if we can't fit more records (max 64)
            if records.push(record).is_err() {
                return Err(Error::FifoOverflow);
            }
        }

        Ok(records)
    }

    /// Parse a single record from raw bytes
    ///
    /// # Arguments
    /// * `data` - Raw bytes for one complete record
    ///
    /// # Returns
    /// Parsed FIFO record
    ///
    /// # Errors
    /// Returns `InvalidConfig` if data length is incorrect
    fn parse_record(&self, data: &[u8]) -> Result<FifoRecord, Error<()>> {
        if data.len() != self.bytes_per_record {
            return Err(Error::InvalidConfig);
        }

        let mut record = FifoRecord::new();
        let mut offset = 0;

        // Parse accelerometer (always 6 bytes if enabled)
        if self.config.enable_accel {
            if offset + 6 > data.len() {
                return Err(Error::InvalidConfig);
            }
            let x = i16::from_be_bytes([data[offset], data[offset + 1]]);
            let y = i16::from_be_bytes([data[offset + 2], data[offset + 3]]);
            let z = i16::from_be_bytes([data[offset + 4], data[offset + 5]]);
            record.accel = Some((x, y, z));
            offset += 6;
        }

        // Parse gyroscope (always 6 bytes if any axis enabled)
        if self.config.enable_gyro_x || self.config.enable_gyro_y || self.config.enable_gyro_z {
            if offset + 6 > data.len() {
                return Err(Error::InvalidConfig);
            }
            let x = i16::from_be_bytes([data[offset], data[offset + 1]]);
            let y = i16::from_be_bytes([data[offset + 2], data[offset + 3]]);
            let z = i16::from_be_bytes([data[offset + 4], data[offset + 5]]);
            record.gyro = Some((x, y, z));
            offset += 6;
        }

        // Parse temperature (2 bytes if enabled)
        if self.config.enable_temp {
            if offset + 2 > data.len() {
                return Err(Error::InvalidConfig);
            }
            let temp = i16::from_be_bytes([data[offset], data[offset + 1]]);
            record.temp = Some(temp);
            offset += 2;
        }

        // Parse external sensor data (I2C slaves)
        // For now, we assume slave 0 is magnetometer (6 bytes)
        if self.config.enable_slv0 {
            if offset + 6 > data.len() {
                return Err(Error::InvalidConfig);
            }
            // Magnetometer data is typically in little-endian
            let x = i16::from_le_bytes([data[offset], data[offset + 1]]);
            let y = i16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let z = i16::from_le_bytes([data[offset + 4], data[offset + 5]]);
            record.mag = Some((x, y, z));
            // offset += 6; // Additional slaves would be parsed here if needed
        }

        // Additional slaves would be parsed here if needed
        // For now, we skip them as they're less common

        Ok(record)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::fifo::FifoMode;

    #[test]
    fn test_parser_empty_config() {
        let config = FifoConfigAdvanced::default();
        let parser = FifoParser::new(&config);
        assert_eq!(parser.bytes_per_record(), 0);

        let data = [0u8; 12];
        let records = parser.parse(&data).unwrap();
        assert_eq!(records.len(), 0);
    }

    #[test]
    fn test_parser_accel_only() {
        let config = FifoConfigAdvanced {
            enable_accel: true,
            mode: FifoMode::Stream,
            ..Default::default()
        };
        let parser = FifoParser::new(&config);
        assert_eq!(parser.bytes_per_record(), 6);

        // Create test data: one record with accel data
        let data = [
            0x01, 0x00, // X = 256
            0x02, 0x00, // Y = 512
            0x03, 0x00, // Z = 768
        ];

        let records = parser.parse(&data).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].accel, Some((256, 512, 768)));
        assert_eq!(records[0].gyro, None);
        assert_eq!(records[0].temp, None);
    }

    #[test]
    fn test_parser_accel_and_gyro() {
        let config = FifoConfigAdvanced {
            enable_accel: true,
            enable_gyro_x: true,
            enable_gyro_y: true,
            enable_gyro_z: true,
            mode: FifoMode::Stream,
            ..Default::default()
        };
        let parser = FifoParser::new(&config);
        assert_eq!(parser.bytes_per_record(), 12);

        // Create test data: one record with accel + gyro
        let data = [
            0x01, 0x00, // Accel X = 256
            0x02, 0x00, // Accel Y = 512
            0x03, 0x00, // Accel Z = 768
            0x04, 0x00, // Gyro X = 1024
            0x05, 0x00, // Gyro Y = 1280
            0x06, 0x00, // Gyro Z = 1536
        ];

        let records = parser.parse(&data).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].accel, Some((256, 512, 768)));
        assert_eq!(records[0].gyro, Some((1024, 1280, 1536)));
        assert_eq!(records[0].temp, None);
    }

    #[test]
    fn test_parser_all_sensors() {
        let config = FifoConfigAdvanced {
            enable_accel: true,
            enable_gyro_x: true,
            enable_gyro_y: true,
            enable_gyro_z: true,
            enable_temp: true,
            enable_slv0: true, // Magnetometer
            mode: FifoMode::Stream,
            ..Default::default()
        };
        let parser = FifoParser::new(&config);
        assert_eq!(parser.bytes_per_record(), 20);

        // Create test data: one complete record
        let data = [
            0x01, 0x00, // Accel X = 256
            0x02, 0x00, // Accel Y = 512
            0x03, 0x00, // Accel Z = 768
            0x04, 0x00, // Gyro X = 1024
            0x05, 0x00, // Gyro Y = 1280
            0x06, 0x00, // Gyro Z = 1536
            0x10, 0x00, // Temp = 4096
            0x07, 0x00, // Mag X = 7 (little-endian)
            0x08, 0x00, // Mag Y = 8
            0x09, 0x00, // Mag Z = 9
        ];

        let records = parser.parse(&data).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].accel, Some((256, 512, 768)));
        assert_eq!(records[0].gyro, Some((1024, 1280, 1536)));
        assert_eq!(records[0].temp, Some(4096));
        assert_eq!(records[0].mag, Some((7, 8, 9)));
    }

    #[test]
    fn test_parser_multiple_records() {
        let config = FifoConfigAdvanced {
            enable_accel: true,
            enable_gyro_x: true,
            mode: FifoMode::Stream,
            ..Default::default()
        };
        let parser = FifoParser::new(&config);

        // Two complete records
        let data = [
            // Record 1
            0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00,
            // Record 2
            0x0A, 0x00, 0x0B, 0x00, 0x0C, 0x00, 0x0D, 0x00, 0x0E, 0x00, 0x0F, 0x00,
        ];

        let records = parser.parse(&data).unwrap();
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].accel, Some((256, 512, 768)));
        assert_eq!(records[1].accel, Some((2560, 2816, 3072)));
    }

    #[test]
    fn test_parser_partial_record() {
        let config = FifoConfigAdvanced {
            enable_accel: true,
            mode: FifoMode::Stream,
            ..Default::default()
        };
        let parser = FifoParser::new(&config);

        // Incomplete record (only 4 bytes instead of 6)
        let data = [0x01, 0x00, 0x02, 0x00];

        let records = parser.parse(&data).unwrap();
        // Should return 0 records since we don't have a complete one
        assert_eq!(records.len(), 0);
    }

    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn test_parser_overflow() {
        // Test that parser returns error when trying to parse more than 64 records
        let config = FifoConfigAdvanced {
            enable_accel: true,
            mode: FifoMode::Stream,
            ..Default::default()
        };
        let parser = FifoParser::new(&config);

        // Create data for 65 records (6 bytes each = 390 bytes)
        let mut data = [0u8; 65 * 6];
        for i in 0..65 {
            let offset = i * 6;
            // Fill with recognizable pattern (index in first byte)
            data[offset] = i as u8;
            data[offset + 1] = 0;
            data[offset + 2] = 0;
            data[offset + 3] = 0;
            data[offset + 4] = 0;
            data[offset + 5] = 0;
        }

        // Should return FifoOverflow error since we can only store 64 records
        let result = parser.parse(&data);
        assert!(result.is_err(), "Should return error for >64 records");
        match result {
            Err(crate::Error::FifoOverflow) => {
                // Expected error
            }
            _ => unreachable!("Expected FifoOverflow error"),
        }
    }

    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn test_parser_exactly_64_records() {
        // Test that parser works correctly with exactly 64 records
        let config = FifoConfigAdvanced {
            enable_accel: true,
            mode: FifoMode::Stream,
            ..Default::default()
        };
        let parser = FifoParser::new(&config);

        // Create data for exactly 64 records (6 bytes each = 384 bytes)
        let mut data = [0u8; 64 * 6];
        for i in 0..64 {
            let offset = i * 6;
            // Fill with recognizable pattern (index in first byte)
            data[offset] = i as u8;
            data[offset + 1] = 0;
            data[offset + 2] = 0;
            data[offset + 3] = 0;
            data[offset + 4] = 0;
            data[offset + 5] = 0;
        }

        // Should succeed with exactly 64 records
        let records = parser.parse(&data).unwrap();
        assert_eq!(records.len(), 64, "Should parse all 64 records");
    }
}
