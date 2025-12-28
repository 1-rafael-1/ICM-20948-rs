//! DMP Firmware Image
//!
//! This module contains the Digital Motion Processor (DMP) firmware binary
//! that must be loaded into the ICM-20948's DMP memory on every power-up.
//!
//! ## Source and Licensing
//!
//! This firmware is originally provided by InvenSense (TDK) and distributed via
//! the SparkFun ICM-20948 Arduino Library under MIT license:
//! <https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary>
//!
//! ```text
//! SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library
//!
//! Copyright (c) 2016 SparkFun Electronics
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights
//! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//! copies of the Software, and to permit persons to whom the Software is
//! furnished to do so, subject to the following conditions:
//!
//! The above copyright notice and this permission notice shall be included in all
//! copies or substantial portions of the Software.
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//! SOFTWARE.
//! ```
//!
//! ## Firmware Details
//!
//! - **Version**: DMP Image 3a (dmp3a)
//! - **Size**: 14,301 bytes
//! - **Format**: Raw binary data organized into 256-byte memory banks
//! - **Loading**: Written to DMP memory via registers 0x7C/0x7D/0x7E
//! - **Start Address**: 0x1000 (set via Bank 2, register 0x50)

/// DMP firmware image (14,301 bytes)
///
/// Complete DMP firmware binary (version 3a) that must be loaded into the
/// ICM-20948's DMP processor memory on every power-up before the DMP can be used.
pub const DMP_FIRMWARE: &[u8] = &include!("firmware_data.rs");

/// Size of the DMP firmware in bytes
pub const DMP_FIRMWARE_SIZE: usize = 14301;

/// DMP firmware start address (to be written to Bank 2, register 0x50)
pub const DMP_START_ADDRESS: u16 = 0x1000;

/// Memory bank select register (Bank 0, address 0x7E)
pub const DMP_MEM_BANK_SEL: u8 = 0x7E;

/// Memory start address register (Bank 0, address 0x7C)
pub const DMP_MEM_START_ADDR: u8 = 0x7C;

/// Memory read/write register (Bank 0, address 0x7D)
pub const DMP_MEM_R_W: u8 = 0x7D;

/// Verify firmware size at compile time
const _: () = {
    // This will fail to compile if the firmware size doesn't match
    assert!(DMP_FIRMWARE.len() == DMP_FIRMWARE_SIZE);
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_firmware_size() {
        assert_eq!(
            DMP_FIRMWARE.len(),
            14301,
            "Firmware size must be 14,301 bytes"
        );
        assert_eq!(DMP_FIRMWARE.len(), DMP_FIRMWARE_SIZE);
    }

    #[test]
    fn test_firmware_not_empty() {
        assert!(!DMP_FIRMWARE.is_empty(), "Firmware must not be empty");
        assert!(DMP_FIRMWARE.len() > 0);
    }

    #[test]
    fn test_firmware_start_address() {
        assert_eq!(
            DMP_START_ADDRESS, 0x1000,
            "DMP start address should be 0x1000"
        );
    }

    #[test]
    fn test_firmware_first_bytes() {
        // Verify first few bytes match expected values from SparkFun library
        // This helps catch any corruption during copying
        assert_eq!(DMP_FIRMWARE[0], 0x00);
        assert_eq!(DMP_FIRMWARE[1], 0x01);
        assert_eq!(DMP_FIRMWARE[2], 0x00);
        assert_eq!(DMP_FIRMWARE[3], 0x00);
    }

    #[test]
    fn test_firmware_last_bytes() {
        // Verify last few bytes
        let len = DMP_FIRMWARE.len();
        assert_eq!(DMP_FIRMWARE[len - 1], 0xe0);
        assert_eq!(DMP_FIRMWARE[len - 2], 0xc9);
        assert_eq!(DMP_FIRMWARE[len - 3], 0xf3);
    }
}
