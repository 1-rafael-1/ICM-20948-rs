//! DMP Firmware Image
//!
//! This module contains the Digital Motion Processor (DMP) firmware binary
//! that must be loaded into the ICM-20948's DMP memory on every power-up.
//!
//! ## Source and Licensing
//!
//! This firmware is originally provided by `InvenSense` (TDK) and distributed via
//! the `SparkFun` ICM-20948 Arduino Library under MIT license:
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

cfg_if::cfg_if!(
    if #[cfg(feature = "dmp-lzss")] {
        include!("firmware_data_lzss.rs");
        /// A reader utility to sequentially read DMP firmware bytes.
        ///
        /// In `dmp-lzss` mode, this struct performs on-the-fly LZSS decompression,
        /// keeping RAM usage minimal (only requiring a 2048-byte sliding window).
        pub struct FirmwareReader {
            in_pos: usize,
            window: [u8; 2048],
            win_pos: usize,
            flags: u8,
            bits_left: u8,
            match_len: usize,
            match_offset: usize,
        }
        impl Default for FirmwareReader {
            fn default() -> Self {
                Self::new()
            }
        }
        impl FirmwareReader {
            /// Creates a new `FirmwareReader` instance initialized for LZSS decompression.
            pub const fn new() -> Self {
                Self {
                    in_pos: 0,
                    window: [0; 2048],
                    win_pos: 0,
                    flags: 0,
                    bits_left: 0,
                    match_len: 0,
                    match_offset: 0,
                }
            }
            /// Fills the provided buffer with decompressed firmware bytes.
            ///
            /// Returns the number of bytes actually read. A return value of `0` indicates
            pub fn read(&mut self, buf: &mut [u8]) -> usize {
                let mut count = 0;
                for b in buf.iter_mut() {
                    if let Some(byte) = self.next_lzss_byte() {
                        *b = byte;
                        count += 1;
                    } else {
                        break;
                    }
                }
                count
            }

            /// Internal helper to decode the next byte from the LZSS stream.
            fn next_lzss_byte(&mut self) -> Option<u8> {
                let input = DMP_FIRMWARE;
                if self.match_len > 0 {
                    self.match_len -= 1;
                    let b = self.window[(self.win_pos + 2048 - self.match_offset) % 2048];
                    self.window[self.win_pos] = b;
                    self.win_pos = (self.win_pos + 1) % 2048;
                    return Some(b);
                }
                if self.bits_left == 0 {
                    if self.in_pos >= input.len() {
                        return None;
                    }
                    self.flags = input[self.in_pos];
                    self.in_pos += 1;
                    self.bits_left = 8;
                }
                if self.in_pos >= input.len() && (self.flags & 1) != 0 {
                    return None;
                }

                let is_match = (self.flags & 1) != 0;
                self.flags >>= 1;
                self.bits_left -= 1;

                if is_match {
                    if self.in_pos + 1 >= input.len() {
                        return None;
                    }
                    let b1 = u16::from(input[self.in_pos]);
                    let b2 = u16::from(input[self.in_pos + 1]);
                    self.in_pos += 2;
                    let val = (b1 << 8) | b2;
                    self.match_offset = ((val >> 5) + 1) as usize;
                    self.match_len = ((val & 0x1F) + 3 - 1) as usize;

                    let b = self.window[(self.win_pos + 2048 - self.match_offset) % 2048];
                    self.window[self.win_pos] = b;
                    self.win_pos = (self.win_pos + 1) % 2048;
                    Some(b)
                } else {
                    if self.in_pos >= input.len() {
                        return None;
                    }
                    let b = input[self.in_pos];
                    self.in_pos += 1;
                    self.window[self.win_pos] = b;
                    self.win_pos = (self.win_pos + 1) % 2048;
                    Some(b)
                }
            }
        }
    } else {
        include!("firmware_data.rs");

        /// A reader utility to sequentially read DMP firmware bytes.
        ///
        /// In uncompressed mode, this struct directly copies slices from the static
        /// firmware array into the provided buffer for maximum performance.
        pub struct FirmwareReader {
            pos: usize,
        }
        impl Default for FirmwareReader {
            fn default() -> Self {
                Self::new()
            }
        }
        impl FirmwareReader {
            /// Creates a new `FirmwareReader` instance initialized to the start of the firmware.
            pub const fn new() -> Self {
                Self { pos: 0 }
            }
            /// Fills the provided buffer with firmware bytes.
            ///
            /// Returns the number of bytes actually read. A return value of `0` indicates
            /// that the end of the firmware has been reached.
            pub fn read(&mut self, buf: &mut [u8]) -> usize {
                let remaining = DMP_FIRMWARE.len() - self.pos;
                let to_read = remaining.min(buf.len());
                if to_read == 0 { return 0; }
                buf[..to_read].copy_from_slice(&DMP_FIRMWARE[self.pos..self.pos + to_read]);
                self.pos += to_read;
                to_read
            }
        }
    }
);

/// Size of the DMP firmware in bytes
pub const DMP_FIRMWARE_SIZE: usize = 14301;

/// DMP firmware start address (to be written to Bank 2, register 0x50)
pub const DMP_START_ADDRESS: u16 = 0x1000;

/// DMP firmware write start pos
pub const DMP_LOAD_START: u16 = 0x90;

/// Memory bank select register (Bank 0, address 0x7E)
pub const DMP_MEM_BANK_SEL: u8 = 0x7E;

/// Memory start address register (Bank 0, address 0x7C)
pub const DMP_MEM_START_ADDR: u8 = 0x7C;

/// Memory read/write register (Bank 0, address 0x7D)
pub const DMP_MEM_R_W: u8 = 0x7D;

// /// Verify firmware size at compile time
// const _: () = {
//     // This will fail to compile if the firmware size doesn't match
//     assert!(DMP_FIRMWARE.len() == DMP_FIRMWARE_SIZE);
// };

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
