//! DMP Firmware Loader
//!
//! This module implements the firmware loading functionality for the ICM-20948's
//! Digital Motion Processor (DMP). The DMP does not have pre-programmed firmware,
//! so the firmware must be uploaded by the host MCU on every power-up.
//!
//! ## Loading Process
//!
//! The firmware is loaded by writing to three special registers in Bank 0:
//! - `MEM_BANK_SEL` (0x7E): Selects the memory bank (256-byte chunks)
//! - `MEM_START_ADDR` (0x7C): Sets the start address within the current bank
//! - `MEM_R_W` (0x7D): Data register for reading/writing firmware bytes
//!
//! The firmware is 14,301 bytes and is organized into memory banks. The loading
//! process writes the firmware in chunks, incrementing the bank and address as needed.
//!
//! After loading the firmware, the DMP start address must be set in Bank 2 registers
//! `PRGM_START_ADDRH` (0x50) and `PRGM_START_ADDRL` (0x51).

use crate::dmp::firmware::{DMP_FIRMWARE, DMP_MEM_BANK_SEL, DMP_MEM_R_W, DMP_MEM_START_ADDR};

/// Maximum size of a single write transaction (bytes per chunk)
/// Writing in smaller chunks improves reliability and reduces bus overhead
const WRITE_CHUNK_SIZE: usize = 16;

/// DMP memory bank size (bytes)
const DMP_BANK_SIZE: usize = 256;

/// Delay needed after firmware loading (microseconds)
/// This allows the DMP to initialize after firmware upload
pub const DMP_LOAD_DELAY_US: u32 = 1000;

/// Firmware loader implementation
pub struct FirmwareLoader;

impl FirmwareLoader {
    /// Load DMP firmware into the device
    ///
    /// Uploads the complete DMP firmware binary to the ICM-20948's DMP processor memory.
    /// The firmware must be loaded every time the device powers up, as the DMP memory
    /// is volatile.
    ///
    /// # Arguments
    ///
    /// * `write_fn` - Function to write a single byte to a register
    ///   Takes (`register_address`, value) and returns Result<(), E>
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` if the firmware was loaded successfully, or an error if
    /// communication failed.
    ///
    /// After this function returns, delay for at least 1ms before configuring or
    /// enabling the DMP.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::dmp::loader::FirmwareLoader;
    /// # let mut device = todo!();
    /// // Load firmware (write_fn is provided by the driver)
    /// FirmwareLoader::load_firmware(|addr, val| {
    ///     device.write_register(addr, val)
    /// })?;
    /// // Wait for DMP to initialize
    /// // delay_us(1000);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn load_firmware<E, F>(mut write_fn: F) -> Result<(), E>
    where
        F: FnMut(u8, u8) -> Result<(), E>,
    {
        // The firmware is loaded as a continuous stream of bytes
        // We need to manage the bank and address as we write
        let mut current_address: u16 = 0;

        // Process firmware in chunks
        for chunk in DMP_FIRMWARE.chunks(WRITE_CHUNK_SIZE) {
            // Calculate which bank and address offset we're at
            #[allow(clippy::cast_possible_truncation)]
            let bank = (current_address / DMP_BANK_SIZE as u16) as u8;
            #[allow(clippy::cast_possible_truncation)]
            let addr_in_bank = (current_address % DMP_BANK_SIZE as u16) as u8;

            // Set the memory bank
            write_fn(DMP_MEM_BANK_SEL, bank)?;

            // Set the start address within the bank
            write_fn(DMP_MEM_START_ADDR, addr_in_bank)?;

            // Write the data bytes sequentially to MEM_R_W
            // The address auto-increments after each write
            for &byte in chunk {
                write_fn(DMP_MEM_R_W, byte)?;
            }

            #[allow(clippy::cast_possible_truncation)]
            let chunk_len = chunk.len() as u16;
            current_address += chunk_len;
        }

        Ok(())
    }

    /// Verify firmware was loaded correctly (optional verification)
    ///
    /// Reads back portions of the loaded firmware to verify the loading process
    /// completed successfully. Checks the first, middle, and last bytes.
    ///
    /// # Arguments
    ///
    /// * `write_fn` - Function to write a byte to a register
    /// * `read_fn` - Function to read a byte from a register
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if verification passed, `Ok(false)` if verification
    /// failed, or an error if communication failed.
    pub fn verify_firmware<E, W, R>(mut write_fn: W, mut read_fn: R) -> Result<bool, E>
    where
        W: FnMut(u8, u8) -> Result<(), E>,
        R: FnMut(u8) -> Result<u8, E>,
    {
        // Verify first 4 bytes
        if !Self::verify_bytes(&mut write_fn, &mut read_fn, 0, 4)? {
            return Ok(false);
        }

        // Verify last 4 bytes
        let last_addr = DMP_FIRMWARE.len() - 4;
        if !Self::verify_bytes(&mut write_fn, &mut read_fn, last_addr, 4)? {
            return Ok(false);
        }

        // Verify some bytes in the middle
        let mid_addr = DMP_FIRMWARE.len() / 2;
        if !Self::verify_bytes(&mut write_fn, &mut read_fn, mid_addr, 4)? {
            return Ok(false);
        }

        Ok(true)
    }

    /// Helper function to verify a range of bytes
    fn verify_bytes<E, W, R>(
        write_fn: &mut W,
        read_fn: &mut R,
        start_addr: usize,
        count: usize,
    ) -> Result<bool, E>
    where
        W: FnMut(u8, u8) -> Result<(), E>,
        R: FnMut(u8) -> Result<u8, E>,
    {
        #[allow(clippy::cast_possible_truncation)]
        let bank = (start_addr / DMP_BANK_SIZE) as u8;
        #[allow(clippy::cast_possible_truncation)]
        let addr_in_bank = (start_addr % DMP_BANK_SIZE) as u8;

        // Set the memory bank
        write_fn(DMP_MEM_BANK_SEL, bank)?;

        // Set the start address
        write_fn(DMP_MEM_START_ADDR, addr_in_bank)?;

        // Read and compare bytes
        for i in 0..count {
            let expected = DMP_FIRMWARE[start_addr + i];
            let actual = read_fn(DMP_MEM_R_W)?;

            if actual != expected {
                return Ok(false);
            }
        }

        Ok(true)
    }

    /// Set the DMP program start address
    ///
    /// Sets the DMP execution start address using the `PRGM_START_ADDRH` and
    /// `PRGM_START_ADDRL` registers in Bank 2. The standard start address is 0x1000.
    ///
    /// # Arguments
    ///
    /// * `write_fn` - Function to write a byte to a register
    /// * `start_addr` - The DMP program start address (typically 0x1000)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` if successful, or an error if communication failed.
    pub fn set_start_address<E, F>(mut write_fn: F, start_addr: u16) -> Result<(), E>
    where
        F: FnMut(u8, u8) -> Result<(), E>,
    {
        // PRGM_START_ADDRH is at 0x50 in Bank 2
        let addr_high = (start_addr >> 8) as u8;
        let addr_low = (start_addr & 0xFF) as u8;

        // Write high byte to 0x50
        write_fn(0x50, addr_high)?;

        // Write low byte to 0x51
        write_fn(0x51, addr_low)?;

        Ok(())
    }
}

/// DMP initialization sequence
pub struct DmpInitializer;

impl DmpInitializer {
    // Reserved for potential future expansion of the API.
}

#[cfg(test)]
mod tests {
    use super::*;

    // Tests that use Vec require std (alloc)
    extern crate std;
    use std::vec::Vec;

    #[test]
    fn test_chunk_size_valid() {
        assert!(WRITE_CHUNK_SIZE > 0);
        assert!(WRITE_CHUNK_SIZE <= 256);
    }

    #[test]
    fn test_bank_size() {
        assert_eq!(DMP_BANK_SIZE, 256);
    }

    #[test]
    fn test_firmware_load_simulation() {
        // Simulate firmware loading to verify the logic
        let mut writes = Vec::new();

        let result = FirmwareLoader::load_firmware(|addr, val| {
            writes.push((addr, val));
            Ok::<(), ()>(())
        });

        assert!(result.is_ok());

        // Verify we got some writes
        assert!(!writes.is_empty());

        // Verify MEM_BANK_SEL and MEM_START_ADDR are set
        let bank_writes: Vec<_> = writes
            .iter()
            .filter(|(addr, _)| *addr == DMP_MEM_BANK_SEL)
            .collect();
        let start_addr_writes: Vec<_> = writes
            .iter()
            .filter(|(addr, _)| *addr == DMP_MEM_START_ADDR)
            .collect();

        assert!(!bank_writes.is_empty());
        assert!(!start_addr_writes.is_empty());
    }

    #[test]
    fn test_set_start_address() {
        let mut writes = Vec::new();

        let result = FirmwareLoader::set_start_address(
            |addr, val| {
                writes.push((addr, val));
                Ok::<(), ()>(())
            },
            0x1000,
        );

        assert!(result.is_ok());
        assert_eq!(writes.len(), 2);

        // High byte should be 0x10, low byte should be 0x00
        assert_eq!(writes[0], (0x50, 0x10));
        assert_eq!(writes[1], (0x51, 0x00));
    }

    #[test]
    fn test_start_address_different_value() {
        let mut writes = Vec::new();

        let result = FirmwareLoader::set_start_address(
            |addr, val| {
                writes.push((addr, val));
                Ok::<(), ()>(())
            },
            0xABCD,
        );

        assert!(result.is_ok());
        assert_eq!(writes.len(), 2);
        assert_eq!(writes[0], (0x50, 0xAB));
        assert_eq!(writes[1], (0x51, 0xCD));
    }
}
