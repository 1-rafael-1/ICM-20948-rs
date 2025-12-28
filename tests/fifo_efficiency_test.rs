//! Test FIFO read efficiency and race condition fixes
//!
//! This test verifies that the FIFO read implementation:
//! 1. Reads the FIFO count only once before reading data
//! 2. Correctly handles cases where available data > buffer size
//! 3. Correctly handles cases where available data < buffer size
//! 4. Returns the correct number of bytes read

#![cfg(feature = "std")]

use icm20948::{I2cInterface, Icm20948Driver, WHO_AM_I_VALUE};
use std::cell::RefCell;
use std::rc::Rc;

// Mock I2C implementation for testing with shared state
#[derive(Clone)]
struct MockI2c {
    state: Rc<RefCell<MockI2cState>>,
}

struct MockI2cState {
    who_am_i_value: u8,
    bank: u8,
    fifo_count: u16,
    fifo_data: [u8; 512],
    fifo_read_index: usize,
    register_reads: Vec<(u8, u8)>, // (bank, register) for tracking reads
}

impl MockI2c {
    fn new() -> Self {
        Self {
            state: Rc::new(RefCell::new(MockI2cState {
                who_am_i_value: WHO_AM_I_VALUE,
                bank: 0,
                fifo_count: 0,
                fifo_data: [0; 512],
                fifo_read_index: 0,
                register_reads: Vec::new(),
            })),
        }
    }

    fn with_fifo_count(count: u16) -> Self {
        let mock = Self::new();
        {
            let mut state = mock.state.borrow_mut();
            state.fifo_count = count;
            // Fill FIFO data with test pattern
            for i in 0..count as usize {
                state.fifo_data[i] = (i % 256) as u8;
            }
        }
        mock
    }

    fn count_fifo_count_reads(&self) -> usize {
        // Count how many times FIFO_COUNTH (0x70) or FIFO_COUNTL (0x71) were read
        let state = self.state.borrow();
        state
            .register_reads
            .iter()
            .filter(|(bank, reg)| *bank == 0 && (*reg == 0x70 || *reg == 0x71))
            .count()
    }
}

#[derive(Debug)]
struct MockError;

impl embedded_hal::i2c::Error for MockError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl embedded_hal::i2c::ErrorType for MockI2c {
    type Error = MockError;
}

impl embedded_hal::i2c::I2c for MockI2c {
    fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
        // Handle register writes
        if write.len() >= 2 {
            let reg = write[0];
            let value = write[1];
            let mut state = self.state.borrow_mut();
            match (state.bank, reg) {
                (_, 0x7F) => {
                    // REG_BANK_SEL register - bank switching
                    state.bank = value >> 4;
                }
                _ => {}
            }
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        _address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Simulate register reads
        if !write.is_empty() {
            let reg = write[0];
            let mut state = self.state.borrow_mut();

            // Track register reads
            let current_bank = state.bank;
            state.register_reads.push((current_bank, reg));

            match (current_bank, reg) {
                (_, 0x00) => {
                    // WHO_AM_I register
                    if !read.is_empty() {
                        read[0] = state.who_am_i_value;
                    }
                }
                (0, 0x70) => {
                    // FIFO_COUNTH register
                    if !read.is_empty() {
                        read[0] = ((state.fifo_count >> 8) & 0xFF) as u8;
                    }
                }
                (0, 0x71) => {
                    // FIFO_COUNTL register
                    if !read.is_empty() {
                        read[0] = (state.fifo_count & 0xFF) as u8;
                    }
                }
                (0, 0x72) => {
                    // FIFO_R_W register - read one byte
                    if !read.is_empty() && state.fifo_read_index < state.fifo_count as usize {
                        read[0] = state.fifo_data[state.fifo_read_index];
                        state.fifo_read_index += 1;
                    }
                }
                (_, 0x7F) => {
                    // REG_BANK_SEL register
                    if write.len() > 1 {
                        // Write operation
                        state.bank = write[1] >> 4;
                    } else if !read.is_empty() {
                        // Read operation
                        read[0] = state.bank << 4;
                    }
                }
                _ => {
                    // Default: return zeros
                    for byte in read.iter_mut() {
                        *byte = 0;
                    }
                }
            }
        }
        Ok(())
    }
}

#[test]
fn test_fifo_read_efficiency_exact_count() {
    // Test case: buffer size matches available data
    let i2c = MockI2c::with_fifo_count(100);
    let interface = I2cInterface::default(i2c);

    let mut driver = Icm20948Driver::new(interface).expect("Failed to create driver");

    let mut buffer = [0u8; 100];
    let bytes_read = driver.fifo_read(&mut buffer).expect("Failed to read FIFO");

    assert_eq!(bytes_read, 100, "Should read exactly 100 bytes");

    // Verify data pattern
    for i in 0..100 {
        assert_eq!(buffer[i], (i % 256) as u8, "Data mismatch at index {}", i);
    }
}

#[test]
fn test_fifo_read_efficiency_buffer_smaller() {
    // Test case: buffer size < available data
    // This is the critical case - should read only buffer.len() bytes
    let i2c = MockI2c::with_fifo_count(200);
    let interface = I2cInterface::default(i2c);

    let mut driver = Icm20948Driver::new(interface).expect("Failed to create driver");

    let mut buffer = [0u8; 100];
    let bytes_read = driver.fifo_read(&mut buffer).expect("Failed to read FIFO");

    assert_eq!(
        bytes_read, 100,
        "Should read only buffer size (100) even though 200 available"
    );

    // Verify data pattern
    for i in 0..100 {
        assert_eq!(buffer[i], (i % 256) as u8, "Data mismatch at index {}", i);
    }
}

#[test]
fn test_fifo_read_efficiency_buffer_larger() {
    // Test case: buffer size > available data
    // Should read only available data
    let i2c = MockI2c::with_fifo_count(50);
    let interface = I2cInterface::default(i2c);

    let mut driver = Icm20948Driver::new(interface).expect("Failed to create driver");

    let mut buffer = [0u8; 100];
    let bytes_read = driver.fifo_read(&mut buffer).expect("Failed to read FIFO");

    assert_eq!(
        bytes_read, 50,
        "Should read only available data (50) even though buffer is 100"
    );

    // Verify data pattern
    for i in 0..50 {
        assert_eq!(buffer[i], (i % 256) as u8, "Data mismatch at index {}", i);
    }
}

#[test]
fn test_fifo_read_efficiency_empty() {
    // Test case: FIFO is empty
    let i2c = MockI2c::with_fifo_count(0);
    let interface = I2cInterface::default(i2c);

    let mut driver = Icm20948Driver::new(interface).expect("Failed to create driver");

    let mut buffer = [0u8; 100];
    let bytes_read = driver.fifo_read(&mut buffer).expect("Failed to read FIFO");

    assert_eq!(bytes_read, 0, "Should read 0 bytes when FIFO is empty");
}

#[test]
fn test_fifo_read_no_repeated_count_checks() {
    // CRITICAL TEST: Verify that FIFO count is only read ONCE
    // The old implementation would read count after every byte (2 register reads per byte!)
    // The new implementation should read count only once (2 register reads total)

    let i2c = MockI2c::with_fifo_count(50);

    // Clear initialization tracking
    {
        let mut state = i2c.state.borrow_mut();
        state.register_reads.clear();
    }

    let interface = I2cInterface::default(i2c.clone());
    let mut driver = Icm20948Driver::new(interface).expect("Failed to create driver");

    let mut buffer = [0u8; 50];
    let _ = driver.fifo_read(&mut buffer).expect("Failed to read FIFO");

    let count_reads = i2c.count_fifo_count_reads();

    // Should have exactly 2 count register reads (FIFO_COUNTH + FIFO_COUNTL)
    // OLD implementation would have 2 * (buffer_len - 1) = 98 reads!
    assert_eq!(
        count_reads, 2,
        "Should read FIFO count registers exactly twice (once for high, once for low), but got {}",
        count_reads
    );
}

#[test]
fn test_fifo_read_large_buffer() {
    // Test with a larger, more realistic buffer size
    let i2c = MockI2c::with_fifo_count(512);
    let interface = I2cInterface::default(i2c);

    let mut driver = Icm20948Driver::new(interface).expect("Failed to create driver");

    let mut buffer = [0u8; 512];
    let bytes_read = driver.fifo_read(&mut buffer).expect("Failed to read FIFO");

    assert_eq!(bytes_read, 512, "Should read all 512 bytes");

    // Verify data pattern
    for i in 0..512 {
        assert_eq!(buffer[i], (i % 256) as u8, "Data mismatch at index {}", i);
    }
}

#[test]
fn test_fifo_read_partial_records() {
    // Test that we can read partial record sets
    // Example: 3.5 records of 12 bytes each = 42 bytes
    let i2c = MockI2c::with_fifo_count(42);
    let interface = I2cInterface::default(i2c);

    let mut driver = Icm20948Driver::new(interface).expect("Failed to create driver");

    let mut buffer = [0u8; 100];
    let bytes_read = driver.fifo_read(&mut buffer).expect("Failed to read FIFO");

    assert_eq!(bytes_read, 42, "Should read exactly 42 bytes");
}
