//! Mock interface implementation for testing the ICM-20948 driver

#[cfg(feature = "async")]
use device_driver::AsyncRegisterInterface;
use device_driver::RegisterInterface;
use icm20948::Bank;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// Records operations performed on the mock interface
#[derive(Debug, Clone, PartialEq)]
pub enum Operation {
    /// Read register operation
    ReadRegister {
        /// Bank where the register was read
        bank: Bank,
        /// Register address
        address: u8,
        /// Value that was returned
        value: u8,
    },
    /// Write register operation
    WriteRegister {
        /// Bank where the register was written
        bank: Bank,
        /// Register address
        address: u8,
        /// Value that was written
        value: u8,
    },
    /// Bank switch operation
    BankSwitch {
        /// Previous bank
        from: Bank,
        /// New bank
        to: Bank,
    },
}

/// Shared state for mock interface (uses interior mutability)
#[derive(Debug)]
struct MockState {
    /// Simulated register values (bank, address) -> value
    registers: HashMap<(Bank, u8), u8>,

    /// Current bank selection
    current_bank: Bank,

    /// Operations log for verification
    operations: Vec<Operation>,

    /// Failure injection flags
    fail_next_read: bool,
    fail_next_write: bool,
    fail_bank_switch: bool,

    /// Sensor data sequences for simulating sensor readings
    accel_sequence: Vec<[i16; 3]>,
    accel_sequence_idx: usize,

    gyro_sequence: Vec<[i16; 3]>,
    gyro_sequence_idx: usize,

    /// Simulated magnetometer (AK09916) registers
    mag_registers: HashMap<u8, u8>,
}

impl MockState {
    fn new() -> Self {
        let mut state = Self {
            registers: HashMap::new(),
            current_bank: Bank::Bank0,
            operations: Vec::new(),
            fail_next_read: false,
            fail_next_write: false,
            fail_bank_switch: false,
            accel_sequence: Vec::new(),
            accel_sequence_idx: 0,
            gyro_sequence: Vec::new(),
            gyro_sequence_idx: 0,
            mag_registers: HashMap::new(),
        };

        // Set default WHO_AM_I value (0xEA)
        state.registers.insert((Bank::Bank0, 0x00), 0xEA);

        // Initialize REG_BANK_SEL to 0 (Bank 0)
        state.registers.insert((Bank::Bank0, 0x7F), 0x00);

        // Initialize AK09916 magnetometer registers
        state.mag_registers.insert(0x01, 0x09); // WHO_AM_I = 0x09
        state.mag_registers.insert(0x10, 0x01); // ST1 - data ready
        state.mag_registers.insert(0x18, 0x00); // ST2 - no overflow

        state
    }

    /// Advance accelerometer sequence and update registers
    fn advance_accel_sequence(&mut self) {
        if !self.accel_sequence.is_empty() {
            let [x, y, z] = self.accel_sequence[self.accel_sequence_idx];
            self.set_accel_data(x, y, z);
            self.accel_sequence_idx = (self.accel_sequence_idx + 1) % self.accel_sequence.len();
        }
    }

    /// Advance gyroscope sequence and update registers
    fn advance_gyro_sequence(&mut self) {
        if !self.gyro_sequence.is_empty() {
            let [x, y, z] = self.gyro_sequence[self.gyro_sequence_idx];
            self.set_gyro_data(x, y, z);
            self.gyro_sequence_idx = (self.gyro_sequence_idx + 1) % self.gyro_sequence.len();
        }
    }

    /// Set accelerometer data (will be returned on next read)
    fn set_accel_data(&mut self, x: i16, y: i16, z: i16) {
        let [x_h, x_l] = x.to_be_bytes();
        let [y_h, y_l] = y.to_be_bytes();
        let [z_h, z_l] = z.to_be_bytes();

        self.registers.insert((Bank::Bank0, 0x2D), x_h);
        self.registers.insert((Bank::Bank0, 0x2E), x_l);
        self.registers.insert((Bank::Bank0, 0x2F), y_h);
        self.registers.insert((Bank::Bank0, 0x30), y_l);
        self.registers.insert((Bank::Bank0, 0x31), z_h);
        self.registers.insert((Bank::Bank0, 0x32), z_l);
    }

    /// Set gyroscope data (will be returned on next read)
    fn set_gyro_data(&mut self, x: i16, y: i16, z: i16) {
        let [x_h, x_l] = x.to_be_bytes();
        let [y_h, y_l] = y.to_be_bytes();
        let [z_h, z_l] = z.to_be_bytes();

        self.registers.insert((Bank::Bank0, 0x33), x_h);
        self.registers.insert((Bank::Bank0, 0x34), x_l);
        self.registers.insert((Bank::Bank0, 0x35), y_h);
        self.registers.insert((Bank::Bank0, 0x36), y_l);
        self.registers.insert((Bank::Bank0, 0x37), z_h);
        self.registers.insert((Bank::Bank0, 0x38), z_l);
    }

    /// Set temperature data (will be returned on next read)
    fn set_temperature_data(&mut self, temp_raw: i16) {
        let [temp_h, temp_l] = temp_raw.to_be_bytes();
        self.registers.insert((Bank::Bank0, 0x39), temp_h);
        self.registers.insert((Bank::Bank0, 0x3A), temp_l);
    }

    /// Set magnetometer data in EXT_SLV_SENS_DATA registers (Bank 0)
    /// Format: ST1, HXL, HXH, HYL, HYH, HZL, HZH, dummy(0x17), ST2 (9 bytes for DMP compatibility)
    fn set_mag_data(&mut self, x: i16, y: i16, z: i16, st1: u8, st2: u8) {
        let [x_l, x_h] = x.to_le_bytes(); // AK09916 uses little-endian
        let [y_l, y_h] = y.to_le_bytes();
        let [z_l, z_h] = z.to_le_bytes();

        // EXT_SLV_SENS_DATA registers start at 0x3B in Bank 0
        // AK09916 format: ST1, data bytes, dummy byte (0x17), ST2 (9 bytes total)
        self.registers.insert((Bank::Bank0, 0x3B), st1); // EXT_SLV_SENS_DATA_00 (ST1)
        self.registers.insert((Bank::Bank0, 0x3C), x_l); // EXT_SLV_SENS_DATA_01 (HXL)
        self.registers.insert((Bank::Bank0, 0x3D), x_h); // EXT_SLV_SENS_DATA_02 (HXH)
        self.registers.insert((Bank::Bank0, 0x3E), y_l); // EXT_SLV_SENS_DATA_03 (HYL)
        self.registers.insert((Bank::Bank0, 0x3F), y_h); // EXT_SLV_SENS_DATA_04 (HYH)
        self.registers.insert((Bank::Bank0, 0x40), z_l); // EXT_SLV_SENS_DATA_05 (HZL)
        self.registers.insert((Bank::Bank0, 0x41), z_h); // EXT_SLV_SENS_DATA_06 (HZH)
        self.registers.insert((Bank::Bank0, 0x42), 0x00); // EXT_SLV_SENS_DATA_07 (dummy byte at reg 0x17)
        self.registers.insert((Bank::Bank0, 0x43), st2); // EXT_SLV_SENS_DATA_08 (ST2)
    }

    /// Simulate I2C master slave 4 transaction
    /// When I2C_SLV4_CTRL is written with enable bit set, simulate the I2C transaction
    fn simulate_i2c_slv4_transaction(&mut self) {
        // Get slave 4 configuration
        // I2C_SLV4_ADDR = 0x13, I2C_SLV4_REG = 0x14, I2C_SLV4_CTRL = 0x15, I2C_SLV4_DO = 0x16, I2C_SLV4_DI = 0x17
        let slv4_addr = self
            .registers
            .get(&(Bank::Bank3, 0x13))
            .copied()
            .unwrap_or(0);
        let slv4_reg = self
            .registers
            .get(&(Bank::Bank3, 0x14))
            .copied()
            .unwrap_or(0);
        let slv4_ctrl = self
            .registers
            .get(&(Bank::Bank3, 0x15))
            .copied()
            .unwrap_or(0);
        let slv4_do = self
            .registers
            .get(&(Bank::Bank3, 0x16))
            .copied()
            .unwrap_or(0);

        // Check if transaction is enabled
        if (slv4_ctrl & 0x80) == 0 {
            return;
        }

        // RNW bit is bit 7 of SLV4_ADDR register
        let is_read = (slv4_addr & 0x80) != 0;
        let i2c_addr = slv4_addr & 0x7F;

        // Only handle AK09916 address (0x0C)
        if i2c_addr == 0x0C {
            if is_read {
                // Read from magnetometer register
                let value = self.mag_registers.get(&slv4_reg).copied().unwrap_or(0);
                // Store result in I2C_SLV4_DI register (0x17)
                self.registers.insert((Bank::Bank3, 0x17), value);
            } else {
                // Write to magnetometer register
                self.mag_registers.insert(slv4_reg, slv4_do);
            }

            // Set I2C_SLV4_DONE bit (bit 6) in I2C_MST_STATUS register (Bank 0, 0x17)
            let current_status = self
                .registers
                .get(&(Bank::Bank0, 0x17))
                .copied()
                .unwrap_or(0);
            self.registers
                .insert((Bank::Bank0, 0x17), current_status | 0x40);
        }
    }
}

/// Mock interface for testing
#[derive(Clone)]
pub struct MockInterface {
    state: Rc<RefCell<MockState>>,
}

impl MockInterface {
    /// Create a new mock interface with default register values
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            state: Rc::new(RefCell::new(MockState::new())),
        }
    }

    /// Set a register value
    #[allow(dead_code)]
    pub fn set_register(&self, bank: Bank, address: u8, value: u8) {
        self.state
            .borrow_mut()
            .registers
            .insert((bank, address), value);
    }

    /// Get a register value
    #[allow(dead_code)]
    pub fn get_register(&self, bank: Bank, address: u8) -> u8 {
        self.state
            .borrow()
            .registers
            .get(&(bank, address))
            .copied()
            .unwrap_or(0)
    }

    /// Set WHO_AM_I register value
    #[allow(dead_code)]
    pub fn set_who_am_i(&self, value: u8) {
        self.set_register(Bank::Bank0, 0x00, value);
    }

    /// Set accelerometer data (will be returned on next read)
    pub fn set_accel_data(&self, x: i16, y: i16, z: i16) {
        self.state.borrow_mut().set_accel_data(x, y, z);
    }

    /// Set a sequence of accelerometer readings
    pub fn set_accel_sequence(&self, sequence: Vec<[i16; 3]>) {
        let mut state = self.state.borrow_mut();
        state.accel_sequence = sequence;
        state.accel_sequence_idx = 0;
    }

    /// Set gyroscope data (will be returned on next read)
    pub fn set_gyro_data(&self, x: i16, y: i16, z: i16) {
        self.state.borrow_mut().set_gyro_data(x, y, z);
    }

    /// Set gyroscope data sequence
    #[allow(dead_code)]
    pub fn set_gyro_sequence(&self, sequence: Vec<[i16; 3]>) {
        let mut state = self.state.borrow_mut();
        state.gyro_sequence = sequence;
        state.gyro_sequence_idx = 0;
    }

    /// Set temperature data (will be returned on next read)
    pub fn set_temperature_data(&self, temp_raw: i16) {
        self.state.borrow_mut().set_temperature_data(temp_raw);
    }

    /// Set magnetometer data (will be returned on next read)
    ///
    /// This simulates the AK09916 magnetometer data being read via the I2C master
    /// and stored in the EXT_SLV_SENS_DATA registers.
    ///
    /// # Arguments
    /// * `x` - X-axis raw magnetometer value
    /// * `y` - Y-axis raw magnetometer value
    /// * `z` - Z-axis raw magnetometer value
    pub fn set_mag_data(&self, x: i16, y: i16, z: i16) {
        // ST1 = 0x01 (data ready), ST2 = 0x00 (no overflow)
        self.state.borrow_mut().set_mag_data(x, y, z, 0x01, 0x00);
    }

    /// Set magnetometer data with custom status registers
    ///
    /// # Arguments
    /// * `x` - X-axis raw magnetometer value
    /// * `y` - Y-axis raw magnetometer value
    /// * `z` - Z-axis raw magnetometer value
    /// * `st1` - Status 1 register value (data ready flag)
    /// * `st2` - Status 2 register value (overflow flag in bit 3)
    #[allow(dead_code)]
    pub fn set_mag_data_with_status(&self, x: i16, y: i16, z: i16, st1: u8, st2: u8) {
        self.state.borrow_mut().set_mag_data(x, y, z, st1, st2);
    }

    /// Simulate magnetometer overflow condition
    pub fn set_mag_overflow(&self) {
        // Set ST2 bit 3 to indicate overflow
        self.state.borrow_mut().set_mag_data(0, 0, 0, 0x01, 0x08);
    }

    /// Simulate successful magnetometer initialization by setting up I2C master registers
    /// This is now handled automatically by the mock's I2C slave 4 simulation
    pub fn setup_mag_initialization(&self) {
        // The mock now automatically handles I2C slave 4 transactions
        // This method is kept for backward compatibility but does nothing
    }

    /// Inject a read failure on the next read operation
    pub fn fail_next_read(&self) {
        self.state.borrow_mut().fail_next_read = true;
    }

    /// Inject a write failure on the next write operation
    #[allow(dead_code)]
    pub fn fail_next_write(&self) {
        self.state.borrow_mut().fail_next_write = true;
    }

    /// Inject a bank switch failure
    pub fn fail_bank_switch(&self, enable: bool) {
        self.state.borrow_mut().fail_bank_switch = enable;
    }

    /// Get the operations log
    pub fn operations(&self) -> Vec<Operation> {
        self.state.borrow().operations.clone()
    }

    /// Clear the operations log
    pub fn clear_operations(&self) {
        self.state.borrow_mut().operations.clear();
    }

    /// Count bank switch operations
    pub fn bank_switch_count(&self) -> usize {
        self.state
            .borrow()
            .operations
            .iter()
            .filter(|op| matches!(op, Operation::BankSwitch { .. }))
            .count()
    }

    /// Verify a register was written with expected value
    #[allow(dead_code)]
    pub fn verify_register(&self, bank: Bank, address: u8, expected: u8) -> bool {
        self.get_register(bank, address) == expected
    }
}

/// Mock error type
#[derive(Debug, Clone, PartialEq)]
pub enum MockError {
    /// Simulated communication error
    Communication,
    /// Simulated bank switch error
    BankSwitch,
}

impl RegisterInterface for MockInterface {
    type Error = MockError;
    type AddressType = u8;

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        read_data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut state = self.state.borrow_mut();

        // Check for injected failure
        if state.fail_next_read {
            state.fail_next_read = false;
            return Err(MockError::Communication);
        }

        // Handle bank switch register specially
        if address == 0x7F {
            let current_bank = state.current_bank;
            read_data[0] = (current_bank as u8) << 4;
            state.operations.push(Operation::ReadRegister {
                bank: current_bank,
                address,
                value: read_data[0],
            });
            return Ok(());
        }

        // Check if reading accelerometer data - advance sequence if configured
        if address == 0x2D {
            state.advance_accel_sequence();
        }

        // Check if reading gyroscope data - advance sequence if configured
        if address == 0x33 {
            state.advance_gyro_sequence();
        }

        // Read from registers
        for (i, byte) in read_data.iter_mut().enumerate() {
            let reg_addr = address.wrapping_add(i as u8);
            let current_bank = state.current_bank;
            *byte = state
                .registers
                .get(&(current_bank, reg_addr))
                .copied()
                .unwrap_or(0);

            state.operations.push(Operation::ReadRegister {
                bank: current_bank,
                address: reg_addr,
                value: *byte,
            });

            // Clear I2C_SLV4_DONE bit (bit 6) when I2C_MST_STATUS register (Bank 0, 0x17) is read
            if current_bank == Bank::Bank0 && reg_addr == 0x17 {
                state.registers.insert((Bank::Bank0, 0x17), *byte & !0x40);
            }
        }

        Ok(())
    }

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        write_data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut state = self.state.borrow_mut();

        // Check for injected failure
        if state.fail_next_write {
            state.fail_next_write = false;
            return Err(MockError::Communication);
        }

        // Handle bank switch register specially
        if address == 0x7F {
            let bank_value = (write_data[0] >> 4) & 0x03;
            let new_bank = match bank_value {
                0 => Bank::Bank0,
                1 => Bank::Bank1,
                2 => Bank::Bank2,
                3 => Bank::Bank3,
                _ => return Err(MockError::BankSwitch),
            };

            // Check for injected bank switch failure
            if state.fail_bank_switch {
                return Err(MockError::BankSwitch);
            }

            let old_bank = state.current_bank;
            state.current_bank = new_bank;

            state.operations.push(Operation::BankSwitch {
                from: old_bank,
                to: new_bank,
            });

            state.registers.insert((Bank::Bank0, 0x7F), write_data[0]);

            return Ok(());
        }

        // Write to registers
        for (i, &byte) in write_data.iter().enumerate() {
            let reg_addr = address.wrapping_add(i as u8);
            let current_bank = state.current_bank;
            state.registers.insert((current_bank, reg_addr), byte);

            state.operations.push(Operation::WriteRegister {
                bank: current_bank,
                address: reg_addr,
                value: byte,
            });

            // Simulate I2C master slave 4 transaction when I2C_SLV4_CTRL is written (0x15)
            if current_bank == Bank::Bank3 && reg_addr == 0x15 {
                state.simulate_i2c_slv4_transaction();
            }
        }

        Ok(())
    }
}

#[cfg(feature = "async")]
impl AsyncRegisterInterface for MockInterface {
    type Error = MockError;
    type AddressType = u8;

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        read_data: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Delegate to synchronous implementation
        RegisterInterface::read_register(self, address, size_bits, read_data)
    }

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        write_data: &[u8],
    ) -> Result<(), Self::Error> {
        // Delegate to synchronous implementation
        RegisterInterface::write_register(self, address, size_bits, write_data)
    }
}

impl Default for MockInterface {
    fn default() -> Self {
        Self::new()
    }
}
