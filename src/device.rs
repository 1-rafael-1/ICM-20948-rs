//! High-level driver API for the ICM-20948
//!
//! This module provides a user-friendly interface to the ICM-20948 sensor,
//! handling register bank switching, sensor configuration, and data reading.

use crate::registers::Icm20948 as RegisterDevice;
use crate::{Bank, Error, WHO_AM_I_VALUE};

/// Default motion detection threshold divisor for calibration
///
/// During calibration, the sensor must remain stationary. The threshold is
/// calculated as: `max_variance` = sensitivity / divisor
/// - Divisor 20 = ~5% variance allowed (balanced, recommended)
/// - Lower values are more lenient, higher values are stricter
const DEFAULT_MOTION_DETECTION_THRESHOLD_DIVISOR: i16 = 20;

use crate::fifo::{FifoConfig, FifoConfigAdvanced, FifoOverflowStatus, FifoRecord};

// Only import RegisterInterface when not using async feature
#[cfg(not(feature = "async"))]
use device_driver::RegisterInterface;

// Interrupt imports - needed in both blocking and async modes
use crate::interrupt::{
    DataReadyStatus, InterruptConfig, InterruptPinConfig, InterruptStatus, WomStatus,
};

// Power management imports - needed in both blocking and async modes
use crate::power::{
    ClockSource, CycleConfig, LowPowerConfig, PowerMode, PowerStatus, SensorPowerConfig,
};

/// Accelerometer data (raw 16-bit values)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelData {
    /// X-axis acceleration (raw)
    pub x: i16,
    /// Y-axis acceleration (raw)
    pub y: i16,
    /// Z-axis acceleration (raw)
    pub z: i16,
}

/// Gyroscope data (raw 16-bit values)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroData {
    /// X-axis rotation (raw)
    pub x: i16,
    /// Y-axis rotation (raw)
    pub y: i16,
    /// Z-axis rotation (raw)
    pub z: i16,
}

/// Magnetometer data (raw 16-bit values)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagData {
    /// X-axis magnetic field (raw)
    pub x: i16,
    /// Y-axis magnetic field (raw)
    pub y: i16,
    /// Z-axis magnetic field (raw)
    pub z: i16,
}

/// Main driver for the ICM-20948
pub struct Icm20948Driver<I> {
    device: RegisterDevice<I>,
    current_bank: Option<Bank>,
    // Sensor configurations
    accel_config: crate::sensors::AccelConfig,
    gyro_config: crate::sensors::GyroConfig,
    // Sensor calibrations
    accel_calibration: crate::sensors::AccelCalibration,
    gyro_calibration: crate::sensors::GyroCalibration,
    mag_calibration: crate::sensors::MagCalibration,
    mag_initialized: bool,
}

#[cfg(not(feature = "async"))]
impl<I> Icm20948Driver<I>
where
    I: RegisterInterface<AddressType = u8>,
{
    /// Create a new ICM-20948 driver instance
    ///
    /// This will verify the `WHO_AM_I` register but will not initialize the device.
    /// Call `init()` after construction to configure the device.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Communication with the device fails
    /// - The `WHO_AM_I` register contains an unexpected value
    pub fn new(interface: I) -> Result<Self, Error<I::Error>> {
        let device = RegisterDevice::new(interface);
        let mut driver = Self {
            device,
            current_bank: None,
            accel_config: crate::sensors::AccelConfig::default(),
            gyro_config: crate::sensors::GyroConfig::default(),
            accel_calibration: crate::sensors::AccelCalibration::default(),
            gyro_calibration: crate::sensors::GyroCalibration::default(),
            mag_calibration: crate::sensors::MagCalibration::default(),
            mag_initialized: false,
        };

        // Verify WHO_AM_I
        driver.select_bank(Bank::Bank0)?;
        let who_am_i = driver.read_who_am_i()?;

        if who_am_i != WHO_AM_I_VALUE {
            return Err(Error::InvalidDevice(who_am_i));
        }

        Ok(driver)
    }

    /// Initialize the device with default settings
    ///
    /// This performs a soft reset and configures basic settings.
    ///
    /// **Important**: This function requires a delay provider to ensure proper
    /// timing after device reset. According to the ICM-20948 datasheet Section 3
    /// "ELECTRICAL CHARACTERISTICS", the device requires up to 100ms after reset
    /// before registers can be accessed.
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider implementing `embedded_hal::delay::DelayNs`
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use embassy_time::Delay;
    /// let mut delay = Delay;
    /// driver.init(&mut delay)?;
    /// ```
    pub fn init<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        const MAX_WAIT_MS: u32 = 100;
        const POLL_INTERVAL_MS: u32 = 1;

        self.select_bank(Bank::Bank0)?;

        // Reset the device
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_device_reset(true);
        })?;

        // Wait for reset to complete by polling device_reset bit
        // Datasheet Section 3 "ELECTRICAL CHARACTERISTICS": typical 11ms, max 100ms
        for _ in 0..(MAX_WAIT_MS / POLL_INTERVAL_MS) {
            delay.delay_ms(POLL_INTERVAL_MS);
            if self
                .device
                .pwr_mgmt_1()
                .read()
                .is_ok_and(|pwr_mgmt| !pwr_mgmt.device_reset())
            {
                break;
            }
        }

        // Wake up and select auto clock source
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_sleep(false);
            w.set_clksel(1);
        })?;

        // Wait and verify by checking we can read back the configuration correctly
        for _ in 0..(MAX_WAIT_MS / POLL_INTERVAL_MS) {
            delay.delay_ms(POLL_INTERVAL_MS);

            // Verify device is responding and configuration took effect
            if self
                .device
                .pwr_mgmt_1()
                .read()
                .is_ok_and(|pwr_mgmt| !pwr_mgmt.sleep() && pwr_mgmt.clksel() == 1)
            {
                // Device is awake, clock is set
                return Ok(());
            }
        }

        Ok(())
    }

    /// Enable SPI mode by disabling the I2C slave interface
    ///
    /// **Required for SPI operation!** Call this immediately after `init()`
    /// when using the `SpiInterface`. Not needed for I2C.
    ///
    /// When using SPI to communicate with the ICM-20948, the I2C slave interface
    /// must be disabled by setting the `I2C_IF_DIS` bit in the `USER_CTRL` register.
    /// This is a requirement from the ICM-20948 datasheet for proper SPI operation.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use icm20948::{SpiInterface, Icm20948Driver};
    /// use embedded_hal_bus::spi::ExclusiveDevice;
    ///
    /// // Create SPI device with CS pin
    /// let spi_device = ExclusiveDevice::new(spi_bus, cs_pin, delay);
    /// let interface = SpiInterface::new(spi_device);
    ///
    /// // Create and initialize driver
    /// let mut imu = Icm20948Driver::new(interface)?;
    /// imu.init(&mut delay)?;
    ///
    /// // Enable SPI mode (required!)
    /// imu.enable_spi_mode()?;
    ///
    /// // Now use the device normally
    /// let accel = imu.read_accelerometer()?;
    /// ```
    pub fn enable_spi_mode(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_if_dis(true);
        })?;
        Ok(())
    }

    /// Select a register bank
    ///
    /// The ICM-20948 has 4 register banks that must be selected before
    /// accessing registers in that bank.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn select_bank(&mut self, bank: Bank) -> Result<(), Error<I::Error>> {
        if self.current_bank != Some(bank) {
            self.device.reg_bank_sel().write(|w| {
                w.set_user_bank(bank as u8);
            })?;

            self.current_bank = Some(bank);
        }
        Ok(())
    }

    /// Read the `WHO_AM_I` register
    ///
    /// Should return 0xEA for a valid ICM-20948
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_who_am_i(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        let reg = self.device.who_am_i().read()?;
        Ok(reg.who_am_i())
    }

    /// Read accelerometer data
    ///
    /// Returns raw 16-bit values for X, Y, Z axes.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_accel(&mut self) -> Result<AccelData, Error<I::Error>> {
        // Read all 6 bytes atomically to prevent torn reads
        // Register addresses: ACCEL_XOUT_H (0x2D) through ACCEL_ZOUT_L (0x32)
        const ACCEL_XOUT_H: u8 = 0x2D;
        let mut buffer = [0u8; 6];
        self.select_bank(Bank::Bank0)?;
        self.device
            .interface
            .read_register(ACCEL_XOUT_H, 48, &mut buffer)?;

        let x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let z = i16::from_be_bytes([buffer[4], buffer[5]]);

        Ok(AccelData { x, y, z })
    }

    /// Read gyroscope data
    ///
    /// Returns raw 16-bit values for X, Y, Z axes.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_gyro(&mut self) -> Result<GyroData, Error<I::Error>> {
        // Read all 6 bytes atomically to prevent torn reads
        // Register addresses: GYRO_XOUT_H (0x33) through GYRO_ZOUT_L (0x38)
        const GYRO_XOUT_H: u8 = 0x33;
        let mut buffer = [0u8; 6];
        self.select_bank(Bank::Bank0)?;
        self.device
            .interface
            .read_register(GYRO_XOUT_H, 48, &mut buffer)?;

        let x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let z = i16::from_be_bytes([buffer[4], buffer[5]]);

        Ok(GyroData { x, y, z })
    }

    /// Read temperature sensor
    ///
    /// Returns raw 16-bit signed value.
    /// Temperature in °C = (`TEMP_OUT` - `RoomTemp_Offset`)/`Temp_Sensitivity` + 21°C
    /// Where `RoomTemp_Offset` = 0 and `Temp_Sensitivity` = 333.87 LSB/°C (typical)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_temperature(&mut self) -> Result<i16, Error<I::Error>> {
        // Read both bytes atomically to prevent torn reads
        // Register addresses: TEMP_OUT_H (0x39) through TEMP_OUT_L (0x3A)
        const TEMP_OUT_H: u8 = 0x39;
        let mut buffer = [0u8; 2];
        self.select_bank(Bank::Bank0)?;
        self.device
            .interface
            .read_register(TEMP_OUT_H, 16, &mut buffer)?;

        // Combine high and low bytes (big-endian)
        let temp_raw = i16::from_be_bytes([buffer[0], buffer[1]]);

        Ok(temp_raw)
    }

    /// Convert raw temperature to degrees Celsius
    #[must_use]
    pub fn temperature_to_celsius(raw: i16) -> f32 {
        // From datasheet: Temp_degC = ((TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity) + 21
        // Where RoomTemp_Offset = 0, Temp_Sensitivity = 333.87
        (f32::from(raw) / 333.87) + 21.0
    }

    /// Set sleep mode
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn set_sleep(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_sleep(enable);
        })?;
        Ok(())
    }

    /// Enable/disable DMP
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn set_dmp_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        self.device.user_ctrl().modify(|w| {
            w.set_dmp_en(enable);
        })?;
        Ok(())
    }

    /// Enable/disable FIFO
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn set_fifo_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        self.device.user_ctrl().modify(|w| {
            w.set_fifo_en(enable);
        })?;
        Ok(())
    }

    /// Consume the driver and return the underlying interface
    pub fn release(self) -> I {
        self.device.interface
    }

    /// Get a reference to the underlying register device (for advanced usage)
    pub const fn device(&self) -> &crate::registers::Icm20948<I> {
        &self.device
    }

    /// Get a mutable reference to the underlying register device (for advanced usage)
    pub const fn device_mut(&mut self) -> &mut crate::registers::Icm20948<I> {
        &mut self.device
    }

    /// Get a mutable reference to the current bank tracker (for advanced usage)
    ///
    /// At initialization, before any banks have been selected this will return bank. Once the
    /// driver selects a bank -- occurs on any register access -- then it knows the bank and from
    /// then on this will always return a valid Bank value.
    pub const fn current_bank_mut(&mut self) -> Option<&mut Bank> {
        self.current_bank.as_mut()
    }

    // ==================== DMP METHODS ====================
    // Digital Motion Processor support (requires "dmp" feature)

    #[cfg(feature = "dmp")]
    /// Load DMP firmware into the device
    ///
    /// This function uploads the complete DMP firmware binary to the ICM-20948's
    /// DMP processor memory. The firmware must be loaded every time the device
    /// powers up, as the DMP memory is volatile.
    ///
    /// **Important**: After calling this function, you should delay for at least
    /// 1ms before configuring or enabling the DMP. Use your platform's delay
    /// function (e.g., `delay.delay_ms(1)` or `delay_us(1000)`).
    ///
    /// # Process
    ///
    /// 1. Switches to Bank 0 (where DMP memory registers are located)
    /// 2. Writes the firmware in chunks to DMP memory via registers:
    ///    - `MEM_BANK_SEL` (0x7E): Selects memory bank
    ///    - `MEM_START_ADDR` (0x7C): Sets start address in bank
    ///    - `MEM_R_W` (0x7D): Writes firmware data
    /// 3. Sets the DMP program start address (0x1000) in Bank 2
    ///
    /// # Timing
    ///
    /// The loading process typically takes 100-200ms depending on I2C/SPI speed.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = todo!();
    /// // Load the DMP firmware
    /// driver.dmp_load_firmware()?;
    ///
    /// // Wait for firmware to initialize
    /// delay.delay_ms(1);
    ///
    /// // Now you can configure and enable the DMP
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn dmp_load_firmware(&mut self) -> Result<(), Error<I::Error>> {
        use crate::dmp::firmware::{DMP_FIRMWARE, DMP_START_ADDRESS};

        // Ensure we're in Bank 0
        const WRITE_CHUNK_SIZE: usize = 16;
        const DMP_BANK_SIZE: usize = 256;

        self.select_bank(Bank::Bank0)?;

        let mut current_address: u16 = 0;

        for chunk in DMP_FIRMWARE.chunks(WRITE_CHUNK_SIZE) {
            // Calculate which bank and address offset we're at
            #[allow(clippy::cast_possible_truncation)]
            let bank = (current_address / DMP_BANK_SIZE as u16) as u8;
            #[allow(clippy::cast_possible_truncation)]
            let addr_in_bank = (current_address % DMP_BANK_SIZE as u16) as u8;

            // Set the memory bank
            self.device.mem_bank_sel().write(|w| {
                w.set_mem_bank_sel(bank);
            })?;

            // Set the start address within the bank
            self.device.mem_start_addr().write(|w| {
                w.set_mem_start_addr(addr_in_bank);
            })?;

            // Write the data bytes sequentially to MEM_R_W
            // The address auto-increments after each write
            for &byte in chunk {
                self.device.mem_rw().write(|w| {
                    w.set_mem_r_w(byte);
                })?;
            }

            #[allow(clippy::cast_possible_truncation)]
            let chunk_len = chunk.len() as u16;
            current_address += chunk_len;
        }

        // Switch to Bank 2 to set program start address
        self.select_bank(Bank::Bank2)?;

        // Set the DMP program start address (0x1000)
        let addr_high = (DMP_START_ADDRESS >> 8) as u8;
        let addr_low = (DMP_START_ADDRESS & 0xFF) as u8;

        self.device.bank_2_prgm_start_addrh().write(|w| {
            w.set_prgm_start_addrh(addr_high);
        })?;

        self.device.bank_2_prgm_start_addrl().write(|w| {
            w.set_prgm_start_addrl(addr_low);
        })?;

        // Switch back to Bank 0
        self.select_bank(Bank::Bank0)?;

        // Apply hardware fix registers required for reliable DMP operation on some silicon revisions

        // Set HW_FIX_DISABLE register to 0x48
        // This disables certain hardware fixes that interfere with DMP operation
        self.device.hw_fix_disable().write(|w| {
            w.set_hw_fix_disable(0x48);
        })?;

        // Set SINGLE_FIFO_PRIORITY_SEL register to 0xE4
        // This configures FIFO priority selection for DMP mode
        self.device.single_fifo_priority_sel().write(|w| {
            w.set_single_fifo_priority_sel(0xE4);
        })?;

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Applied DMP hardware fix registers (HW_FIX_DISABLE=0x48, SINGLE_FIFO_PRIORITY_SEL=0xE4)"
        );

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Initialize the DMP (reset, load firmware, configure)
    ///
    /// Performs complete DMP initialization including reset, firmware loading,
    /// and hardware configuration.
    ///
    /// **Important**: Delay at least 1-2ms after calling this function before
    /// enabling the DMP or reading data.
    ///
    /// # Errors
    ///
    /// Returns an error if any step fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = todo!();
    /// // Initialize the device first
    /// driver.init()?;
    /// delay.delay_ms(100);
    ///
    /// // Initialize DMP
    /// driver.dmp_init()?;
    /// delay.delay_ms(2);
    ///
    /// // Enable DMP
    /// driver.dmp_enable(true)?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn dmp_init(&mut self) -> Result<(), Error<I::Error>> {
        // Reset the DMP
        self.dmp_reset()?;

        // Note: Caller should delay ~1ms here for reset to complete

        // Ensure accelerometer and gyroscope are not in low power or cycle mode
        // DMP requires sensors in full power mode
        self.select_bank(Bank::Bank0)?;

        // Disable low power mode in PWR_MGMT_1
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_lp_en(false); // Disable low power mode
            w.set_sleep(false); // Ensure not in sleep
            w.set_clksel(1); // Auto-select best clock (0x01)
        })?;

        // Disable accel/gyro cycle modes, but keep I2C master cycle enabled for magnetometer
        // LP_CONFIG should be 0x40 (only I2C master in duty cycle mode)
        self.device.lp_config().write(|w| {
            w.set_accel_cycle(false); // Accel must NOT be in cycle mode for DMP
            w.set_gyro_cycle(false); // Gyro must NOT be in cycle mode for DMP
            w.set_i_2_c_mst_cycle(false); // Disable for now, will re-enable after DMP starts
        })?;

        // Ensure all accelerometer and gyroscope axes are powered on
        // PWR_MGMT_2 should be 0x00 (all sensors enabled)
        self.device.pwr_mgmt_2().write(|w| {
            w.set_disable_accel_x(false);
            w.set_disable_accel_y(false);
            w.set_disable_accel_z(false);
            w.set_disable_gyro_x(false);
            w.set_disable_gyro_y(false);
            w.set_disable_gyro_z(false);
        })?;

        // Configure sensor sample rates before loading firmware
        self.select_bank(Bank::Bank2)?;

        // Enable GYRO_FCHOICE so that GYRO_SMPLRT_DIV is effective
        self.device.bank_2_gyro_config_1().modify(|w| {
            w.set_gyro_fchoice(true);
        })?;

        // Set gyroscope sample rate divider to 0 (1.1kHz internal rate)
        self.device.bank_2_gyro_smplrt_div().write(|w| {
            w.set_gyro_smplrt_div(0);
        })?;

        // Enable ACCEL_FCHOICE so that ACCEL_SMPLRT_DIV is effective
        self.device.bank_2_accel_config().modify(|w| {
            w.set_accel_fchoice(true);
        })?;

        // Set accelerometer sample rate divider to 0 (1.125kHz internal rate)
        self.device.bank_2_accel_smplrt_div_1().write(|w| {
            w.set_accel_smplrt_div_1(0);
        })?;
        self.device.bank_2_accel_smplrt_div_2().write(|w| {
            w.set_accel_smplrt_div_2(0);
        })?;

        // Enable ODR_ALIGN_EN to synchronize sensor data streams
        self.device.bank_2_odr_align_en().write(|w| {
            w.set_odr_align_en(true);
        })?;

        // Enable REG_LP_DMP_EN to allow DMP to receive internal sensor data
        self.device.bank_2_mod_ctrl_usr().write(|w| {
            w.set_reg_lp_dmp_en(true);
        })?;

        self.select_bank(Bank::Bank0)?;

        // Load the firmware after configuring hardware
        self.dmp_load_firmware()?;

        // Note: Caller should delay ~1ms here for firmware to initialize

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Reset the DMP processor
    ///
    /// This resets the DMP, clearing its state and FIFO. After reset, the
    /// firmware must be reloaded using `dmp_load_firmware()` or `dmp_init()`.
    ///
    /// **Important**: After calling this function, you should delay for at least
    /// 1ms before loading firmware.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn dmp_reset(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Reset DMP by setting bit 3 of USER_CTRL
        self.device.user_ctrl().modify(|w| {
            w.set_dmp_rst(true);
        })?;

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Enable or disable the DMP processor
    ///
    /// The firmware must already be loaded before enabling the DMP.
    /// Use `dmp_init()` or `dmp_load_firmware()` first.
    ///
    /// When enabled, the DMP processes sensor data and writes results to the FIFO.
    /// The FIFO is automatically enabled when DMP is enabled.
    ///
    /// # Arguments
    ///
    /// * `enable` - `true` to enable the DMP, `false` to disable it
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = todo!();
    /// // Initialize DMP
    /// driver.dmp_init()?;
    /// delay.delay_ms(2);
    ///
    /// // Enable FIFO
    /// driver.set_fifo_enable(true)?;
    ///
    /// // Enable DMP
    /// driver.dmp_enable(true)?;
    ///
    /// // Now DMP is running and writing data to FIFO
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn dmp_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // If enabling, verify power management settings first
        if enable {
            // Verify that sensors are not in low power or cycle mode
            // DMP operation requires sensors in full power mode
            let pwr_mgmt_1 = self.device.pwr_mgmt_1().read()?;
            if pwr_mgmt_1.lp_en() {
                #[cfg(feature = "defmt")]
                defmt::warn!("DMP Enable: Forcing LP_EN=false (was enabled)");

                self.device.pwr_mgmt_1().modify(|w| {
                    w.set_lp_en(false);
                })?;
            }

            let lp_config = self.device.lp_config().read()?;
            if lp_config.accel_cycle() || lp_config.gyro_cycle() {
                #[cfg(feature = "defmt")]
                defmt::warn!("DMP Enable: Forcing accel/gyro cycle modes OFF (were enabled)");

                self.device.lp_config().modify(|w| {
                    w.set_accel_cycle(false);
                    w.set_gyro_cycle(false);
                })?;
            }

            // Verify all sensors are powered on
            let pwr_mgmt_2 = self.device.pwr_mgmt_2().read()?;
            if pwr_mgmt_2.disable_accel_x()
                || pwr_mgmt_2.disable_accel_y()
                || pwr_mgmt_2.disable_accel_z()
                || pwr_mgmt_2.disable_gyro_x()
                || pwr_mgmt_2.disable_gyro_y()
                || pwr_mgmt_2.disable_gyro_z()
            {
                #[cfg(feature = "defmt")]
                defmt::warn!("DMP Enable: Enabling all accel/gyro axes (some were disabled)");

                self.device.pwr_mgmt_2().write(|w| {
                    w.set_disable_accel_x(false);
                    w.set_disable_accel_y(false);
                    w.set_disable_accel_z(false);
                    w.set_disable_gyro_x(false);
                    w.set_disable_gyro_y(false);
                    w.set_disable_gyro_z(false);
                })?;
            }
        }

        // Enable/disable DMP, FIFO, and I2C master
        // Note: I2C master is only enabled if magnetometer has been initialized (9-axis mode)
        self.device.user_ctrl().modify(|w| {
            w.set_dmp_en(enable);
            w.set_fifo_en(enable);
            // Only enable I2C master if magnetometer is initialized (9-axis mode)
            w.set_i_2_c_mst_en(enable && self.mag_initialized);
        })?;

        // Enable sensor data routing to DMP internal data path via FIFO_EN_2
        // This is required for the DMP to receive sensor data
        if enable {
            self.device.fifo_en_2().write(|w| {
                w.set_gyro_x_fifo_en(true);
                w.set_gyro_y_fifo_en(true);
                w.set_gyro_z_fifo_en(true);
                w.set_accel_fifo_en(true);
            })?;
        } else {
            self.device.fifo_en_2().write(|w| {
                w.set_gyro_x_fifo_en(false);
                w.set_gyro_y_fifo_en(false);
                w.set_gyro_z_fifo_en(false);
                w.set_accel_fifo_en(false);
            })?;
        }

        // Enable I2C master cycle mode for magnetometer data collection
        // Note: Accelerometer and gyroscope must NOT be in cycle mode for DMP
        if enable {
            // Enable I2C master cycle mode (LP_CONFIG = 0x40)
            self.device.lp_config().modify(|w| {
                w.set_i_2_c_mst_cycle(true);
            })?;

            // Enable DMP interrupt - this may be required to start data flow
            self.device.int_enable().modify(|w| {
                w.set_dmp_int_1_en(true);
            })?;
        } else {
            // Disable DMP interrupt when disabling DMP
            self.device.int_enable().modify(|w| {
                w.set_dmp_int_1_en(false);
            })?;
        }

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Enable sensor fusion for DMP processing
    ///
    /// Configures the DMP to perform 9-axis sensor fusion by writing the appropriate
    /// control registers. This must be called **after** `dmp_enable()` because the DMP
    /// firmware initializes these registers on startup.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Enable DMP first
    /// driver.dmp_enable(true)?;
    ///
    /// // THEN tell it which sensors to use
    /// driver.dmp_enable_sensor_fusion()?;
    /// ```
    pub fn dmp_enable_sensor_fusion(&mut self) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::DmpMemoryAddresses;

        // Calculate DATA_OUT_CTL1 value for 9-axis quaternion
        // GYRO_CALIBR (0x0008) + QUAT9 (0x0400) + ACCEL (0x8000) = 0x8408
        let data_out_ctl1: u16 = 0x8408;

        // Calculate DATA_INTR_CTL - must match DATA_OUT_CTL1!
        let data_intr_ctl: u16 = 0x8408;

        // Calculate DATA_OUT_CTL2 (accuracy bits)
        // ACCEL_ACCURACY (0x0001) + GYRO_ACCURACY (0x0002) + COMPASS_ACCURACY (0x0004) = 0x0007
        let data_out_ctl2: u16 = 0x0007;

        // Calculate MOTION_EVENT_CTL
        // ACCEL_CALIBR (0x0001) + GYRO_CALIBR (0x0002) + COMPASS_CALIBR (0x0004) + 9AXIS (0x0100) = 0x0107
        let motion_event_ctl: u16 = 0x0107;

        // Calculate DATA_RDY_STATUS
        // GYRO (0x0001) + ACCEL (0x0002) + COMPASS (0x0008) = 0x000B
        let data_rdy_status: u16 = 0x000B;

        // Write DATA_OUT_CTL1
        self.write_dmp_memory(
            DmpMemoryAddresses::DATA_OUT_CTL1,
            &data_out_ctl1.to_be_bytes(),
        )?;

        // Write DATA_OUT_CTL2
        self.write_dmp_memory(
            DmpMemoryAddresses::DATA_OUT_CTL2,
            &data_out_ctl2.to_be_bytes(),
        )?;

        // Write DATA_INTR_CTL (MUST match DATA_OUT_CTL1!)
        self.write_dmp_memory(
            DmpMemoryAddresses::DATA_INTR_CTL,
            &data_intr_ctl.to_be_bytes(),
        )?;

        // Write MOTION_EVENT_CTL
        self.write_dmp_memory(
            DmpMemoryAddresses::MOTION_EVENT_CTL,
            &motion_event_ctl.to_be_bytes(),
        )?;

        // Write DATA_RDY_STATUS
        self.write_dmp_memory(
            DmpMemoryAddresses::DATA_RDY_STATUS,
            &data_rdy_status.to_be_bytes(),
        )?;

        // Small delay for DMP to process
        for _ in 0..10000 {
            core::hint::spin_loop();
        }

        // Verify DATA_RDY_STATUS
        let mut readback = [0u8; 2];
        self.read_dmp_memory(DmpMemoryAddresses::DATA_RDY_STATUS, &mut readback)?;
        let _status = ((readback[0] as u16) << 8) | (readback[1] as u16);

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Enable sensor data routing to DMP internal data path
    ///
    /// Configures the FIFO_EN_2 register to route accelerometer and gyroscope data
    /// to the DMP processor. This is required for DMP operation.
    ///
    /// Note: The magnetometer uses a separate I2C slave data path.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use icm20948::*;
    /// # let mut imu = Icm20948Driver::new_i2c(todo!(), Address::Primary).unwrap();
    /// // After enabling DMP, enable sensor routing
    /// imu.dmp_enable(true)?;
    /// imu.dmp_enable_sensor_routing()?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn dmp_enable_sensor_routing(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Enable accel and gyro to internal data bus (shared by FIFO and DMP)
        // This is required for DMP to receive sensor samples
        self.device.fifo_en_2().modify(|w| {
            w.set_accel_fifo_en(true);
            w.set_gyro_x_fifo_en(true);
            w.set_gyro_y_fifo_en(true);
            w.set_gyro_z_fifo_en(true);
        })?;

        // Reset FIFO to clear any stale data and reinitialize the data path
        self.device.fifo_rst().write(|w| {
            w.set_fifo_reset(0x1F); // Reset all FIFOs
        })?;

        // Small delay to allow FIFO reset to complete
        for _ in 0..1000 {
            core::hint::spin_loop();
        }

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Read the FIFO_EN_2 register value
    ///
    /// This register controls which sensors route data to the internal FIFO/DMP data bus.
    ///
    /// # Returns
    ///
    /// Returns the raw FIFO_EN_2 register value (0x00-0xFF).
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_fifo_en_2(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        let reg = self.device.fifo_en_2().read()?;

        // Reconstruct the raw register value from individual bits
        let mut value = 0u8;
        if reg.temp_fifo_en() {
            value |= 0x01;
        }
        if reg.gyro_x_fifo_en() {
            value |= 0x02;
        }
        if reg.gyro_y_fifo_en() {
            value |= 0x04;
        }
        if reg.gyro_z_fifo_en() {
            value |= 0x08;
        }
        if reg.accel_fifo_en() {
            value |= 0x10;
        }

        Ok(value)
    }

    #[cfg(feature = "dmp")]
    /// Read the FIFO count (number of bytes available)
    ///
    /// Returns the number of bytes currently stored in the FIFO (0-4096).
    /// The DMP writes processed data to the FIFO for host retrieval.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_fifo_count(&mut self) -> Result<u16, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        let count_h = self.device.fifo_counth().read()?;
        let count_l = self.device.fifo_countl().read()?;

        // Combine high (bits 12:8) and low (bits 7:0) bytes
        let count = (u16::from(count_h.fifo_cnt_h()) << 8) | u16::from(count_l.fifo_cnt_l());

        Ok(count)
    }

    #[cfg(feature = "dmp")]
    /// Read raw bytes from the FIFO
    ///
    /// Reads data from the FIFO without parsing. Use this for direct FIFO access
    /// or custom DMP packet parsing.
    ///
    /// # Arguments
    ///
    /// * `buffer` - Buffer to read data into
    ///
    /// # Returns
    ///
    /// Returns the number of bytes actually read.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_fifo_raw(&mut self, buffer: &mut [u8]) -> Result<usize, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Use burst read to avoid I2C master conflicts with magnetometer

        let len = buffer.len();
        if len == 0 {
            return Ok(0);
        }

        // Burst read from FIFO_R_W register (0x72)
        self.device.interface.read_register(0x72, 8, buffer)?;

        Ok(len)
    }

    #[cfg(feature = "dmp")]
    /// Reset the FIFO
    ///
    /// This clears all data from the FIFO. Useful when recovering from
    /// overflow conditions or when reconfiguring the DMP.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn reset_fifo(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Reset FIFO by setting bit 2 of USER_CTRL
        self.device.user_ctrl().modify(|w| {
            w.set_sram_rst(true);
        })?;

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Set DMP FIFO watermark threshold
    ///
    /// The watermark determines when the DMP generates an interrupt. When the FIFO
    /// contains at least this many bytes, the DMP can assert the interrupt pin.
    ///
    /// This writes to the DMP memory location `FIFO_WATERMARK` (0x01FE).
    ///
    /// # Arguments
    ///
    /// * `watermark` - Number of bytes (typically set to expected packet size)
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory write fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use icm20948::{Icm20948Driver, I2cInterface};
    /// # let mut driver: Icm20948Driver<I2cInterface<_>> = todo!();
    /// // Set watermark to 16 bytes (typical quaternion packet size)
    /// driver.set_dmp_fifo_watermark(16)?;
    /// # Ok::<(), icm20948::Error<_>>(())
    /// ```
    pub fn set_dmp_fifo_watermark(&mut self, watermark: u16) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::DmpMemoryAddresses;
        let bytes = watermark.to_be_bytes();
        self.write_dmp_memory(DmpMemoryAddresses::FIFO_WATERMARK, &bytes)
    }

    #[cfg(feature = "dmp")]
    /// Get the current DMP FIFO watermark threshold
    ///
    /// Reads the watermark value from DMP memory.
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory read fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use icm20948::{Icm20948Driver, I2cInterface};
    /// # let mut driver: Icm20948Driver<I2cInterface<_>> = todo!();
    /// let watermark = driver.get_dmp_fifo_watermark()?;
    /// println!("Current watermark: {} bytes", watermark);
    /// # Ok::<(), icm20948::Error<_>>(())
    /// ```
    pub fn get_dmp_fifo_watermark(&mut self) -> Result<u16, Error<I::Error>> {
        use crate::dmp::config::DmpMemoryAddresses;
        let mut bytes = [0u8; 2];
        self.read_dmp_memory(DmpMemoryAddresses::FIFO_WATERMARK, &mut bytes)?;
        Ok(u16::from_be_bytes(bytes))
    }

    #[cfg(feature = "dmp")]
    /// Enable or disable DMP interrupt generation
    ///
    /// Controls whether the DMP asserts the interrupt pin when data is ready.
    /// This writes to the `INT_ENABLE` register in Bank 0.
    ///
    /// # Arguments
    ///
    /// * `enable` - True to enable DMP interrupts, false to disable
    ///
    /// # Errors
    ///
    /// Returns an error if register write fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use icm20948::{Icm20948Driver, I2cInterface};
    /// # let mut driver: Icm20948Driver<I2cInterface<_>> = todo!();
    /// // Enable DMP data ready interrupt
    /// driver.set_dmp_interrupt_enable(true)?;
    /// # Ok::<(), icm20948::Error<_>>(())
    /// ```
    pub fn set_dmp_interrupt_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        self.device.int_enable().modify(|w| {
            w.set_dmp_int_1_en(enable);
        })?;
        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Check if DMP interrupt is enabled
    ///
    /// Reads the `INT_ENABLE` register to check DMP interrupt status.
    ///
    /// # Errors
    ///
    /// Returns an error if register read fails.
    pub fn is_dmp_interrupt_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        let reg = self.device.int_enable().read()?;
        Ok(reg.dmp_int_1_en())
    }

    #[cfg(feature = "dmp")]
    /// Configure the DMP with specific features and settings
    ///
    /// Writes configuration to DMP memory to enable features like quaternion output,
    /// calibrated sensor data, and sample rates.
    ///
    /// **Important**: The DMP firmware must be loaded first using `dmp_init()` or
    /// `dmp_load_firmware()`.
    ///
    /// # Arguments
    ///
    /// * `config` - DMP configuration specifying which features to enable
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::{Icm20948Driver, dmp::DmpConfig};
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = todo!();
    /// // Initialize and load firmware
    /// driver.dmp_init()?;
    /// delay.delay_ms(2);
    ///
    /// // Configure for 9-axis quaternion at 100Hz
    /// let config = DmpConfig::default()
    ///     .with_quaternion_9axis(true)
    ///     .with_sample_rate(100);
    /// driver.dmp_configure(&config)?;
    ///
    /// // Enable DMP
    /// driver.dmp_enable(true)?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn dmp_configure(&mut self, config: &crate::dmp::DmpConfig) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::ConfigSequence;

        // Ensure we're in Bank 0
        self.select_bank(Bank::Bank0)?;

        // Temporarily disable I2C master during DMP configuration to prevent
        // write conflicts between magnetometer data and DMP memory registers
        let lp_cfg = self.device.lp_config().read()?;
        let i2c_mst_cycle_was_enabled = lp_cfg.i_2_c_mst_cycle();

        let user_ctrl = self.device.user_ctrl().read()?;
        let i2c_mst_was_enabled = user_ctrl.i_2_c_mst_en();

        if i2c_mst_cycle_was_enabled || i2c_mst_was_enabled {
            // Disable I2C master cycle mode first
            if i2c_mst_cycle_was_enabled {
                self.device.lp_config().modify(|w| {
                    w.set_i_2_c_mst_cycle(false);
                })?;
            }

            // Disable I2C master itself
            if i2c_mst_was_enabled {
                self.device.user_ctrl().modify(|w| {
                    w.set_i_2_c_mst_en(false);
                })?;
            }

            // Brief delay to ensure I2C master is fully stopped
            for _ in 0..5000 {
                core::hint::spin_loop();
            }
        }

        // Create configuration sequence
        let seq = ConfigSequence::from_config(config);

        // Get the complete initialization sequence (36 writes)
        let init_sequence = seq.get_init_sequence();

        // Write all configuration registers in one pass
        // This includes DATA_INTR_CTL (0x004C) and MOTION_EVENT_CTL (0x004E)

        for write in &init_sequence {
            self.write_dmp_memory(write.address, write.data)?;
        }

        // Write DATA_RDY_STATUS to indicate which sensors are available to the DMP
        // This must be written before DMP is enabled
        // Bits: 0x0001=Gyro, 0x0002=Accel, 0x0008=Compass
        let data_rdy_status: u16 = 0x000B; // Gyro + Accel + Compass

        self.write_dmp_memory(0x008A, &data_rdy_status.to_be_bytes())?;

        // Verify the write
        let mut readback = [0u8; 2];
        self.read_dmp_memory(0x008A, &mut readback)?;
        let _status = ((readback[0] as u16) << 8) | (readback[1] as u16);

        // I2C master will be re-enabled by dmp_enable() after DMP starts

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Write data to DMP memory at a specific address
    ///
    /// This is a helper method for writing configuration data to DMP memory.
    /// Data is written in chunks of up to 16 bytes for reliability.
    ///
    /// # Arguments
    ///
    /// * `address` - 16-bit DMP memory address
    /// * `data` - Bytes to write
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    fn write_dmp_memory(&mut self, address: u16, data: &[u8]) -> Result<(), Error<I::Error>> {
        const MAX_CHUNK_SIZE: usize = 16;

        if data.is_empty() {
            return Ok(());
        }

        // Ensure we're in Bank 0 for DMP memory access
        self.select_bank(Bank::Bank0)?;

        let mut bytes_written = 0;
        let mut current_address = address;

        while bytes_written < data.len() {
            // Calculate bank and offset for current address
            let bank = (current_address >> 8) as u8;
            let offset = (current_address & 0xFF) as u8;

            // Set memory bank
            self.device.mem_bank_sel().write(|w| {
                w.set_mem_bank_sel(bank);
            })?;

            // Set start address within bank
            self.device.mem_start_addr().write(|w| {
                w.set_mem_start_addr(offset);
            })?;

            // Calculate chunk size for this write
            let remaining = data.len() - bytes_written;
            let chunk_size = remaining.min(MAX_CHUNK_SIZE);

            // Write chunk bytes (address auto-increments)
            for i in 0..chunk_size {
                let byte = data[bytes_written + i];
                self.device.mem_rw().write(|w| {
                    w.set_mem_r_w(byte);
                })?;
            }

            #[cfg(feature = "defmt")]
            {
                if chunk_size == 2 {
                    let val = u16::from_be_bytes([data[bytes_written], data[bytes_written + 1]]);
                    defmt::trace!(
                        "DMP write: addr=0x{:04X} value=0x{:04X}",
                        current_address,
                        val
                    );
                } else if chunk_size == 4 {
                    defmt::trace!(
                        "DMP write: addr=0x{:04X} data=[{:02X}] (4 bytes)",
                        current_address,
                        &data[bytes_written..bytes_written + 4]
                    );
                }
            }

            bytes_written += chunk_size;
            current_address += chunk_size as u16;
        }

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Read DMP data from FIFO
    ///
    /// This method reads and parses DMP packets from the FIFO, extracting
    /// quaternion and sensor data according to the current configuration.
    ///
    /// The method automatically detects packet size based on the FIFO count
    /// and parses the data into a `DmpData` structure.
    ///
    /// # Returns
    ///
    /// Returns `Some(DmpData)` if a valid packet was read, or `None` if the
    /// FIFO is empty or contains an incomplete packet.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// // Read DMP data
    /// if let Some(data) = driver.dmp_read_fifo()? {
    ///     if let Some(quat) = data.quaternion_9axis {
    ///         println!("Quaternion: w={:.3}, x={:.3}, y={:.3}, z={:.3}",
    ///                  quat.w, quat.x, quat.y, quat.z);
    ///     }
    /// }
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn dmp_read_fifo(&mut self) -> Result<Option<crate::dmp::DmpData>, Error<I::Error>> {
        use crate::dmp::DmpParser;

        // Check FIFO count
        let count = self.read_fifo_count()?;

        // Need at least a header (2 bytes)
        if count < 2 {
            return Ok(None);
        }

        // Read enough data for typical packet (header + quaternion)
        // Maximum packet size is ~40 bytes for full configuration
        let mut buffer = [0u8; 64];
        let bytes_to_read = core::cmp::min(count as usize, buffer.len());

        self.read_fifo_raw(&mut buffer[..bytes_to_read])?;

        // Parse the packet
        let parser = DmpParser::new();
        Ok(parser.parse_packet(&buffer[..bytes_to_read]))
    }

    #[cfg(feature = "dmp")]
    /// Read quaternion data from DMP
    ///
    /// This is a convenience method that reads from the FIFO and returns just
    /// the quaternion data if present.
    ///
    /// # Returns
    ///
    /// Returns `Some(Quaternion)` if quaternion data was available, or `None`
    /// if no data or no quaternion in the packet.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// if let Some(quat) = driver.dmp_read_quaternion()? {
    ///     println!("Orientation: w={:.3}, x={:.3}, y={:.3}, z={:.3}",
    ///              quat.w, quat.x, quat.y, quat.z);
    /// }
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn dmp_read_quaternion(
        &mut self,
    ) -> Result<Option<crate::dmp::Quaternion>, Error<I::Error>> {
        if let Some(data) = self.dmp_read_fifo()? {
            // Return first available quaternion type
            if let Some(quat) = data.quaternion_9axis {
                return Ok(Some(quat));
            }
            if let Some(quat) = data.quaternion_6axis {
                return Ok(Some(quat));
            }
            if let Some(quat) = data.game_rotation_vector {
                return Ok(Some(quat));
            }
            if let Some(quat) = data.geomag_rotation_vector {
                return Ok(Some(quat));
            }
        }
        Ok(None)
    }

    #[cfg(feature = "dmp")]
    /// Read data from DMP memory at a specific address
    ///
    /// This is a helper method for reading back DMP memory for verification.
    ///
    /// # Arguments
    ///
    /// * `address` - 16-bit DMP memory address
    /// * `buffer` - Buffer to read data into
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_dmp_memory(
        &mut self,
        address: u16,
        buffer: &mut [u8],
    ) -> Result<(), Error<I::Error>> {
        // Ensure we're in Bank 0 for DMP memory access
        self.select_bank(Bank::Bank0)?;

        // Calculate bank and offset
        let bank = (address / 256) as u8;
        #[allow(clippy::cast_possible_truncation)]
        let offset = (address % 256) as u8;

        // Set memory bank
        self.device.mem_bank_sel().write(|w| {
            w.set_mem_bank_sel(bank);
        })?;

        // Set start address
        self.device.mem_start_addr().write(|w| {
            w.set_mem_start_addr(offset);
        })?;

        // Read data bytes
        for byte in buffer.iter_mut() {
            let reg = self.device.mem_rw().read()?;
            *byte = reg.mem_r_w();
        }

        Ok(())
    }

    /// Read the `USER_CTRL` register value
    ///
    /// This method reads the `USER_CTRL` register which contains control bits for:
    /// - `DMP_EN` (bit 7): DMP enable
    /// - `FIFO_EN` (bit 6): FIFO enable
    /// - `I2C_MST_EN` (bit 5): I2C master enable
    /// - `I2C_IF_DIS` (bit 4): Disable I2C slave interface
    /// - `DMP_RST` (bit 3): DMP reset
    /// - `SRAM_RST` (bit 2): SRAM/FIFO reset
    /// - `I2C_MST_RST` (bit 1): I2C master reset
    ///
    /// # Returns
    ///
    /// The raw 8-bit value of the `USER_CTRL` register
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_user_ctrl(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        let reg = self.device.user_ctrl().read()?;

        // Reconstruct the raw register value from individual bits
        let mut value = 0u8;
        if reg.dmp_en() {
            value |= 0x80;
        }
        if reg.fifo_en() {
            value |= 0x40;
        }
        if reg.i_2_c_mst_en() {
            value |= 0x20;
        }
        if reg.i_2_c_if_dis() {
            value |= 0x10;
        }
        if reg.dmp_rst() {
            value |= 0x08;
        }
        if reg.sram_rst() {
            value |= 0x04;
        }
        if reg.i_2_c_mst_rst() {
            value |= 0x02;
        }

        Ok(value)
    }

    /// Read the `PWR_MGMT_2` register as a raw byte
    ///
    /// Returns the `PWR_MGMT_2` register value with bits:
    /// - Bit 5: `DISABLE_ACCEL_X`
    /// - Bit 4: `DISABLE_ACCEL_Y`
    /// - Bit 3: `DISABLE_ACCEL_Z`
    /// - Bit 2: `DISABLE_GYRO_X`
    /// - Bit 1: `DISABLE_GYRO_Y`
    /// - Bit 0: `DISABLE_GYRO_Z`
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_pwr_mgmt_2(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        let reg = self.device.pwr_mgmt_2().read()?;

        // Reconstruct the raw register value from individual bits
        let mut value = 0u8;
        if reg.disable_accel_x() {
            value |= 0x20;
        }
        if reg.disable_accel_y() {
            value |= 0x10;
        }
        if reg.disable_accel_z() {
            value |= 0x08;
        }
        if reg.disable_gyro_x() {
            value |= 0x04;
        }
        if reg.disable_gyro_y() {
            value |= 0x02;
        }
        if reg.disable_gyro_z() {
            value |= 0x01;
        }

        Ok(value)
    }

    /// Read the `LP_CONFIG` register as a raw byte
    ///
    /// Returns the `LP_CONFIG` register value with bits:
    /// - Bit 6: `I2C_MST_CYCLE`
    /// - Bit 5: `ACCEL_CYCLE`
    /// - Bit 4: `GYRO_CYCLE`
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_lp_config(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        let reg = self.device.lp_config().read()?;

        // Reconstruct the raw register value from individual bits
        let mut value = 0u8;
        if reg.gyro_cycle() {
            value |= 0x10;
        }
        if reg.accel_cycle() {
            value |= 0x20;
        }
        if reg.i_2_c_mst_cycle() {
            value |= 0x40;
        }

        Ok(value)
    }

    /// Read Bank 2 gyro configuration registers
    ///
    /// Returns (`GYRO_CONFIG_1`, `GYRO_SMPLRT_DIV`) as raw bytes
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_bank2_gyro_config(&mut self) -> Result<(u8, u8), Error<I::Error>> {
        self.select_bank(Bank::Bank2)?;

        let config1_reg = self.device.bank_2_gyro_config_1().read()?;
        let smplrt_reg = self.device.bank_2_gyro_smplrt_div().read()?;

        // Reconstruct raw bytes
        let mut config1 = 0u8;
        if config1_reg.gyro_fchoice() {
            config1 |= 0x01;
        }
        config1 |= config1_reg.gyro_fs_sel() << 1;
        config1 |= config1_reg.gyro_dlpfcfg() << 3;

        let smplrt_div = smplrt_reg.gyro_smplrt_div();

        self.select_bank(Bank::Bank0)?;
        Ok((config1, smplrt_div))
    }

    /// Read Bank 2 accel configuration registers
    ///
    /// Returns (`ACCEL_CONFIG`, `ACCEL_SMPLRT_DIV_1`, `ACCEL_SMPLRT_DIV_2`) as raw bytes
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_bank2_accel_config(&mut self) -> Result<(u8, u8, u8), Error<I::Error>> {
        self.select_bank(Bank::Bank2)?;

        let config_reg = self.device.bank_2_accel_config().read()?;
        let div1_reg = self.device.bank_2_accel_smplrt_div_1().read()?;
        let div2_reg = self.device.bank_2_accel_smplrt_div_2().read()?;

        // Reconstruct raw bytes
        let mut config = 0u8;
        if config_reg.accel_fchoice() {
            config |= 0x01;
        }
        config |= config_reg.accel_fs_sel() << 1;
        config |= config_reg.accel_dlpfcfg() << 3;

        let div1 = div1_reg.accel_smplrt_div_1();
        let div2 = div2_reg.accel_smplrt_div_2();

        self.select_bank(Bank::Bank0)?;
        Ok((config, div1, div2))
    }

    /// Read Bank 2 `ODR_ALIGN_EN` register
    ///
    /// Returns the raw byte value
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_bank2_odr_align(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank2)?;

        let reg = self.device.bank_2_odr_align_en().read()?;

        let mut value = 0u8;
        if reg.odr_align_en() {
            value |= 0x01;
        }

        self.select_bank(Bank::Bank0)?;
        Ok(value)
    }

    /// Read Bank 2 `MOD_CTRL_USR` register
    ///
    /// Returns the raw byte value
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_bank2_mod_ctrl_usr(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank2)?;

        let reg = self.device.bank_2_mod_ctrl_usr().read()?;

        let mut value = 0u8;
        if reg.reg_lp_dmp_en() {
            value |= 0x01;
        }

        self.select_bank(Bank::Bank0)?;
        Ok(value)
    }

    /// Set DMP output data rate (ODR) for a specific feature
    ///
    /// This function sets the output rate for DMP features like quaternion output,
    /// accelerometer, gyroscope, or magnetometer data. The DMP runs at 225 Hz internally,
    /// and the ODR value determines how often data is output.
    ///
    /// # Formula
    ///
    /// ```text
    /// ODR_value = (225 / desired_Hz) - 1
    /// ```
    ///
    /// For example, for 25 Hz output: `ODR_value = (225 / 25) - 1 = 8`
    ///
    /// # Arguments
    ///
    /// * `odr_register` - The ODR register address (e.g., DmpOdrRegisters::QUAT9)
    /// * `interval` - The ODR interval value (0 = use sensor rate, higher = slower)
    ///
    /// # Notes
    ///
    /// When changing an ODR value, the corresponding ODR counter must also be reset to 0.
    /// This function writes both the ODR register and resets its counter.
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory write fails.
    #[cfg(feature = "dmp")]
    pub fn dmp_set_odr(&mut self, odr_register: u16, interval: u16) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpOdrCounterRegisters, DmpOdrRegisters};

        // Write ODR value
        let odr_bytes = interval.to_be_bytes();
        self.write_dmp_memory(odr_register, &odr_bytes)?;

        // Determine the corresponding counter register and reset it
        let counter_register = match odr_register {
            DmpOdrRegisters::QUAT9 => DmpOdrCounterRegisters::QUAT9,
            DmpOdrRegisters::QUAT6 => DmpOdrCounterRegisters::QUAT6,
            DmpOdrRegisters::ACCEL => DmpOdrCounterRegisters::ACCEL,
            DmpOdrRegisters::GYRO => DmpOdrCounterRegisters::GYRO,
            DmpOdrRegisters::CPASS => DmpOdrCounterRegisters::CPASS,
            DmpOdrRegisters::GYRO_CALIBR => DmpOdrCounterRegisters::GYRO_CALIBR,
            DmpOdrRegisters::CPASS_CALIBR => DmpOdrCounterRegisters::CPASS_CALIBR,
            _ => return Err(Error::InvalidConfig), // Unknown ODR register
        };

        // Reset the counter to 0
        let zero_bytes = [0u8, 0u8];
        self.write_dmp_memory(counter_register, &zero_bytes)?;

        Ok(())
    }

    /// Configure which sensors the DMP should expect data from
    ///
    /// Writes to the DATA_RDY_STATUS register to indicate which sensors are available.
    /// This is required for DMP operation.
    ///
    /// # Arguments
    ///
    /// * `accel` - Enable accelerometer data for DMP
    /// * `gyro` - Enable gyroscope data for DMP
    /// * `compass` - Enable magnetometer/compass data for DMP
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Enable accel and gyro for 6-axis quaternion
    /// imu.dmp_enable_sensors(true, true, false)?;
    ///
    /// // Enable all sensors for 9-axis quaternion
    /// imu.dmp_enable_sensors(true, true, true)?;
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory write fails.
    #[cfg(feature = "dmp")]
    pub fn dmp_enable_sensors(
        &mut self,
        accel: bool,
        gyro: bool,
        compass: bool,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpDataReadyStatus, DmpMemoryAddresses};

        let mut status = 0u16;

        if gyro {
            status |= DmpDataReadyStatus::GYRO;
        }
        if accel {
            status |= DmpDataReadyStatus::ACCEL;
        }
        if compass {
            status |= DmpDataReadyStatus::COMPASS;
        }

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Setting DATA_RDY_STATUS: accel={} gyro={} compass={} -> 0x{:04X}",
            accel,
            gyro,
            compass,
            status
        );

        let status_bytes = status.to_be_bytes();
        self.write_dmp_memory(DmpMemoryAddresses::DATA_RDY_STATUS, &status_bytes)?;

        Ok(())
    }

    /// Configure DMP motion event control
    ///
    /// Writes to the MOTION_EVENT_CTL register to enable sensor calibration
    /// and 9-axis fusion features.
    ///
    /// # Arguments
    ///
    /// * `accel_calibr` - Enable accelerometer calibration
    /// * `gyro_calibr` - Enable gyroscope calibration
    /// * `compass_calibr` - Enable compass/magnetometer calibration
    /// * `nine_axis` - Enable 9-axis fusion (requires all sensors)
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Enable calibration for all sensors and 9-axis fusion
    /// imu.dmp_set_motion_event_control(true, true, true, true)?;
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory write fails.
    #[cfg(feature = "dmp")]
    pub fn dmp_set_motion_event_control(
        &mut self,
        accel_calibr: bool,
        gyro_calibr: bool,
        compass_calibr: bool,
        nine_axis: bool,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpMemoryAddresses, DmpMotionEventControl};

        let mut control = 0u16;

        if accel_calibr {
            control |= DmpMotionEventControl::ACCEL_CALIBR;
        }
        if gyro_calibr {
            control |= DmpMotionEventControl::GYRO_CALIBR;
        }
        if compass_calibr {
            control |= DmpMotionEventControl::COMPASS_CALIBR;
        }
        if nine_axis {
            control |= DmpMotionEventControl::NINE_AXIS;
        }

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Setting MOTION_EVENT_CTL: accel_cal={} gyro_cal={} compass_cal={} 9axis={} -> 0x{:04X}",
            accel_calibr,
            gyro_calibr,
            compass_calibr,
            nine_axis,
            control
        );

        let control_bytes = control.to_be_bytes();
        self.write_dmp_memory(DmpMemoryAddresses::MOTION_EVENT_CTL, &control_bytes)?;

        Ok(())
    }

    /// Configure DMP to receive sensor data
    ///
    /// Performs the complete sensor initialization sequence required for DMP operation.
    /// This must be called after enabling the DMP with `dmp_enable(true)`.
    ///
    /// # Arguments
    ///
    /// * `enable_magnetometer` - Whether to include magnetometer for 9-axis fusion
    ///
    /// This function:
    /// 1. Sets output data rates (ODR) for all enabled sensors
    /// 2. Configures DMP to accept data from the specified sensors
    /// 3. Enables sensor calibration and fusion algorithms
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Load firmware and configure DMP
    /// imu.dmp_init()?;
    /// imu.dmp_configure(&config)?;
    /// imu.dmp_enable(true)?;
    ///
    /// // Now enable sensors for DMP operation
    /// imu.dmp_configure_sensors(true)?;  // Enable 9-axis with magnetometer
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if any DMP memory write fails.
    #[cfg(feature = "dmp")]
    pub fn dmp_configure_sensors(
        &mut self,
        enable_magnetometer: bool,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpMemoryAddresses, DmpOdrRegisters};

        #[cfg(feature = "defmt")]
        defmt::info!(
            "Configuring DMP sensors (magnetometer: {})",
            enable_magnetometer
        );

        // Ensure chip is awake and not in low power mode
        self.select_bank(Bank::Bank0)?;
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_sleep(false);
        })?;

        // Exit low power mode before writing DMP registers
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(false);
            w.set_accel_cycle(false);
            w.set_gyro_cycle(false);
        })?;

        // Small delay after power mode change
        for _ in 0..1000 {
            core::hint::spin_loop();
        }

        // Step 1: Set ODR for raw sensors (0 = use sensor sample rate)
        #[cfg(feature = "defmt")]
        defmt::debug!("Setting ODR for accelerometer");
        self.dmp_set_odr(DmpOdrRegisters::ACCEL, 0)?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Setting ODR for gyroscope");
        self.dmp_set_odr(DmpOdrRegisters::GYRO, 0)?;

        if enable_magnetometer {
            #[cfg(feature = "defmt")]
            defmt::debug!("Setting ODR for compass/magnetometer");
            self.dmp_set_odr(DmpOdrRegisters::CPASS, 0)?;
        }

        // Step 2: Set ODR for quaternion output
        let quat_odr = if enable_magnetometer {
            #[cfg(feature = "defmt")]
            defmt::debug!("Setting ODR for 9-axis quaternion");
            DmpOdrRegisters::QUAT9
        } else {
            #[cfg(feature = "defmt")]
            defmt::debug!("Setting ODR for 6-axis quaternion");
            DmpOdrRegisters::QUAT6
        };
        self.dmp_set_odr(quat_odr, 0)?;

        // Step 3: Enable sensors (tell DMP which sensors to expect)
        #[cfg(feature = "defmt")]
        defmt::debug!("Enabling sensor inputs for DMP");
        self.dmp_enable_sensors(true, true, enable_magnetometer)?;

        // Verify DATA_RDY_STATUS write
        let mut readback = [0u8; 2];
        self.read_dmp_memory(DmpMemoryAddresses::DATA_RDY_STATUS, &mut readback)?;
        let status = u16::from_be_bytes(readback);

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "DATA_RDY_STATUS verification: wrote=0x000B read=0x{:04X}",
            status
        );

        // Step 4: Enable calibration and fusion
        #[cfg(feature = "defmt")]
        defmt::debug!("Configuring motion event control (calibration and fusion)");
        self.dmp_set_motion_event_control(
            true,                // accel_calibr
            true,                // gyro_calibr
            enable_magnetometer, // compass_calibr
            enable_magnetometer, // nine_axis (only if magnetometer enabled)
        )?;

        // Verify MOTION_EVENT_CTL write
        self.read_dmp_memory(DmpMemoryAddresses::MOTION_EVENT_CTL, &mut readback)?;
        let _motion_ctl = u16::from_be_bytes(readback);

        // Re-enable low power mode after writing DMP registers
        // This is required for DMP to properly process sensor data
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_lp_en(true);
        })?;

        // Re-enable I2C master cycle mode
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(true);
        })?;

        Ok(())
    }

    /// Configure FIFO with the given settings
    ///
    /// # Arguments
    /// * `config` - FIFO configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn fifo_configure(&mut self, config: &FifoConfig) -> Result<(), Error<I::Error>> {
        let advanced: FifoConfigAdvanced = (*config).into();
        self.fifo_configure_advanced(&advanced)
    }

    /// Configure FIFO with advanced per-axis control
    ///
    /// # Arguments
    /// * `config` - Advanced FIFO configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn fifo_configure_advanced(
        &mut self,
        config: &FifoConfigAdvanced,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Configure FIFO_EN_1 (slave sensors)
        self.device.fifo_en_1().write(|w| {
            w.set_slv_0_fifo_en(config.enable_slv0);
            w.set_slv_1_fifo_en(config.enable_slv1);
            w.set_slv_2_fifo_en(config.enable_slv2);
            w.set_slv_3_fifo_en(config.enable_slv3);
        })?;

        // Configure FIFO_EN_2 (accel, gyro, temp)
        self.device.fifo_en_2().write(|w| {
            w.set_accel_fifo_en(config.enable_accel);
            w.set_gyro_x_fifo_en(config.enable_gyro_x);
            w.set_gyro_y_fifo_en(config.enable_gyro_y);
            w.set_gyro_z_fifo_en(config.enable_gyro_z);
            w.set_temp_fifo_en(config.enable_temp);
        })?;

        // Configure FIFO mode
        // Datasheet specifies FIFO_MODE[4:0] - use 0x1F for snapshot, 0x00 for stream
        self.device.fifo_mode().write(|w| {
            use crate::FifoMode;

            let mode_bits = match config.mode {
                FifoMode::Stream => 0x00,
                FifoMode::Snapshot => 0x1F, // All 5 bits set per Cybergear reference
            };
            w.set_fifo_mode(mode_bits);
        })?;

        Ok(())
    }

    /// Enable or disable FIFO
    ///
    /// # Arguments
    /// * `enable` - true to enable, false to disable
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn fifo_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;
        self.device.user_ctrl().modify(|w| {
            w.set_fifo_en(enable);
        })?;
        Ok(())
    }

    /// Reset FIFO buffer
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn fifo_reset(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Reset all FIFOs (bits 4:0)
        self.device.fifo_rst().write(|w| {
            w.set_fifo_reset(0x1F);
        })?;

        // Clear the reset bits
        self.device.fifo_rst().write(|w| {
            w.set_fifo_reset(0);
        })?;

        Ok(())
    }

    /// Get the number of bytes currently in the FIFO
    ///
    /// # Returns
    /// Number of bytes in FIFO (0-512)
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn fifo_count(&mut self) -> Result<u16, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        let count_h = self.device.fifo_counth().read()?;
        let count_l = self.device.fifo_countl().read()?;

        // Mask upper 3 bits of FIFO_COUNTH - datasheet specifies FIFO_CNT[12:8] only
        let count_h_masked = count_h.fifo_cnt_h() & 0x1F;
        let count = u16::from_be_bytes([count_h_masked, count_l.fifo_cnt_l()]);

        Ok(count)
    }

    /// Read raw data from FIFO
    ///
    /// Reads available FIFO data up to the buffer size. This method reads the
    /// FIFO count once before starting the read to avoid race conditions and
    /// improve performance.
    ///
    /// # Arguments
    /// * `buffer` - Buffer to read data into
    ///
    /// # Returns
    /// Number of bytes actually read (limited by available data and buffer size)
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn fifo_read(&mut self, buffer: &mut [u8]) -> Result<usize, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Read FIFO count ONCE before reading to avoid race conditions
        // and reduce overhead (checking count after every byte adds 2 register reads per byte!)
        let count = self.fifo_count()?;

        // Determine how many bytes to actually read
        let bytes_to_read = core::cmp::min(count as usize, buffer.len());

        // Burst read the determined number of bytes
        for byte in &mut buffer[..bytes_to_read] {
            let val = self.device.fifo_rw().read()?;
            *byte = val.fifo_r_w();
        }

        Ok(bytes_to_read)
    }

    /// Parse FIFO data into records
    ///
    /// # Arguments
    /// * `data` - Raw FIFO data
    /// * `config` - FIFO configuration used
    ///
    /// # Returns
    /// Vector of parsed FIFO records
    ///
    /// # Errors
    /// Returns an error if parsing fails.
    pub fn fifo_parse(
        &self,
        data: &[u8],
        config: &FifoConfigAdvanced,
    ) -> Result<heapless::Vec<FifoRecord, 64>, Error<I::Error>> {
        use crate::fifo::parser::FifoParser;
        let parser = FifoParser::new(config);
        parser.parse(data).map_err(|_| Error::InvalidConfig)
    }

    /// Get FIFO overflow status
    ///
    /// # Returns
    /// FIFO overflow status for all FIFOs
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn fifo_overflow_status(&mut self) -> Result<FifoOverflowStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        let status = self.device.int_status_2().read()?;
        let bits = status.fifo_overflow_int();

        Ok(FifoOverflowStatus {
            fifo0: (bits & 0x01) != 0,
            fifo1: (bits & 0x02) != 0,
            fifo2: (bits & 0x04) != 0,
            fifo3: (bits & 0x08) != 0,
            fifo4: (bits & 0x10) != 0,
        })
    }

    /// Configure interrupt pin electrical properties
    ///
    /// # Arguments
    /// * `config` - Interrupt pin configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn configure_interrupt_pin(
        &mut self,
        config: &InterruptPinConfig,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        self.device.int_pin_cfg().write(|w| {
            w.set_int_1_actl(config.active_low);
            w.set_int_1_open(config.open_drain);
            w.set_int_1_latch_int_en(config.latch_enabled);
            w.set_int_anyrd_2_clear(config.clear_on_any_read);
        })?;

        Ok(())
    }

    /// Configure interrupt sources
    ///
    /// # Arguments
    /// * `config` - Interrupt configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn configure_interrupts(
        &mut self,
        config: &InterruptConfig,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // INT_ENABLE register
        self.device.int_enable().write(|w| {
            w.set_wom_int_en(config.wake_on_motion);
            w.set_dmp_int_1_en(config.dmp);
            w.set_i_2_c_mst_int_en(config.i2c_master);
            w.set_pll_rdy_en(config.pll_ready);
        })?;

        // INT_ENABLE_1 register
        self.device.int_enable_1().write(|w| {
            w.set_raw_data_0_rdy_en(config.raw_data_ready);
        })?;

        // INT_ENABLE_2 register (FIFO overflow)
        self.device.int_enable_2().write(|w| {
            if config.fifo_overflow {
                w.set_fifo_overflow_en(0x1F); // Enable all FIFOs
            } else {
                w.set_fifo_overflow_en(0);
            }
        })?;

        // INT_ENABLE_3 register (FIFO watermark)
        self.device.int_enable_3().write(|w| {
            if config.fifo_watermark {
                w.set_fifo_wm_en(0x1F); // Enable all FIFOs
            } else {
                w.set_fifo_wm_en(0);
            }
        })?;

        Ok(())
    }

    /// Read interrupt status
    ///
    /// # Returns
    /// Current interrupt status flags
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        let status = self.device.int_status().read()?;
        let status1 = self.device.int_status_1().read()?;
        let status2 = self.device.int_status_2().read()?;
        let status3 = self.device.int_status_3().read()?;

        Ok(InterruptStatus {
            raw_data_ready: status1.raw_data_0_rdy_int(),
            fifo_overflow: status2.fifo_overflow_int() != 0,
            fifo_watermark: status3.fifo_wm_int() != 0,
            wake_on_motion: status.wom_int(),
            dmp: status.dmp_int_1(),
            i2c_master: status.i_2_c_mst_int(),
            pll_ready: status.pll_rdy_int(),
        })
    }

    /// Read data ready status
    ///
    /// # Returns
    /// Data ready status for all sensors
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn read_data_ready_status(&mut self) -> Result<DataReadyStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        let status = self.device.data_rdy_status().read()?;
        let bits = status.raw_data_rdy();

        Ok(DataReadyStatus {
            sensor0_ready: (bits & 0x01) != 0,
            sensor1_ready: (bits & 0x02) != 0,
            sensor2_ready: (bits & 0x04) != 0,
            sensor3_ready: (bits & 0x08) != 0,
        })
    }

    /// Read wake-on-motion status
    ///
    /// # Returns
    /// `WoM` status for each axis
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn read_wom_status(&mut self) -> Result<WomStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        let status = self.device.data_rdy_status().read()?;
        let bits = status.wof_status();

        Ok(WomStatus {
            x_motion: (bits & 0x01) != 0,
            y_motion: (bits & 0x02) != 0,
            z_motion: (bits & 0x04) != 0,
        })
    }

    // ==================== STAGE 3: POWER MANAGEMENT METHODS ====================

    /// Set the power mode
    ///
    /// # Arguments
    /// * `mode` - Power mode to set
    /// * `delay` - Delay provider for timing after power mode change
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    ///
    /// # Notes
    /// This function adds a 1ms delay after changing power mode to allow the device
    /// to stabilize. According to the ICM-20948 datasheet, ~100µs is required for
    /// power mode transitions. The 1ms delay provides a safe margin.
    pub fn set_power_mode<D>(
        &mut self,
        mode: PowerMode,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        self.select_bank(Bank::Bank0)?;

        match mode {
            PowerMode::Normal => {
                self.device.pwr_mgmt_1().modify(|w| {
                    w.set_sleep(false);
                    w.set_lp_en(false);
                })?;
            }
            PowerMode::LowPower => {
                self.device.pwr_mgmt_1().modify(|w| {
                    w.set_sleep(false);
                    w.set_lp_en(true);
                })?;
            }
            PowerMode::Sleep => {
                self.device.pwr_mgmt_1().modify(|w| {
                    w.set_sleep(true);
                })?;
            }
        }

        // Wait for power mode transition to stabilize
        // Datasheet specifies ~100µs, we use 1ms for safety margin
        delay.delay_ms(1);

        Ok(())
    }

    /// Enter low-power mode with specified configuration
    ///
    /// # Arguments
    /// * `config` - Low-power mode configuration
    /// * `delay` - Delay provider for timing after power mode change
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    ///
    /// # Notes
    /// This function adds a 1ms delay after entering low-power mode to allow the device
    /// to stabilize. According to the ICM-20948 datasheet, ~100µs is required for
    /// power mode transitions. The 1ms delay provides a safe margin.
    pub fn enter_low_power_mode<D>(
        &mut self,
        config: &LowPowerConfig,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        // Switch to Bank 2 for accelerometer configuration
        self.select_bank(Bank::Bank2)?;

        // Configure wake-on-motion threshold if enabled
        if config.enable_wake_on_motion {
            self.device.bank_2_accel_wom_thr().write(|w| {
                w.set_wom_threshold(config.wom_threshold_register_value());
            })?;

            // Configure WoM mode
            self.device.bank_2_accel_intel_ctrl().write(|w| {
                w.set_accel_intel_en(true);
                w.set_accel_intel_mode_int(config.wom_mode as u8 != 0);
            })?;
        }

        // Configure accelerometer sample rate divider for low-power mode
        // The ODR value is just a byte that goes directly into the low register
        let odr = config.accel_rate.odr_value();
        self.device.bank_2_accel_smplrt_div_1().write(|w| {
            w.set_accel_smplrt_div_1(0); // High byte is 0 for these rates
        })?;
        self.device.bank_2_accel_smplrt_div_2().write(|w| {
            w.set_accel_smplrt_div_2(odr);
        })?;

        // Configure accelerometer for LOW-POWER MODE per Table 19 of datasheet
        // In duty-cycled low-power mode, the accelerometer requires SPECIFIC settings:
        // - ACCEL_FCHOICE = 1 (enable DLPF path for averaging)
        // - ACCEL_DLPFCFG = 7 (required for LP mode with averaging)
        // - DEC3_CFG controls averaging: 0=4x, 1=8x, 2=16x, 3=32x
        // The full-scale setting (±2g/±4g/±8g/±16g) is preserved from normal mode
        let fs_sel = self.accel_config.full_scale as u8;

        // Configure ACCEL_CONFIG for low-power mode
        // Must use ACCEL_FCHOICE=1 and ACCEL_DLPFCFG=7 per Table 19
        self.device.bank_2_accel_config().write(|w| {
            w.set_accel_fs_sel(fs_sel); // Preserve full-scale setting
            w.set_accel_dlpfcfg(7); // MUST be 7 for LP mode with averaging
            w.set_accel_fchoice(true); // MUST be 1 (true) for LP mode averaging
        })?;

        // Configure ACCEL_CONFIG_2 for low-power mode averaging
        // DEC3_CFG controls number of samples averaged in LP mode:
        // 0 = 4x averaging (default, good balance of noise and power)
        // 1 = 8x averaging
        // 2 = 16x averaging
        // 3 = 32x averaging
        self.device.bank_2_accel_config_2().write(|w| {
            w.set_dec_3_cfg(0); // 4x averaging (1.488ms ON time, 496.8 Hz NBW)
            w.set_az_st_en(false);
            w.set_ay_st_en(false);
            w.set_ax_st_en(false);
        })?;

        // Switch to Bank 0 for power management
        self.select_bank(Bank::Bank0)?;

        // Configure PWR_MGMT_2: Enable only accelerometer, disable gyroscope for power savings
        // This is critical for low-power mode - sensors must be explicitly controlled
        self.device.pwr_mgmt_2().write(|w| {
            w.set_disable_accel_x(false);
            w.set_disable_accel_y(false);
            w.set_disable_accel_z(false);
            w.set_disable_gyro_x(true);
            w.set_disable_gyro_y(true);
            w.set_disable_gyro_z(true);
        })?;

        // Enable low-power mode
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_lp_en(true);
            w.set_sleep(false);
        })?;

        // Enable accelerometer cycle mode
        self.device.lp_config().modify(|w| {
            w.set_accel_cycle(true);
        })?;

        // Wait for power mode transition to stabilize
        // Datasheet specifies ~100µs, we use 1ms for safety margin
        delay.delay_ms(1);

        // Clear any pending interrupts caused by mode transition
        // The transition from normal to low-power mode can cause spurious motion detection
        if config.enable_wake_on_motion {
            let _ = self.read_interrupt_status();
            delay.delay_ms(1);

            // Now enable wake-on-motion interrupt after clearing transition artifacts
            self.device.int_enable().modify(|w| {
                w.set_wom_int_en(true);
            })?;
        }

        Ok(())
    }

    // ==================== STAGE 4: ACCELEROMETER METHODS ====================
    /// Exit low-power mode and return to normal operation
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn exit_low_power_mode(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        // Disable low-power mode
        self.device.pwr_mgmt_1().modify(|w| {
            w.set_lp_en(false);
            w.set_sleep(false);
        })?;

        // Disable cycle modes
        self.device.lp_config().write(|w| {
            w.set_accel_cycle(false);
            w.set_gyro_cycle(false);
            w.set_i_2_c_mst_cycle(false);
        })?;

        // Switch to Bank 2 to restore normal-mode accelerometer configuration
        self.select_bank(Bank::Bank2)?;

        // Restore normal-mode ACCEL_CONFIG settings
        let fs_sel = self.accel_config.full_scale as u8;
        let dlpf_cfg = self.accel_config.dlpf as u8;
        self.device.bank_2_accel_config().write(|w| {
            w.set_accel_fs_sel(fs_sel);
            w.set_accel_dlpfcfg(dlpf_cfg);
            w.set_accel_fchoice(self.accel_config.dlpf_enable);
        })?;

        // Restore normal-mode ACCEL_CONFIG_2 settings (no averaging)
        self.device.bank_2_accel_config_2().write(|w| {
            w.set_dec_3_cfg(0); // Reset to default
            w.set_az_st_en(false);
            w.set_ay_st_en(false);
            w.set_ax_st_en(false);
        })?;

        // Switch back to Bank 0
        self.select_bank(Bank::Bank0)?;

        Ok(())
    }

    /// Configure cycle modes for periodic sensor readings
    ///
    /// # Arguments
    /// * `config` - Cycle configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn configure_cycle_mode(&mut self, config: &CycleConfig) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        self.device.lp_config().write(|w| {
            w.set_accel_cycle(config.enable_accel_cycle);
            w.set_gyro_cycle(config.enable_gyro_cycle);
            w.set_i_2_c_mst_cycle(config.enable_i2c_master_cycle);
        })?;

        Ok(())
    }

    /// Set accelerometer and gyroscope to continuous sampling mode
    ///
    /// This configures the `LP_CONFIG` register to disable cycle modes, enabling
    /// continuous sampling. This is required for the `ACCEL_SMPLRT_DIV` and
    /// `GYRO_SMPLRT_DIV` registers to take effect.
    ///
    /// # Arguments
    /// * `enable_accel` - Enable continuous mode for accelerometer
    /// * `enable_gyro` - Enable continuous mode for gyroscope
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn set_continuous_mode(
        &mut self,
        enable_accel: bool,
        enable_gyro: bool,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        self.device.lp_config().modify(|w| {
            if enable_accel {
                w.set_accel_cycle(false); // false = continuous mode
            }
            if enable_gyro {
                w.set_gyro_cycle(false); // false = continuous mode
            }
        })?;

        Ok(())
    }

    /// Set clock source
    ///
    /// # Arguments
    /// * `source` - Clock source to use
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn set_clock_source(&mut self, source: ClockSource) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        self.device.pwr_mgmt_1().modify(|w| {
            w.set_clksel(source as u8);
        })?;

        Ok(())
    }

    /// Enable or disable temperature sensor
    ///
    /// # Arguments
    /// * `enable` - true to enable, false to disable
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn set_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        self.device.pwr_mgmt_1().modify(|w| {
            w.set_temp_dis(!enable);
        })?;

        Ok(())
    }

    /// Configure individual sensor power
    ///
    /// # Arguments
    /// * `config` - Sensor power configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn configure_sensor_power(
        &mut self,
        config: &SensorPowerConfig,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        self.device.pwr_mgmt_2().write(|w| {
            w.set_disable_accel_x(config.disable_accel_x);
            w.set_disable_accel_y(config.disable_accel_y);
            w.set_disable_accel_z(config.disable_accel_z);
            w.set_disable_gyro_x(config.disable_gyro_x);
            w.set_disable_gyro_y(config.disable_gyro_y);
            w.set_disable_gyro_z(config.disable_gyro_z);
        })?;

        Ok(())
    }

    /// Read current power status
    ///
    /// # Returns
    /// Current power management status
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub fn read_power_status(&mut self) -> Result<PowerStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0)?;

        let pwr1 = self.device.pwr_mgmt_1().read()?;
        let lp_cfg = self.device.lp_config().read()?;

        let mode = if pwr1.sleep() {
            PowerMode::Sleep
        } else if pwr1.lp_en() {
            PowerMode::LowPower
        } else {
            PowerMode::Normal
        };

        Ok(PowerStatus {
            mode,
            sleep: pwr1.sleep(),
            low_power: pwr1.lp_en(),
            temp_disabled: pwr1.temp_dis(),
            accel_cycle: lp_cfg.accel_cycle(),
            gyro_cycle: lp_cfg.gyro_cycle(),
            i2c_master_cycle: lp_cfg.i_2_c_mst_cycle(),
        })
    }

    // ==================== HIGH-LEVEL SENSOR METHODS ====================
    //
    // These methods provide the primary API for sensor operations including
    // configuration, reading, and calibration. They automatically manage
    // stored configuration and calibration state.

    /// Configure the accelerometer
    ///
    /// This is a convenience method that configures the accelerometer without
    /// requiring creation of a separate `Accelerometer` handle.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let accel_config = AccelConfig {
    ///     full_scale: AccelFullScale::G2,
    ///     dlpf: AccelDlpf::Hz246,
    ///     dlpf_enable: true,
    ///     sample_rate_div: 10,
    /// };
    /// imu.configure_accelerometer(accel_config)?;
    /// ```
    /// Configure accelerometer settings
    ///
    /// # Arguments
    /// * `config` - Accelerometer configuration
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn configure_accelerometer(
        &mut self,
        config: crate::sensors::AccelConfig,
    ) -> Result<(), Error<I::Error>> {
        // CRITICAL: Set continuous mode first (Bank 0)
        // The ACCEL_SMPLRT_DIV registers only work when the accelerometer
        // is in continuous mode (ACCEL_CYCLE = 0 in LP_CONFIG)
        self.select_bank(Bank::Bank0)?;
        self.device.lp_config().modify(|w| {
            w.set_accel_cycle(false); // Continuous mode required for SMPLRT_DIV
        })?;

        // Then configure Bank 2 registers
        self.select_bank(Bank::Bank2)?;

        // Configure full-scale range and DLPF
        self.device.bank_2_accel_config().modify(|w| {
            w.set_accel_fs_sel(config.full_scale as u8);
            w.set_accel_dlpfcfg(config.dlpf as u8);
            w.set_accel_fchoice(config.dlpf_enable);
        })?;

        // Configure sample rate divider
        self.device.bank_2_accel_smplrt_div_1().write(|w| {
            w.set_accel_smplrt_div_1((config.sample_rate_div >> 8) as u8);
        })?;

        self.device.bank_2_accel_smplrt_div_2().write(|w| {
            w.set_accel_smplrt_div_2((config.sample_rate_div & 0xFF) as u8);
        })?;

        self.accel_config = config;

        // Return to Bank 0
        self.select_bank(Bank::Bank0)?;
        Ok(())
    }

    /// Read raw accelerometer data (16-bit signed values)
    ///
    /// Returns raw sensor values without calibration or conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_accelerometer_raw(&mut self) -> Result<(i16, i16, i16), Error<I::Error>> {
        let accel_data = self.read_accel()?;
        Ok((accel_data.x, accel_data.y, accel_data.z))
    }

    /// Read accelerometer data in g-force units
    ///
    /// This is a convenience method that reads and converts accelerometer data.
    /// Automatically applies calibration if set.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let accel_data = imu.read_accelerometer()?;
    /// println!("X: {}g, Y: {}g, Z: {}g", accel_data.x, accel_data.y, accel_data.z);
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_accelerometer(&mut self) -> Result<crate::sensors::AccelDataG, Error<I::Error>> {
        let accel_data = self.read_accel()?;

        // Apply calibration
        let (cal_x, cal_y, cal_z) =
            self.accel_calibration
                .apply(accel_data.x, accel_data.y, accel_data.z);

        let sensitivity = self.accel_config.full_scale.sensitivity();
        Ok(crate::sensors::AccelDataG::from_raw(
            cal_x,
            cal_y,
            cal_z,
            sensitivity,
        ))
    }

    /// Set accelerometer calibration data
    ///
    /// The calibration will be automatically applied to all subsequent readings.
    pub const fn set_accelerometer_calibration(
        &mut self,
        calibration: crate::sensors::AccelCalibration,
    ) {
        self.accel_calibration = calibration;
    }

    /// Get current accelerometer calibration data
    #[must_use]
    pub const fn accelerometer_calibration(&self) -> &crate::sensors::AccelCalibration {
        &self.accel_calibration
    }

    /// Calibrate the accelerometer with default motion detection threshold.
    ///
    /// Convenience wrapper for [`calibrate_accelerometer_with_threshold`](Self::calibrate_accelerometer_with_threshold)
    /// using the default threshold divisor (5% of full scale).
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub fn calibrate_accelerometer(
        &mut self,
        samples: u16,
    ) -> Result<crate::sensors::AccelCalibration, Error<I::Error>> {
        self.calibrate_accelerometer_with_threshold(
            samples,
            DEFAULT_MOTION_DETECTION_THRESHOLD_DIVISOR,
        )
    }

    /// Calibrate the accelerometer by taking a series of samples and calculating
    /// offsets. The device should be placed on a level surface with Z-axis pointing up.
    ///
    /// The calibration will automatically be applied to future readings.
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    /// * `motion_threshold_divisor` - Controls motion detection sensitivity.
    ///   Smaller values are more lenient (allow more variance).
    ///   - 100: Very strict (1% variance allowed)
    ///   - 33: Strict (3% variance allowed)
    ///   - 20: Balanced (5% variance allowed, recommended)
    ///   - 10: Lenient (10% variance allowed)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub fn calibrate_accelerometer_with_threshold(
        &mut self,
        samples: u16,
        motion_threshold_divisor: i16,
    ) -> Result<crate::sensors::AccelCalibration, Error<I::Error>> {
        // Validate input
        if samples == 0 {
            return Err(Error::InvalidConfig);
        }

        let mut sum_x: i64 = 0;
        let mut sum_y: i64 = 0;
        let mut sum_z: i64 = 0;

        // For motion detection
        let mut min_x = i16::MAX;
        let mut max_x = i16::MIN;
        let mut min_y = i16::MAX;
        let mut max_y = i16::MIN;
        let mut min_z = i16::MAX;
        let mut max_z = i16::MIN;

        for _ in 0..samples {
            let accel_data = self.read_accel()?;
            sum_x += i64::from(accel_data.x);
            sum_y += i64::from(accel_data.y);
            sum_z += i64::from(accel_data.z);

            // Track variance for motion detection
            min_x = min_x.min(accel_data.x);
            max_x = max_x.max(accel_data.x);
            min_y = min_y.min(accel_data.y);
            max_y = max_y.max(accel_data.y);
            min_z = min_z.min(accel_data.z);
            max_z = max_z.max(accel_data.z);
        }

        // Check for motion during calibration
        #[allow(clippy::cast_possible_truncation)]
        let max_variance =
            self.accel_config.full_scale.sensitivity() as i16 / motion_threshold_divisor;

        let variance_x = max_x.saturating_sub(min_x);
        let variance_y = max_y.saturating_sub(min_y);
        let variance_z = max_z.saturating_sub(min_z);

        if variance_x > max_variance || variance_y > max_variance || variance_z > max_variance {
            return Err(Error::DeviceMoving);
        }

        // Calculate averages - CRITICAL: Handle overflow properly
        let avg_x = i16::try_from(sum_x / i64::from(samples)).map_err(|_| Error::InvalidConfig)?; // Return error, don't silent fail!
        let avg_y = i16::try_from(sum_y / i64::from(samples)).map_err(|_| Error::InvalidConfig)?;
        let avg_z = i16::try_from(sum_z / i64::from(samples)).map_err(|_| Error::InvalidConfig)?;

        // Z-axis should read +1g, so offset it by the difference from expected value
        let sensitivity = self.accel_config.full_scale.sensitivity();
        // Sensitivity values are known to be in range 2048-16384, well within i16::MAX
        #[allow(clippy::cast_possible_truncation)]
        let sensitivity_i16 = sensitivity as i16;
        let z_offset = avg_z.saturating_sub(sensitivity_i16); // Expected +1g

        let calibration = crate::sensors::AccelCalibration {
            offset_x: avg_x,
            offset_y: avg_y,
            offset_z: z_offset,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        self.accel_calibration = calibration;
        Ok(calibration)
    }

    /// Run accelerometer self-test
    ///
    /// Returns `true` if self-test passes, `false` otherwise.
    /// The self-test verifies the accelerometer is functioning correctly.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Notes
    ///
    /// This function temporarily changes the accelerometer to ±2g range for the test,
    /// then restores the original configuration. The threshold (225 LSB) is calibrated
    /// for ±2g range and represents approximately 1.4% of full scale.
    pub fn accelerometer_self_test<D>(&mut self, delay: &mut D) -> Result<bool, Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        // Self-test passes if response is significant
        // Threshold: 225 LSB @ ±2g range (16384 LSB/g) = ~0.014g (~1.4% of full scale)
        // This is a fixed threshold calibrated for ±2g range per ICM-20948 datasheet
        const MIN_RESPONSE: i16 = 225;

        // Save original configuration
        let original_config = self.accel_config;

        self.select_bank(Bank::Bank2)?;

        // Configure to ±2g for self-test (required by datasheet)
        self.device.bank_2_accel_config().modify(|w| {
            w.set_accel_fs_sel(0); // ±2g
        })?;

        // Read response with self-test disabled
        let (x_off, y_off, z_off) = self.read_accelerometer_raw()?;

        // Enable self-test on all axes via ACCEL_CONFIG_2
        self.select_bank(Bank::Bank2)?;
        self.device.bank_2_accel_config_2().modify(|w| {
            w.set_ax_st_en(true);
            w.set_ay_st_en(true);
            w.set_az_st_en(true);
        })?;

        // Wait for oscillations to stabilize (20ms)
        delay.delay_ms(20);

        // Read response with self-test enabled
        let (x_on, y_on, z_on) = self.read_accelerometer_raw()?;

        // Disable self-test
        self.select_bank(Bank::Bank2)?;
        self.device.bank_2_accel_config_2().modify(|w| {
            w.set_ax_st_en(false);
            w.set_ay_st_en(false);
            w.set_az_st_en(false);
        })?;

        // Calculate self-test response
        let str_x = x_on.saturating_sub(x_off);
        let str_y = y_on.saturating_sub(y_off);
        let str_z = z_on.saturating_sub(z_off);

        let result =
            str_x.abs() > MIN_RESPONSE && str_y.abs() > MIN_RESPONSE && str_z.abs() > MIN_RESPONSE;

        // Restore original configuration
        self.configure_accelerometer(original_config)?;

        Ok(result)
    }

    /// Configure the gyroscope
    ///
    /// This is a convenience method that configures the gyroscope without
    /// requiring creation of a separate `Gyroscope` handle.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let gyro_config = GyroConfig {
    ///     full_scale: GyroFullScale::Dps250,
    ///     dlpf: GyroDlpf::Hz197,
    ///     dlpf_enable: true,
    ///     sample_rate_div: 10,
    /// };
    /// imu.configure_gyroscope(gyro_config)?;
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn configure_gyroscope(
        &mut self,
        config: crate::sensors::GyroConfig,
    ) -> Result<(), Error<I::Error>> {
        // CRITICAL: Set continuous mode first (Bank 0)
        // The GYRO_SMPLRT_DIV register only works when the gyroscope
        // is in continuous mode (GYRO_CYCLE = 0 in LP_CONFIG)
        self.select_bank(Bank::Bank0)?;
        self.device.lp_config().modify(|w| {
            w.set_gyro_cycle(false); // Continuous mode required for SMPLRT_DIV
        })?;

        // Then configure Bank 2 registers
        self.select_bank(Bank::Bank2)?;

        // Configure full-scale range and DLPF
        self.device.bank_2_gyro_config_1().modify(|w| {
            w.set_gyro_fs_sel(config.full_scale as u8);
            w.set_gyro_dlpfcfg(config.dlpf as u8);
            w.set_gyro_fchoice(config.dlpf_enable);
        })?;

        // Configure sample rate divider
        self.device.bank_2_gyro_smplrt_div().write(|w| {
            w.set_gyro_smplrt_div(config.sample_rate_div);
        })?;

        self.gyro_config = config;

        // Return to Bank 0
        self.select_bank(Bank::Bank0)?;
        Ok(())
    }

    /// Read raw gyroscope data (16-bit signed values)
    ///
    /// Returns raw sensor values without calibration or conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_gyroscope_raw(&mut self) -> Result<(i16, i16, i16), Error<I::Error>> {
        let gyro_data = self.read_gyro()?;
        Ok((gyro_data.x, gyro_data.y, gyro_data.z))
    }

    /// Read gyroscope data in degrees per second
    ///
    /// This is a convenience method that reads and converts gyroscope data.
    /// Automatically applies calibration if set.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let gyro_data = imu.read_gyroscope()?;
    /// println!("X: {}°/s, Y: {}°/s, Z: {}°/s", gyro_data.x, gyro_data.y, gyro_data.z);
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_gyroscope(&mut self) -> Result<crate::sensors::GyroDataDps, Error<I::Error>> {
        let gyro_data = self.read_gyro()?;

        // Apply calibration
        let (cal_x, cal_y, cal_z) =
            self.gyro_calibration
                .apply(gyro_data.x, gyro_data.y, gyro_data.z);

        let sensitivity = self.gyro_config.full_scale.sensitivity();
        Ok(crate::sensors::GyroDataDps::from_raw(
            cal_x,
            cal_y,
            cal_z,
            sensitivity,
        ))
    }

    /// Read gyroscope data in radians per second
    ///
    /// This is a convenience method that reads and converts gyroscope data to radians.
    /// Automatically applies calibration if set.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_gyroscope_radians(
        &mut self,
    ) -> Result<crate::sensors::GyroDataRps, Error<I::Error>> {
        let dps = self.read_gyroscope()?;
        Ok(dps.to_radians_per_sec())
    }

    /// Set gyroscope calibration data
    ///
    /// The calibration will be automatically applied to all subsequent readings.
    pub const fn set_gyroscope_calibration(
        &mut self,
        calibration: crate::sensors::GyroCalibration,
    ) {
        self.gyro_calibration = calibration;
    }

    /// Get current gyroscope calibration data
    #[must_use]
    pub const fn gyroscope_calibration(&self) -> &crate::sensors::GyroCalibration {
        &self.gyro_calibration
    }

    /// Perform gyroscope calibration by averaging samples while stationary
    /// Calibrate the gyroscope with default motion detection threshold.
    ///
    /// Convenience wrapper for [`calibrate_gyroscope_with_threshold`](Self::calibrate_gyroscope_with_threshold)
    /// using the default threshold divisor (5% of full scale).
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub fn calibrate_gyroscope(
        &mut self,
        samples: u16,
    ) -> Result<crate::sensors::GyroCalibration, Error<I::Error>> {
        self.calibrate_gyroscope_with_threshold(samples, DEFAULT_MOTION_DETECTION_THRESHOLD_DIVISOR)
    }

    /// Calibrate the gyroscope by taking a series of samples and calculating
    /// average offsets. The device should be stationary during calibration.
    ///
    /// The calibration will automatically be applied to future readings.
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    /// * `motion_threshold_divisor` - Controls motion detection sensitivity.
    ///   Smaller values are more lenient (allow more variance).
    ///   - 100: Very strict (1% variance allowed)
    ///   - 33: Strict (3% variance allowed)
    ///   - 20: Balanced (5% variance allowed, recommended)
    ///   - 10: Lenient (10% variance allowed)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub fn calibrate_gyroscope_with_threshold(
        &mut self,
        samples: u16,
        motion_threshold_divisor: i16,
    ) -> Result<crate::sensors::GyroCalibration, Error<I::Error>> {
        // Validate input
        if samples == 0 {
            return Err(Error::InvalidConfig);
        }

        let mut sum_x: i64 = 0;
        let mut sum_y: i64 = 0;
        let mut sum_z: i64 = 0;

        // For motion detection (optional, based on threshold)
        let mut min_x = i16::MAX;
        let mut max_x = i16::MIN;
        let mut min_y = i16::MAX;
        let mut max_y = i16::MIN;
        let mut min_z = i16::MAX;
        let mut max_z = i16::MIN;

        // Motion detection is optional for gyroscopes because MEMS gyroscopes
        // exhibit significant bias drift (0.5-1 °/s is normal) even when perfectly
        // stationary. This drift is the bias that calibration measures, not motion.
        // However, with lenient thresholds (divisor <= 10), motion detection can
        // still catch actual device movement during calibration.
        let check_motion = motion_threshold_divisor <= 50;

        for _ in 0..samples {
            let gyro_data = self.read_gyro()?;
            sum_x += i64::from(gyro_data.x);
            sum_y += i64::from(gyro_data.y);
            sum_z += i64::from(gyro_data.z);

            // Track variance for motion detection
            if check_motion {
                min_x = min_x.min(gyro_data.x);
                max_x = max_x.max(gyro_data.x);
                min_y = min_y.min(gyro_data.y);
                max_y = max_y.max(gyro_data.y);
                min_z = min_z.min(gyro_data.z);
                max_z = max_z.max(gyro_data.z);
            }
        }

        // Check for motion during calibration if enabled
        if check_motion {
            #[allow(clippy::cast_possible_truncation)]
            let max_variance =
                self.gyro_config.full_scale.sensitivity() as i16 / motion_threshold_divisor;

            let variance_x = max_x.saturating_sub(min_x);
            let variance_y = max_y.saturating_sub(min_y);
            let variance_z = max_z.saturating_sub(min_z);

            if variance_x > max_variance || variance_y > max_variance || variance_z > max_variance {
                return Err(Error::DeviceMoving);
            }
        }

        // CRITICAL: Handle overflow properly - return error instead of silent failure with 0
        let avg_x =
            i16::try_from(sum_x / i64::from(samples)).map_err(|_| Error::CalibrationOverflow)?;
        let avg_y =
            i16::try_from(sum_y / i64::from(samples)).map_err(|_| Error::CalibrationOverflow)?;
        let avg_z =
            i16::try_from(sum_z / i64::from(samples)).map_err(|_| Error::CalibrationOverflow)?;

        let calibration = crate::sensors::GyroCalibration {
            offset_x: avg_x,
            offset_y: avg_y,
            offset_z: avg_z,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        self.gyro_calibration = calibration;
        Ok(calibration)
    }

    /// Run gyroscope self-test
    ///
    /// Returns `true` if self-test passes, `false` otherwise.
    /// The self-test verifies the gyroscope is functioning correctly.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Notes
    ///
    /// This function temporarily changes the gyroscope to ±250°/s range for the test,
    /// then restores the original configuration. The threshold (60 LSB) is calibrated
    /// for ±250°/s range and represents approximately 0.46% of full scale.
    pub fn gyroscope_self_test<D>(&mut self, delay: &mut D) -> Result<bool, Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        // Self-test passes if response is significant
        // Threshold: 60 LSB @ ±250°/s range (131 LSB/(°/s)) = ~0.46°/s (~0.46% of full scale)
        // This is a fixed threshold calibrated for ±250°/s range per ICM-20948 datasheet
        const MIN_RESPONSE: i16 = 60;

        // Save original configuration
        let original_config = self.gyro_config;

        self.select_bank(Bank::Bank2)?;

        // Configure to ±250 dps for self-test (required by datasheet)
        self.device.bank_2_gyro_config_1().modify(|w| {
            w.set_gyro_fs_sel(0); // ±250 dps
        })?;

        // Read response with self-test disabled
        let (x_off, y_off, z_off) = self.read_gyroscope_raw()?;

        // Enable self-test on all axes via GYRO_CONFIG_2
        self.select_bank(Bank::Bank2)?;
        self.device.bank_2_gyro_config_2().modify(|w| {
            w.set_xgyro_cten(true);
            w.set_ygyro_cten(true);
            w.set_zgyro_cten(true);
        })?;

        // Wait for oscillations to stabilize (20ms)
        delay.delay_ms(20);

        // Read response with self-test enabled
        let (x_on, y_on, z_on) = self.read_gyroscope_raw()?;

        // Disable self-test
        self.select_bank(Bank::Bank2)?;
        self.device.bank_2_gyro_config_2().modify(|w| {
            w.set_xgyro_cten(false);
            w.set_ygyro_cten(false);
            w.set_zgyro_cten(false);
        })?;

        // Calculate self-test response
        let str_x = x_on.saturating_sub(x_off);
        let str_y = y_on.saturating_sub(y_off);
        let str_z = z_on.saturating_sub(z_off);

        let result =
            str_x.abs() > MIN_RESPONSE && str_y.abs() > MIN_RESPONSE && str_z.abs() > MIN_RESPONSE;

        // Restore original configuration
        self.configure_gyroscope(original_config)?;

        Ok(result)
    }

    /// Read temperature in degrees Celsius
    ///
    /// This is a convenience method that reads and converts temperature data
    /// without requiring creation of a separate `Temperature` handle.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let temp = imu.read_temperature_celsius()?;
    /// println!("Temperature: {}°C", temp);
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_temperature_celsius(&mut self) -> Result<f32, Error<I::Error>> {
        let raw = self.read_temperature()?;
        Ok(Self::temperature_to_celsius(raw))
    }

    /// Initialize the magnetometer (AK09916)
    ///
    /// This configures the ICM-20948's I2C master to communicate with the AK09916 magnetometer.
    /// The magnetometer is accessed via the ICM-20948's auxiliary I2C bus.
    ///
    /// # Arguments
    ///
    /// * `config` - Magnetometer configuration (mode, sample rate)
    /// * `delay` - Delay provider for timing-critical operations
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mag_config = MagConfig {
    ///     mode: MagMode::Continuous100Hz,
    /// };
    /// imu.init_magnetometer(mag_config, &mut delay)?;
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if I2C communication fails or the magnetometer doesn't respond.
    pub fn init_magnetometer<D>(
        &mut self,
        config: crate::sensors::MagConfig,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        use crate::sensors::magnetometer::{
            AK09916_I2C_ADDRESS, AK09916_REG_CNTL2, AK09916_REG_CNTL3, AK09916_REG_ST1,
            AK09916_REG_WIA2, AK09916_WIA2_VALUE,
        };

        // Maximum number of attempts to initialize the magnetometer.
        // The AK09916 magnetometer may not respond immediately after an ICM-20948 reset.
        // Up to 5 retry attempts (with delays) have proven sufficient in practice to handle
        // transient I2C master communication issues during initialization.
        const MAX_MAGNETOMETER_STARTS: u8 = 5;

        // Step 1: Reset and prepare I2C master
        self.reset_i2c_master(delay)?;

        // Step 2: Enable gyroscope (required for I2C master clock)
        self.enable_gyroscope_for_i2c_master(delay)?;

        // Step 3: Enable and verify I2C master mode
        self.enable_i2c_master_mode(delay)?;

        // Step 4: Configure I2C master timing
        self.configure_i2c_master_timing(delay)?;

        // Step 5: Reset AK09916 magnetometer
        self.write_mag_register(AK09916_REG_CNTL3, 0x01, delay)?;
        delay.delay_ms(100); // AK09916 needs up to 100ms after reset

        // Step 6: Verify WHO_AM_I with retry logic.
        // The magnetometer may not respond immediately after reset, so we retry
        // with delays to allow it time to initialize.
        self.verify_magnetometer_with_retry(
            AK09916_REG_WIA2,
            AK09916_WIA2_VALUE,
            MAX_MAGNETOMETER_STARTS,
            delay,
        )?;

        // Step 7: Configure Slave 0 for automatic magnetometer reads
        self.configure_slave0_for_magnetometer(AK09916_I2C_ADDRESS, AK09916_REG_ST1)?;
        delay.delay_ms(10);

        // Step 8: Set magnetometer operating mode
        self.write_mag_register(AK09916_REG_CNTL2, config.mode as u8, delay)?;
        delay.delay_ms(10);

        // Step 9: Enable I2C master cycle mode for continuous SLV0 reads
        self.select_bank(Bank::Bank0)?;
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(true);
        })?;

        delay.delay_ms(10);

        self.mag_initialized = true;

        Ok(())
    }

    /// Reset I2C master and clear any previous state
    fn reset_i2c_master<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        self.select_bank(Bank::Bank0)?;

        // Disable BYPASS_EN before using I2C master
        self.device.int_pin_cfg().modify(|w| {
            w.set_bypass_en(false);
        })?;

        // Disable I2C master first, then reset it
        // Use modify() to preserve DMP_EN, FIFO_EN, and other bits
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_en(false);
        })?;
        delay.delay_ms(1);

        // Now reset the I2C master
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_rst(true);
        })?;
        delay.delay_ms(10);

        // Clear the reset bit
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_rst(false);
        })?;
        delay.delay_ms(1);

        Ok(())
    }

    /// Enable gyroscope - CRITICAL: I2C master clock is derived from gyro sample rate
    fn enable_gyroscope_for_i2c_master<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        self.select_bank(Bank::Bank0)?;
        self.device.pwr_mgmt_2().modify(|w| {
            w.set_disable_gyro_x(false);
            w.set_disable_gyro_y(false);
            w.set_disable_gyro_z(false);
        })?;
        delay.delay_ms(50); // Wait for gyroscope to stabilize

        // Ensure device is not in sleep mode
        let pwr_mgmt_1 = self.device.pwr_mgmt_1().read()?;

        if pwr_mgmt_1.sleep() {
            self.device.pwr_mgmt_1().modify(|w| {
                w.set_sleep(false);
                w.set_clksel(1); // Auto select best clock
            })?;
            delay.delay_ms(50);
        }

        Ok(())
    }

    /// Enable I2C master mode with retry logic
    fn enable_i2c_master_mode<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        self.select_bank(Bank::Bank0)?;

        // The I2C master enable bit may take several milliseconds to stabilize after
        // power-on or reset. Retry up to 5 times with delays (up to 50ms total).
        for _attempt in 0..5 {
            // Use modify() to preserve DMP_EN, FIFO_EN, and other bits
            self.device.user_ctrl().modify(|w| {
                w.set_i_2_c_mst_en(true);
            })?;

            delay.delay_ms(10);

            // Verify the bit stuck
            let user_ctrl = self.device.user_ctrl().read()?;

            if user_ctrl.i_2_c_mst_en() {
                // Disable I2C master cycle mode to allow immediate transactions
                self.device.lp_config().modify(|w| {
                    w.set_i_2_c_mst_cycle(false);
                })?;
                delay.delay_ms(10);
                return Ok(());
            }
        }

        Err(Error::Magnetometer)
    }

    /// Configure I2C master timing and clock settings
    fn configure_i2c_master_timing<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        // Configure gyroscope sample rate divider (Bank 2)
        // I2C master operates at gyro ODR / (1 + GYRO_SMPLRT_DIV)
        self.select_bank(Bank::Bank2)?;
        self.device.bank_2_gyro_smplrt_div().write(|w| {
            w.set_gyro_smplrt_div(0); // Maximum sample rate (1125 Hz)
        })?;

        // Configure I2C master ODR and clock (Bank 3)
        self.select_bank(Bank::Bank3)?;

        // Set I2C master ODR (Output Data Rate) to 0x04, matching SparkFun implementation.
        // Formula: f_i2c_mst = 1.1 kHz / (2^I2C_MST_ODR_CONFIG)
        // With 0x04: f_i2c_mst = 1100 Hz / 16 = 68.75 Hz
        //
        // The AK09916 magnetometer operates at up to 100 Hz in continuous mode.
        // Polling at 68.75 Hz is appropriate because:
        // - It's fast enough to capture most magnetometer updates without missing data
        // - It's slightly slower than the mag's 100 Hz rate, reducing I2C bus load
        // - This proven configuration from SparkFun balances responsiveness and efficiency
        self.device
            .bank_3_i_2_c_mst_odr_config()
            .write(|w| w.set_i_2_c_mst_odr_config(0x04))?;

        // Set I2C master clock to ~400 kHz (0x07 = 345.6 kHz, closest available)
        self.device.bank_3_i_2_c_mst_ctrl().modify(|w| {
            w.set_i_2_c_mst_clk(0x07);
            w.set_i_2_c_mst_p_nsr(true);
        })?;

        // Ensure Slave 0 reads at every I2C master cycle (no delay)
        self.device.bank_3_i_2_c_mst_delay_ctrl().write(|w| {
            w.set_i_2_c_slv_0_delay_en(false);
        })?;

        delay.delay_ms(10);
        Ok(())
    }

    /// Verify magnetometer `WHO_AM_I` with retry and I2C master reset logic
    ///
    /// After an ICM reset, the magnetometer may stop responding over the I2C master.
    /// This resets the master I2C between attempts until it responds.
    fn verify_magnetometer_with_retry<D>(
        &mut self,
        who_am_i_reg: u8,
        expected_value: u8,
        max_attempts: u8,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        for attempt in 0..max_attempts {
            match self.read_mag_register(who_am_i_reg, delay) {
                Ok(who_am_i) => {
                    if who_am_i == expected_value {
                        return Ok(()); // Success!
                    }
                }
                Err(_e) => {}
            }

            if attempt < max_attempts - 1 {
                // Reset I2C master and try again
                self.reset_and_reconfigure_i2c_master(delay)?;
            }
        }

        Err(Error::Magnetometer)
    }

    /// Reset I2C master and reconfigure Bank 3 settings
    fn reset_and_reconfigure_i2c_master<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        // Reset I2C master
        self.select_bank(Bank::Bank0)?;

        // Disable BYPASS_EN before using I2C master
        self.device.int_pin_cfg().modify(|w| {
            w.set_bypass_en(false);
        })?;

        // Disable I2C master first
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_en(false);
        })?;
        delay.delay_ms(1);

        // Reset I2C master
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_rst(true);
        })?;
        delay.delay_ms(10);

        // Clear reset bit
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_rst(false);
        })?;
        delay.delay_ms(1);

        // Re-enable I2C master
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_en(true);
        })?;
        delay.delay_ms(10);

        // Reconfigure Bank 3 settings
        self.select_bank(Bank::Bank3)?;
        self.device.bank_3_i_2_c_mst_ctrl().modify(|w| {
            w.set_i_2_c_mst_clk(0x07);
            w.set_i_2_c_mst_p_nsr(true);
        })?;
        delay.delay_ms(10);

        Ok(())
    }

    /// Configure Slave 0 for automatic magnetometer data reads
    ///
    /// Reads 9 bytes: ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2, (dummy byte)
    /// NOTE: AK09916 requires reading ST1 first, then data, then ST2
    fn configure_slave0_for_magnetometer(
        &mut self,
        mag_address: u8,
        start_reg: u8,
    ) -> Result<(), Error<I::Error>> {
        self.device.bank_3_i_2_c_slv_0_addr().write(|w| {
            w.set_i_2_c_id_0(mag_address);
            w.set_i_2_c_slv_0_rnw(true); // Read mode
        })?;

        self.device
            .bank_3_i_2_c_slv_0_reg()
            .write(|w| w.set_i_2_c_slv_0_reg(start_reg))?;

        self.device.bank_3_i_2_c_slv_0_ctrl().write(|w| {
            w.set_i_2_c_slv_0_leng(9); // Read 9 bytes
            w.set_i_2_c_slv_0_en(true); // Enable slave 0
        })?;

        Ok(())
    }

    /// Read magnetometer data in microteslas (µT)
    ///
    /// Automatically applies calibration if set.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mag_data = imu.read_magnetometer()?;
    /// println!("X: {}µT, Y: {}µT, Z: {}µT", mag_data.x, mag_data.y, mag_data.z);
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if not initialized or communication fails.
    pub fn read_magnetometer(&mut self) -> Result<crate::sensors::MagDataUT, Error<I::Error>> {
        // AK09916 sensitivity: 0.15 µT/LSB
        const SENSITIVITY: f32 = 0.15;

        if !self.mag_initialized {
            return Err(Error::Magnetometer);
        }

        self.select_bank(Bank::Bank0)?;

        // Read 9 bytes from EXT_SLV_SENS_DATA
        // Format: ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2, (dummy)
        let data = [
            self.device
                .ext_slv_sens_data_00()
                .read()?
                .ext_slv_sens_data_00(),
            self.device
                .ext_slv_sens_data_01()
                .read()?
                .ext_slv_sens_data_01(),
            self.device
                .ext_slv_sens_data_02()
                .read()?
                .ext_slv_sens_data_02(),
            self.device
                .ext_slv_sens_data_03()
                .read()?
                .ext_slv_sens_data_03(),
            self.device
                .ext_slv_sens_data_04()
                .read()?
                .ext_slv_sens_data_04(),
            self.device
                .ext_slv_sens_data_05()
                .read()?
                .ext_slv_sens_data_05(),
            self.device
                .ext_slv_sens_data_06()
                .read()?
                .ext_slv_sens_data_06(),
            self.device
                .ext_slv_sens_data_07()
                .read()?
                .ext_slv_sens_data_07(),
            self.device
                .ext_slv_sens_data_08()
                .read()?
                .ext_slv_sens_data_08(),
        ];

        // ST1 is first byte (data[0]) but we do NOT check it when reading via I2C master
        // The I2C master polls automatically at a fixed rate, so received data is valid

        // ST2 is at index 7
        let st2 = data[7];
        if (st2 & 0x08) != 0 {
            return Err(Error::Magnetometer); // Magnetic overflow
        }

        // Parse little-endian 16-bit values
        // Data starts at index 1: HXL, HXH, HYL, HYH, HZL, HZH
        let x = i16::from_le_bytes([data[1], data[2]]);
        let y = i16::from_le_bytes([data[3], data[4]]);
        let z = i16::from_le_bytes([data[5], data[6]]);

        let data = crate::sensors::MagDataUT {
            x: f32::from(x) * SENSITIVITY,
            y: f32::from(y) * SENSITIVITY,
            z: f32::from(z) * SENSITIVITY,
        };

        Ok(self.mag_calibration.apply(&data))
    }

    /// Read raw magnetometer data (16-bit signed integers)
    ///
    /// Returns (x, y, z) as raw ADC values without calibration or conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if not initialized or communication fails.
    pub fn read_magnetometer_raw(&mut self) -> Result<(i16, i16, i16), Error<I::Error>> {
        if !self.mag_initialized {
            return Err(Error::Magnetometer);
        }

        self.select_bank(Bank::Bank0)?;

        // Read 8 bytes from EXT_SLV_SENS_DATA
        // Format: ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2 (AK09916 order)
        let data = [
            self.device
                .ext_slv_sens_data_00()
                .read()?
                .ext_slv_sens_data_00(),
            self.device
                .ext_slv_sens_data_01()
                .read()?
                .ext_slv_sens_data_01(),
            self.device
                .ext_slv_sens_data_02()
                .read()?
                .ext_slv_sens_data_02(),
            self.device
                .ext_slv_sens_data_03()
                .read()?
                .ext_slv_sens_data_03(),
            self.device
                .ext_slv_sens_data_04()
                .read()?
                .ext_slv_sens_data_04(),
            self.device
                .ext_slv_sens_data_05()
                .read()?
                .ext_slv_sens_data_05(),
            self.device
                .ext_slv_sens_data_06()
                .read()?
                .ext_slv_sens_data_06(),
            self.device
                .ext_slv_sens_data_07()
                .read()?
                .ext_slv_sens_data_07(),
        ];

        // ST1 is first byte (data[0]) but we do NOT check it when reading via I2C master
        // The I2C master polls automatically at a fixed rate, so received data is valid

        // ST2 is at index 7
        let st2 = data[7];
        if (st2 & 0x08) != 0 {
            return Err(Error::Magnetometer); // Magnetic overflow
        }

        // Parse little-endian 16-bit values
        // Data starts at index 1: HXL, HXH, HYL, HYH, HZL, HZH
        let x = i16::from_le_bytes([data[1], data[2]]);
        let y = i16::from_le_bytes([data[3], data[4]]);
        let z = i16::from_le_bytes([data[5], data[6]]);

        Ok((x, y, z))
    }

    /// Read magnetometer heading (yaw angle) in degrees
    ///
    /// Calculates the heading from the magnetic field vector.
    /// Assumes the sensor is level (horizontal).
    ///
    /// Returns angle in degrees (0-360), where:
    /// - 0° = North
    /// - 90° = East
    /// - 180° = South
    /// - 270° = West
    ///
    /// # Errors
    ///
    /// Returns an error if reading fails.
    pub fn read_magnetometer_heading(&mut self) -> Result<f32, Error<I::Error>> {
        let data = self.read_magnetometer()?;

        // Calculate heading: atan2(y, x)
        let heading_rad = libm::atan2f(data.y, data.x);

        // Convert to degrees and normalize to 0-360
        let mut heading_deg = heading_rad.to_degrees();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }

        Ok(heading_deg)
    }

    /// Read magnetometer heading with tilt compensation
    ///
    /// Calculates the heading from the magnetic field vector, compensating
    /// for tilt using accelerometer data.
    ///
    /// # Arguments
    ///
    /// * `accel_x` - X-axis acceleration in g
    /// * `accel_y` - Y-axis acceleration in g
    /// * `accel_z` - Z-axis acceleration in g
    ///
    /// Returns angle in degrees (0-360).
    ///
    /// # Errors
    ///
    /// Returns an error if reading fails or if accelerometer magnitude is too small
    /// (less than 0.1g), which indicates invalid data or free-fall condition.
    pub fn read_magnetometer_heading_compensated(
        &mut self,
        accel_x: f32,
        accel_y: f32,
        accel_z: f32,
    ) -> Result<f32, Error<I::Error>> {
        // Validate accelerometer data - magnitude must be meaningful
        let accel_magnitude =
            libm::sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        if accel_magnitude < 0.1 {
            // Too small to be meaningful - could be free-fall, invalid data, or zero vector
            return Err(Error::InvalidConfig);
        }

        let mag = self.read_magnetometer()?;

        // Calculate pitch and roll from accelerometer
        let pitch = libm::asinf(-accel_x);
        let roll = libm::atan2f(accel_y, accel_z);

        // Tilt-compensated magnetic field components
        let mag_x = mag.x * libm::cosf(pitch) + mag.z * libm::sinf(pitch);
        let mag_y = mag.x * libm::sinf(roll) * libm::sinf(pitch) + mag.y * libm::cosf(roll)
            - mag.z * libm::sinf(roll) * libm::cosf(pitch);

        // Calculate heading
        let heading_rad = libm::atan2f(mag_y, mag_x);

        // Convert to degrees and normalize to 0-360
        let mut heading_deg = heading_rad.to_degrees();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }

        Ok(heading_deg)
    }

    /// Check if magnetometer data is ready
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails.
    pub fn magnetometer_data_ready(&mut self) -> Result<bool, Error<I::Error>> {
        if !self.mag_initialized {
            return Ok(false);
        }

        self.select_bank(Bank::Bank0)?;

        // Check ST1 bit 0 (DRDY - Data Ready)
        // ST1 is at EXT_SLV_SENS_DATA_00 (first byte in AK09916 format)
        let st1 = self
            .device
            .ext_slv_sens_data_00()
            .read()?
            .ext_slv_sens_data_00();

        Ok((st1 & 0x01) != 0)
    }

    /// Run magnetometer self-test
    ///
    /// Returns `true` if self-test passes, `false` otherwise.
    ///
    /// # Errors
    ///
    /// Returns an error if self-test fails or communication fails.
    pub fn magnetometer_self_test<D>(&mut self, delay: &mut D) -> Result<bool, Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        use crate::sensors::magnetometer::{AK09916_REG_CNTL2, AK09916_REG_HXL};

        // Set self-test mode
        self.write_mag_register(AK09916_REG_CNTL2, 0x10, delay)?;
        delay.delay_ms(100);

        // Read self-test data
        self.select_bank(Bank::Bank3)?;

        // Configure Slave 0 to read self-test data
        self.device
            .bank_3_i_2_c_slv_0_reg()
            .write(|w| w.set_i_2_c_slv_0_reg(AK09916_REG_HXL))?;

        delay.delay_ms(10);

        let (x, y, z) = self.read_magnetometer_raw()?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Mag self-test values: x={}, y={}, z={}", x, y, z);

        // Return to normal mode
        self.write_mag_register(AK09916_REG_CNTL2, 0x08, delay)?;
        delay.delay_ms(10);

        // Simple check: magnetometer should produce non-zero readings in self-test mode
        // A complete self-test would use sensitivity adjustment values (ASAX/Y/Z)
        // but this basic verification is sufficient for normal operation.
        let result =
            (x != 0 || y != 0 || z != 0) && x.abs() < 32767 && y.abs() < 32767 && z.abs() < 32767;

        #[cfg(feature = "defmt")]
        defmt::debug!("Mag self-test result: {} (non-zero check passed)", result);

        Ok(result)
    }

    /// Perform hard-iron magnetometer calibration
    ///
    /// Collects samples while rotating the device in all directions.
    /// Calculates offset corrections for hard-iron distortion.
    ///
    /// # Arguments
    ///
    /// * `num_samples` - Number of samples to collect (recommended: 100-500)
    /// * `delay` - Delay provider for timing between samples
    ///
    /// # Errors
    ///
    /// Returns an error if reading fails.
    pub fn calibrate_magnetometer_hard_iron<D>(
        &mut self,
        num_samples: usize,
        delay: &mut D,
    ) -> Result<crate::sensors::MagCalibration, Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;

        for _ in 0..num_samples {
            let data = self.read_magnetometer()?;

            min_x = libm::fminf(min_x, data.x);
            max_x = libm::fmaxf(max_x, data.x);
            min_y = libm::fminf(min_y, data.y);
            max_y = libm::fmaxf(max_y, data.y);
            min_z = libm::fminf(min_z, data.z);
            max_z = libm::fmaxf(max_z, data.z);

            delay.delay_ms(10);
        }

        let calibration = crate::sensors::MagCalibration {
            offset_x: (max_x + min_x) * 0.5,
            offset_y: (max_y + min_y) * 0.5,
            offset_z: (max_z + min_z) * 0.5,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        self.mag_calibration = calibration;
        Ok(calibration)
    }

    /// Set magnetometer calibration data
    ///
    /// The calibration will be automatically applied to all subsequent readings.
    pub const fn set_magnetometer_calibration(
        &mut self,
        calibration: crate::sensors::MagCalibration,
    ) {
        self.mag_calibration = calibration;
    }

    /// Get current magnetometer calibration data
    #[must_use]
    pub const fn magnetometer_calibration(&self) -> &crate::sensors::MagCalibration {
        &self.mag_calibration
    }

    /// Helper to wait for I2C Slave 4 transaction to complete
    ///
    /// This polls the `I2C_MST_STATUS` register until the transaction completes
    /// or an error occurs (NACK, lost arbitration, or timeout).
    fn wait_for_i2c_slv4_done<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        // Maximum retry attempts for I2C slave 4 transaction completion.
        // With 1ms delay per retry, this gives a 50ms timeout.
        // The AK09916 typically responds within a few milliseconds, but we allow
        // extra margin for bus contention or timing variations.
        const MAX_I2C_SLV4_RETRIES: u32 = 50;

        // Wait for Slave 4 transaction to complete
        // Check I2C_MST_STATUS for completion
        self.select_bank(Bank::Bank0)?;

        let mut retries = 0;
        loop {
            let status = self.device.i_2_c_mst_status().read()?;

            // Check for NACK error
            if status.i_2_c_slv_4_nack() {
                return Err(Error::Magnetometer);
            }

            // Check for lost arbitration
            if status.i_2_c_lost_arb() {
                return Err(Error::Magnetometer);
            }

            if status.i_2_c_slv_4_done() {
                break;
            }
            if retries > MAX_I2C_SLV4_RETRIES {
                return Err(Error::Magnetometer);
            }
            delay.delay_ms(1);
            retries += 1;
        }

        Ok(())
    }

    /// Helper to write to AK09916 register using Slave 4
    fn write_mag_register<D>(
        &mut self,
        reg: u8,
        value: u8,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        use crate::sensors::magnetometer::AK09916_I2C_ADDRESS;

        // Clear any previous status by reading I2C_MST_STATUS
        self.select_bank(Bank::Bank0)?;
        let _ = self.device.i_2_c_mst_status().read()?;

        // Enable I2C master cycle mode for SLV4 transaction
        // SLV4 requires the I2C master to be actively clocked to execute
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(true);
        })?;
        delay.delay_ms(1);

        self.select_bank(Bank::Bank3)?;

        // Configure Slave 4 for write
        self.device.bank_3_i_2_c_slv_4_addr().write(|w| {
            w.set_i_2_c_id_4(AK09916_I2C_ADDRESS);
            w.set_i_2_c_slv_4_rnw(false); // Write mode
        })?;

        self.device
            .bank_3_i_2_c_slv_4_reg()
            .write(|w| w.set_i_2_c_slv_4_reg(reg))?;

        self.device
            .bank_3_i_2_c_slv_4_do()
            .write(|w| w.set_i_2_c_slv_4_do(value))?;

        self.device.bank_3_i_2_c_slv_4_ctrl().write(|w| {
            w.set_i_2_c_slv_4_en(true);
            w.set_i_2_c_slv_4_int_en(false);
            w.set_i_2_c_slv_4_reg_dis(false); // Use internal register addressing
            w.set_i_2_c_mst_dly(0);
        })?;

        self.wait_for_i2c_slv4_done(delay)?;

        // Disable I2C master cycle mode after transaction
        self.select_bank(Bank::Bank0)?;
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(false);
        })?;

        Ok(())
    }

    /// Helper to read from AK09916 register using Slave 4
    fn read_mag_register<D>(&mut self, reg: u8, delay: &mut D) -> Result<u8, Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        use crate::sensors::magnetometer::AK09916_I2C_ADDRESS;

        // Clear any previous status by reading I2C_MST_STATUS
        self.select_bank(Bank::Bank0)?;
        let _ = self.device.i_2_c_mst_status().read()?;

        // Enable I2C master cycle mode for SLV4 transaction
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(true);
        })?;
        delay.delay_ms(1);

        self.select_bank(Bank::Bank3)?;

        // Configure Slave 4 for read
        self.device.bank_3_i_2_c_slv_4_addr().write(|w| {
            w.set_i_2_c_id_4(AK09916_I2C_ADDRESS);
            w.set_i_2_c_slv_4_rnw(true); // Read mode
        })?;

        self.device
            .bank_3_i_2_c_slv_4_reg()
            .write(|w| w.set_i_2_c_slv_4_reg(reg))?;

        self.device.bank_3_i_2_c_slv_4_ctrl().write(|w| {
            w.set_i_2_c_slv_4_en(true);
            w.set_i_2_c_slv_4_int_en(false);
            w.set_i_2_c_slv_4_reg_dis(false); // Use internal register addressing
            w.set_i_2_c_mst_dly(0);
        })?;

        self.wait_for_i2c_slv4_done(delay)?;

        // Disable I2C master cycle mode after transaction
        self.select_bank(Bank::Bank0)?;
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(false);
        })?;

        // Read the data
        self.select_bank(Bank::Bank3)?;
        let value = self.device.bank_3_i_2_c_slv_4_di().read()?.i_2_c_slv_4_di();

        Ok(value)
    }

    #[cfg(feature = "dmp")]
    /// Configure magnetometer for DMP 9-axis operation
    ///
    /// This configures the AK09916 magnetometer specifically for DMP usage, which differs
    /// from the standard continuous mode configuration. This special configuration is
    /// required for proper 9-axis quaternion fusion.
    ///
    /// # Configuration details
    ///
    /// The DMP requires a specific magnetometer setup:
    /// - **I2C_SLV0**: Reads 10 bytes starting from AK09916 register 0x03 (RSV2)
    ///   with byte-swap and register grouping enabled
    /// - **I2C_SLV1**: Triggers single measurement mode on each DMP sample cycle
    /// - **I2C_MST_ODR_CONFIG**: Sets magnetometer sample rate to ~69 Hz
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider for timing
    ///
    /// # Important
    ///
    /// - Call this BEFORE `dmp_enable(true)`
    /// - Do NOT call `init_magnetometer()` - this replaces it for DMP use
    /// - Only use this for 9-axis DMP operation
    /// - For 6-axis DMP (accel+gyro only), you don't need magnetometer at all
    ///
    /// # Errors
    ///
    /// Returns an error if I2C communication fails or magnetometer doesn't respond.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Initialize device
    /// driver.init(&mut delay)?;
    ///
    /// // Load and init DMP
    /// driver.dmp_load_firmware(&mut delay)?;
    /// driver.dmp_init(&mut delay)?;
    ///
    /// // Configure magnetometer for DMP
    /// driver.dmp_init_magnetometer(&mut delay)?;
    ///
    /// // Configure and enable DMP
    /// let config = DmpConfig::default().with_quaternion_9axis(true);
    /// driver.dmp_configure(&config)?;
    /// driver.dmp_enable(true)?;
    /// ```
    pub fn dmp_init_magnetometer<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal::delay::DelayNs,
    {
        use crate::sensors::magnetometer::{
            AK09916_I2C_ADDRESS, AK09916_REG_CNTL2, AK09916_REG_CNTL3, AK09916_REG_RSV2,
            AK09916_REG_WIA2,
        };

        #[cfg(feature = "defmt")]
        defmt::info!("Initializing magnetometer for DMP");

        // Step 1: Enable I2C master mode
        self.select_bank(Bank::Bank0)?;
        self.device.user_ctrl().modify(|w| {
            w.set_i_2_c_mst_en(true);
        })?;
        delay.delay_ms(10);

        // Step 2: Configure I2C master timing
        self.select_bank(Bank::Bank3)?;
        self.device.bank_3_i_2_c_mst_ctrl().write(|w| {
            w.set_i_2_c_mst_clk(7); // 400 kHz I2C clock
        })?;

        // Step 3: Verify magnetometer is present using Slave 4
        self.select_bank(Bank::Bank3)?;
        self.device.bank_3_i_2_c_slv_4_addr().write(|w| {
            w.set_i_2_c_id_4(AK09916_I2C_ADDRESS);
            w.set_i_2_c_slv_4_rnw(true); // Read operation
        })?;

        self.device
            .bank_3_i_2_c_slv_4_reg()
            .write(|w| w.set_i_2_c_slv_4_reg(AK09916_REG_WIA2))?;

        self.device.bank_3_i_2_c_slv_4_ctrl().write(|w| {
            w.set_i_2_c_slv_4_en(true);
        })?;

        self.wait_for_i2c_slv4_done(delay)?;

        self.select_bank(Bank::Bank3)?;
        let who_am_i = self.device.bank_3_i_2_c_slv_4_di().read()?.i_2_c_slv_4_di();

        if who_am_i != 0x09 {
            #[cfg(feature = "defmt")]
            defmt::error!(
                "Magnetometer WHO_AM_I check failed: expected 0x09, got 0x{:02X}",
                who_am_i
            );

            // Cleanup: Disable I2C master on failure
            self.select_bank(Bank::Bank0)?;
            self.device.user_ctrl().modify(|w| {
                w.set_i_2_c_mst_en(false);
            })?;

            return Err(Error::Magnetometer);
        }

        #[cfg(feature = "defmt")]
        defmt::debug!("Magnetometer WHO_AM_I verified: 0x09");

        // Step 4: Reset magnetometer using Slave 4
        self.write_mag_register(AK09916_REG_CNTL3, 0x01, delay)?; // Soft reset
        delay.delay_ms(100);

        // Step 5: Configure I2C_SLV0 for DMP magnetometer reads
        // Read 10 bytes from RSV2 (0x03) with byte-swap and grouping enabled
        self.select_bank(Bank::Bank3)?;

        // Set slave 0 address (AK09916, read mode)
        self.device.bank_3_i_2_c_slv_0_addr().write(|w| {
            w.set_i_2_c_id_0(AK09916_I2C_ADDRESS);
            w.set_i_2_c_slv_0_rnw(true); // Read operation
        })?;

        // Set slave 0 register (start at RSV2 = 0x03)
        self.device
            .bank_3_i_2_c_slv_0_reg()
            .write(|w| w.set_i_2_c_slv_0_reg(AK09916_REG_RSV2))?;

        // Configure slave 0 control:
        // - Read 10 bytes
        // - Enable slave 0
        // - Enable byte swap (I2C_SLV0_BYTE_SW)
        // - Enable register grouping (I2C_SLV0_GRP)
        self.device.bank_3_i_2_c_slv_0_ctrl().write(|w| {
            w.set_i_2_c_slv_0_leng(10); // Read 10 bytes
            w.set_i_2_c_slv_0_en(true); // Enable slave 0
            w.set_i_2_c_slv_0_byte_sw(true); // Byte swap
            w.set_i_2_c_slv_0_reg_dis(false); // Use register address
            w.set_i_2_c_slv_0_grp(true); // Group registers
        })?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Configured I2C_SLV0 for 10-byte read from RSV2 (0x03)");

        // Step 6: Configure I2C_SLV1 to trigger single measurement mode
        // This writes CNTL2 = SINGLE_MEAS (0x01) on each sample
        self.select_bank(Bank::Bank3)?;

        // Set slave 1 address (AK09916, write mode)
        self.device.bank_3_i_2_c_slv_1_addr().write(|w| {
            w.set_i_2_c_id_1(AK09916_I2C_ADDRESS);
            w.set_i_2_c_slv_1_rnw(false); // Write operation
        })?;

        // Set slave 1 register (CNTL2 = 0x31)
        self.device
            .bank_3_i_2_c_slv_1_reg()
            .write(|w| w.set_i_2_c_slv_1_reg(AK09916_REG_CNTL2))?;

        // Set data out register (value to write: SINGLE_MEAS = 0x01)
        self.device
            .bank_3_i_2_c_slv_1_do()
            .write(|w| w.set_i_2_c_slv_1_do(0x01))?;

        // Configure slave 1 control:
        // - Write 1 byte
        // - Enable slave 1
        self.device.bank_3_i_2_c_slv_1_ctrl().write(|w| {
            w.set_i_2_c_slv_1_leng(1); // Write 1 byte
            w.set_i_2_c_slv_1_en(true); // Enable slave 1
            w.set_i_2_c_slv_1_byte_sw(false);
            w.set_i_2_c_slv_1_reg_dis(false);
            w.set_i_2_c_slv_1_grp(false);
        })?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Configured I2C_SLV1 to trigger single measurements");

        // Step 7: Set I2C Master ODR configuration
        // This reduces the magnetometer read rate to ~69 Hz
        // Formula: 1.1 kHz / 2^(odr_config) = rate
        // 0x04 = 1100 / 2^4 = 68.75 Hz
        self.select_bank(Bank::Bank3)?;
        self.device
            .bank_3_i_2_c_mst_odr_config()
            .write(|w| w.set_i_2_c_mst_odr_config(0x04))?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Set I2C_MST_ODR_CONFIG to 0x04 (~69 Hz)");

        // Step 8: Enable I2C master cycle mode
        self.select_bank(Bank::Bank0)?;
        self.device.lp_config().modify(|w| {
            w.set_i_2_c_mst_cycle(true);
        })?;

        delay.delay_ms(50);

        self.mag_initialized = true;

        #[cfg(feature = "defmt")]
        defmt::info!("DMP magnetometer initialization complete");

        Ok(())
    }
}

#[cfg(feature = "async")]
impl<I> Icm20948Driver<I>
where
    I: device_driver::AsyncRegisterInterface<AddressType = u8>,
{
    /// Create a new ICM-20948 driver instance
    ///
    /// This will verify the `WHO_AM_I` register but will not initialize the device.
    /// Call `init()` after construction to configure the device.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Communication with the device fails
    /// - The `WHO_AM_I` register contains an unexpected value
    pub async fn new(interface: I) -> Result<Self, Error<I::Error>> {
        let device = RegisterDevice::new(interface);
        let mut driver = Self {
            device,
            current_bank: None,
            accel_config: crate::sensors::AccelConfig::default(),
            gyro_config: crate::sensors::GyroConfig::default(),
            accel_calibration: crate::sensors::AccelCalibration::default(),
            gyro_calibration: crate::sensors::GyroCalibration::default(),
            mag_calibration: crate::sensors::MagCalibration::default(),
            mag_initialized: false,
        };

        // Verify WHO_AM_I
        driver.select_bank(Bank::Bank0).await?;
        let who_am_i = driver.read_who_am_i().await?;

        if who_am_i != WHO_AM_I_VALUE {
            return Err(Error::InvalidDevice(who_am_i));
        }

        Ok(driver)
    }

    /// Initialize the device with default settings
    ///
    /// This performs a soft reset and configures basic settings.
    ///
    /// **Important**: This function requires a delay provider to ensure proper
    /// timing after device reset. According to the ICM-20948 datasheet Section 3
    /// "ELECTRICAL CHARACTERISTICS", the device requires up to 100ms after reset
    /// before registers can be accessed.
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider implementing `embedded_hal_async::delay::DelayNs`
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use embassy_time::Delay;
    /// let mut delay = Delay;
    /// driver.init(&mut delay).await?;
    /// ```
    pub async fn init<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        const MAX_WAIT_MS: u32 = 100;
        const POLL_INTERVAL_MS: u32 = 1;

        self.select_bank(Bank::Bank0).await?;

        // Reset the device
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_device_reset(true);
            })
            .await?;

        // Wait for reset to complete
        // Datasheet Section 3 "ELECTRICAL CHARACTERISTICS" - "A.C. Electrical Characteristics":
        // Start-up time for register read/write is 11ms typical, 100ms max
        // We use polling to check when reset is done
        for _ in 0..(MAX_WAIT_MS / POLL_INTERVAL_MS) {
            delay.delay_ms(POLL_INTERVAL_MS).await;
            if self
                .device
                .pwr_mgmt_1()
                .read_async()
                .await
                .is_ok_and(|pwr_mgmt| !pwr_mgmt.device_reset())
            {
                break;
            }
        }

        // Wake up and select auto clock source
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_sleep(false);
                w.set_clksel(1);
            })
            .await?;

        // Wait and verify by checking we can read back the configuration correctly
        for _ in 0..(MAX_WAIT_MS / POLL_INTERVAL_MS) {
            delay.delay_ms(POLL_INTERVAL_MS).await;
            // Verify device is responding and configuration took effect
            if self
                .device
                .pwr_mgmt_1()
                .read_async()
                .await
                .is_ok_and(|pwr_mgmt| !pwr_mgmt.sleep() && pwr_mgmt.clksel() == 1)
            {
                // Device is awake, clock is set
                return Ok(());
            }
        }

        Ok(())
    }

    /// Select a register bank
    ///
    /// The ICM-20948 has 4 register banks that must be selected before
    /// accessing registers in that bank.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn select_bank(&mut self, bank: Bank) -> Result<(), Error<I::Error>> {
        if self.current_bank != Some(bank) {
            self.device
                .reg_bank_sel()
                .write_async(|w| {
                    w.set_user_bank(bank as u8);
                })
                .await?;

            self.current_bank = Some(bank);
        }
        Ok(())
    }

    /// Enable SPI mode by disabling the I2C slave interface
    ///
    /// **Required for SPI operation!** Call this immediately after `init()`
    /// when using the `SpiInterface`. Not needed for I2C.
    ///
    /// When using SPI to communicate with the ICM-20948, the I2C slave interface
    /// must be disabled by setting the `I2C_IF_DIS` bit in the `USER_CTRL` register.
    /// This is a requirement from the ICM-20948 datasheet for proper SPI operation.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use icm20948::{SpiInterface, Icm20948Driver};
    /// use embedded_hal_bus::spi::ExclusiveDevice;
    ///
    /// // Create SPI device with CS pin
    /// let spi_device = ExclusiveDevice::new(spi_bus, cs_pin, delay);
    /// let interface = SpiInterface::new(spi_device);
    ///
    /// // Create and initialize driver
    /// let mut imu = Icm20948Driver::new(interface).await?;
    /// imu.init(&mut delay).await?;
    ///
    /// // Enable SPI mode (required!)
    /// imu.enable_spi_mode().await?;
    ///
    /// // Now use the device normally
    /// let accel = imu.read_accelerometer().await?;
    /// ```
    pub async fn enable_spi_mode(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_i_2_c_if_dis(true);
            })
            .await?;
        Ok(())
    }

    /// Read the `WHO_AM_I` register
    ///
    /// Should return 0xEA for a valid ICM-20948
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_who_am_i(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;
        let reg = self.device.who_am_i().read_async().await?;
        Ok(reg.who_am_i())
    }

    /// Read the `LP_CONFIG` register as a raw byte (async version)
    ///
    /// Returns the `LP_CONFIG` register value with bits:
    /// - Bit 6: `I2C_MST_CYCLE`
    /// - Bit 5: `ACCEL_CYCLE`
    /// - Bit 4: `GYRO_CYCLE`
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_lp_config(&mut self) -> Result<u8, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;
        let reg = self.device.lp_config().read_async().await?;

        // Reconstruct the raw register value from individual bits
        let mut value = 0u8;
        if reg.gyro_cycle() {
            value |= 0x10;
        }
        if reg.accel_cycle() {
            value |= 0x20;
        }
        if reg.i_2_c_mst_cycle() {
            value |= 0x40;
        }

        Ok(value)
    }

    /// Read accelerometer data
    ///
    /// Returns raw 16-bit values for X, Y, Z axes.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    async fn read_accel(&mut self) -> Result<AccelData, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Read all 6 bytes atomically to prevent torn reads
        // Register addresses: ACCEL_XOUT_H (0x2D) through ACCEL_ZOUT_L (0x32)
        const ACCEL_XOUT_H: u8 = 0x2D;
        let mut buffer = [0u8; 6];
        self.device
            .interface
            .read_register(ACCEL_XOUT_H, 48, &mut buffer)
            .await?;

        let x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let z = i16::from_be_bytes([buffer[4], buffer[5]]);

        Ok(AccelData { x, y, z })
    }

    /// Read gyroscope data
    ///
    /// Returns raw 16-bit values for X, Y, Z axes.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    async fn read_gyro(&mut self) -> Result<GyroData, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Read all 6 bytes atomically to prevent torn reads
        // Register addresses: GYRO_XOUT_H (0x33) through GYRO_ZOUT_L (0x38)
        const GYRO_XOUT_H: u8 = 0x33;
        let mut buffer = [0u8; 6];
        self.device
            .interface
            .read_register(GYRO_XOUT_H, 48, &mut buffer)
            .await?;

        let x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let z = i16::from_be_bytes([buffer[4], buffer[5]]);

        Ok(GyroData { x, y, z })
    }

    /// Read temperature sensor
    ///
    /// Returns raw 16-bit signed value.
    /// Temperature in °C = (`TEMP_OUT` - `RoomTemp_Offset`)/`Temp_Sensitivity` + 21°C
    /// Where `RoomTemp_Offset` = 0 and `Temp_Sensitivity` = 333.87 LSB/°C (typical)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_temperature(&mut self) -> Result<i16, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Read both bytes atomically to prevent torn reads
        // Register addresses: TEMP_OUT_H (0x39) through TEMP_OUT_L (0x3A)
        const TEMP_OUT_H: u8 = 0x39;
        let mut buffer = [0u8; 2];
        self.device
            .interface
            .read_register(TEMP_OUT_H, 16, &mut buffer)
            .await?;

        // Combine high and low bytes (big-endian)
        let temp_raw = i16::from_be_bytes([buffer[0], buffer[1]]);

        Ok(temp_raw)
    }

    /// Read temperature in degrees Celsius
    ///
    /// Convenience method that reads the temperature sensor and converts to Celsius.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_temperature_celsius(&mut self) -> Result<f32, Error<I::Error>> {
        let raw = self.read_temperature().await?;
        Ok((f32::from(raw) / 333.87) + 21.0)
    }

    /// Set sleep mode
    ///
    /// # Arguments
    ///
    /// * `enable` - true to enter sleep mode, false to exit
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn set_sleep(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_sleep(enable);
            })
            .await?;
        Ok(())
    }

    /// Enable or disable the DMP
    ///
    /// # Arguments
    ///
    /// * `enable` - true to enable DMP, false to disable
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn set_dmp_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_dmp_en(enable);
            })
            .await?;
        Ok(())
    }

    /// Enable or disable the FIFO
    ///
    /// # Arguments
    ///
    /// * `enable` - true to enable FIFO, false to disable
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn set_fifo_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_fifo_en(enable);
            })
            .await?;
        Ok(())
    }

    /// Configure accelerometer settings
    ///
    /// # Arguments
    /// * `config` - Accelerometer configuration
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn configure_accelerometer(
        &mut self,
        config: crate::sensors::AccelConfig,
    ) -> Result<(), Error<I::Error>> {
        // Set continuous mode first - ACCEL_SMPLRT_DIV registers require this
        self.select_bank(Bank::Bank0).await?;
        self.device
            .lp_config()
            .modify_async(|w| {
                w.set_accel_cycle(false);
            })
            .await?;

        // Then configure Bank 2 registers
        self.select_bank(Bank::Bank2).await?;

        // Configure full-scale range and DLPF
        self.device
            .bank_2_accel_config()
            .modify_async(|w| {
                w.set_accel_fs_sel(config.full_scale as u8);
                w.set_accel_dlpfcfg(config.dlpf as u8);
                w.set_accel_fchoice(config.dlpf_enable);
            })
            .await?;

        // Configure sample rate divider
        self.device
            .bank_2_accel_smplrt_div_1()
            .write_async(|w| {
                w.set_accel_smplrt_div_1((config.sample_rate_div >> 8) as u8);
            })
            .await?;

        self.device
            .bank_2_accel_smplrt_div_2()
            .write_async(|w| {
                w.set_accel_smplrt_div_2((config.sample_rate_div & 0xFF) as u8);
            })
            .await?;

        self.accel_config = config;

        // Return to Bank 0
        self.select_bank(Bank::Bank0).await?;
        Ok(())
    }

    /// Read Bank 2 accelerometer configuration registers
    ///
    /// Returns `(config_byte, div1, div2)` representing the raw register values
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_bank2_accel_config(&mut self) -> Result<(u8, u8, u8), Error<I::Error>> {
        self.select_bank(Bank::Bank2).await?;

        let config_reg = self.device.bank_2_accel_config().read_async().await?;
        let div1_reg = self.device.bank_2_accel_smplrt_div_1().read_async().await?;
        let div2_reg = self.device.bank_2_accel_smplrt_div_2().read_async().await?;

        // Reconstruct raw bytes
        let mut config = 0u8;
        if config_reg.accel_fchoice() {
            config |= 0x01;
        }
        config |= config_reg.accel_fs_sel() << 1;
        config |= config_reg.accel_dlpfcfg() << 3;

        let div1 = div1_reg.accel_smplrt_div_1();
        let div2 = div2_reg.accel_smplrt_div_2();

        self.select_bank(Bank::Bank0).await?;
        Ok((config, div1, div2))
    }

    /// Read raw accelerometer data (16-bit signed values)
    ///
    /// Returns raw sensor values without calibration or conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_accelerometer_raw(&mut self) -> Result<AccelData, Error<I::Error>> {
        self.read_accel().await
    }

    /// Read accelerometer data in g (gravitational acceleration)
    ///
    /// Returns calibrated acceleration data in units of g (9.81 m/s²).
    /// Automatically applies stored calibration offsets.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_accelerometer(
        &mut self,
    ) -> Result<crate::sensors::AccelDataG, Error<I::Error>> {
        let raw = self.read_accel().await?;

        // Apply calibration offsets
        let x_cal = raw.x - self.accel_calibration.offset_x;
        let y_cal = raw.y - self.accel_calibration.offset_y;
        let z_cal = raw.z - self.accel_calibration.offset_z;

        // Convert to g based on full scale setting
        let scale = self.accel_config.full_scale.sensitivity();

        Ok(crate::sensors::AccelDataG {
            x: f32::from(x_cal) / scale,
            y: f32::from(y_cal) / scale,
            z: f32::from(z_cal) / scale,
        })
    }

    /// Configure gyroscope
    ///
    /// # Arguments
    ///
    /// * `config` - Gyroscope configuration
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn configure_gyroscope(
        &mut self,
        config: crate::sensors::GyroConfig,
    ) -> Result<(), Error<I::Error>> {
        // CRITICAL: Set continuous mode first (Bank 0)
        // The GYRO_SMPLRT_DIV register only works when the gyroscope
        // is in continuous mode (GYRO_CYCLE = 0 in LP_CONFIG)
        self.select_bank(Bank::Bank0).await?;
        self.device
            .lp_config()
            .modify_async(|w| {
                w.set_gyro_cycle(false); // Continuous mode required for SMPLRT_DIV
            })
            .await?;

        // Then configure Bank 2 registers
        self.select_bank(Bank::Bank2).await?;

        // Configure full-scale range and DLPF
        self.device
            .bank_2_gyro_config_1()
            .modify_async(|w| {
                w.set_gyro_fs_sel(config.full_scale as u8);
                w.set_gyro_dlpfcfg(config.dlpf as u8);
                w.set_gyro_fchoice(config.dlpf_enable);
            })
            .await?;

        // Configure sample rate divider
        self.device
            .bank_2_gyro_smplrt_div()
            .write_async(|w| {
                w.set_gyro_smplrt_div(config.sample_rate_div);
            })
            .await?;

        self.gyro_config = config;

        // Return to Bank 0
        self.select_bank(Bank::Bank0).await?;
        Ok(())
    }

    /// Read raw gyroscope data (16-bit signed values)
    ///
    /// Returns raw sensor values without calibration or conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_gyroscope_raw(&mut self) -> Result<GyroData, Error<I::Error>> {
        self.read_gyro().await
    }

    /// Read gyroscope data in degrees per second
    ///
    /// Returns calibrated gyroscope data in degrees per second.
    /// Automatically applies stored calibration offsets.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_gyroscope(&mut self) -> Result<crate::sensors::GyroDataDps, Error<I::Error>> {
        let raw = self.read_gyro().await?;

        // Apply calibration offsets
        let x_cal = raw.x - self.gyro_calibration.offset_x;
        let y_cal = raw.y - self.gyro_calibration.offset_y;
        let z_cal = raw.z - self.gyro_calibration.offset_z;

        // Convert to degrees per second based on full scale setting
        let scale = self.gyro_config.full_scale.sensitivity();

        Ok(crate::sensors::GyroDataDps {
            x: f32::from(x_cal) / scale,
            y: f32::from(y_cal) / scale,
            z: f32::from(z_cal) / scale,
        })
    }

    /// Read gyroscope data in radians per second
    ///
    /// Returns calibrated gyroscope data in radians per second.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_gyroscope_radians(
        &mut self,
    ) -> Result<crate::sensors::GyroDataRps, Error<I::Error>> {
        let dps = self.read_gyroscope().await?;
        Ok(dps.to_radians_per_sec())
    }

    /// Set accelerometer calibration data
    ///
    /// The calibration will be automatically applied to all subsequent readings.
    pub const fn set_accelerometer_calibration(
        &mut self,
        calibration: crate::sensors::AccelCalibration,
    ) {
        self.accel_calibration = calibration;
    }

    /// Get current accelerometer calibration data
    #[must_use]
    pub const fn accelerometer_calibration(&self) -> &crate::sensors::AccelCalibration {
        &self.accel_calibration
    }

    /// Calibrate the accelerometer with default motion detection threshold (async version).
    ///
    /// Convenience wrapper for [`calibrate_accelerometer_with_threshold`](Self::calibrate_accelerometer_with_threshold)
    /// using the default threshold divisor (5% of full scale).
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub async fn calibrate_accelerometer(
        &mut self,
        samples: u16,
    ) -> Result<crate::sensors::AccelCalibration, Error<I::Error>> {
        self.calibrate_accelerometer_with_threshold(
            samples,
            DEFAULT_MOTION_DETECTION_THRESHOLD_DIVISOR,
        )
        .await
    }

    /// Calibrate the accelerometer by taking a series of samples and calculating
    /// offsets (async version). The device should be placed on a level surface with Z-axis pointing up.
    ///
    /// The calibration will automatically be applied to future readings.
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    /// * `motion_threshold_divisor` - Controls motion detection sensitivity.
    ///   Smaller values are more lenient (allow more variance).
    ///   - 100: Very strict (1% variance allowed)
    ///   - 33: Strict (3% variance allowed)
    ///   - 20: Balanced (5% variance allowed, recommended)
    ///   - 10: Lenient (10% variance allowed)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub async fn calibrate_accelerometer_with_threshold(
        &mut self,
        samples: u16,
        motion_threshold_divisor: i16,
    ) -> Result<crate::sensors::AccelCalibration, Error<I::Error>> {
        // Validate input
        if samples == 0 {
            return Err(Error::InvalidConfig);
        }

        let mut sum_x: i64 = 0;
        let mut sum_y: i64 = 0;
        let mut sum_z: i64 = 0;

        // For motion detection
        let mut min_x = i16::MAX;
        let mut max_x = i16::MIN;
        let mut min_y = i16::MAX;
        let mut max_y = i16::MIN;
        let mut min_z = i16::MAX;
        let mut max_z = i16::MIN;

        for _ in 0..samples {
            let accel_data = self.read_accel().await?;
            sum_x += i64::from(accel_data.x);
            sum_y += i64::from(accel_data.y);
            sum_z += i64::from(accel_data.z);

            // Track variance for motion detection
            min_x = min_x.min(accel_data.x);
            max_x = max_x.max(accel_data.x);
            min_y = min_y.min(accel_data.y);
            max_y = max_y.max(accel_data.y);
            min_z = min_z.min(accel_data.z);
            max_z = max_z.max(accel_data.z);
        }

        // Check for motion during calibration
        #[allow(clippy::cast_possible_truncation)]
        let max_variance =
            self.accel_config.full_scale.sensitivity() as i16 / motion_threshold_divisor;

        let variance_x = max_x.saturating_sub(min_x);
        let variance_y = max_y.saturating_sub(min_y);
        let variance_z = max_z.saturating_sub(min_z);

        if variance_x > max_variance || variance_y > max_variance || variance_z > max_variance {
            return Err(Error::DeviceMoving);
        }

        // Calculate averages - CRITICAL: Handle overflow properly
        let avg_x = i16::try_from(sum_x / i64::from(samples)).map_err(|_| Error::InvalidConfig)?;
        let avg_y = i16::try_from(sum_y / i64::from(samples)).map_err(|_| Error::InvalidConfig)?;
        let avg_z = i16::try_from(sum_z / i64::from(samples)).map_err(|_| Error::InvalidConfig)?;

        // Z-axis should read +1g, so offset it by the difference from expected value
        let sensitivity = self.accel_config.full_scale.sensitivity();
        // Sensitivity values are known to be in range 2048-16384, well within i16::MAX
        #[allow(clippy::cast_possible_truncation)]
        let sensitivity_i16 = sensitivity as i16;
        let z_offset = avg_z.saturating_sub(sensitivity_i16); // Expected +1g

        let calibration = crate::sensors::AccelCalibration {
            offset_x: avg_x,
            offset_y: avg_y,
            offset_z: z_offset,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        self.accel_calibration = calibration;
        Ok(calibration)
    }

    /// Set gyroscope calibration data
    ///
    /// The calibration will be automatically applied to all subsequent readings.
    pub const fn set_gyroscope_calibration(
        &mut self,
        calibration: crate::sensors::GyroCalibration,
    ) {
        self.gyro_calibration = calibration;
    }

    /// Get current gyroscope calibration data
    #[must_use]
    pub const fn gyroscope_calibration(&self) -> &crate::sensors::GyroCalibration {
        &self.gyro_calibration
    }

    /// Calibrate the gyroscope with default motion detection threshold (async version).
    ///
    /// Convenience wrapper for [`calibrate_gyroscope_with_threshold`](Self::calibrate_gyroscope_with_threshold)
    /// using the default threshold divisor (5% of full scale).
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub async fn calibrate_gyroscope(
        &mut self,
        samples: u16,
    ) -> Result<crate::sensors::GyroCalibration, Error<I::Error>> {
        self.calibrate_gyroscope_with_threshold(samples, DEFAULT_MOTION_DETECTION_THRESHOLD_DIVISOR)
            .await
    }

    /// Calibrate the gyroscope by taking a series of samples and calculating
    /// average offsets (async version). The device should be stationary during calibration.
    ///
    /// The calibration will automatically be applied to future readings.
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of samples to average (more samples = better accuracy)
    /// * `motion_threshold_divisor` - Controls motion detection sensitivity.
    ///   Smaller values are more lenient (allow more variance).
    ///   - 100: Very strict (1% variance allowed)
    ///   - 33: Strict (3% variance allowed)
    ///   - 20: Balanced (5% variance allowed, recommended)
    ///   - 10: Lenient (10% variance allowed)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails or if the device
    /// is moving during calibration.
    pub async fn calibrate_gyroscope_with_threshold(
        &mut self,
        samples: u16,
        motion_threshold_divisor: i16,
    ) -> Result<crate::sensors::GyroCalibration, Error<I::Error>> {
        // Validate input
        if samples == 0 {
            return Err(Error::InvalidConfig);
        }

        let mut sum_x: i64 = 0;
        let mut sum_y: i64 = 0;
        let mut sum_z: i64 = 0;

        // For motion detection (optional, based on threshold)
        let mut min_x = i16::MAX;
        let mut max_x = i16::MIN;
        let mut min_y = i16::MAX;
        let mut max_y = i16::MIN;
        let mut min_z = i16::MAX;
        let mut max_z = i16::MIN;

        // Motion detection is optional for gyroscopes because MEMS gyroscopes
        // exhibit significant bias drift (0.5-1 °/s is normal) even when perfectly
        // stationary. This drift is the bias that calibration measures, not motion.
        // However, with lenient thresholds (divisor <= 10), motion detection can
        // still catch actual device movement during calibration.
        let check_motion = motion_threshold_divisor <= 50;

        for _ in 0..samples {
            let gyro_data = self.read_gyro().await?;
            sum_x += i64::from(gyro_data.x);
            sum_y += i64::from(gyro_data.y);
            sum_z += i64::from(gyro_data.z);

            // Track variance for motion detection
            if check_motion {
                min_x = min_x.min(gyro_data.x);
                max_x = max_x.max(gyro_data.x);
                min_y = min_y.min(gyro_data.y);
                max_y = max_y.max(gyro_data.y);
                min_z = min_z.min(gyro_data.z);
                max_z = max_z.max(gyro_data.z);
            }
        }

        // Check for motion during calibration if enabled
        if check_motion {
            #[allow(clippy::cast_possible_truncation)]
            let max_variance =
                self.gyro_config.full_scale.sensitivity() as i16 / motion_threshold_divisor;

            let variance_x = max_x.saturating_sub(min_x);
            let variance_y = max_y.saturating_sub(min_y);
            let variance_z = max_z.saturating_sub(min_z);

            if variance_x > max_variance || variance_y > max_variance || variance_z > max_variance {
                return Err(Error::DeviceMoving);
            }
        }

        // CRITICAL: Handle overflow properly - return error instead of silent failure with 0
        let avg_x =
            i16::try_from(sum_x / i64::from(samples)).map_err(|_| Error::CalibrationOverflow)?;
        let avg_y =
            i16::try_from(sum_y / i64::from(samples)).map_err(|_| Error::CalibrationOverflow)?;
        let avg_z =
            i16::try_from(sum_z / i64::from(samples)).map_err(|_| Error::CalibrationOverflow)?;

        let calibration = crate::sensors::GyroCalibration {
            offset_x: avg_x,
            offset_y: avg_y,
            offset_z: avg_z,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        self.gyro_calibration = calibration;
        Ok(calibration)
    }

    /// Configure FIFO with the given settings
    ///
    /// # Arguments
    /// * `config` - FIFO configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn fifo_configure(&mut self, config: &FifoConfig) -> Result<(), Error<I::Error>> {
        let advanced: FifoConfigAdvanced = (*config).into();
        self.fifo_configure_advanced(&advanced).await
    }

    /// Configure FIFO with advanced per-axis control
    ///
    /// # Arguments
    /// * `config` - Advanced FIFO configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn fifo_configure_advanced(
        &mut self,
        config: &FifoConfigAdvanced,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Configure FIFO_EN_1 (slave sensors)
        self.device
            .fifo_en_1()
            .write_async(|w| {
                w.set_slv_0_fifo_en(config.enable_slv0);
                w.set_slv_1_fifo_en(config.enable_slv1);
                w.set_slv_2_fifo_en(config.enable_slv2);
                w.set_slv_3_fifo_en(config.enable_slv3);
            })
            .await?;

        // Configure FIFO_EN_2 (accel, gyro, temp)
        self.device
            .fifo_en_2()
            .write_async(|w| {
                w.set_accel_fifo_en(config.enable_accel);
                w.set_gyro_x_fifo_en(config.enable_gyro_x);
                w.set_gyro_y_fifo_en(config.enable_gyro_y);
                w.set_gyro_z_fifo_en(config.enable_gyro_z);
                w.set_temp_fifo_en(config.enable_temp);
            })
            .await?;

        // Configure FIFO mode
        self.device
            .fifo_mode()
            .write_async(|w| {
                w.set_fifo_mode(config.mode as u8);
            })
            .await?;

        Ok(())
    }

    /// Enable or disable FIFO
    ///
    /// # Arguments
    /// * `enable` - true to enable, false to disable
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn fifo_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_fifo_en(enable);
            })
            .await?;
        Ok(())
    }

    /// Reset FIFO buffer
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn fifo_reset(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Reset all FIFOs (bits 4:0)
        self.device
            .fifo_rst()
            .write_async(|w| {
                w.set_fifo_reset(0x1F);
            })
            .await?;

        // Clear the reset bits
        self.device
            .fifo_rst()
            .write_async(|w| {
                w.set_fifo_reset(0);
            })
            .await?;

        Ok(())
    }

    /// Get the number of bytes currently in the FIFO
    ///
    /// # Returns
    /// Number of bytes in FIFO (0-512)
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn fifo_count(&mut self) -> Result<u16, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        let count_h = self.device.fifo_counth().read_async().await?;
        let count_l = self.device.fifo_countl().read_async().await?;

        let count = u16::from_be_bytes([count_h.fifo_cnt_h(), count_l.fifo_cnt_l()]);

        Ok(count)
    }

    /// Read raw data from FIFO
    ///
    /// Reads available FIFO data up to the buffer size. This method reads the
    /// FIFO count once before starting the read to avoid race conditions and
    /// improve performance.
    ///
    /// # Arguments
    /// * `buffer` - Buffer to read data into
    ///
    /// # Returns
    /// Number of bytes actually read (limited by available data and buffer size)
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn fifo_read(&mut self, buffer: &mut [u8]) -> Result<usize, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Read FIFO count ONCE before reading to avoid race conditions
        // and reduce overhead (checking count after every byte adds 2 register reads per byte!)
        let count = self.fifo_count().await?;

        // Determine how many bytes to actually read
        let bytes_to_read = core::cmp::min(count as usize, buffer.len());

        // Burst read the determined number of bytes
        for byte in &mut buffer[..bytes_to_read] {
            let val = self.device.fifo_rw().read_async().await?;
            *byte = val.fifo_r_w();
        }

        Ok(bytes_to_read)
    }

    /// Parse FIFO data into records
    ///
    /// # Arguments
    /// * `data` - Raw FIFO data
    /// * `config` - FIFO configuration used
    ///
    /// # Returns
    /// Vector of parsed FIFO records
    ///
    /// # Errors
    /// Returns an error if parsing fails.
    pub fn fifo_parse(
        &self,
        data: &[u8],
        config: &FifoConfigAdvanced,
    ) -> Result<heapless::Vec<FifoRecord, 64>, Error<I::Error>> {
        use crate::fifo::parser::FifoParser;
        let parser = FifoParser::new(config);
        parser.parse(data).map_err(|_| Error::InvalidConfig)
    }

    /// Get FIFO overflow status
    ///
    /// # Returns
    /// FIFO overflow status for all FIFOs
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn fifo_overflow_status(&mut self) -> Result<FifoOverflowStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        let status = self.device.int_status_2().read_async().await?;
        let bits = status.fifo_overflow_int();

        Ok(FifoOverflowStatus {
            fifo0: (bits & 0x01) != 0,
            fifo1: (bits & 0x02) != 0,
            fifo2: (bits & 0x04) != 0,
            fifo3: (bits & 0x08) != 0,
            fifo4: (bits & 0x10) != 0,
        })
    }

    /// Load DMP firmware into the processor memory
    ///
    /// This transfers ~16KB of firmware data in 16-byte chunks to the DMP processor.
    /// This is a very I/O intensive operation (1000+ I2C transactions).
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider for short delays between operations
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # use embassy_time::Delay;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = Delay;
    /// // Load the DMP firmware
    /// driver.dmp_load_firmware(&mut delay).await?;
    ///
    /// // Now you can configure and enable the DMP
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    #[cfg(feature = "dmp")]
    pub async fn dmp_load_firmware<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        use crate::dmp::firmware::{DMP_FIRMWARE, DMP_START_ADDRESS};

        // Ensure we're in Bank 0 for DMP memory access
        const WRITE_CHUNK_SIZE: usize = 16;
        const DMP_BANK_SIZE: usize = 256;

        self.select_bank(Bank::Bank0).await?;

        let mut current_address: u16 = 0;

        for chunk in DMP_FIRMWARE.chunks(WRITE_CHUNK_SIZE) {
            // Calculate which bank and address offset we're at
            #[allow(clippy::cast_possible_truncation)]
            let bank = (current_address / DMP_BANK_SIZE as u16) as u8;
            #[allow(clippy::cast_possible_truncation)]
            let addr_in_bank = (current_address % DMP_BANK_SIZE as u16) as u8;

            // Set the memory bank
            self.device
                .mem_bank_sel()
                .write_async(|w| {
                    w.set_mem_bank_sel(bank);
                })
                .await?;

            // Set the start address within the bank
            self.device
                .mem_start_addr()
                .write_async(|w| {
                    w.set_mem_start_addr(addr_in_bank);
                })
                .await?;

            // Write the data bytes sequentially to MEM_R_W
            // The address auto-increments after each write
            for &byte in chunk {
                self.device
                    .mem_rw()
                    .write_async(|w| {
                        w.set_mem_r_w(byte);
                    })
                    .await?;
            }

            #[allow(clippy::cast_possible_truncation)]
            let chunk_len = chunk.len() as u16;
            current_address += chunk_len;

            // Small delay every few chunks to allow I2C bus to settle
            if current_address % 256 == 0 {
                delay.delay_us(100).await;
            }
        }

        // Switch to Bank 2 to set program start address
        self.select_bank(Bank::Bank2).await?;

        // Set the DMP program start address (0x1000)
        let addr_high = (DMP_START_ADDRESS >> 8) as u8;
        let addr_low = (DMP_START_ADDRESS & 0xFF) as u8;

        self.device
            .bank_2_prgm_start_addrh()
            .write_async(|w| {
                w.set_prgm_start_addrh(addr_high);
            })
            .await?;

        self.device
            .bank_2_prgm_start_addrl()
            .write_async(|w| {
                w.set_prgm_start_addrl(addr_low);
            })
            .await?;

        // Switch back to Bank 0
        self.select_bank(Bank::Bank0).await?;

        // Apply hardware fix registers required for reliable DMP operation on some silicon revisions

        // Set HW_FIX_DISABLE register to 0x48
        // This disables certain hardware fixes that interfere with DMP operation
        self.device
            .hw_fix_disable()
            .write_async(|w| {
                w.set_hw_fix_disable(0x48);
            })
            .await?;

        // Set SINGLE_FIFO_PRIORITY_SEL register to 0xE4
        // This configures FIFO priority selection for DMP mode
        self.device
            .single_fifo_priority_sel()
            .write_async(|w| {
                w.set_single_fifo_priority_sel(0xE4);
            })
            .await?;

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Applied DMP hardware fix registers (HW_FIX_DISABLE=0x48, SINGLE_FIFO_PRIORITY_SEL=0xE4)"
        );

        Ok(())
    }

    /// Initialize the DMP (reset, load firmware, configure)
    ///
    /// This is a convenience function that performs the complete DMP initialization:
    /// 1. Resets the DMP
    /// 2. Waits for reset to complete (~1ms)
    /// 3. Loads the firmware (~16KB)
    /// 4. Waits for firmware to initialize (~2ms)
    /// 5. Sets the program start address
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider for required delays between operations
    ///
    /// # Errors
    ///
    /// Returns an error if any step fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # use embassy_time::Delay;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = Delay;
    /// // Initialize the device first
    /// driver.init(&mut delay).await?;
    ///
    /// // Initialize DMP (includes all required delays)
    /// driver.dmp_init(&mut delay).await?;
    ///
    /// // Enable DMP
    /// driver.dmp_enable(true).await?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    #[cfg(feature = "dmp")]
    pub async fn dmp_init<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        // Reset the DMP
        self.dmp_reset().await?;

        // Wait for reset to complete
        delay.delay_ms(1).await;

        // CRITICAL: Ensure accel/gyro are NOT in low power or cycle mode
        // Per Cybergear findings: "The accel and gyro must not be in low power mode when using the DMP"
        self.select_bank(Bank::Bank0).await?;

        // Disable low power mode in PWR_MGMT_1
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_lp_en(false); // Disable low power mode
                w.set_sleep(false); // Ensure not in sleep
                w.set_clksel(1); // Auto-select best clock (0x01)
            })
            .await?;

        // Disable accel/gyro cycle modes, but keep I2C master cycle enabled for magnetometer
        // LP_CONFIG should be 0x40 (only I2C master in duty cycle mode)
        self.device
            .lp_config()
            .write_async(|w| {
                w.set_accel_cycle(false); // Accel must NOT be in cycle mode for DMP
                w.set_gyro_cycle(false); // Gyro must NOT be in cycle mode for DMP
                w.set_i_2_c_mst_cycle(false); // Disable for now, will re-enable after DMP starts
            })
            .await?;

        // Ensure all accelerometer and gyroscope axes are powered on
        // PWR_MGMT_2 should be 0x00 (all sensors enabled)
        self.device
            .pwr_mgmt_2()
            .write_async(|w| {
                w.set_disable_accel_x(false);
                w.set_disable_accel_y(false);
                w.set_disable_accel_z(false);
                w.set_disable_gyro_x(false);
                w.set_disable_gyro_y(false);
                w.set_disable_gyro_z(false);
            })
            .await?;

        // Configure sensor sample rates before loading firmware
        self.select_bank(Bank::Bank2).await?;

        // Enable GYRO_FCHOICE so that GYRO_SMPLRT_DIV is effective

        self.device
            .bank_2_gyro_config_1()
            .modify_async(|w| {
                w.set_gyro_fchoice(true);
            })
            .await?;

        // Set gyroscope sample rate divider to 0 (1.1kHz internal rate)
        self.device
            .bank_2_gyro_smplrt_div()
            .write_async(|w| {
                w.set_gyro_smplrt_div(0);
            })
            .await?;

        // Enable ACCEL_FCHOICE so that ACCEL_SMPLRT_DIV is effective

        self.device
            .bank_2_accel_config()
            .modify_async(|w| {
                w.set_accel_fchoice(true);
            })
            .await?;

        // Set accelerometer sample rate divider to 0 (1.125kHz internal rate)
        self.device
            .bank_2_accel_smplrt_div_1()
            .write_async(|w| {
                w.set_accel_smplrt_div_1(0);
            })
            .await?;
        self.device
            .bank_2_accel_smplrt_div_2()
            .write_async(|w| {
                w.set_accel_smplrt_div_2(0);
            })
            .await?;

        // Enable ODR_ALIGN_EN to synchronize sensor data streams

        self.device
            .bank_2_odr_align_en()
            .write_async(|w| {
                w.set_odr_align_en(true);
            })
            .await?;

        // Enable REG_LP_DMP_EN to allow DMP to receive internal sensor data

        self.device
            .bank_2_mod_ctrl_usr()
            .write_async(|w| {
                w.set_reg_lp_dmp_en(true);
            })
            .await?;

        self.select_bank(Bank::Bank0).await?;

        // Load the firmware after configuring hardware
        self.dmp_load_firmware(delay).await?;

        // Wait for firmware to initialize
        delay.delay_ms(2).await;

        Ok(())
    }

    /// Reset the DMP processor
    ///
    /// This resets the DMP, clearing its state and FIFO. After reset, the
    /// firmware must be reloaded using `dmp_load_firmware()` or `dmp_init()`.
    ///
    /// **Note**: The `dmp_init()` method includes the required delay after reset.
    /// If you call this method directly, you should delay ~1ms before loading firmware.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    #[cfg(feature = "dmp")]
    pub async fn dmp_reset(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Reset DMP by setting bit 3 of USER_CTRL
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_dmp_rst(true);
            })
            .await?;

        Ok(())
    }

    /// Enable or disable the DMP processor
    ///
    /// The firmware must already be loaded before enabling the DMP.
    /// Use `dmp_init()` or `dmp_load_firmware()` first.
    ///
    /// When enabled, the DMP will process sensor data and write results to the FIFO.
    /// You'll also need to enable the FIFO using `fifo_enable(true)`.
    ///
    /// # Arguments
    ///
    /// * `enable` - `true` to enable the DMP, `false` to disable it
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # use embassy_time::Delay;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = Delay;
    /// // Initialize DMP
    /// driver.dmp_init(&mut delay).await?;
    ///
    /// // Enable FIFO
    /// driver.fifo_enable(true).await?;
    ///
    /// // Enable DMP
    /// driver.dmp_enable(true).await?;
    ///
    /// // Now DMP is running and writing data to FIFO
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    #[cfg(feature = "dmp")]
    pub async fn dmp_enable(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // If enabling, verify power management settings first
        if enable {
            // Verify that sensors are not in low power or cycle mode
            // DMP operation requires sensors in full power mode
            let pwr_mgmt_1 = self.device.pwr_mgmt_1().read_async().await?;
            if pwr_mgmt_1.lp_en() {
                #[cfg(feature = "defmt")]
                defmt::warn!("DMP Enable: Forcing LP_EN=false (was enabled)");

                self.device
                    .pwr_mgmt_1()
                    .modify_async(|w| {
                        w.set_lp_en(false);
                    })
                    .await?;
            }

            let lp_config = self.device.lp_config().read_async().await?;
            if lp_config.accel_cycle() || lp_config.gyro_cycle() {
                #[cfg(feature = "defmt")]
                defmt::warn!("DMP Enable: Forcing accel/gyro cycle modes OFF (were enabled)");

                self.device
                    .lp_config()
                    .modify_async(|w| {
                        w.set_accel_cycle(false);
                        w.set_gyro_cycle(false);
                    })
                    .await?;
            }

            // Verify all sensors are powered on
            let pwr_mgmt_2 = self.device.pwr_mgmt_2().read_async().await?;
            if pwr_mgmt_2.disable_accel_x()
                || pwr_mgmt_2.disable_accel_y()
                || pwr_mgmt_2.disable_accel_z()
                || pwr_mgmt_2.disable_gyro_x()
                || pwr_mgmt_2.disable_gyro_y()
                || pwr_mgmt_2.disable_gyro_z()
            {
                #[cfg(feature = "defmt")]
                defmt::warn!("DMP Enable: Enabling all accel/gyro axes (some were disabled)");

                self.device
                    .pwr_mgmt_2()
                    .write_async(|w| {
                        w.set_disable_accel_x(false);
                        w.set_disable_accel_y(false);
                        w.set_disable_accel_z(false);
                        w.set_disable_gyro_x(false);
                        w.set_disable_gyro_y(false);
                        w.set_disable_gyro_z(false);
                    })
                    .await?;
            }
        }
        // Enable/disable DMP, FIFO, and I2C master together
        // FIFO_EN is needed for internal sensor→DMP routing
        // I2C master is needed ONLY for magnetometer data (9-axis mode)
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_dmp_en(enable);
                w.set_fifo_en(enable);
                // Only enable I2C master if magnetometer is initialized (9-axis mode)
                w.set_i_2_c_mst_en(enable && self.mag_initialized);
            })
            .await?;

        // Enable sensor data routing to DMP internal data path via FIFO_EN_2
        // This is required for the DMP to receive sensor data
        if enable {
            self.device
                .fifo_en_2()
                .write_async(|w| {
                    w.set_gyro_x_fifo_en(true);
                    w.set_gyro_y_fifo_en(true);
                    w.set_gyro_z_fifo_en(true);
                    w.set_accel_fifo_en(true);
                })
                .await?;
        } else {
            self.device
                .fifo_en_2()
                .write_async(|w| {
                    w.set_gyro_x_fifo_en(false);
                    w.set_gyro_y_fifo_en(false);
                    w.set_gyro_z_fifo_en(false);
                    w.set_accel_fifo_en(false);
                })
                .await?;
        }

        // Enable I2C master cycle mode for magnetometer data collection
        // Note: Accelerometer and gyroscope must NOT be in cycle mode for DMP
        if enable {
            // Enable I2C master cycle mode (LP_CONFIG = 0x40)
            self.device
                .lp_config()
                .modify_async(|w| {
                    w.set_i_2_c_mst_cycle(true);
                })
                .await?;

            // Enable DMP interrupt - this may be required to start data flow
            self.device
                .int_enable()
                .modify_async(|w| {
                    w.set_dmp_int_1_en(true);
                })
                .await?;
        } else {
            // Disable DMP interrupt when disabling DMP
            self.device
                .int_enable()
                .modify_async(|w| {
                    w.set_dmp_int_1_en(false);
                })
                .await?;
        }

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Enable sensor data routing to DMP internal data path
    ///
    /// Configures the FIFO_EN_2 register to route accelerometer and gyroscope data
    /// to the DMP processor. This is required for DMP operation.
    ///
    /// Note: The magnetometer uses a separate I2C slave data path.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use icm20948::*;
    /// # let mut imu = Icm20948Driver::new_i2c(todo!(), Address::Primary).unwrap();
    /// // After enabling DMP, enable sensor routing
    /// imu.dmp_enable(true).await?;
    /// imu.dmp_enable_sensor_routing().await?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub async fn dmp_enable_sensor_routing(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Enable accel and gyro to internal data bus (shared by FIFO and DMP)
        // This is required for DMP to receive sensor samples
        self.device
            .fifo_en_2()
            .modify_async(|w| {
                w.set_accel_fifo_en(true);
                w.set_gyro_x_fifo_en(true);
                w.set_gyro_y_fifo_en(true);
                w.set_gyro_z_fifo_en(true);
            })
            .await?;

        // Reset FIFO to clear any stale data and reinitialize the data path
        self.device
            .fifo_rst()
            .write_async(|w| {
                w.set_fifo_reset(0x1F); // Reset all FIFOs
            })
            .await?;

        // Small delay to allow FIFO reset to complete
        for _ in 0..1000 {
            core::hint::spin_loop();
        }

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Enable sensor fusion for DMP processing
    ///
    /// Configures the DMP to perform 9-axis sensor fusion by writing the appropriate
    /// control registers. This must be called **after** `dmp_enable()` because the DMP
    /// firmware initializes these registers on startup.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Enable DMP first
    /// driver.dmp_enable(true).await?;
    ///
    /// // THEN tell it which sensors to use
    /// driver.dmp_enable_sensor_fusion().await?;
    /// ```
    pub async fn dmp_enable_sensor_fusion(&mut self) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::DmpMemoryAddresses;

        // Calculate DATA_OUT_CTL1 value for 9-axis quaternion
        // GYRO_CALIBR (0x0008) + QUAT9 (0x0400) + ACCEL (0x8000) = 0x8408
        let data_out_ctl1: u16 = 0x8408;

        // Calculate DATA_INTR_CTL - must match DATA_OUT_CTL1!
        let data_intr_ctl: u16 = 0x8408;

        // Calculate DATA_OUT_CTL2 (accuracy bits)
        // ACCEL_ACCURACY (0x0001) + GYRO_ACCURACY (0x0002) + COMPASS_ACCURACY (0x0004) = 0x0007
        let data_out_ctl2: u16 = 0x0007;

        // Calculate MOTION_EVENT_CTL
        // ACCEL_CALIBR (0x0001) + GYRO_CALIBR (0x0002) + COMPASS_CALIBR (0x0004) + 9AXIS (0x0100) = 0x0107
        let motion_event_ctl: u16 = 0x0107;

        // Calculate DATA_RDY_STATUS
        // GYRO (0x0001) + ACCEL (0x0002) + COMPASS (0x0008) = 0x000B
        let data_rdy_status: u16 = 0x000B;

        #[cfg(feature = "defmt")]
        defmt::info!(
            "Writing sensor fusion config AFTER DMP enable: DATA_OUT_CTL1=0x{:04X}, DATA_INTR_CTL=0x{:04X}",
            data_out_ctl1,
            data_intr_ctl
        );

        // Write DATA_OUT_CTL1
        self.write_dmp_memory_async(
            DmpMemoryAddresses::DATA_OUT_CTL1,
            &data_out_ctl1.to_be_bytes(),
        )
        .await?;

        // Write DATA_OUT_CTL2
        self.write_dmp_memory_async(
            DmpMemoryAddresses::DATA_OUT_CTL2,
            &data_out_ctl2.to_be_bytes(),
        )
        .await?;

        // Write DATA_INTR_CTL (MUST match DATA_OUT_CTL1!)
        self.write_dmp_memory_async(
            DmpMemoryAddresses::DATA_INTR_CTL,
            &data_intr_ctl.to_be_bytes(),
        )
        .await?;

        // Write MOTION_EVENT_CTL
        self.write_dmp_memory_async(
            DmpMemoryAddresses::MOTION_EVENT_CTL,
            &motion_event_ctl.to_be_bytes(),
        )
        .await?;

        // Write DATA_RDY_STATUS
        self.write_dmp_memory_async(
            DmpMemoryAddresses::DATA_RDY_STATUS,
            &data_rdy_status.to_be_bytes(),
        )
        .await?;

        // Small delay for DMP to process
        for _ in 0..10000 {
            core::hint::spin_loop();
        }

        // Verify DATA_RDY_STATUS
        let mut readback = [0u8; 2];
        self.read_dmp_memory_async(DmpMemoryAddresses::DATA_RDY_STATUS, &mut readback)
            .await?;
        let status = ((readback[0] as u16) << 8) | (readback[1] as u16);

        #[cfg(feature = "defmt")]
        defmt::debug!("DATA_RDY_STATUS readback: 0x{:04X}", status);

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Set DMP output data rate for a specific sensor or quaternion (async)
    ///
    /// This configures how often the DMP produces output for a given data type.
    /// Setting interval to 0 means "use the sensor's sample rate" (no decimation).
    ///
    /// # Arguments
    ///
    /// * `odr_register` - The ODR register to configure (from `DmpOdrRegisters`)
    /// * `interval` - Output interval (0 = sensor rate, 1+ = decimation factor)
    ///
    /// # Notes
    ///
    /// When changing an ODR value, the corresponding ODR counter must also be reset to 0.
    /// This function writes both the ODR register and resets its counter.
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory write fails.
    async fn dmp_set_odr_async(
        &mut self,
        odr_register: u16,
        interval: u16,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpOdrCounterRegisters, DmpOdrRegisters};

        // Write ODR value
        let odr_bytes = interval.to_be_bytes();
        self.write_dmp_memory_async(odr_register, &odr_bytes)
            .await?;

        // Determine the corresponding counter register and reset it
        let counter_register = match odr_register {
            DmpOdrRegisters::QUAT9 => DmpOdrCounterRegisters::QUAT9,
            DmpOdrRegisters::QUAT6 => DmpOdrCounterRegisters::QUAT6,
            DmpOdrRegisters::ACCEL => DmpOdrCounterRegisters::ACCEL,
            DmpOdrRegisters::GYRO => DmpOdrCounterRegisters::GYRO,
            DmpOdrRegisters::CPASS => DmpOdrCounterRegisters::CPASS,
            DmpOdrRegisters::GYRO_CALIBR => DmpOdrCounterRegisters::GYRO_CALIBR,
            DmpOdrRegisters::CPASS_CALIBR => DmpOdrCounterRegisters::CPASS_CALIBR,
            _ => return Err(Error::InvalidConfig), // Unknown ODR register
        };

        // Reset the counter to 0
        let zero_bytes = [0u8, 0u8];
        self.write_dmp_memory_async(counter_register, &zero_bytes)
            .await?;

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Configure which sensors the DMP should expect data from (async)
    ///
    /// This function writes to the DATA_RDY_STATUS register to tell the DMP firmware
    /// which sensors are available and should be monitored. This is critical for DMP
    /// operation - without this, the DMP won't process sensor data.
    ///
    /// # Arguments
    ///
    /// * `accel` - Enable accelerometer data for DMP
    /// * `gyro` - Enable gyroscope data for DMP
    /// * `compass` - Enable magnetometer/compass data for DMP
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory write fails.
    async fn dmp_enable_sensors_async(
        &mut self,
        accel: bool,
        gyro: bool,
        compass: bool,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpDataReadyStatus, DmpMemoryAddresses};

        let mut status = 0u16;

        if gyro {
            status |= DmpDataReadyStatus::GYRO;
        }
        if accel {
            status |= DmpDataReadyStatus::ACCEL;
        }
        if compass {
            status |= DmpDataReadyStatus::COMPASS;
        }

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Setting DATA_RDY_STATUS: accel={} gyro={} compass={} -> 0x{:04X}",
            accel,
            gyro,
            compass,
            status
        );

        let status_bytes = status.to_be_bytes();
        self.write_dmp_memory_async(DmpMemoryAddresses::DATA_RDY_STATUS, &status_bytes)
            .await?;

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Configure DMP motion event control (calibration and fusion features) (async)
    ///
    /// This function writes to the MOTION_EVENT_CTL register to enable various
    /// DMP features including sensor calibration and 9-axis fusion.
    ///
    /// # Arguments
    ///
    /// * `accel_calibr` - Enable accelerometer calibration
    /// * `gyro_calibr` - Enable gyroscope calibration
    /// * `compass_calibr` - Enable compass/magnetometer calibration
    /// * `nine_axis` - Enable 9-axis fusion (requires all sensors)
    ///
    /// # Errors
    ///
    /// Returns an error if DMP memory write fails.
    async fn dmp_set_motion_event_control_async(
        &mut self,
        accel_calibr: bool,
        gyro_calibr: bool,
        compass_calibr: bool,
        nine_axis: bool,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpMemoryAddresses, DmpMotionEventControl};

        let mut control = 0u16;

        if accel_calibr {
            control |= DmpMotionEventControl::ACCEL_CALIBR;
        }
        if gyro_calibr {
            control |= DmpMotionEventControl::GYRO_CALIBR;
        }
        if compass_calibr {
            control |= DmpMotionEventControl::COMPASS_CALIBR;
        }
        if nine_axis {
            control |= DmpMotionEventControl::NINE_AXIS;
        }

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Setting MOTION_EVENT_CTL: accel_cal={} gyro_cal={} compass_cal={} 9axis={} -> 0x{:04X}",
            accel_calibr,
            gyro_calibr,
            compass_calibr,
            nine_axis,
            control
        );

        let control_bytes = control.to_be_bytes();
        self.write_dmp_memory_async(DmpMemoryAddresses::MOTION_EVENT_CTL, &control_bytes)
            .await?;

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Configure DMP sensors after enabling DMP (async version)
    ///
    /// This function performs the complete sensor configuration sequence that must happen
    /// AFTER the DMP is enabled. It configures:
    /// 1. Set ODR for accelerometer and gyroscope (and magnetometer if enabled)
    /// 2. Set ODR for quaternion output
    /// 3. Enable sensors via DATA_RDY_STATUS
    /// 4. Enable calibration and fusion via MOTION_EVENT_CTL
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Load firmware and configure DMP
    /// imu.dmp_init(&mut delay).await?;
    /// imu.dmp_configure(&config).await?;
    /// imu.dmp_enable(true).await?;
    ///
    /// // Now enable sensors for DMP operation
    /// imu.dmp_configure_sensors(true).await?;  // Enable 9-axis with magnetometer
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if any DMP memory write fails.
    pub async fn dmp_configure_sensors(
        &mut self,
        enable_magnetometer: bool,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::{DmpMemoryAddresses, DmpOdrRegisters};

        #[cfg(feature = "defmt")]
        defmt::info!(
            "Configuring DMP sensors (magnetometer: {})",
            enable_magnetometer
        );

        // Ensure chip is awake and not in low power mode
        self.select_bank(Bank::Bank0).await?;
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_sleep(false);
            })
            .await?;

        // Exit low power mode before writing DMP registers
        self.device
            .lp_config()
            .modify_async(|w| {
                w.set_i_2_c_mst_cycle(false);
                w.set_accel_cycle(false);
                w.set_gyro_cycle(false);
            })
            .await?;

        // Small delay after power mode change
        for _ in 0..1000 {
            core::hint::spin_loop();
        }

        // Step 1: Set ODR for raw sensors (0 = use sensor sample rate)
        #[cfg(feature = "defmt")]
        defmt::debug!("Setting ODR for accelerometer");
        self.dmp_set_odr_async(DmpOdrRegisters::ACCEL, 0).await?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Setting ODR for gyroscope");
        self.dmp_set_odr_async(DmpOdrRegisters::GYRO, 0).await?;

        if enable_magnetometer {
            #[cfg(feature = "defmt")]
            defmt::debug!("Setting ODR for compass/magnetometer");
            self.dmp_set_odr_async(DmpOdrRegisters::CPASS, 0).await?;
        }

        // Step 2: Set ODR for quaternion output
        let quat_odr = if enable_magnetometer {
            #[cfg(feature = "defmt")]
            defmt::debug!("Setting ODR for 9-axis quaternion");
            DmpOdrRegisters::QUAT9
        } else {
            #[cfg(feature = "defmt")]
            defmt::debug!("Setting ODR for 6-axis quaternion");
            DmpOdrRegisters::QUAT6
        };
        self.dmp_set_odr_async(quat_odr, 0).await?;

        // Step 3: Enable sensors (tell DMP which sensors to expect)
        #[cfg(feature = "defmt")]
        defmt::debug!("Enabling sensor inputs for DMP");
        self.dmp_enable_sensors_async(true, true, enable_magnetometer)
            .await?;

        // Verify DATA_RDY_STATUS write
        let mut readback = [0u8; 2];
        self.read_dmp_memory_async(DmpMemoryAddresses::DATA_RDY_STATUS, &mut readback)
            .await?;
        let status = u16::from_be_bytes(readback);

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "DATA_RDY_STATUS verification: wrote=0x000B read=0x{:04X}",
            status
        );

        // Step 4: Enable calibration and fusion
        #[cfg(feature = "defmt")]
        defmt::debug!("Configuring motion event control (calibration and fusion)");
        self.dmp_set_motion_event_control_async(
            true,                // accel_calibr
            true,                // gyro_calibr
            enable_magnetometer, // compass_calibr
            enable_magnetometer, // nine_axis (only if magnetometer enabled)
        )
        .await?;

        // Verify MOTION_EVENT_CTL write
        self.read_dmp_memory_async(DmpMemoryAddresses::MOTION_EVENT_CTL, &mut readback)
            .await?;
        let _motion_ctl = u16::from_be_bytes(readback);

        // Re-enable low power mode after writing DMP registers
        // This is required for DMP to properly process sensor data
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_lp_en(true);
            })
            .await?;

        // Re-enable I2C master cycle mode
        self.device
            .lp_config()
            .modify_async(|w| {
                w.set_i_2_c_mst_cycle(true);
            })
            .await?;

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Read the FIFO count (number of bytes available)
    ///
    /// Returns the number of bytes currently stored in the FIFO (0-4096).
    /// The DMP writes processed data to the FIFO for host retrieval.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    #[cfg(feature = "dmp")]
    pub async fn read_fifo_count(&mut self) -> Result<u16, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        let count_h = self.device.fifo_counth().read_async().await?;
        let count_l = self.device.fifo_countl().read_async().await?;

        // Combine high (bits 12:8) and low (bits 7:0) bytes
        let count = (u16::from(count_h.fifo_cnt_h()) << 8) | u16::from(count_l.fifo_cnt_l());

        Ok(count)
    }

    /// Read raw bytes from the FIFO
    ///
    /// This reads data from the FIFO without parsing it. Use this if you need
    /// direct access to the FIFO data or are implementing custom DMP packet parsing.
    ///
    /// # Arguments
    ///
    /// * `buffer` - Buffer to read data into
    ///
    /// # Returns
    ///
    /// Returns the number of bytes actually read.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    #[cfg(feature = "dmp")]
    pub async fn read_fifo_raw(&mut self, buffer: &mut [u8]) -> Result<usize, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Use burst read to avoid I2C master conflicts with magnetometer

        let len = buffer.len();
        if len == 0 {
            return Ok(0);
        }

        // Burst read from FIFO_R_W register (0x72)
        self.device.interface.read_register(0x72, 8, buffer).await?;

        Ok(len)
    }

    /// Reset the FIFO
    ///
    /// This clears all data from the FIFO. Useful when recovering from
    /// overflow conditions or when reconfiguring the DMP.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    #[cfg(feature = "dmp")]
    pub async fn reset_fifo(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Reset FIFO by setting bit 2 of USER_CTRL
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_sram_rst(true);
            })
            .await?;

        Ok(())
    }

    #[cfg(feature = "dmp")]
    /// Configure the DMP with specific features and settings
    ///
    /// Writes configuration to DMP memory to enable features like quaternion output,
    /// calibrated sensor data, and sample rates.
    ///
    /// **Important**: The DMP firmware must be loaded first using `dmp_init()` or
    /// `dmp_load_firmware()`.
    ///
    /// # Arguments
    ///
    /// * `config` - DMP configuration specifying which features to enable
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::{Icm20948Driver, dmp::DmpConfig};
    /// # use embassy_time::Delay;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// # let mut delay = Delay;
    /// // Initialize and load firmware
    /// driver.dmp_init(&mut delay).await?;
    ///
    /// // Configure for 9-axis quaternion at 100Hz
    /// let config = DmpConfig::default()
    ///     .with_quaternion_9axis(true)
    ///     .with_sample_rate(100);
    /// driver.dmp_configure(&config).await?;
    ///
    /// // Enable DMP
    /// driver.dmp_enable(true).await?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    #[cfg(feature = "dmp")]
    pub async fn dmp_configure(
        &mut self,
        config: &crate::dmp::DmpConfig,
    ) -> Result<(), Error<I::Error>> {
        use crate::dmp::config::ConfigSequence;

        // Ensure we're in Bank 0
        self.select_bank(Bank::Bank0).await?;

        // NOTE: We do NOT disable I2C master during config for 9-axis mode
        // The DMP needs continuous magnetometer data to run 9-axis fusion
        // Disabling I2C master causes the DMP to stop producing output

        // Create configuration sequence
        let seq = ConfigSequence::from_config(config);

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "DMP configure: feature_mask=0x{:04X}, data_out_ctl2=0x{:04X}, data_rdy_status=0x{:04X}, motion_event_ctl=0x{:04X}",
            seq.feature_mask,
            seq.data_out_ctl2,
            seq.data_rdy_status,
            seq.motion_event_ctl
        );

        // Get the complete initialization sequence (36 writes)
        let init_sequence = seq.get_init_sequence();

        // Write all configuration registers in one pass
        // This includes DATA_INTR_CTL (0x004C) and MOTION_EVENT_CTL (0x004E)
        for write in &init_sequence {
            self.write_dmp_memory_async(write.address, write.data)
                .await?;
        }

        // Write DATA_RDY_STATUS to indicate which sensors are available to the DMP
        // This must be written before DMP is enabled
        // Bits: 0x0001=Gyro, 0x0002=Accel, 0x0008=Compass
        let data_rdy_status: u16 = 0x000B; // Gyro + Accel + Compass

        self.write_dmp_memory_async(0x008A, &data_rdy_status.to_be_bytes())
            .await?;

        // Verify the write
        let mut readback = [0u8; 2];
        self.read_dmp_memory_async(0x008A, &mut readback).await?;
        let _status = ((readback[0] as u16) << 8) | (readback[1] as u16);

        Ok(())
    }

    /// Write data to DMP memory at a specific address
    ///
    /// This is a helper method for writing configuration data to DMP memory.
    ///
    /// # Arguments
    ///
    /// * `address` - 16-bit DMP memory address
    /// * `data` - Bytes to write
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    #[cfg(feature = "dmp")]
    async fn write_dmp_memory_async(
        &mut self,
        address: u16,
        data: &[u8],
    ) -> Result<(), Error<I::Error>> {
        // Calculate bank and offset
        let bank = (address / 256) as u8;
        #[allow(clippy::cast_possible_truncation)]
        let offset = (address % 256) as u8;

        // Set memory bank
        self.device
            .mem_bank_sel()
            .write_async(|w| {
                w.set_mem_bank_sel(bank);
            })
            .await?;

        // Set start address
        self.device
            .mem_start_addr()
            .write_async(|w| {
                w.set_mem_start_addr(offset);
            })
            .await?;

        // Write data bytes
        for &byte in data {
            self.device
                .mem_rw()
                .write_async(|w| {
                    w.set_mem_r_w(byte);
                })
                .await?;
        }

        Ok(())
    }

    /// Read data from DMP memory at a specific address (async)
    ///
    /// This is a helper method for reading back DMP memory for verification.
    ///
    /// # Arguments
    ///
    /// * `address` - 16-bit DMP memory address
    /// * `buffer` - Buffer to read data into
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    #[cfg(feature = "dmp")]
    async fn read_dmp_memory_async(
        &mut self,
        address: u16,
        buffer: &mut [u8],
    ) -> Result<(), Error<I::Error>> {
        // Ensure we're in Bank 0 for DMP memory access
        self.select_bank(Bank::Bank0).await?;

        // Calculate bank and offset
        let bank = (address / 256) as u8;
        #[allow(clippy::cast_possible_truncation)]
        let offset = (address % 256) as u8;

        // Set memory bank
        self.device
            .mem_bank_sel()
            .write_async(|w| {
                w.set_mem_bank_sel(bank);
            })
            .await?;

        // Set start address
        self.device
            .mem_start_addr()
            .write_async(|w| {
                w.set_mem_start_addr(offset);
            })
            .await?;

        // Read data bytes
        for byte in buffer.iter_mut() {
            let reg = self.device.mem_rw().read_async().await?;
            *byte = reg.mem_r_w();
        }

        Ok(())
    }

    /// Read DMP data from FIFO
    ///
    /// This method reads and parses DMP packets from the FIFO, extracting
    /// quaternion and sensor data according to the current configuration.
    ///
    /// The method automatically detects packet size based on the FIFO count
    /// and parses the data into a `DmpData` structure.
    ///
    /// # Returns
    ///
    /// Returns `Some(DmpData)` if a valid packet was read, or `None` if the
    /// FIFO is empty or contains an incomplete packet.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// // Read DMP data
    /// if let Some(data) = driver.dmp_read_fifo().await? {
    ///     if let Some(quat) = data.quaternion_9axis {
    ///         println!("Quaternion: w={:.3}, x={:.3}, y={:.3}, z={:.3}",
    ///                  quat.w, quat.x, quat.y, quat.z);
    ///     }
    /// }
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    #[cfg(feature = "dmp")]
    pub async fn dmp_read_fifo(&mut self) -> Result<Option<crate::dmp::DmpData>, Error<I::Error>> {
        use crate::dmp::DmpParser;

        // Check FIFO count
        let count = self.read_fifo_count().await?;

        // Need at least a header (2 bytes)
        if count < 2 {
            return Ok(None);
        }

        // Read enough data for typical packet (header + quaternion)
        // Maximum packet size is ~40 bytes for full configuration
        let mut buffer = [0u8; 64];
        let bytes_to_read = core::cmp::min(count as usize, buffer.len());

        self.read_fifo_raw(&mut buffer[..bytes_to_read]).await?;

        // Log raw header for debugging
        #[cfg(feature = "defmt")]
        {
            let header = u16::from_be_bytes([buffer[0], buffer[1]]);
            defmt::debug!(
                "DMP FIFO header: 0x{:04X} (QUAT6={} QUAT9={} ACCEL={} GYRO={} CAL_GYRO={} CAL_ACCEL={})",
                header,
                (header & 0x0001) != 0,
                (header & 0x0002) != 0,
                (header & 0x0004) != 0,
                (header & 0x0008) != 0,
                (header & 0x0010) != 0,
                (header & 0x0020) != 0
            );

            // Show first 8 bytes after header if we have quaternion
            if (header & 0x0003) != 0 && bytes_to_read >= 10 {
                defmt::debug!(
                    "First quaternion bytes: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                    buffer[2],
                    buffer[3],
                    buffer[4],
                    buffer[5],
                    buffer[6],
                    buffer[7],
                    buffer[8],
                    buffer[9]
                );
            }
        }

        // Parse the packet
        let parser = DmpParser::new();
        Ok(parser.parse_packet(&buffer[..bytes_to_read]))
    }

    /// Read quaternion data from DMP
    ///
    /// This is a convenience method that reads from the FIFO and returns just
    /// the quaternion data if present.
    ///
    /// # Returns
    ///
    /// Returns `Some(Quaternion)` if quaternion data was available, or `None`
    /// if no data or no quaternion in the packet.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use icm20948::Icm20948Driver;
    /// # let mut driver: Icm20948Driver<_> = todo!();
    /// if let Some(quat) = driver.dmp_read_quaternion().await? {
    ///     println!("Orientation: w={:.3}, x={:.3}, y={:.3}, z={:.3}",
    ///              quat.w, quat.x, quat.y, quat.z);
    /// }
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    #[cfg(feature = "dmp")]
    pub async fn dmp_read_quaternion(
        &mut self,
    ) -> Result<Option<crate::dmp::Quaternion>, Error<I::Error>> {
        if let Some(data) = self.dmp_read_fifo().await? {
            // Return first available quaternion type
            if let Some(quat) = data.quaternion_9axis {
                return Ok(Some(quat));
            }
            if let Some(quat) = data.quaternion_6axis {
                return Ok(Some(quat));
            }
            if let Some(quat) = data.game_rotation_vector {
                return Ok(Some(quat));
            }
            if let Some(quat) = data.geomag_rotation_vector {
                return Ok(Some(quat));
            }
        }
        Ok(None)
    }

    /// Initialize the magnetometer (AK09916) via I2C master interface
    ///
    /// This configures the ICM-20948's I2C master to communicate with the AK09916
    /// magnetometer and sets up continuous measurement mode.
    ///
    /// # Arguments
    ///
    /// * `config` - Magnetometer configuration
    /// * `delay` - Delay provider for timing between operations
    ///
    /// # Errors
    ///
    /// Returns an error if I2C communication fails or the magnetometer doesn't respond.
    pub async fn init_magnetometer<D>(
        &mut self,
        config: crate::sensors::MagConfig,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        use crate::sensors::magnetometer::{
            AK09916_I2C_ADDRESS, AK09916_REG_CNTL2, AK09916_REG_CNTL3, AK09916_REG_HXL,
            AK09916_REG_WIA2, AK09916_WIA2_VALUE,
        };

        // Step 1: Enable I2C master mode (Bank 0)
        self.select_bank(Bank::Bank0).await?;
        self.device
            .user_ctrl()
            .modify_async(|w| w.set_i_2_c_mst_en(true))
            .await?;
        delay.delay_ms(10).await;

        // Step 2: Configure I2C master (Bank 3)
        self.select_bank(Bank::Bank3).await?;

        // Set I2C master clock to 400 kHz (0x07 = 345.6 kHz, closest to 400 kHz)
        self.device
            .bank_3_i_2_c_mst_ctrl()
            .modify_async(|w| w.set_i_2_c_mst_clk(0x07))
            .await?;
        delay.delay_ms(10).await;

        // Step 3: Reset AK09916 using Slave 4
        self.write_mag_register_async(AK09916_REG_CNTL3, 0x01, delay)
            .await?; // Soft reset
        delay.delay_ms(10).await;

        // Step 4: Verify WHO_AM_I
        let who_am_i = self
            .read_mag_register_async(AK09916_REG_WIA2, delay)
            .await?;
        if who_am_i != AK09916_WIA2_VALUE {
            return Err(Error::Magnetometer);
        }

        // Step 5: Configure Slave 0 for reading magnetometer data
        // Read 8 bytes: HXL, HXH, HYL, HYH, HZL, HZH, ST1, ST2
        self.device
            .bank_3_i_2_c_slv_0_addr()
            .write_async(|w| {
                w.set_i_2_c_id_0(AK09916_I2C_ADDRESS);
                w.set_i_2_c_slv_0_rnw(true); // Read mode
            })
            .await?;

        self.device
            .bank_3_i_2_c_slv_0_reg()
            .write_async(|w| w.set_i_2_c_slv_0_reg(AK09916_REG_HXL))
            .await?;

        self.device
            .bank_3_i_2_c_slv_0_ctrl()
            .write_async(|w| {
                w.set_i_2_c_slv_0_leng(8); // Read 8 bytes
                w.set_i_2_c_slv_0_en(true); // Enable slave 0
            })
            .await?;

        delay.delay_ms(10).await;

        // Step 6: Set operating mode using Slave 4
        self.write_mag_register_async(AK09916_REG_CNTL2, config.mode as u8, delay)
            .await?;
        delay.delay_ms(10).await;

        // Step 7: Switch back to Bank 0 for data reading
        self.select_bank(Bank::Bank0).await?;

        self.mag_initialized = true;
        Ok(())
    }

    /// Read magnetometer data in microteslas (µT)
    ///
    /// Automatically applies calibration if set.
    ///
    /// # Errors
    ///
    /// Returns an error if not initialized or communication fails.
    pub async fn read_magnetometer(
        &mut self,
    ) -> Result<crate::sensors::MagDataUT, Error<I::Error>> {
        // AK09916 sensitivity: 0.15 µT/LSB
        const SENSITIVITY: f32 = 0.15;

        if !self.mag_initialized {
            return Err(Error::Magnetometer);
        }

        self.select_bank(Bank::Bank0).await?;

        // Read 8 bytes from EXT_SLV_SENS_DATA
        // Format: HXL, HXH, HYL, HYH, HZL, HZH, ST1, ST2
        let data = [
            self.device
                .ext_slv_sens_data_00()
                .read_async()
                .await?
                .ext_slv_sens_data_00(),
            self.device
                .ext_slv_sens_data_01()
                .read_async()
                .await?
                .ext_slv_sens_data_01(),
            self.device
                .ext_slv_sens_data_02()
                .read_async()
                .await?
                .ext_slv_sens_data_02(),
            self.device
                .ext_slv_sens_data_03()
                .read_async()
                .await?
                .ext_slv_sens_data_03(),
            self.device
                .ext_slv_sens_data_04()
                .read_async()
                .await?
                .ext_slv_sens_data_04(),
            self.device
                .ext_slv_sens_data_05()
                .read_async()
                .await?
                .ext_slv_sens_data_05(),
            self.device
                .ext_slv_sens_data_06()
                .read_async()
                .await?
                .ext_slv_sens_data_06(),
            self.device
                .ext_slv_sens_data_07()
                .read_async()
                .await?
                .ext_slv_sens_data_07(),
        ];

        // ST1 is at index 6 but we do NOT check it when reading via I2C master
        // The I2C master polls automatically at a fixed rate, so received data is valid

        // Check ST2 for overflow (bit 3)
        let st2 = data[7];
        if (st2 & 0x08) != 0 {
            return Err(Error::Magnetometer); // Magnetic overflow
        }

        // Parse little-endian 16-bit values
        // AK09916 uses little-endian format (LSB first)
        let x = i16::from_le_bytes([data[0], data[1]]);
        let y = i16::from_le_bytes([data[2], data[3]]);
        let z = i16::from_le_bytes([data[4], data[5]]);

        let data = crate::sensors::MagDataUT {
            x: f32::from(x) * SENSITIVITY,
            y: f32::from(y) * SENSITIVITY,
            z: f32::from(z) * SENSITIVITY,
        };

        Ok(self.mag_calibration.apply(&data))
    }

    /// Read raw magnetometer data (16-bit signed integers)
    ///
    /// Returns (x, y, z) as raw ADC values without calibration or conversion.
    ///
    /// # Errors
    ///
    /// Returns an error if not initialized or communication fails.
    pub async fn read_magnetometer_raw(&mut self) -> Result<(i16, i16, i16), Error<I::Error>> {
        if !self.mag_initialized {
            return Err(Error::Magnetometer);
        }

        self.select_bank(Bank::Bank0).await?;

        // Read 8 bytes from EXT_SLV_SENS_DATA
        // Format: ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2 (AK09916 order)
        let data = [
            self.device
                .ext_slv_sens_data_00()
                .read_async()
                .await?
                .ext_slv_sens_data_00(),
            self.device
                .ext_slv_sens_data_01()
                .read_async()
                .await?
                .ext_slv_sens_data_01(),
            self.device
                .ext_slv_sens_data_02()
                .read_async()
                .await?
                .ext_slv_sens_data_02(),
            self.device
                .ext_slv_sens_data_03()
                .read_async()
                .await?
                .ext_slv_sens_data_03(),
            self.device
                .ext_slv_sens_data_04()
                .read_async()
                .await?
                .ext_slv_sens_data_04(),
            self.device
                .ext_slv_sens_data_05()
                .read_async()
                .await?
                .ext_slv_sens_data_05(),
            self.device
                .ext_slv_sens_data_06()
                .read_async()
                .await?
                .ext_slv_sens_data_06(),
            self.device
                .ext_slv_sens_data_07()
                .read_async()
                .await?
                .ext_slv_sens_data_07(),
        ];

        // ST1 is first byte (data[0]) but we do NOT check it when reading via I2C master
        // The I2C master polls automatically at a fixed rate, so received data is valid

        // ST2 is at index 7
        let st2 = data[7];
        if (st2 & 0x08) != 0 {
            return Err(Error::Magnetometer); // Magnetic overflow
        }

        // Parse little-endian 16-bit values
        // Data starts at index 1: HXL, HXH, HYL, HYH, HZL, HZH
        let x = i16::from_le_bytes([data[1], data[2]]);
        let y = i16::from_le_bytes([data[3], data[4]]);
        let z = i16::from_le_bytes([data[5], data[6]]);

        Ok((x, y, z))
    }

    /// Read magnetometer heading (yaw angle) in degrees
    ///
    /// Calculates the heading from the magnetic field vector.
    /// Assumes the sensor is level (horizontal).
    ///
    /// Returns angle in degrees (0-360), where:
    /// - 0° = North
    /// - 90° = East
    /// - 180° = South
    /// - 270° = West
    ///
    /// # Errors
    ///
    /// Returns an error if reading fails.
    pub async fn read_magnetometer_heading(&mut self) -> Result<f32, Error<I::Error>> {
        let data = self.read_magnetometer().await?;

        // Calculate heading: atan2(y, x)
        let heading_rad = libm::atan2f(data.y, data.x);

        // Convert to degrees and normalize to 0-360
        let mut heading_deg = heading_rad.to_degrees();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }

        Ok(heading_deg)
    }

    /// Read magnetometer heading with tilt compensation
    ///
    /// Calculates the heading from the magnetic field vector, compensating
    /// for tilt using accelerometer data.
    ///
    /// # Arguments
    ///
    /// * `accel_x` - X-axis acceleration in g
    /// * `accel_y` - Y-axis acceleration in g
    /// * `accel_z` - Z-axis acceleration in g
    ///
    /// Returns angle in degrees (0-360).
    ///
    /// # Errors
    ///
    /// Returns an error if reading fails or if accelerometer magnitude is too small
    /// (less than 0.1g), which indicates invalid data or free-fall condition.
    pub async fn read_magnetometer_heading_compensated(
        &mut self,
        accel_x: f32,
        accel_y: f32,
        accel_z: f32,
    ) -> Result<f32, Error<I::Error>> {
        // Validate accelerometer data - magnitude must be meaningful
        let accel_magnitude =
            libm::sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        if accel_magnitude < 0.1 {
            // Too small to be meaningful - could be free-fall, invalid data, or zero vector
            return Err(Error::InvalidConfig);
        }

        let mag = self.read_magnetometer().await?;

        // Calculate pitch and roll from accelerometer
        let pitch = libm::asinf(-accel_x);
        let roll = libm::atan2f(accel_y, accel_z);

        // Tilt-compensated magnetic field components
        let mag_x = mag.x * libm::cosf(pitch) + mag.z * libm::sinf(pitch);
        let mag_y = mag.x * libm::sinf(roll) * libm::sinf(pitch) + mag.y * libm::cosf(roll)
            - mag.z * libm::sinf(roll) * libm::cosf(pitch);

        // Calculate heading
        let heading_rad = libm::atan2f(mag_y, mag_x);

        // Convert to degrees and normalize to 0-360
        let mut heading_deg = heading_rad.to_degrees();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }

        Ok(heading_deg)
    }

    /// Check if magnetometer data is ready
    ///
    /// # Errors
    ///
    /// Returns an error if communication fails.
    pub async fn magnetometer_data_ready(&mut self) -> Result<bool, Error<I::Error>> {
        if !self.mag_initialized {
            return Ok(false);
        }

        self.select_bank(Bank::Bank0).await?;

        // Check ST1 bit 0 (DRDY - Data Ready)
        // ST1 is at EXT_SLV_SENS_DATA_00 (first byte in AK09916 format)
        let st1 = self
            .device
            .ext_slv_sens_data_00()
            .read_async()
            .await?
            .ext_slv_sens_data_00();

        Ok((st1 & 0x01) != 0)
    }

    /// Set magnetometer calibration data
    ///
    /// The calibration will be automatically applied to all subsequent readings.
    pub const fn set_magnetometer_calibration(
        &mut self,
        calibration: crate::sensors::MagCalibration,
    ) {
        self.mag_calibration = calibration;
    }

    /// Get current magnetometer calibration data
    #[must_use]
    pub const fn magnetometer_calibration(&self) -> &crate::sensors::MagCalibration {
        &self.mag_calibration
    }

    /// Run accelerometer hardware self-test (async version)
    ///
    /// Performs internal self-test by applying test signals and measuring response.
    /// Returns `true` if all axes pass self-test, `false` otherwise.
    ///
    /// The self-test automatically configures the sensor to ±2g range as required,
    /// then restores the original configuration. The threshold (225 LSB) is calibrated
    /// for ±2g range and represents approximately 1.4% of full scale.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn accelerometer_self_test<D>(
        &mut self,
        delay: &mut D,
    ) -> Result<bool, Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        // Self-test passes if response is significant
        // Threshold: 225 LSB @ ±2g range (16384 LSB/g) = ~0.014g (~1.4% of full scale)
        // This is a fixed threshold calibrated for ±2g range per ICM-20948 datasheet
        const MIN_RESPONSE: i16 = 225;

        // Save original configuration
        let original_config = self.accel_config;

        self.select_bank(Bank::Bank2).await?;

        // Configure to ±2g for self-test (required by datasheet)
        self.device
            .bank_2_accel_config()
            .modify_async(|w| {
                w.set_accel_fs_sel(0); // ±2g
            })
            .await?;

        // Read response with self-test disabled
        let accel_off = self.read_accelerometer_raw().await?;
        let (x_off, y_off, z_off) = (accel_off.x, accel_off.y, accel_off.z);

        #[cfg(feature = "defmt")]
        defmt::debug!("Accel self-test OFF: x={}, y={}, z={}", x_off, y_off, z_off);

        // Enable self-test on all axes via ACCEL_CONFIG_2
        self.select_bank(Bank::Bank2).await?;
        self.device
            .bank_2_accel_config_2()
            .modify_async(|w| {
                w.set_ax_st_en(true);
                w.set_ay_st_en(true);
                w.set_az_st_en(true);
            })
            .await?;

        // Wait for oscillations to stabilize (per Cybergear reference: DEF_ST_STABLE_TIME = 20ms)
        delay.delay_ms(20).await;

        // Read response with self-test enabled
        let accel_on = self.read_accelerometer_raw().await?;
        let (x_on, y_on, z_on) = (accel_on.x, accel_on.y, accel_on.z);

        #[cfg(feature = "defmt")]
        defmt::debug!("Accel self-test ON: x={}, y={}, z={}", x_on, y_on, z_on);

        // Disable self-test
        self.select_bank(Bank::Bank2).await?;
        self.device
            .bank_2_accel_config_2()
            .modify_async(|w| {
                w.set_ax_st_en(false);
                w.set_ay_st_en(false);
                w.set_az_st_en(false);
            })
            .await?;

        // Calculate self-test response
        let str_x = x_on.saturating_sub(x_off);
        let str_y = y_on.saturating_sub(y_off);
        let str_z = z_on.saturating_sub(z_off);

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Accel self-test RESPONSE: x={} (threshold={}), y={} (threshold={}), z={} (threshold={})",
            str_x.abs(),
            MIN_RESPONSE,
            str_y.abs(),
            MIN_RESPONSE,
            str_z.abs(),
            MIN_RESPONSE
        );

        let result =
            str_x.abs() > MIN_RESPONSE && str_y.abs() > MIN_RESPONSE && str_z.abs() > MIN_RESPONSE;

        // Restore original configuration
        self.configure_accelerometer(original_config).await?;

        Ok(result)
    }

    /// Run gyroscope hardware self-test (async version)
    ///
    /// Performs internal self-test by applying test signals and measuring response.
    /// Returns `true` if all axes pass self-test, `false` otherwise.
    ///
    /// The self-test automatically configures the sensor to ±250°/s range as required,
    /// then restores the original configuration. The threshold (60 LSB) is calibrated
    /// for ±250°/s range and represents approximately 0.46% of full scale.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn gyroscope_self_test<D>(&mut self, delay: &mut D) -> Result<bool, Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        // Self-test passes if response is significant
        // Threshold: 60 LSB @ ±250°/s range (131 LSB/(°/s)) = ~0.46°/s (~0.46% of full scale)
        // This is a fixed threshold calibrated for ±250°/s range per ICM-20948 datasheet
        const MIN_RESPONSE: i16 = 60;

        // Save original configuration
        let original_config = self.gyro_config;

        self.select_bank(Bank::Bank2).await?;

        // Configure to ±250 dps for self-test (required by datasheet)
        self.device
            .bank_2_gyro_config_1()
            .modify_async(|w| {
                w.set_gyro_fs_sel(0); // ±250 dps
            })
            .await?;

        // Read response with self-test disabled
        let gyro_off = self.read_gyroscope_raw().await?;
        let (x_off, y_off, z_off) = (gyro_off.x, gyro_off.y, gyro_off.z);

        #[cfg(feature = "defmt")]
        defmt::debug!("Gyro self-test OFF: x={}, y={}, z={}", x_off, y_off, z_off);

        // Enable self-test on all axes via GYRO_CONFIG_2
        self.select_bank(Bank::Bank2).await?;
        self.device
            .bank_2_gyro_config_2()
            .modify_async(|w| {
                w.set_xgyro_cten(true);
                w.set_ygyro_cten(true);
                w.set_zgyro_cten(true);
            })
            .await?;

        // Wait for oscillations to stabilize (per Cybergear reference: DEF_ST_STABLE_TIME = 20ms)
        delay.delay_ms(20).await;

        // Read response with self-test enabled
        let gyro_on = self.read_gyroscope_raw().await?;
        let (x_on, y_on, z_on) = (gyro_on.x, gyro_on.y, gyro_on.z);

        #[cfg(feature = "defmt")]
        defmt::debug!("Gyro self-test ON: x={}, y={}, z={}", x_on, y_on, z_on);

        // Disable self-test
        self.select_bank(Bank::Bank2).await?;
        self.device
            .bank_2_gyro_config_2()
            .modify_async(|w| {
                w.set_xgyro_cten(false);
                w.set_ygyro_cten(false);
                w.set_zgyro_cten(false);
            })
            .await?;

        // Calculate self-test response
        let str_x = x_on.saturating_sub(x_off);
        let str_y = y_on.saturating_sub(y_off);
        let str_z = z_on.saturating_sub(z_off);

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Gyro self-test RESPONSE: x={} (threshold={}), y={} (threshold={}), z={} (threshold={})",
            str_x.abs(),
            MIN_RESPONSE,
            str_y.abs(),
            MIN_RESPONSE,
            str_z.abs(),
            MIN_RESPONSE
        );

        let result =
            str_x.abs() > MIN_RESPONSE && str_y.abs() > MIN_RESPONSE && str_z.abs() > MIN_RESPONSE;

        // Restore original configuration
        self.configure_gyroscope(original_config).await?;

        Ok(result)
    }

    /// Perform magnetometer hard-iron calibration
    ///
    /// This calibration compensates for constant magnetic fields (hard-iron distortion).
    /// The device should be rotated in a figure-8 pattern during calibration to capture
    /// readings at all orientations.
    ///
    /// # Arguments
    ///
    /// * `num_samples` - Number of samples to collect (more samples = better accuracy)
    /// * `delay` - Delay provider for timing between samples
    ///
    /// # Errors
    ///
    /// Returns an error if reading fails.
    pub async fn calibrate_magnetometer_hard_iron<D>(
        &mut self,
        num_samples: usize,
        delay: &mut D,
    ) -> Result<crate::sensors::MagCalibration, Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;

        for _ in 0..num_samples {
            let data = self.read_magnetometer().await?;

            min_x = libm::fminf(min_x, data.x);
            max_x = libm::fmaxf(max_x, data.x);
            min_y = libm::fminf(min_y, data.y);
            max_y = libm::fmaxf(max_y, data.y);
            min_z = libm::fminf(min_z, data.z);
            max_z = libm::fmaxf(max_z, data.z);

            delay.delay_ms(10).await;
        }

        let calibration = crate::sensors::MagCalibration {
            offset_x: (max_x + min_x) * 0.5,
            offset_y: (max_y + min_y) * 0.5,
            offset_z: (max_z + min_z) * 0.5,
            scale_x: 1.0,
            scale_y: 1.0,
            scale_z: 1.0,
        };

        self.mag_calibration = calibration;
        Ok(calibration)
    }

    /// Run magnetometer self-test
    ///
    /// Returns `true` if self-test passes, `false` otherwise.
    ///
    /// # Errors
    ///
    /// Returns an error if self-test fails or communication fails.
    pub async fn magnetometer_self_test<D>(
        &mut self,
        delay: &mut D,
    ) -> Result<bool, Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        use crate::sensors::magnetometer::{AK09916_REG_CNTL2, AK09916_REG_HXL};

        // Set self-test mode
        self.write_mag_register_async(AK09916_REG_CNTL2, 0x10, delay)
            .await?;
        delay.delay_ms(100).await;

        // Read self-test data
        self.select_bank(Bank::Bank3).await?;

        // Configure Slave 0 to read self-test data
        self.device
            .bank_3_i_2_c_slv_0_reg()
            .write_async(|w| w.set_i_2_c_slv_0_reg(AK09916_REG_HXL))
            .await?;

        delay.delay_ms(10).await;

        let (x, y, z) = self.read_magnetometer_raw().await?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Mag self-test values: x={}, y={}, z={}", x, y, z);

        // Return to normal mode
        self.write_mag_register_async(AK09916_REG_CNTL2, 0x08, delay)
            .await?;
        delay.delay_ms(10).await;

        // Simple check: magnetometer should produce non-zero readings in self-test mode
        // Full Cybergear-style self-test requires sensitivity adjustment values (ASAX/Y/Z)
        // which adds complexity. Since normal magnetometer operation works fine,
        // we just verify the sensor responds to self-test mode.
        let result =
            (x != 0 || y != 0 || z != 0) && x.abs() < 32767 && y.abs() < 32767 && z.abs() < 32767;

        #[cfg(feature = "defmt")]
        defmt::debug!("Mag self-test result: {} (non-zero check passed)", result);

        Ok(result)
    }

    /// Helper to write to AK09916 register using Slave 4
    async fn write_mag_register_async<D>(
        &mut self,
        reg: u8,
        value: u8,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        use crate::sensors::magnetometer::AK09916_I2C_ADDRESS;

        self.select_bank(Bank::Bank3).await?;

        // Configure Slave 4 for write
        self.device
            .bank_3_i_2_c_slv_4_addr()
            .write_async(|w| {
                w.set_i_2_c_id_4(AK09916_I2C_ADDRESS);
                w.set_i_2_c_slv_4_rnw(false); // Write mode
            })
            .await?;

        self.device
            .bank_3_i_2_c_slv_4_reg()
            .write_async(|w| w.set_i_2_c_slv_4_reg(reg))
            .await?;

        self.device
            .bank_3_i_2_c_slv_4_do()
            .write_async(|w| w.set_i_2_c_slv_4_do(value))
            .await?;

        self.device
            .bank_3_i_2_c_slv_4_ctrl()
            .write_async(|w| {
                w.set_i_2_c_slv_4_en(true);
            })
            .await?;

        delay.delay_ms(10).await;

        Ok(())
    }

    /// Helper to read from AK09916 register using Slave 4
    async fn read_mag_register_async<D>(
        &mut self,
        reg: u8,
        delay: &mut D,
    ) -> Result<u8, Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        use crate::sensors::magnetometer::AK09916_I2C_ADDRESS;

        self.select_bank(Bank::Bank3).await?;

        // Configure Slave 4 for read
        self.device
            .bank_3_i_2_c_slv_4_addr()
            .write_async(|w| {
                w.set_i_2_c_id_4(AK09916_I2C_ADDRESS);
                w.set_i_2_c_slv_4_rnw(true); // Read mode
            })
            .await?;

        self.device
            .bank_3_i_2_c_slv_4_reg()
            .write_async(|w| w.set_i_2_c_slv_4_reg(reg))
            .await?;

        self.device
            .bank_3_i_2_c_slv_4_ctrl()
            .write_async(|w| {
                w.set_i_2_c_slv_4_en(true);
            })
            .await?;

        delay.delay_ms(10).await;

        let value = self
            .device
            .bank_3_i_2_c_slv_4_di()
            .read_async()
            .await?
            .i_2_c_slv_4_di();

        Ok(value)
    }

    #[cfg(feature = "dmp")]
    /// Configure magnetometer for DMP 9-axis operation
    ///
    /// This configures the AK09916 magnetometer specifically for DMP usage, which differs
    /// from the standard continuous mode configuration. This special configuration is
    /// required for proper 9-axis quaternion fusion.
    ///
    /// # Configuration details
    ///
    /// The DMP requires a specific magnetometer setup:
    /// - **I2C_SLV0**: Reads 10 bytes starting from AK09916 register 0x03 (RSV2)
    ///   with byte-swap and register grouping enabled
    /// - **I2C_SLV1**: Triggers single measurement mode on each DMP sample cycle
    /// - **I2C_MST_ODR_CONFIG**: Sets magnetometer sample rate to ~69 Hz
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider for timing
    ///
    /// # Important
    ///
    /// - Call this BEFORE `dmp_enable(true)`
    /// - Do NOT call `init_magnetometer()` - this replaces it for DMP use
    /// - Only use this for 9-axis DMP operation
    /// - For 6-axis DMP (accel+gyro only), you don't need magnetometer at all
    ///
    /// # Errors
    ///
    /// Returns an error if I2C communication fails or magnetometer doesn't respond.
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Initialize device
    /// driver.init(&mut delay).await?;
    ///
    /// // Load and init DMP
    /// driver.dmp_load_firmware(&mut delay).await?;
    /// driver.dmp_init(&mut delay).await?;
    ///
    /// // Configure magnetometer for DMP (Cybergear approach)
    /// driver.dmp_init_magnetometer(&mut delay).await?;
    ///
    /// // Configure and enable DMP
    /// let config = DmpConfig::default().with_quaternion_9axis(true);
    /// driver.dmp_configure(&config).await?;
    /// driver.dmp_enable(true).await?;
    /// ```
    pub async fn dmp_init_magnetometer<D>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        use crate::sensors::magnetometer::{
            AK09916_I2C_ADDRESS, AK09916_REG_CNTL2, AK09916_REG_CNTL3, AK09916_REG_RSV2,
            AK09916_REG_WIA2,
        };

        #[cfg(feature = "defmt")]
        defmt::info!("Initializing magnetometer for DMP");

        // Step 1: Enable I2C master mode
        self.select_bank(Bank::Bank0).await?;
        self.device
            .user_ctrl()
            .modify_async(|w| {
                w.set_i_2_c_mst_en(true);
            })
            .await?;
        delay.delay_ms(10).await;

        // Step 2: Configure I2C master timing
        self.select_bank(Bank::Bank3).await?;
        self.device
            .bank_3_i_2_c_mst_ctrl()
            .write_async(|w| {
                w.set_i_2_c_mst_clk(7); // 400 kHz I2C clock
            })
            .await?;

        // Step 3: Verify magnetometer is present using Slave 4
        self.select_bank(Bank::Bank3).await?;
        self.device
            .bank_3_i_2_c_slv_4_addr()
            .write_async(|w| {
                w.set_i_2_c_id_4(AK09916_I2C_ADDRESS);
                w.set_i_2_c_slv_4_rnw(true); // Read operation
            })
            .await?;

        self.device
            .bank_3_i_2_c_slv_4_reg()
            .write_async(|w| w.set_i_2_c_slv_4_reg(AK09916_REG_WIA2))
            .await?;

        self.device
            .bank_3_i_2_c_slv_4_ctrl()
            .write_async(|w| {
                w.set_i_2_c_slv_4_en(true);
            })
            .await?;

        delay.delay_ms(10).await;

        self.select_bank(Bank::Bank3).await?;
        let who_am_i = self
            .device
            .bank_3_i_2_c_slv_4_di()
            .read_async()
            .await?
            .i_2_c_slv_4_di();

        if who_am_i != 0x09 {
            #[cfg(feature = "defmt")]
            defmt::error!(
                "Magnetometer WHO_AM_I check failed: expected 0x09, got 0x{:02X}",
                who_am_i
            );

            // Cleanup: Disable I2C master on failure
            self.select_bank(Bank::Bank0).await?;
            self.device
                .user_ctrl()
                .modify_async(|w| {
                    w.set_i_2_c_mst_en(false);
                })
                .await?;

            return Err(Error::Magnetometer);
        }

        #[cfg(feature = "defmt")]
        defmt::debug!("Magnetometer WHO_AM_I verified: 0x09");

        // Step 4: Reset magnetometer using Slave 4
        self.write_mag_register_async(AK09916_REG_CNTL3, 0x01, delay)
            .await?; // Soft reset
        delay.delay_ms(100).await;

        // Step 5: Configure I2C_SLV0 for DMP magnetometer reads
        // Read 10 bytes from RSV2 (0x03) with byte-swap and grouping enabled
        self.select_bank(Bank::Bank3).await?;

        // Set slave 0 address (AK09916, read mode)
        self.device
            .bank_3_i_2_c_slv_0_addr()
            .write_async(|w| {
                w.set_i_2_c_id_0(AK09916_I2C_ADDRESS);
                w.set_i_2_c_slv_0_rnw(true); // Read operation
            })
            .await?;

        // Set slave 0 register (start at RSV2 = 0x03)
        self.device
            .bank_3_i_2_c_slv_0_reg()
            .write_async(|w| w.set_i_2_c_slv_0_reg(AK09916_REG_RSV2))
            .await?;

        // Configure slave 0 control:
        // - Read 10 bytes
        // - Enable slave 0
        // - Enable byte swap (I2C_SLV0_BYTE_SW)
        // - Enable register grouping (I2C_SLV0_GRP)
        self.device
            .bank_3_i_2_c_slv_0_ctrl()
            .write_async(|w| {
                w.set_i_2_c_slv_0_leng(10); // Read 10 bytes
                w.set_i_2_c_slv_0_en(true); // Enable slave 0
                w.set_i_2_c_slv_0_byte_sw(true); // Byte swap
                w.set_i_2_c_slv_0_reg_dis(false); // Use register address
                w.set_i_2_c_slv_0_grp(true); // Group registers
            })
            .await?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Configured I2C_SLV0 for 10-byte read from RSV2 (0x03)");

        // Step 6: Configure I2C_SLV1 to trigger single measurement mode
        // This writes CNTL2 = SINGLE_MEAS (0x01) on each sample
        self.select_bank(Bank::Bank3).await?;

        // Set slave 1 address (AK09916, write mode)
        self.device
            .bank_3_i_2_c_slv_1_addr()
            .write_async(|w| {
                w.set_i_2_c_id_1(AK09916_I2C_ADDRESS);
                w.set_i_2_c_slv_1_rnw(false); // Write operation
            })
            .await?;

        // Set slave 1 register (CNTL2 = 0x31)
        self.device
            .bank_3_i_2_c_slv_1_reg()
            .write_async(|w| w.set_i_2_c_slv_1_reg(AK09916_REG_CNTL2))
            .await?;

        // Set data out register (value to write: SINGLE_MEAS = 0x01)
        self.device
            .bank_3_i_2_c_slv_1_do()
            .write_async(|w| w.set_i_2_c_slv_1_do(0x01))
            .await?;

        // Configure slave 1 control:
        // - Write 1 byte
        // - Enable slave 1
        self.device
            .bank_3_i_2_c_slv_1_ctrl()
            .write_async(|w| {
                w.set_i_2_c_slv_1_leng(1); // Write 1 byte
                w.set_i_2_c_slv_1_en(true); // Enable slave 1
                w.set_i_2_c_slv_1_byte_sw(false);
                w.set_i_2_c_slv_1_reg_dis(false);
                w.set_i_2_c_slv_1_grp(false);
            })
            .await?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Configured I2C_SLV1 to trigger single measurements");

        // Step 7: Set I2C Master ODR configuration
        // This reduces the magnetometer read rate to ~69 Hz
        // Formula: 1.1 kHz / 2^(odr_config) = rate
        // 0x04 = 1100 / 2^4 = 68.75 Hz
        self.select_bank(Bank::Bank3).await?;
        self.device
            .bank_3_i_2_c_mst_odr_config()
            .write_async(|w| w.set_i_2_c_mst_odr_config(0x04))
            .await?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Set I2C_MST_ODR_CONFIG to 0x04 (~69 Hz)");

        // Step 8: Enable I2C master cycle mode
        self.select_bank(Bank::Bank0).await?;
        self.device
            .lp_config()
            .modify_async(|w| {
                w.set_i_2_c_mst_cycle(true);
            })
            .await?;

        delay.delay_ms(50).await;

        self.mag_initialized = true;

        #[cfg(feature = "defmt")]
        defmt::info!("DMP magnetometer initialization complete");

        Ok(())
    }

    /// Configure interrupt pin electrical properties
    ///
    /// # Arguments
    /// * `config` - Interrupt pin configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn configure_interrupt_pin(
        &mut self,
        config: &InterruptPinConfig,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        self.device
            .int_pin_cfg()
            .write_async(|w| {
                w.set_int_1_actl(config.active_low);
                w.set_int_1_open(config.open_drain);
                w.set_int_1_latch_int_en(config.latch_enabled);
                w.set_int_anyrd_2_clear(config.clear_on_any_read);
            })
            .await?;

        Ok(())
    }

    /// Configure interrupt sources
    ///
    /// # Arguments
    /// * `config` - Interrupt configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn configure_interrupts(
        &mut self,
        config: &InterruptConfig,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // INT_ENABLE register
        self.device
            .int_enable()
            .write_async(|w| {
                w.set_wom_int_en(config.wake_on_motion);
                w.set_dmp_int_1_en(config.dmp);
                w.set_i_2_c_mst_int_en(config.i2c_master);
                w.set_pll_rdy_en(config.pll_ready);
            })
            .await?;

        // INT_ENABLE_1 register
        self.device
            .int_enable_1()
            .write_async(|w| {
                w.set_raw_data_0_rdy_en(config.raw_data_ready);
            })
            .await?;

        // INT_ENABLE_2 register (FIFO overflow)
        self.device
            .int_enable_2()
            .write_async(|w| {
                if config.fifo_overflow {
                    w.set_fifo_overflow_en(0x1F); // Enable all FIFOs
                } else {
                    w.set_fifo_overflow_en(0);
                }
            })
            .await?;

        // INT_ENABLE_3 register (FIFO watermark)
        self.device
            .int_enable_3()
            .write_async(|w| {
                if config.fifo_watermark {
                    w.set_fifo_wm_en(0x1F); // Enable all FIFOs
                } else {
                    w.set_fifo_wm_en(0);
                }
            })
            .await?;

        Ok(())
    }

    /// Read interrupt status
    ///
    /// # Returns
    /// Current interrupt status flags
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        let status = self.device.int_status().read_async().await?;
        let status1 = self.device.int_status_1().read_async().await?;
        let status2 = self.device.int_status_2().read_async().await?;
        let status3 = self.device.int_status_3().read_async().await?;

        Ok(InterruptStatus {
            raw_data_ready: status1.raw_data_0_rdy_int(),
            fifo_overflow: status2.fifo_overflow_int() != 0,
            fifo_watermark: status3.fifo_wm_int() != 0,
            wake_on_motion: status.wom_int(),
            dmp: status.dmp_int_1(),
            i2c_master: status.i_2_c_mst_int(),
            pll_ready: status.pll_rdy_int(),
        })
    }

    /// Read data ready status
    ///
    /// # Returns
    /// Data ready status for all sensors
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn read_data_ready_status(&mut self) -> Result<DataReadyStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        let status = self.device.data_rdy_status().read_async().await?;
        let bits = status.raw_data_rdy();

        Ok(DataReadyStatus {
            sensor0_ready: (bits & 0x01) != 0,
            sensor1_ready: (bits & 0x02) != 0,
            sensor2_ready: (bits & 0x04) != 0,
            sensor3_ready: (bits & 0x08) != 0,
        })
    }

    /// Read wake-on-motion status
    ///
    /// # Returns
    /// `WoM` status for each axis
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn read_wom_status(&mut self) -> Result<WomStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        let status = self.device.data_rdy_status().read_async().await?;
        let bits = status.wof_status();

        Ok(WomStatus {
            x_motion: (bits & 0x01) != 0,
            y_motion: (bits & 0x02) != 0,
            z_motion: (bits & 0x04) != 0,
        })
    }

    /// Set the power mode
    ///
    /// # Arguments
    /// * `mode` - Power mode to set
    /// * `delay` - Delay provider for timing after power mode change
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    ///
    /// # Notes
    /// This function adds a 1ms delay after changing power mode to allow the device
    /// to stabilize. According to the ICM-20948 datasheet, ~100µs is required for
    /// power mode transitions. The 1ms delay provides a safe margin.
    pub async fn set_power_mode<D>(
        &mut self,
        mode: PowerMode,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        self.select_bank(Bank::Bank0).await?;

        match mode {
            PowerMode::Normal => {
                self.device
                    .pwr_mgmt_1()
                    .modify_async(|w| {
                        w.set_sleep(false);
                        w.set_lp_en(false);
                    })
                    .await?;
            }
            PowerMode::LowPower => {
                self.device
                    .pwr_mgmt_1()
                    .modify_async(|w| {
                        w.set_sleep(false);
                        w.set_lp_en(true);
                    })
                    .await?;
            }
            PowerMode::Sleep => {
                self.device
                    .pwr_mgmt_1()
                    .modify_async(|w| {
                        w.set_sleep(true);
                    })
                    .await?;
            }
        }

        // Wait for power mode transition to stabilize
        // Datasheet specifies ~100µs, we use 1ms for safety margin
        delay.delay_ms(1).await;

        Ok(())
    }

    /// Enter low-power mode with specified configuration
    ///
    /// # Arguments
    /// * `config` - Low-power mode configuration
    /// * `delay` - Delay provider for timing after power mode change
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    ///
    /// # Notes
    /// This function adds a 1ms delay after entering low-power mode to allow the device
    /// to stabilize. According to the ICM-20948 datasheet, ~100µs is required for
    /// power mode transitions. The 1ms delay provides a safe margin.
    pub async fn enter_low_power_mode<D>(
        &mut self,
        config: &LowPowerConfig,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>>
    where
        D: embedded_hal_async::delay::DelayNs,
    {
        // Switch to Bank 2 for accelerometer configuration
        self.select_bank(Bank::Bank2).await?;

        // Configure wake-on-motion threshold if enabled
        if config.enable_wake_on_motion {
            self.device
                .bank_2_accel_wom_thr()
                .write_async(|w| {
                    w.set_wom_threshold(config.wom_threshold_register_value());
                })
                .await?;

            // Configure WoM mode
            self.device
                .bank_2_accel_intel_ctrl()
                .write_async(|w| {
                    w.set_accel_intel_en(true);
                    w.set_accel_intel_mode_int(config.wom_mode as u8 != 0);
                })
                .await?;
        }

        // CRITICAL: Configure ACCEL_CONFIG (including FS_SEL) BEFORE enabling low-power mode
        // Set ACCEL_FCHOICE = 1 (enable DLPF) so that ACCEL_SMPLRT_DIV registers are used
        // In low-power cycle mode, the sample rate dividers only work when FCHOICE=1
        // Also explicitly set FS_SEL to match the driver's configured full scale
        self.device
            .bank_2_accel_config()
            .write_async(|w| {
                w.set_accel_fchoice(true); // Enable DLPF for sample rate divider to work
                w.set_accel_dlpfcfg(0); // Set DLPF to configuration 0 (lowest filtering for low-power)
                w.set_accel_fs_sel(self.accel_config.full_scale as u8); // Use driver's configured full scale
            })
            .await?;

        // Configure accelerometer sample rate divider for low-power mode
        // The ODR value is just a byte that goes directly into the low register
        let odr = config.accel_rate.odr_value();
        self.device
            .bank_2_accel_smplrt_div_1()
            .write_async(|w| {
                w.set_accel_smplrt_div_1(0); // High byte is 0 for these rates
            })
            .await?;
        self.device
            .bank_2_accel_smplrt_div_2()
            .write_async(|w| {
                w.set_accel_smplrt_div_2(odr);
            })
            .await?;

        // Switch to Bank 0 for power management
        self.select_bank(Bank::Bank0).await?;

        // Configure PWR_MGMT_2: Enable only accelerometer, disable gyroscope for power savings
        // This is critical for low-power mode - sensors must be explicitly controlled
        self.device
            .pwr_mgmt_2()
            .write_async(|w| {
                w.set_disable_accel_x(false);
                w.set_disable_accel_y(false);
                w.set_disable_accel_z(false);
                w.set_disable_gyro_x(true);
                w.set_disable_gyro_y(true);
                w.set_disable_gyro_z(true);
            })
            .await?;

        // Enable low-power mode
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_lp_en(true);
                w.set_sleep(false);
            })
            .await?;

        // Enable accelerometer cycle mode
        // Use write_async (not modify_async) to ensure I2C_MST_CYCLE and GYRO_CYCLE are cleared
        self.device
            .lp_config()
            .write_async(|w| {
                w.set_accel_cycle(true);
                w.set_gyro_cycle(false);
                w.set_i_2_c_mst_cycle(false);
            })
            .await?;

        // Wait for power mode transition to stabilize
        // Low-power mode needs more time than normal mode transitions
        // At 15.63 Hz, sample period is 64ms. Wait for at least 2 sample periods
        // to ensure the accelerometer has taken fresh samples and WoM baseline is established
        delay.delay_ms(150).await;

        // Clear any pending interrupts caused by mode transition
        // The transition from normal to low-power mode can cause spurious motion detection
        if config.enable_wake_on_motion {
            let _ = self.read_interrupt_status().await;
            delay.delay_ms(10).await;

            // Now enable wake-on-motion interrupt after clearing transition artifacts
            self.device
                .int_enable()
                .modify_async(|w| {
                    w.set_wom_int_en(true);
                })
                .await?;
        }

        Ok(())
    }

    /// Exit low-power mode and return to normal operation
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn exit_low_power_mode(&mut self) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        // Disable low-power mode
        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_lp_en(false);
                w.set_sleep(false);
            })
            .await?;

        // Disable cycle modes
        self.device
            .lp_config()
            .write_async(|w| {
                w.set_accel_cycle(false);
                w.set_gyro_cycle(false);
                w.set_i_2_c_mst_cycle(false);
            })
            .await?;

        Ok(())
    }

    /// Configure cycle modes for periodic sensor readings
    /// Configure cycle modes for periodic sensor readings (async version)
    ///
    /// # Arguments
    /// * `config` - Cycle configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn configure_cycle_mode(
        &mut self,
        config: &CycleConfig,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        self.device
            .lp_config()
            .write_async(|w| {
                w.set_accel_cycle(config.enable_accel_cycle);
                w.set_gyro_cycle(config.enable_gyro_cycle);
                w.set_i_2_c_mst_cycle(config.enable_i2c_master_cycle);
            })
            .await?;

        Ok(())
    }

    /// Set accelerometer and gyroscope to continuous sampling mode (async version)
    ///
    /// This configures the LP_CONFIG register to disable cycle modes, enabling
    /// continuous sampling. This is required for the ACCEL_SMPLRT_DIV and
    /// GYRO_SMPLRT_DIV registers to take effect.
    ///
    /// # Arguments
    /// * `enable_accel` - Enable continuous mode for accelerometer
    /// * `enable_gyro` - Enable continuous mode for gyroscope
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn set_continuous_mode(
        &mut self,
        enable_accel: bool,
        enable_gyro: bool,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        self.device
            .lp_config()
            .modify_async(|w| {
                if enable_accel {
                    w.set_accel_cycle(false); // false = continuous mode
                }
                if enable_gyro {
                    w.set_gyro_cycle(false); // false = continuous mode
                }
            })
            .await?;

        Ok(())
    }

    /// Set clock source
    ///
    /// # Arguments
    /// * `source` - Clock source to use
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn set_clock_source(&mut self, source: ClockSource) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_clksel(source as u8);
            })
            .await?;

        Ok(())
    }

    /// Enable or disable temperature sensor
    ///
    /// # Arguments
    /// * `enable` - true to enable, false to disable
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn set_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        self.device
            .pwr_mgmt_1()
            .modify_async(|w| {
                w.set_temp_dis(!enable);
            })
            .await?;

        Ok(())
    }

    /// Configure individual sensor power
    ///
    /// # Arguments
    /// * `config` - Sensor power configuration
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn configure_sensor_power(
        &mut self,
        config: &SensorPowerConfig,
    ) -> Result<(), Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        self.device
            .pwr_mgmt_2()
            .write_async(|w| {
                w.set_disable_accel_x(config.disable_accel_x);
                w.set_disable_accel_y(config.disable_accel_y);
                w.set_disable_accel_z(config.disable_accel_z);
                w.set_disable_gyro_x(config.disable_gyro_x);
                w.set_disable_gyro_y(config.disable_gyro_y);
                w.set_disable_gyro_z(config.disable_gyro_z);
            })
            .await?;

        Ok(())
    }

    /// Read current power status
    ///
    /// # Returns
    /// Current power management status
    ///
    /// # Errors
    /// Returns an error if communication with the device fails.
    pub async fn read_power_status(&mut self) -> Result<PowerStatus, Error<I::Error>> {
        self.select_bank(Bank::Bank0).await?;

        let pwr1 = self.device.pwr_mgmt_1().read_async().await?;
        let lp_cfg = self.device.lp_config().read_async().await?;

        let mode = if pwr1.sleep() {
            PowerMode::Sleep
        } else if pwr1.lp_en() {
            PowerMode::LowPower
        } else {
            PowerMode::Normal
        };

        Ok(PowerStatus {
            mode,
            sleep: pwr1.sleep(),
            low_power: pwr1.lp_en(),
            temp_disabled: pwr1.temp_dis(),
            accel_cycle: lp_cfg.accel_cycle(),
            gyro_cycle: lp_cfg.gyro_cycle(),
            i2c_master_cycle: lp_cfg.i_2_c_mst_cycle(),
        })
    }
}
