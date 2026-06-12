//! Async tests for ICM-20948 driver
//!
//! These tests verify the async API functionality for Priority 1 and Priority 2 operations.

#![cfg(feature = "async")]

use icm20948::{
    AccelConfig, AccelDlpf, AccelFullScale, Bank, Error, GyroConfig, GyroDlpf, GyroFullScale,
    I2cInterface, Icm20948Driver, WHO_AM_I_VALUE,
    fifo::{FifoConfig, FifoConfigAdvanced, FifoMode, FifoWatermark},
    interrupt::{InterruptConfig, InterruptPinConfig},
    power::{ClockSource, CycleConfig, LowPowerConfig, LowPowerRate, PowerMode, SensorPowerConfig},
    sensors::MagConfig,
};

#[cfg(feature = "dmp")]
use icm20948::dmp::DmpConfig;

// Mock async I2C implementation for testing
struct MockAsyncI2c {
    who_am_i_value: u8,
    bank: u8,
    fail_next: bool,
    fifo_count: u16,
    fifo_data: [u8; 512],
    mag_initialized: bool,
    i2c_slv_4_di: u8, // I2C Slave 4 data in register
    mag_ext_slv_data: [u8; 8],
    pwr_mgmt_1: u8,
}

impl MockAsyncI2c {
    fn new() -> Self {
        Self {
            who_am_i_value: WHO_AM_I_VALUE,
            bank: 0,
            fail_next: false,
            fifo_count: 0,
            fifo_data: [0; 512],
            mag_initialized: false,
            i2c_slv_4_di: 0x09, // AK09916 WHO_AM_I value
            mag_ext_slv_data: [0; 8],
            pwr_mgmt_1: 0x00,
        }
    }

    fn with_fifo_data(count: u16) -> Self {
        let mut mock = Self::new();
        mock.fifo_count = count;
        // Fill with dummy sensor data pattern (accel + gyro = 12 bytes per sample)
        for i in 0..count as usize {
            mock.fifo_data[i] = (i % 256) as u8;
        }
        mock.mag_initialized = false;
        mock.i2c_slv_4_di = 0x09;
        mock
    }

    fn with_invalid_who_am_i() -> Self {
        Self {
            who_am_i_value: 0xFF,
            bank: 0,
            fail_next: false,
            fifo_count: 0,
            fifo_data: [0; 512],
            mag_initialized: false,
            i2c_slv_4_di: 0x09,
            mag_ext_slv_data: [0; 8],
            pwr_mgmt_1: 0x00,
        }
    }

    fn with_mag_data(x: i16, y: i16, z: i16) -> Self {
        let mut mock = Self::new();
        let [x_l, x_h] = x.to_le_bytes();
        let [y_l, y_h] = y.to_le_bytes();
        let [z_l, z_h] = z.to_le_bytes();
        mock.mag_ext_slv_data = [x_l, x_h, y_l, y_h, z_l, z_h, 0x00, 0x00];
        mock
    }

    /// Create a mock with magnetometer data and a custom ST2 register value.
    ///
    /// The 8-byte layout matches the async atomic read (HXL..ST2):
    /// `[HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2]`.
    /// Use `st2 = 0x08` to simulate a magnetic sensor overflow.
    fn with_mag_data_and_st2(x: i16, y: i16, z: i16, st2: u8) -> Self {
        let mut mock = Self::new();
        let [x_l, x_h] = x.to_le_bytes();
        let [y_l, y_h] = y.to_le_bytes();
        let [z_l, z_h] = z.to_le_bytes();
        mock.mag_ext_slv_data = [x_l, x_h, y_l, y_h, z_l, z_h, 0x00, st2];
        mock
    }
}

// Mock error type
#[derive(Debug)]
struct MockError;

impl embedded_hal::i2c::Error for MockError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

type TestInterface = I2cInterface<MockAsyncI2c>;

type TestDriver = Icm20948Driver<TestInterface>;

async fn new_verified(interface: TestInterface) -> TestDriver {
    let mut imu = Icm20948Driver::new(interface);
    imu.verify_who_am_i()
        .await
        .expect("Failed to verify WHO_AM_I");
    imu
}

impl embedded_hal_async::i2c::ErrorType for MockAsyncI2c {
    type Error = MockError;
}

impl embedded_hal_async::i2c::I2c for MockAsyncI2c {
    async fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        if self.fail_next {
            self.fail_next = false;
            return Err(MockError);
        }
        Ok(())
    }

    async fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
        if self.fail_next {
            self.fail_next = false;
            return Err(MockError);
        }
        Ok(())
    }

    async fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
        if self.fail_next {
            self.fail_next = false;
            return Err(MockError);
        }

        // Handle register writes
        if write.len() >= 2 {
            let reg = write[0];
            let value = write[1];
            match (self.bank, reg) {
                (_, 0x7F) => {
                    // REG_BANK_SEL register - bank switching
                    self.bank = value >> 4;
                }
                (0, 0x06) => {
                    // PWR_MGMT_1 register
                    self.pwr_mgmt_1 = value;
                    // Simulate reset bit auto-clear
                    if (value & 0x80) != 0 {
                        self.pwr_mgmt_1 &= !0x80;
                    }
                }
                (3, 0x15) => {
                    // BANK_3_I2C_SLV_4_CTRL - I2C Slave 4 control
                    // When enabled, simulate magnetometer WHO_AM_I read
                    if (value & 0x80) != 0 {
                        self.mag_initialized = true;
                    }
                }
                _ => {
                    // Other registers - just accept the write
                }
            }
        }

        Ok(())
    }

    async fn write_read(
        &mut self,
        _address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        if self.fail_next {
            self.fail_next = false;
            return Err(MockError);
        }

        // Simulate register reads
        if !write.is_empty() {
            let reg = write[0];
            match reg {
                0x00 => {
                    // WHO_AM_I register
                    if !read.is_empty() {
                        read[0] = self.who_am_i_value;
                    }
                }
                0x06 => {
                    // PWR_MGMT_1 register
                    if self.bank == 0 && !read.is_empty() {
                        read[0] = self.pwr_mgmt_1;
                    }
                }
                0x70 => {
                    // FifoCount (16-bit): new impl reads both bytes in one transaction
                    // (big-endian: high byte at read[0], low byte at read[1]).
                    if read.len() >= 2 {
                        read[0] = ((self.fifo_count >> 8) & 0xFF) as u8;
                        read[1] = (self.fifo_count & 0xFF) as u8;
                    } else if !read.is_empty() {
                        read[0] = ((self.fifo_count >> 8) & 0xFF) as u8;
                    }
                }
                0x71 => {
                    // FIFO_COUNTL register
                    if !read.is_empty() {
                        read[0] = (self.fifo_count & 0xFF) as u8;
                    }
                }
                0x72 => {
                    // FIFO_R_W register
                    if !read.is_empty() && self.fifo_count > 0 {
                        let idx = (512 - self.fifo_count) as usize;
                        read[0] = self.fifo_data[idx];
                        self.fifo_count = self.fifo_count.saturating_sub(1);
                    }
                }
                0x7F => {
                    // REG_BANK_SEL register
                    if write.len() > 1 {
                        // Write operation
                        self.bank = write[1] >> 4;
                    } else if !read.is_empty() {
                        // Read operation
                        read[0] = self.bank << 4;
                    }
                }
                0x17 => {
                    // BANK_3_I2C_SLV_4_DI (register 0x17) - magnetometer read data
                    if self.bank == 3 && !read.is_empty() {
                        read[0] = self.i2c_slv_4_di;
                    } else {
                        read.fill(0);
                    }
                }
                0x3B..=0x52 => {
                    // EXT_SLV_SENS_DATA_00 through EXT_SLV_SENS_DATA_23
                    // Magnetometer data registers (Bank 0)
                    // Fill all requested bytes sequentially (supports multi-byte atomic reads)
                    if self.bank == 0 {
                        let base = (reg - 0x3B) as usize;
                        for (i, byte) in read.iter_mut().enumerate() {
                            *byte = self.mag_ext_slv_data.get(base + i).copied().unwrap_or(0);
                        }
                    } else {
                        read.fill(0);
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

// Mock async delay implementation
struct MockDelay;

impl embedded_hal_async::delay::DelayNs for MockDelay {
    async fn delay_ns(&mut self, _ns: u32) {
        // No actual delay in tests
    }

    async fn delay_us(&mut self, _us: u32) {
        // No actual delay in tests
    }

    async fn delay_ms(&mut self, _ms: u32) {
        // No actual delay in tests
    }
}

// Helper to create a test runtime for async tests
fn block_on<F: core::future::Future>(f: F) -> F::Output {
    // Simple blocking executor for tests
    futures::executor::block_on(f)
}

#[test]
fn test_verify_who_am_i_success() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = Icm20948Driver::new(interface);
        let result = imu.verify_who_am_i().await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_verify_who_am_i_invalid() {
    block_on(async {
        let i2c = MockAsyncI2c::with_invalid_who_am_i();
        let interface = I2cInterface::default(i2c);

        let mut imu = Icm20948Driver::new(interface);
        let result = imu.verify_who_am_i().await;
        assert!(result.is_err());

        if let Err(Error::InvalidDevice(value)) = result {
            assert_eq!(value, 0xFF);
        } else {
            panic!("Expected InvalidDevice error");
        }
    });
}

#[test]
fn test_try_new_success() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let result = Icm20948Driver::try_new(interface).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_try_new_invalid_who_am_i() {
    block_on(async {
        let i2c = MockAsyncI2c::with_invalid_who_am_i();
        let interface = I2cInterface::default(i2c);

        let result = Icm20948Driver::try_new(interface).await;
        assert!(result.is_err());

        if let Err(Error::InvalidDevice(value)) = result {
            assert_eq!(value, 0xFF);
        } else {
            panic!("Expected InvalidDevice error");
        }
    });
}

#[test]
fn test_init() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;
        let result = imu.init(&mut delay).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_select_bank() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Test switching between banks
        assert!(imu.select_bank(Bank::Bank0).await.is_ok());
        assert!(imu.select_bank(Bank::Bank1).await.is_ok());
        assert!(imu.select_bank(Bank::Bank2).await.is_ok());
        assert!(imu.select_bank(Bank::Bank3).await.is_ok());

        // Switching to same bank should be a no-op
        assert!(imu.select_bank(Bank::Bank3).await.is_ok());
    });
}

#[test]
fn test_read_who_am_i() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let who_am_i = imu.read_who_am_i().await.expect("Failed to read WHO_AM_I");
        assert_eq!(who_am_i, WHO_AM_I_VALUE);
    });
}

#[test]
fn test_read_temperature() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Read raw temperature
        let result = imu.read_temperature().await;
        assert!(result.is_ok());

        // Read temperature in Celsius
        let result = imu.read_temperature_celsius().await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_configure_accelerometer() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let config = AccelConfig {
            full_scale: AccelFullScale::G2,
            dlpf: AccelDlpf::Hz246,
            dlpf_enable: true,
            sample_rate_div: 10,
        };

        let result = imu.configure_accelerometer(config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_configure_gyroscope() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let config = GyroConfig {
            full_scale: GyroFullScale::Dps250,
            dlpf: GyroDlpf::Hz197,
            dlpf_enable: true,
            sample_rate_div: 10,
        };

        let result = imu.configure_gyroscope(config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_read_accelerometer() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Read raw data
        let result = imu.read_accelerometer_raw().await;
        assert!(result.is_ok());

        // Read calibrated data
        let result = imu.read_accelerometer().await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_read_gyroscope() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Read raw data
        let result = imu.read_gyroscope_raw().await;
        assert!(result.is_ok());

        // Read in degrees per second
        let result = imu.read_gyroscope().await;
        assert!(result.is_ok());

        // Read in radians per second
        let result = imu.read_gyroscope_radians().await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_set_sleep() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Enter sleep mode
        assert!(imu.set_sleep(true).await.is_ok());

        // Exit sleep mode
        assert!(imu.set_sleep(false).await.is_ok());
    });
}

#[test]
fn test_set_dmp_enable() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        #[cfg(feature = "dmp")]
        {
            // Enable without firmware should fail
            assert!(matches!(
                imu.set_dmp_enable(true).await,
                Err(Error::DmpFirmwareNotLoaded)
            ));

            // Load firmware
            let mut delay = MockDelay;
            imu.dmp_load_firmware(&mut delay)
                .await
                .expect("Firmware load failed");

            // Enable without configuration should fail
            assert!(matches!(
                imu.set_dmp_enable(true).await,
                Err(Error::DmpNotConfigured)
            ));

            // Configure (6-axis, no magnetometer required)
            let config = DmpConfig::six_axis().with_sample_rate(100);

            imu.dmp_configure(&config)
                .await
                .expect("Failed to configure DMP");

            // Enable/disable should now succeed
            assert!(imu.set_dmp_enable(true).await.is_ok());
            assert!(imu.set_dmp_enable(false).await.is_ok());
        }

        #[cfg(not(feature = "dmp"))]
        {
            // Without DMP feature, set_dmp_enable remains a simple register toggle
            assert!(imu.set_dmp_enable(true).await.is_ok());
            assert!(imu.set_dmp_enable(false).await.is_ok());
        }
    });
}

#[test]
fn test_set_fifo_enable() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Enable FIFO
        assert!(imu.set_fifo_enable(true).await.is_ok());

        // Disable FIFO
        assert!(imu.set_fifo_enable(false).await.is_ok());
    });
}

// ==================== PRIORITY 2: FIFO ASYNC TESTS ====================

#[test]
fn test_fifo_configure() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let config = FifoConfig {
            enable_accel: true,
            enable_gyro: true,
            enable_temp: false,
            enable_mag: false,
            mode: FifoMode::Stream,
        };

        let result = imu.fifo_configure(&config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_fifo_configure_advanced() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let config = FifoConfigAdvanced {
            enable_accel: true,
            enable_gyro_x: true,
            enable_gyro_y: true,
            enable_gyro_z: false,
            enable_temp: true,
            enable_slv0: false,
            enable_slv1: false,
            enable_slv2: false,
            enable_slv3: false,
            mode: FifoMode::Snapshot,
        };

        let result = imu.fifo_configure_advanced(&config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_fifo_enable() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Enable FIFO
        assert!(imu.fifo_enable(true).await.is_ok());

        // Disable FIFO
        assert!(imu.fifo_enable(false).await.is_ok());
    });
}

#[test]
fn test_fifo_reset() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.fifo_reset().await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_fifo_count() {
    block_on(async {
        let i2c = MockAsyncI2c::with_fifo_data(120);
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let count = imu.fifo_count().await.expect("Failed to read FIFO count");
        assert_eq!(count, 120);
    });
}

#[test]
fn test_fifo_read() {
    block_on(async {
        let i2c = MockAsyncI2c::with_fifo_data(24); // 2 samples worth
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut buffer = [0u8; 24];
        let bytes_read = imu
            .fifo_read(&mut buffer)
            .await
            .expect("Failed to read FIFO");

        assert!(bytes_read > 0);
        assert!(bytes_read <= 24);
    });
}

#[test]
fn test_fifo_overflow_status() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let status = imu
            .fifo_overflow_status()
            .await
            .expect("Failed to read overflow status");

        // With mock data, no overflow expected
        assert!(!status.any_overflow());
    });
}

#[test]
fn test_fifo_watermark_construction() {
    block_on(async {
        // Test watermark struct construction (no I/O)
        let watermark = FifoWatermark::new(256).expect("Invalid watermark");
        assert_eq!(watermark.threshold, 256);

        // Test from_samples helper
        let watermark = FifoWatermark::from_samples(10, 12).expect("Invalid watermark");
        assert_eq!(watermark.threshold, 120);
    });
}

#[test]
fn test_fifo_parse() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let imu = new_verified(interface).await;

        let config = FifoConfigAdvanced {
            enable_accel: true,
            enable_gyro_x: true,
            enable_gyro_y: true,
            enable_gyro_z: true,
            enable_temp: false,
            enable_slv0: false,
            enable_slv1: false,
            enable_slv2: false,
            enable_slv3: false,
            mode: FifoMode::Stream,
        };

        // Create dummy FIFO data (12 bytes = 1 sample with accel + gyro)
        let data = [
            0x00, 0x01, // Accel X
            0x00, 0x02, // Accel Y
            0x00, 0x03, // Accel Z
            0x00, 0x04, // Gyro X
            0x00, 0x05, // Gyro Y
            0x00, 0x06, // Gyro Z
        ];

        let result = imu.fifo_parse(&data, &config);
        assert!(result.is_ok());

        let records = result.unwrap();
        assert_eq!(records.len(), 1);

        let record = &records[0];
        assert!(record.accel.is_some());
        assert!(record.gyro.is_some());
        assert!(record.temp.is_none());
    });
}

#[test]
fn test_fifo_full_workflow() {
    block_on(async {
        let i2c = MockAsyncI2c::with_fifo_data(120); // 10 samples
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Configure FIFO
        let config = FifoConfig {
            enable_accel: true,
            enable_gyro: true,
            enable_temp: false,
            enable_mag: false,
            mode: FifoMode::Stream,
        };

        imu.fifo_configure(&config)
            .await
            .expect("Failed to configure FIFO");

        // Reset FIFO
        imu.fifo_reset().await.expect("Failed to reset FIFO");

        // Enable FIFO
        imu.fifo_enable(true).await.expect("Failed to enable FIFO");

        // Check count
        let count = imu.fifo_count().await.expect("Failed to read count");
        assert_eq!(count, 120);

        // Read data
        let mut buffer = [0u8; 120];
        let bytes_read = imu.fifo_read(&mut buffer).await.expect("Failed to read");
        assert!(bytes_read > 0);

        // Disable FIFO
        imu.fifo_enable(false)
            .await
            .expect("Failed to disable FIFO");
    });
}

// ==================== PRIORITY 3: DMP ASYNC TESTS ====================

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_reset() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.dmp_reset().await;
        assert!(result.is_ok());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_enable() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Enable without firmware should fail
        assert!(matches!(
            imu.dmp_enable(true).await,
            Err(Error::DmpFirmwareNotLoaded)
        ));

        // Load firmware
        let mut delay = MockDelay;
        imu.dmp_load_firmware(&mut delay)
            .await
            .expect("Firmware load failed");

        // Enable without configuration should fail
        assert!(matches!(
            imu.dmp_enable(true).await,
            Err(Error::DmpNotConfigured)
        ));

        // Configure (6-axis, no magnetometer required)
        let config = DmpConfig::six_axis().with_sample_rate(100);

        imu.dmp_configure(&config)
            .await
            .expect("Failed to configure DMP");

        // Enable/disable should now succeed
        assert!(imu.dmp_enable(true).await.is_ok());
        assert!(imu.dmp_enable(false).await.is_ok());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_load_firmware() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;
        let result = imu.dmp_load_firmware(&mut delay).await;
        assert!(result.is_ok());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_init() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;
        let result = imu.dmp_init(&mut delay).await;
        assert!(result.is_ok());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_configure() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;
        imu.dmp_load_firmware(&mut delay)
            .await
            .expect("Firmware load failed");

        let config = DmpConfig::six_axis().with_sample_rate(100);

        let result = imu.dmp_configure(&config).await;
        assert!(result.is_ok());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_configure_requires_mag_init_for_mag_outputs_async() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;
        imu.dmp_load_firmware(&mut delay)
            .await
            .expect("Firmware load failed");

        let config = DmpConfig::nine_axis().with_sample_rate(100);

        let result = imu.dmp_configure(&config).await;
        assert!(matches!(result, Err(Error::MagnetometerNotInitialized)));
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_read_fifo_count() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.read_fifo_count().await;
        assert!(result.is_ok());

        let count = result.unwrap();
        assert!(count < 4096); // FIFO max size
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_read_fifo_raw() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut buffer = [0u8; 64];
        let result = imu.read_fifo_raw(&mut buffer).await;
        assert!(result.is_ok());

        let bytes_read = result.unwrap();
        assert_eq!(bytes_read, 64);
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_reset_fifo() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.reset_fifo().await;
        assert!(result.is_ok());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_read_fifo_empty() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.dmp_read_fifo().await;
        assert!(result.is_ok());

        // Should return None when FIFO is empty (count < 2)
        assert!(result.unwrap().is_none());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_read_quaternion_empty() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.dmp_read_quaternion().await;
        assert!(result.is_ok());

        // Should return None when FIFO is empty
        assert!(result.unwrap().is_none());
    });
}

#[test]
#[cfg(feature = "dmp")]
fn test_dmp_full_workflow() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;

        // Full DMP initialization workflow
        imu.dmp_init(&mut delay)
            .await
            .expect("Failed to initialize DMP");

        // Use 6-axis (no magnetometer required) so the mock can complete the full workflow.
        // 9-axis magnetometer-init requirements are covered by
        // `test_dmp_configure_requires_mag_init_for_mag_outputs_async`.
        let config = DmpConfig::six_axis().with_sample_rate(100);

        imu.dmp_configure(&config)
            .await
            .expect("Failed to configure DMP");

        // Enable FIFO
        imu.fifo_enable(true).await.expect("Failed to enable FIFO");

        // Enable DMP
        imu.dmp_enable(true).await.expect("Failed to enable DMP");

        // Try to read (will be empty in mock)
        let result = imu.dmp_read_quaternion().await;
        assert!(result.is_ok());

        // Disable DMP
        imu.dmp_enable(false).await.expect("Failed to disable DMP");
    });
}

// ==================== PRIORITY 4: MAGNETOMETER TESTS ====================

#[test]
fn test_magnetometer_init() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;

        let mag_config = MagConfig::default();
        let result = imu.init_magnetometer(mag_config, &mut delay).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_magnetometer_read() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Read without initialization should fail
        let result = imu.read_magnetometer().await;
        assert!(result.is_err());
    });
}

#[test]
fn test_magnetometer_read_raw() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Read without initialization should fail
        let result = imu.read_magnetometer_raw().await;
        assert!(result.is_err());
    });
}

#[test]
fn test_magnetometer_read_raw_parses_hxl_layout() {
    block_on(async {
        const EXPECTED_X: i16 = 0x1234;
        const EXPECTED_Y: i16 = -0x567;
        const EXPECTED_Z: i16 = 0x7A55;

        let i2c = MockAsyncI2c::with_mag_data(EXPECTED_X, EXPECTED_Y, EXPECTED_Z);
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;
        let mut delay = MockDelay;

        imu.init_magnetometer(MagConfig::default(), &mut delay)
            .await
            .unwrap();

        let result = imu.read_magnetometer_raw().await.unwrap();
        assert_eq!(result, (EXPECTED_X, EXPECTED_Y, EXPECTED_Z));
    });
}

#[test]
fn test_magnetometer_read_raw_rejects_st2_overflow() {
    block_on(async {
        const EXPECTED_X: i16 = 0x1234;
        const EXPECTED_Y: i16 = -0x567;
        const EXPECTED_Z: i16 = 0x7A55;

        // ST2 at index 7 with overflow bit (0x08) set
        let i2c = MockAsyncI2c::with_mag_data_and_st2(EXPECTED_X, EXPECTED_Y, EXPECTED_Z, 0x08);
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;
        let mut delay = MockDelay;

        imu.init_magnetometer(MagConfig::default(), &mut delay)
            .await
            .unwrap();

        let result = imu.read_magnetometer_raw().await;
        assert!(
            matches!(result, Err(Error::Magnetometer)),
            "ST2 overflow should be rejected as Error::Magnetometer, got {:?}",
            result
        );
    });
}

#[test]
fn test_magnetometer_read_rejects_st2_overflow() {
    block_on(async {
        const EXPECTED_X: i16 = 0x1234;
        const EXPECTED_Y: i16 = -0x567;
        const EXPECTED_Z: i16 = 0x7A55;

        // ST2 at index 7 with overflow bit (0x08) set
        let i2c = MockAsyncI2c::with_mag_data_and_st2(EXPECTED_X, EXPECTED_Y, EXPECTED_Z, 0x08);
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;
        let mut delay = MockDelay;

        imu.init_magnetometer(MagConfig::default(), &mut delay)
            .await
            .unwrap();

        let result = imu.read_magnetometer().await;
        assert!(
            matches!(result, Err(Error::Magnetometer)),
            "ST2 overflow should be rejected as Error::Magnetometer, got {:?}",
            result
        );
    });
}

#[test]
fn test_magnetometer_heading() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Read without initialization should fail
        let result = imu.read_magnetometer_heading().await;
        assert!(result.is_err());
    });
}

#[test]
fn test_magnetometer_heading_compensated() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Read without initialization should fail
        let result = imu
            .read_magnetometer_heading_compensated(0.0, 0.0, 1.0)
            .await;
        assert!(result.is_err());
    });
}

#[test]
fn test_magnetometer_data_ready() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Check data ready status - returns false when not initialized
        let result = imu.magnetometer_data_ready().await;
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), false);
    });
}

#[test]
fn test_accelerometer_self_test_async() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Initialize the device
        let mut delay = MockDelay;
        imu.init(&mut delay)
            .await
            .expect("Failed to initialize device");

        // Configure accelerometer
        let config = AccelConfig {
            full_scale: AccelFullScale::G2,
            dlpf: AccelDlpf::Hz246,
            dlpf_enable: true,
            sample_rate_div: 10,
        };
        imu.configure_accelerometer(config)
            .await
            .expect("Failed to configure accelerometer");

        // Run self-test
        let result = imu.accelerometer_self_test(&mut delay).await;

        // Mock interface may not fully simulate self-test behavior,
        // but we verify the function can be called without panicking
        if let Ok(test_result) = result {
            println!("Accelerometer self-test result: {:?}", test_result);
        }
    });
}

#[test]
fn test_gyroscope_self_test_async() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Initialize the device
        let mut delay = MockDelay;
        imu.init(&mut delay)
            .await
            .expect("Failed to initialize device");

        // Configure gyroscope
        let config = GyroConfig {
            full_scale: GyroFullScale::Dps250,
            dlpf: GyroDlpf::Hz197,
            dlpf_enable: true,
            sample_rate_div: 10,
        };
        imu.configure_gyroscope(config)
            .await
            .expect("Failed to configure gyroscope");

        // Run self-test
        let result = imu.gyroscope_self_test(&mut delay).await;

        // Mock interface may not fully simulate self-test behavior,
        // but we verify the function can be called without panicking
        if let Ok(test_result) = result {
            println!("Gyroscope self-test result: {:?}", test_result);
        }
    });
}

#[test]
fn test_magnetometer_self_test() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;

        // Self-test without initialization should fail
        let result = imu.magnetometer_self_test(&mut delay).await;
        assert!(result.is_err());
    });
}

#[test]
fn test_self_test_restores_sensor_state_async() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Initialize the device
        let mut delay = MockDelay;
        imu.init(&mut delay)
            .await
            .expect("Failed to initialize device");

        // Configure accelerometer
        let config = AccelConfig {
            full_scale: AccelFullScale::G2,
            dlpf: AccelDlpf::Hz246,
            dlpf_enable: true,
            sample_rate_div: 10,
        };
        imu.configure_accelerometer(config)
            .await
            .expect("Failed to configure accelerometer");

        // Run self-test
        let _ = imu.accelerometer_self_test(&mut delay).await;

        // Verify device is still operational after self-test
        let result = imu.read_accelerometer().await;
        assert!(
            result.is_ok(),
            "Device should be operational after self-test"
        );
    });
}

#[test]
fn test_multiple_self_tests_sequence_async() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Initialize the device
        let mut delay = MockDelay;
        imu.init(&mut delay)
            .await
            .expect("Failed to initialize device");

        // Configure sensors
        let accel_config = AccelConfig {
            full_scale: AccelFullScale::G2,
            dlpf: AccelDlpf::Hz246,
            dlpf_enable: true,
            sample_rate_div: 10,
        };
        imu.configure_accelerometer(accel_config)
            .await
            .expect("Failed to configure accelerometer");

        let gyro_config = GyroConfig {
            full_scale: GyroFullScale::Dps250,
            dlpf: GyroDlpf::Hz197,
            dlpf_enable: true,
            sample_rate_div: 10,
        };
        imu.configure_gyroscope(gyro_config)
            .await
            .expect("Failed to configure gyroscope");

        // Run both self-tests in sequence
        let _ = imu.accelerometer_self_test(&mut delay).await;
        let _ = imu.gyroscope_self_test(&mut delay).await;

        // Verify both sensors are still operational
        assert!(imu.read_accelerometer().await.is_ok());
        assert!(imu.read_gyroscope().await.is_ok());
    });
}

// ==================== PRIORITY 5: INTERRUPT TESTS ====================

#[test]
fn test_configure_interrupt_pin() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let pin_config = InterruptPinConfig::i2c_default();
        let result = imu.configure_interrupt_pin(&pin_config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_configure_interrupts() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let int_config = InterruptConfig::data_ready_only();
        let result = imu.configure_interrupts(&int_config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_read_interrupt_status() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.read_interrupt_status().await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_read_data_ready_status() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.read_data_ready_status().await;
        assert!(result.is_ok());
    });
}

// ==================== PRIORITY 5: POWER MANAGEMENT TESTS ====================

#[test]
fn test_set_power_mode() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let mut delay = MockDelay;

        // Test normal mode
        let result = imu.set_power_mode(PowerMode::Normal, &mut delay).await;
        assert!(result.is_ok());

        // Test low-power mode
        let result = imu.set_power_mode(PowerMode::LowPower, &mut delay).await;
        assert!(result.is_ok());

        // Test sleep mode
        let result = imu.set_power_mode(PowerMode::Sleep, &mut delay).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_enter_low_power_mode() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let lp_config = LowPowerConfig {
            accel_rate: LowPowerRate::Hz31_25,
            enable_wake_on_motion: true,
            wom_threshold: 20,
            wom_mode: icm20948::power::WomMode::CompareCurrentSample,
        };

        let mut delay = MockDelay;
        let result = imu.enter_low_power_mode(&lp_config, &mut delay).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_exit_low_power_mode() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.exit_low_power_mode().await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_configure_cycle_mode() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let cycle_config = CycleConfig {
            enable_accel_cycle: true,
            enable_gyro_cycle: false,
            enable_i2c_master_cycle: false,
        };

        let result = imu.configure_cycle_mode(&cycle_config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_set_clock_source() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.set_clock_source(ClockSource::AutoSelect).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_set_temperature_sensor() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        // Enable temperature sensor
        let result = imu.set_temperature_sensor(true).await;
        assert!(result.is_ok());

        // Disable temperature sensor
        let result = imu.set_temperature_sensor(false).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_configure_sensor_power() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let power_config = SensorPowerConfig::all_enabled();
        let result = imu.configure_sensor_power(&power_config).await;
        assert!(result.is_ok());
    });
}

#[test]
fn test_read_power_status() {
    block_on(async {
        let i2c = MockAsyncI2c::new();
        let interface = I2cInterface::default(i2c);

        let mut imu = new_verified(interface).await;

        let result = imu.read_power_status().await;
        assert!(result.is_ok());
    });
}
