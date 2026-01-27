#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

pub mod device;
pub mod interface;
pub mod registers;
pub mod sensors;

pub mod fifo;
pub mod interrupt;
pub mod power;

// DMP support (feature-gated)
#[cfg(feature = "dmp")]
pub mod dmp;

// Re-export main types
pub use device::{AccelData, GyroData, Icm20948Driver, MagData};
pub use interface::{I2cInterface, SpiInterface};
pub use sensors::{
    AccelCalibration, AccelConfig, AccelDataG, AccelDlpf, AccelFullScale, GyroCalibration,
    GyroConfig, GyroDataDps, GyroDataRps, GyroDlpf, GyroFullScale, MagCalibration, MagConfig,
    MagDataUT, MagMode,
};

pub use fifo::{
    FIFO_SIZE, FifoConfig, FifoConfigAdvanced, FifoMode, FifoOverflowStatus, FifoRecord,
    FifoWatermark,
};
pub use interrupt::{
    DataReadyStatus, FsyncConfig, InterruptConfig, InterruptPinConfig, InterruptStatus, WomStatus,
};
pub use power::{
    ClockSource, CycleConfig, LowPowerConfig, LowPowerRate, PowerMode, PowerStatus,
    SensorPowerConfig, WomMode,
};

/// ICM-20948 I2C address when AD0 pin is low (default: 0x68)
///
/// This is the most common configuration. The AD0 pin is typically pulled low
/// or left floating (has internal pull-down). Use [`I2cInterface::default()`]
/// for this configuration.
pub const I2C_ADDRESS_AD0_LOW: u8 = 0x68;

/// ICM-20948 I2C address when AD0 pin is high (alternative: 0x69)
///
/// Use this address when the AD0 pin is explicitly pulled high to VDD.
/// Use [`I2cInterface::alternative()`] for this configuration.
pub const I2C_ADDRESS_AD0_HIGH: u8 = 0x69;

/// Expected value of `WHO_AM_I` register
pub const WHO_AM_I_VALUE: u8 = 0xEA;

/// Register bank identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bank {
    /// Bank 0 - Primary configuration and data registers
    Bank0 = 0,
    /// Bank 1 - Self-test and advanced features
    Bank1 = 1,
    /// Bank 2 - Gyro and accelerometer configuration
    Bank2 = 2,
    /// Bank 3 - I2C master configuration
    Bank3 = 3,
}

/// Driver errors
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// Communication error with the device
    Bus(E),
    /// Invalid `WHO_AM_I` register value (contains the actual value read)
    InvalidDevice(u8),
    /// Invalid configuration parameter
    InvalidConfig,
    /// Magnetometer error
    Magnetometer,
    /// Device is moving during calibration (variance exceeds threshold)
    DeviceMoving,
    /// Calibration overflow (averaged samples exceed i16 range)
    CalibrationOverflow,
    /// FIFO buffer overflow - more records than can fit in output vector (max 64)
    FifoOverflow,
    /// Initialization timed out (device did not reset or wake as expected)
    InitializationTimeout,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}
