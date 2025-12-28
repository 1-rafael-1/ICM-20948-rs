//! Bus interface implementations for the ICM-20948
//!
//! This module provides implementations of the `device-driver` traits for
//! I2C and SPI communication with the ICM-20948.

use crate::I2C_ADDRESS_AD0_LOW;

use crate::Error;
use device_driver::RegisterInterface;

/// I2C interface for the ICM-20948
pub struct I2cInterface<I2C> {
    i2c: I2C,
    #[allow(dead_code)]
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Create a new I2C interface with the default address (0x68, AD0 pin LOW)
    ///
    /// This is the most common configuration where the AD0 pin is pulled low
    /// or left floating (has internal pull-down on most breakout boards).
    ///
    /// # Arguments
    /// * `i2c` - The I2C peripheral
    ///
    /// # Example
    /// ```ignore
    /// let interface = I2cInterface::default(i2c);
    /// let mut imu = Icm20948Driver::new(interface)?;
    /// ```
    pub const fn default(i2c: I2C) -> Self {
        Self {
            i2c,
            address: I2C_ADDRESS_AD0_LOW,
        }
    }

    /// Create a new I2C interface with the alternative address (0x69, AD0 pin HIGH)
    ///
    /// Use this when the AD0 pin is explicitly pulled high to VDD.
    ///
    /// # Arguments
    /// * `i2c` - The I2C peripheral
    ///
    /// # Example
    /// ```ignore
    /// let interface = I2cInterface::alternative(i2c);
    /// let mut imu = Icm20948Driver::new(interface)?;
    /// ```
    pub const fn alternative(i2c: I2C) -> Self {
        Self {
            i2c,
            address: crate::I2C_ADDRESS_AD0_HIGH,
        }
    }

    /// Create a new I2C interface with a custom device address
    ///
    /// This allows specifying any I2C address for advanced use cases.
    /// For standard ICM-20948 configurations, prefer [`default()`](Self::default)
    /// or [`alternative()`](Self::alternative).
    ///
    /// # Arguments
    /// * `i2c` - The I2C peripheral
    /// * `address` - The I2C device address
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Consume the interface and return the I2C peripheral
    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C, E> RegisterInterface for I2cInterface<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    type Error = E;
    type AddressType = u8;

    fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        read_data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in read_data.len() for I2C
        self.i2c.write_read(self.address, &[address], read_data)
    }

    fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        write_data: &[u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in write_data.len() for I2C
        // Create a buffer with address + data
        let mut buffer = [0u8; 33]; // Max: 1 address + 32 data bytes
        buffer[0] = address;
        let len = write_data.len().min(32);
        buffer[1..=len].copy_from_slice(&write_data[..len]);

        self.i2c.write(self.address, &buffer[..=len])
    }
}

#[cfg(feature = "async")]
impl<I2C, E> device_driver::AsyncRegisterInterface for I2cInterface<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    type Error = E;
    type AddressType = u8;

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        read_data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in read_data.len() for I2C
        self.i2c
            .write_read(self.address, &[address], read_data)
            .await
    }

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        write_data: &[u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in write_data.len() for I2C
        // Create a buffer with address + data
        let mut buffer = [0u8; 33]; // Max: 1 address + 32 data bytes
        buffer[0] = address;
        let len = write_data.len().min(32);
        buffer[1..=len].copy_from_slice(&write_data[..len]);

        self.i2c.write(self.address, &buffer[..=len]).await
    }
}

/// SPI interface for the ICM-20948
///
/// # Note on Chip Select
///
/// This interface uses the `SpiDevice` trait from `embedded-hal`, which manages
/// the chip select (CS) pin automatically. The CS pin is handled internally by
/// the SPI device implementation you provide, so you don't need to pass it separately.
///
/// If using `embedded-hal-bus`, you would typically create an `SpiDevice` like:
/// ```ignore
/// let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi_bus, cs_pin, delay);
/// let interface = SpiInterface::new(spi_device);
/// ```
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Create a new SPI interface with the given SPI device
    ///
    /// The SPI device should already include chip select management via the
    /// `SpiDevice` trait (e.g., using `embedded_hal_bus::spi::ExclusiveDevice`).
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Consume the interface and return the SPI device
    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI, E> RegisterInterface for SpiInterface<SPI>
where
    SPI: embedded_hal::spi::SpiDevice<Error = E>,
{
    type Error = Error<E>;
    type AddressType = u8;

    fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        read_data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in read_data.len() for SPI
        // For SPI reads, set MSB to 1
        let read_address = address | 0x80;

        let mut operations = [
            embedded_hal::spi::Operation::Write(&[read_address]),
            embedded_hal::spi::Operation::Read(read_data),
        ];

        self.spi.transaction(&mut operations).map_err(Error::Bus)
    }

    fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        write_data: &[u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in write_data.len() for SPI
        // For SPI writes, MSB should be 0 (clear it just in case)
        let write_address = address & 0x7F;

        // Create buffer with address + data
        let mut buffer = [0u8; 33];
        buffer[0] = write_address;
        let len = write_data.len().min(32);
        buffer[1..=len].copy_from_slice(&write_data[..len]);

        self.spi.write(&buffer[..=len]).map_err(Error::Bus)
    }
}

#[cfg(feature = "async")]
impl<SPI, E> device_driver::AsyncRegisterInterface for SpiInterface<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice<Error = E>,
{
    type Error = Error<E>;
    type AddressType = u8;

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        read_data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in read_data.len() for SPI
        // For SPI reads, set MSB to 1
        let read_address = address | 0x80;

        let mut operations = [
            embedded_hal_async::spi::Operation::Write(&[read_address]),
            embedded_hal_async::spi::Operation::Read(read_data),
        ];

        self.spi
            .transaction(&mut operations)
            .await
            .map_err(Error::Bus)
    }

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        write_data: &[u8],
    ) -> Result<(), Self::Error> {
        let _ = size_bits; // Size is implicit in write_data.len() for SPI
        // For SPI writes, MSB should be 0 (clear it just in case)
        let write_address = address & 0x7F;

        // Create buffer with address + data
        let mut buffer = [0u8; 33];
        buffer[0] = write_address;
        let len = write_data.len().min(32);
        buffer[1..=len].copy_from_slice(&write_data[..len]);

        self.spi.write(&buffer[..=len]).await.map_err(Error::Bus)
    }
}
