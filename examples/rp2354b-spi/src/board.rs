//! Shared board support for the RP2354B ICM-20948 SPI examples.
//!
//! The robot board wires the ICM-20948 to SPI0 behind a level shifter. Every
//! example in this workspace brings the IMU up through [`init_imu_or_idle`],
//! which wraps [`init_imu`], so the pin map, DMA channels, SPI mode and clock
//! can only change in one place.
//!
//! Default bus: SPI0, mode 0, 7 MHz. For signal-integrity triage on the
//! level-shifted bus, pass [`SPI_CONTROL_HZ`] instead of [`SPI_DEFAULT_HZ`];
//! that single argument is the only change required.

use defmt::{error, info};
use embassy_rp::{
    bind_interrupts, dma,
    gpio::{Level, Output},
    peripherals::{DMA_CH1, DMA_CH2, SPI0},
    spi::{Async, Config as SpiConfig, Phase, Polarity, Spi},
    Peripherals,
};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use icm20948::{Icm20948Driver, SpiInterface};

/// SPI0 clock (SCK) pin on the robot board.
pub const SPI0_SCK_PIN: u8 = 18;
/// SPI0 MOSI (data to the IMU) pin on the robot board.
pub const SPI0_MOSI_PIN: u8 = 19;
/// SPI0 MISO (data from the IMU) pin on the robot board.
pub const SPI0_MISO_PIN: u8 = 20;
/// SPI0 chip-select pin on the robot board, driven as a plain GPIO.
pub const SPI0_CS_PIN: u8 = 21;

/// DMA channel used for SPI0 TX on the robot board.
pub const DMA_CH_TX: u8 = 1;
/// DMA channel used for SPI0 RX on the robot board.
pub const DMA_CH_RX: u8 = 2;

/// Default SPI clock: the robot's 7 MHz bus.
pub const SPI_DEFAULT_HZ: u32 = 7_000_000;
/// Signal-integrity control clock: the same firmware at 1 MHz.
///
/// Pass this to [`init_imu`] instead of [`SPI_DEFAULT_HZ`] to separate bus
/// integrity problems from protocol problems.
pub const SPI_CONTROL_HZ: u32 = 1_000_000;

// Both SPI0 DMA channels share `DMA_IRQ_0` on the RP235x, so they are bound
// to the same interrupt line.
bind_interrupts!(pub struct DmaIrqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

/// Initialized ICM-20948 on the robot's SPI0 bus.
///
/// The driver borrows the SPI0, pin and DMA singletons out of
/// [`embassy_rp::Peripherals`], so the lifetime is tied to that borrow.
pub type BoardImu<'d> =
    Icm20948Driver<SpiInterface<ExclusiveDevice<Spi<'d, SPI0, Async>, Output<'d>, Delay>>>;

/// Errors that can occur while bringing the board up.
///
/// The underlying transport error type is not `defmt`-formattable in this
/// crate combination, so each bring-up stage maps to a distinct variant.
#[derive(defmt::Format, Clone, Copy, PartialEq, Eq)]
pub enum BoardError {
    /// The device did not answer the `WHO_AM_I` identity probe.
    DeviceProbe,
    /// A bring-up register transaction failed.
    InitBus,
}

/// Build the SPI0 bus and initialize the ICM-20948 over SPI.
///
/// [`Icm20948Driver::init_spi`] resets the device, restores SPI mode before any
/// read, wakes it and verifies `WHO_AM_I`, so no manual reset sequence is needed.
/// The identity probe therefore happens inside `init_spi`, after SPI mode is
/// restored. That matters when the MCU restarts after the IMU's soft reset but
/// before `I2C_IF_DIS` is restored: a probe issued before `init_spi` would fail
/// on the still-disabled SPI interface, while `init_spi` can recover.
///
/// # Errors
///
/// Returns [`BoardError::DeviceProbe`] if the `WHO_AM_I` probe inside
/// `init_spi` fails and [`BoardError::InitBus`] if the SPI reset/wake sequence
/// fails.
pub async fn init_imu<'d>(p: &'d mut Peripherals, spi_hz: u32) -> Result<BoardImu<'d>, BoardError> {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = spi_hz;
    // Mode 0: sample on the first clock transition, clock idles low.
    spi_config.phase = Phase::CaptureOnFirstTransition;
    spi_config.polarity = Polarity::IdleLow;

    let spi = Spi::new(
        p.SPI0.reborrow(),
        p.PIN_18.reborrow(),
        p.PIN_19.reborrow(),
        p.PIN_20.reborrow(),
        p.DMA_CH1.reborrow(),
        p.DMA_CH2.reborrow(),
        DmaIrqs,
        spi_config,
    );

    // CS idles high so the IMU ignores the bus until a transaction starts.
    let cs = Output::new(p.PIN_21.reborrow(), Level::High);

    let spi_device = match ExclusiveDevice::new(spi, cs, Delay) {
        Ok(device) => device,
        // `Output` uses `Infallible` as its error type: driving CS high
        // cannot fail, so this branch is unreachable without panicking.
        Err(infallible) => match infallible {},
    };

    // Construct without a bus probe: the IMU may still have SPI disabled after
    // an MCU restart, and `init_spi` restores SPI mode before verifying
    // `WHO_AM_I`.
    let mut imu = Icm20948Driver::new(SpiInterface::new(spi_device));

    // Reset, restore SPI mode before any read, wake and verify `WHO_AM_I`.
    let mut delay = Delay;
    imu.init_spi(&mut delay).await.map_err(|e| match e {
        icm20948::Error::InvalidDevice(_) => BoardError::DeviceProbe,
        _ => BoardError::InitBus,
    })?;

    Ok(imu)
}

/// Bring the IMU up over SPI or park the firmware with a clear error.
///
/// This is the bring-up path shared by the example binaries: a failure is
/// logged together with the wiring checklist, then the firmware idles instead
/// of resetting. Binaries that need stage-specific handling can call
/// [`init_imu`] directly.
pub async fn init_imu_or_idle<'d>(p: &'d mut Peripherals, spi_hz: u32) -> BoardImu<'d> {
    print_board_info();
    match init_imu(p, spi_hz).await {
        Ok(imu) => {
            info!("ICM-20948 detected and initialized over SPI0");
            imu
        }
        Err(e) => {
            error!("IMU bring-up failed: {:?}", e);
            error!("Check 3V3/GND, the SPI0 pin map and that CS is not stuck low");
            idle_forever().await;
        }
    }
}

/// Log one error message and park the firmware.
///
/// For stages that need more than one line of context, log the extra lines and
/// then call [`idle_forever`] instead.
pub async fn idle_on_error(message: &str) -> ! {
    error!("{}", message);
    idle_forever().await;
}

/// Park the firmware after an unrecoverable error.
pub async fn idle_forever() -> ! {
    loop {
        Timer::after_millis(1000).await;
    }
}

/// Log the board profile: pin map, DMA channels, clock and the HIL hint.
///
/// Call this once at the start of an example, before [`init_imu_or_idle`].
pub fn print_board_info() {
    info!("Board: RP2354B (RP235x B variant) with ICM-20948 on SPI0, mode 0");
    info!(
        "Pin map: SCK=GP{} MOSI=GP{} MISO=GP{} CS=GP{}",
        SPI0_SCK_PIN, SPI0_MOSI_PIN, SPI0_MISO_PIN, SPI0_CS_PIN
    );
    info!("DMA: TX=CH{} RX=CH{}", DMA_CH_TX, DMA_CH_RX);
    info!(
        "SPI clock: {} Hz default, {} Hz control (change the init_imu_or_idle argument)",
        SPI_DEFAULT_HZ, SPI_CONTROL_HZ
    );
    info!("HIL: rotate the board 90 deg about Z and expect heading_deg to move by ~90 deg");
}
