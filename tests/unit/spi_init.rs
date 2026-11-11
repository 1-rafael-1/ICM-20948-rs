//! Regression tests for the SPI-aware initialization path.
//!
//! The ICM-20948 soft reset clears `USER_CTRL.I2C_IF_DIS`. Over SPI, reads are
//! not served again until that bit is written, so init must write it back before
//! any read.

use crate::common::test_utils::MockDelay;
use crate::common::{Operation, create_mock_driver};
use icm20948::{Bank, Error};

#[cfg(feature = "dmp")]
use icm20948::dmp::{DmpConfig, DmpSampleRate};

const WHO_AM_I: u8 = 0x00;
const USER_CTRL: u8 = 0x03;
const PWR_MGMT_1: u8 = 0x06;

/// `init_spi` must restore SPI mode with a write-only `USER_CTRL` access before
/// any register read happens after the reset.
#[test]
fn init_spi_restores_spi_mode_before_any_read() {
    let (mut driver, interface) = create_mock_driver();
    interface.set_spi_only(true);

    driver
        .init_spi(&mut MockDelay)
        .expect("init_spi should succeed over SPI");

    assert_eq!(
        interface.spi_mode_violations(),
        0,
        "init_spi must not attempt a read before restoring SPI mode"
    );

    let ops = interface.operations();

    // Find the reset write rather than assuming a fixed offset: the bank-select
    // write and any earlier identity probe may precede it.
    let reset_index = ops
        .iter()
        .position(|op| {
            matches!(
                op,
                Operation::WriteRegister {
                    bank: Bank::Bank0,
                    address: PWR_MGMT_1,
                    value,
                } if value & 0x80 != 0
            )
        })
        .expect("PWR_MGMT_1 reset write not found");

    // The very next operation must be the write-only SPI-mode restore.
    match ops.get(reset_index + 1) {
        Some(Operation::WriteRegister {
            bank: Bank::Bank0,
            address: USER_CTRL,
            value: 0x10,
        }) => {}
        other => panic!("expected USER_CTRL=0x10 immediately after reset, got {other:?}"),
    }

    // Nothing at all may occur between the reset and the restore, and in
    // particular no read.
    assert!(
        !matches!(
            ops.get(reset_index + 1),
            Some(Operation::ReadRegister { .. })
        ),
        "no read may occur between reset and SPI-mode restore"
    );

    // After the restore, the device is woken with a write-only PWR_MGMT_1=0x01.
    let wake_index = ops
        .iter()
        .position(|op| {
            matches!(
                op,
                Operation::WriteRegister {
                    bank: Bank::Bank0,
                    address: PWR_MGMT_1,
                    value: 0x01,
                }
            )
        })
        .expect("PWR_MGMT_1 wake write not found");
    assert!(
        wake_index > reset_index + 1,
        "wake write must follow the SPI-mode restore"
    );

    // WHO_AM_I is read only after the restore.
    assert!(
        ops.iter().skip(wake_index + 1).any(|op| {
            matches!(
                op,
                Operation::ReadRegister {
                    bank: Bank::Bank0,
                    address: WHO_AM_I,
                    ..
                }
            )
        }),
        "WHO_AM_I must be read after the wake write"
    );
}

/// The legacy `init` path polls `PWR_MGMT_1` over SPI after the reset. Against a
/// device that enforces SPI mode it must time out, proving the mock models the
/// defect and that the SPI path is genuinely different.
#[test]
fn legacy_init_times_out_when_spi_mode_is_required() {
    let (mut driver, interface) = create_mock_driver();
    interface.set_spi_only(true);

    let result = driver.init(&mut MockDelay);

    assert!(
        matches!(result, Err(Error::InitializationTimeout)),
        "legacy init should time out over SPI, got {result:?}"
    );
    assert!(
        interface.spi_mode_violations() > 0,
        "legacy init should have attempted reads before restoring SPI mode"
    );
}

/// `init_spi` resets the device, so the host-side DMP flags must be invalidated.
/// Otherwise a recovered driver would accept configuration the reset erased.
#[test]
#[cfg(feature = "dmp")]
fn init_spi_invalidates_dmp_state() {
    let (mut driver, interface) = create_mock_driver();
    interface.set_spi_only(true);

    driver
        .dmp_load_firmware()
        .expect("firmware load should succeed");

    let config = DmpConfig::six_axis().with_sample_rate(DmpSampleRate::Hz112);
    driver
        .dmp_configure(&config)
        .expect("DMP configuration should succeed before init_spi");

    driver
        .init_spi(&mut MockDelay)
        .expect("init_spi should succeed over SPI");

    assert!(
        matches!(
            driver.dmp_configure(&config),
            Err(Error::DmpFirmwareNotLoaded)
        ),
        "init_spi must invalidate the loaded-firmware flag"
    );
    assert!(
        matches!(driver.dmp_enable(true), Err(Error::DmpFirmwareNotLoaded)),
        "init_spi must invalidate DMP state before the reset"
    );
}
