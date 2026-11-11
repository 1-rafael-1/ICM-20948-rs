//! Bank 3 magnetometer configuration snapshot tests.
//!
//! These tests drive the public `read_mag_config_snapshot` API through the
//! blocking mock register interface and assert every reported raw byte.

use crate::common::Operation;
use crate::common::test_utils::create_mock_driver;
use icm20948::Bank;
use icm20948::dmp::DmpMagConfigSnapshot;

/// The bank selected by the most recent bank-switch operation, if any.
fn last_bank_switch(operations: &[Operation]) -> Option<Bank> {
    operations
        .iter()
        .rev()
        .find_map(|operation| match operation {
            Operation::BankSwitch { to, .. } => Some(*to),
            _ => None,
        })
}

#[test]
fn snapshot_reports_all_bank3_and_related_registers() {
    let (mut driver, interface) = create_mock_driver();

    interface.set_register(Bank::Bank3, 0x03, 0x8C); // SLV0_ADDR: id 0x0C, read
    interface.set_register(Bank::Bank3, 0x04, 0x03); // SLV0_REG
    interface.set_register(Bank::Bank3, 0x05, 0x9A); // SLV0_CTRL: leng 10, grp, en
    interface.set_register(Bank::Bank3, 0x07, 0x0C); // SLV1_ADDR: id 0x0C, write
    interface.set_register(Bank::Bank3, 0x08, 0x31); // SLV1_REG
    interface.set_register(Bank::Bank3, 0x0A, 0x08); // SLV1_DO
    interface.set_register(Bank::Bank3, 0x09, 0x81); // SLV1_CTRL: leng 1, en
    interface.set_register(Bank::Bank3, 0x00, 0x04); // I2C_MST_ODR_CONFIG
    interface.set_register(Bank::Bank0, 0x05, 0x40); // LP_CONFIG: i2c_mst_cycle
    interface.set_register(Bank::Bank0, 0x03, 0xE0); // USER_CTRL

    let snapshot = driver
        .read_mag_config_snapshot()
        .expect("read_mag_config_snapshot failed");

    assert_eq!(
        snapshot,
        DmpMagConfigSnapshot {
            slv0_addr: 0x8C,
            slv0_reg: 0x03,
            slv0_ctrl: 0x9A,
            slv1_addr: 0x0C,
            slv1_reg: 0x31,
            slv1_do: 0x08,
            slv1_ctrl: 0x81,
            i2c_mst_odr_config: 0x04,
            lp_config: 0x40,
            user_ctrl: 0xE0,
        }
    );

    // The snapshot must leave the driver in Bank 0 for normal register access.
    assert_eq!(last_bank_switch(&interface.operations()), Some(Bank::Bank0));
    assert_eq!(driver.read_who_am_i().expect("read_who_am_i failed"), 0xEA);
}

#[test]
fn snapshot_reads_actual_registers_not_defaults() {
    let (mut driver, interface) = create_mock_driver();

    interface.set_register(Bank::Bank3, 0x03, 0x0C); // SLV0 disabled (write)
    interface.set_register(Bank::Bank3, 0x04, 0x42);
    interface.set_register(Bank::Bank3, 0x05, 0x00); // SLV0_CTRL: disabled
    interface.set_register(Bank::Bank3, 0x07, 0x8C); // SLV1 read
    interface.set_register(Bank::Bank3, 0x08, 0x0B);
    interface.set_register(Bank::Bank3, 0x0A, 0xFF);
    interface.set_register(Bank::Bank3, 0x09, 0xF2); // leng 2, grp, reg_dis, byte_sw, en
    interface.set_register(Bank::Bank3, 0x00, 0x0F); // fast ODR
    interface.set_register(Bank::Bank0, 0x05, 0x30); // accel + gyro cycle
    interface.set_register(Bank::Bank0, 0x03, 0x30); // i2c_mst_en + i2c_if_dis

    let snapshot = driver
        .read_mag_config_snapshot()
        .expect("read_mag_config_snapshot failed");

    assert_eq!(
        snapshot,
        DmpMagConfigSnapshot {
            slv0_addr: 0x0C,
            slv0_reg: 0x42,
            slv0_ctrl: 0x00,
            slv1_addr: 0x8C,
            slv1_reg: 0x0B,
            slv1_do: 0xFF,
            slv1_ctrl: 0xF2,
            i2c_mst_odr_config: 0x0F,
            lp_config: 0x30,
            user_ctrl: 0x30,
        }
    );

    assert_eq!(last_bank_switch(&interface.operations()), Some(Bank::Bank0));
}
