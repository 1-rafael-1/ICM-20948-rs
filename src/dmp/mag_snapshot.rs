//! Read-only snapshot of the DMP magnetometer (Bank 3) configuration.
//!
//! The DMP nine-axis path depends on the host having configured the I2C master
//! for the AK09916: `I2C_SLV0` streams magnetometer data into the DMP, `I2C_SLV1`
//! performs the periodic single-byte measurement trigger, `I2C_MST_ODR_CONFIG`
//! sets the master sample rate, and `LP_CONFIG`/`USER_CTRL` enable the master
//! reachable from the DMP. Enabling the DMP can rewrite some of these registers,
//! which is one candidate explanation for a magnetometer path that goes stale
//! after `dmp_enable`.
//!
//! [`DmpMagConfigSnapshot`] captures those registers as **raw bytes** so a
//! before/after comparison is a simple field-by-field diff.
//!
//! The snapshot is a diagnostic only:
//!
//! - It is **read-only**: it never writes a register and does not change any
//!   configuration.
//! - It reads only the Bank 3 registers listed on the struct plus `LP_CONFIG`
//!   and `USER_CTRL` in Bank 0. It does **not** touch interrupt or status
//!   registers, so taking a snapshot cannot clear a latched condition.
//! - The raw bytes are reconstructed from the typed register fields, so
//!   reserved bits that have no field in the register definition read as zero.

use crate::{Bank, Error, Icm20948Driver};

/// Await an expression only in the async expansion of [`read_mag_snapshot_impl`].
macro_rules! maybe_await {
    ($expr:expr, blocking) => {
        $expr
    };
    ($expr:expr, asynchronous) => {
        $expr.await
    };
}

/// Read a register for the selected blocking or async mode.
macro_rules! read_register {
    ($reg:expr, blocking) => {
        $reg.read()
    };
    ($reg:expr, asynchronous) => {
        $reg.read_async().await
    };
}

/// Single implementation of the snapshot reads and bit-packing.
///
/// `$mode` selects the only difference between the blocking and async paths:
/// whether register I/O is awaited. The raw-byte reconstruction below is
/// defined once.
macro_rules! read_mag_snapshot_impl {
    ($driver:ident, $mode:ident) => {{
        maybe_await!($driver.select_bank(Bank::Bank3), $mode)?;

        let slv0_addr_reg = read_register!($driver.device_mut().bank_3_i_2_c_slv_0_addr(), $mode)?;
        let slv0_reg_reg = read_register!($driver.device_mut().bank_3_i_2_c_slv_0_reg(), $mode)?;
        let slv0_ctrl_reg = read_register!($driver.device_mut().bank_3_i_2_c_slv_0_ctrl(), $mode)?;
        let slv1_addr_reg = read_register!($driver.device_mut().bank_3_i_2_c_slv_1_addr(), $mode)?;
        let slv1_reg_reg = read_register!($driver.device_mut().bank_3_i_2_c_slv_1_reg(), $mode)?;
        let slv1_do_reg = read_register!($driver.device_mut().bank_3_i_2_c_slv_1_do(), $mode)?;
        let slv1_ctrl_reg = read_register!($driver.device_mut().bank_3_i_2_c_slv_1_ctrl(), $mode)?;
        let odr_reg = read_register!($driver.device_mut().bank_3_i_2_c_mst_odr_config(), $mode)?;

        maybe_await!($driver.select_bank(Bank::Bank0), $mode)?;
        let lp_config_reg = read_register!($driver.device_mut().lp_config(), $mode)?;
        let user_ctrl_reg = read_register!($driver.device_mut().user_ctrl(), $mode)?;

        Ok(DmpMagConfigSnapshot {
            slv0_addr: slv0_addr_reg.i_2_c_id_0()
                | (u8::from(slv0_addr_reg.i_2_c_slv_0_rnw()) << 7),
            slv0_reg: slv0_reg_reg.i_2_c_slv_0_reg(),
            slv0_ctrl: slv0_ctrl_reg.i_2_c_slv_0_leng()
                | (u8::from(slv0_ctrl_reg.i_2_c_slv_0_grp()) << 4)
                | (u8::from(slv0_ctrl_reg.i_2_c_slv_0_reg_dis()) << 5)
                | (u8::from(slv0_ctrl_reg.i_2_c_slv_0_byte_sw()) << 6)
                | (u8::from(slv0_ctrl_reg.i_2_c_slv_0_en()) << 7),
            slv1_addr: slv1_addr_reg.i_2_c_id_1()
                | (u8::from(slv1_addr_reg.i_2_c_slv_1_rnw()) << 7),
            slv1_reg: slv1_reg_reg.i_2_c_slv_1_reg(),
            slv1_do: slv1_do_reg.i_2_c_slv_1_do(),
            slv1_ctrl: slv1_ctrl_reg.i_2_c_slv_1_leng()
                | (u8::from(slv1_ctrl_reg.i_2_c_slv_1_grp()) << 4)
                | (u8::from(slv1_ctrl_reg.i_2_c_slv_1_reg_dis()) << 5)
                | (u8::from(slv1_ctrl_reg.i_2_c_slv_1_byte_sw()) << 6)
                | (u8::from(slv1_ctrl_reg.i_2_c_slv_1_en()) << 7),
            i2c_mst_odr_config: odr_reg.i_2_c_mst_odr_config(),
            lp_config: (u8::from(lp_config_reg.gyro_cycle()) << 4)
                | (u8::from(lp_config_reg.accel_cycle()) << 5)
                | (u8::from(lp_config_reg.i_2_c_mst_cycle()) << 6),
            user_ctrl: (u8::from(user_ctrl_reg.i_2_c_mst_rst()) << 1)
                | (u8::from(user_ctrl_reg.sram_rst()) << 2)
                | (u8::from(user_ctrl_reg.dmp_rst()) << 3)
                | (u8::from(user_ctrl_reg.i_2_c_if_dis()) << 4)
                | (u8::from(user_ctrl_reg.i_2_c_mst_en()) << 5)
                | (u8::from(user_ctrl_reg.fifo_en()) << 6)
                | (u8::from(user_ctrl_reg.dmp_en()) << 7),
        })
    }};
}

/// Raw Bank 3 (and related Bank 0) magnetometer configuration.
///
/// Every field is the raw register byte, reconstructed from the typed register
/// fields, so snapshots taken at different points in the lifecycle can be
/// compared field by field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpMagConfigSnapshot {
    /// Raw `I2C_SLV0_ADDR` (Bank 3, 0x03): `id | rnw << 7`.
    pub slv0_addr: u8,
    /// Raw `I2C_SLV0_REG` (Bank 3, 0x04).
    pub slv0_reg: u8,
    /// Raw `I2C_SLV0_CTRL` (Bank 3, 0x05):
    /// `leng | grp << 4 | reg_dis << 5 | byte_sw << 6 | en << 7`.
    pub slv0_ctrl: u8,
    /// Raw `I2C_SLV1_ADDR` (Bank 3, 0x07): `id | rnw << 7`.
    pub slv1_addr: u8,
    /// Raw `I2C_SLV1_REG` (Bank 3, 0x08).
    pub slv1_reg: u8,
    /// Raw `I2C_SLV1_DO` (Bank 3, 0x0A).
    pub slv1_do: u8,
    /// Raw `I2C_SLV1_CTRL` (Bank 3, 0x09):
    /// `leng | grp << 4 | reg_dis << 5 | byte_sw << 6 | en << 7`.
    pub slv1_ctrl: u8,
    /// Raw `I2C_MST_ODR_CONFIG` (Bank 3, 0x00), low nibble.
    pub i2c_mst_odr_config: u8,
    /// Raw `LP_CONFIG` (Bank 0, 0x05).
    pub lp_config: u8,
    /// Raw `USER_CTRL` (Bank 0, 0x03).
    pub user_ctrl: u8,
}

#[cfg(not(feature = "async"))]
impl<I> Icm20948Driver<I>
where
    I: device_driver::RegisterInterface<AddressType = u8>,
{
    /// Read the current DMP magnetometer configuration as raw register bytes.
    ///
    /// The driver switches to Bank 3 to read `I2C_SLV0_ADDR`, `I2C_SLV0_REG`,
    /// `I2C_SLV0_CTRL`, `I2C_SLV1_ADDR`, `I2C_SLV1_REG`, `I2C_SLV1_DO`,
    /// `I2C_SLV1_CTRL` and `I2C_MST_ODR_CONFIG`, then switches back to Bank 0 for
    /// `LP_CONFIG` and `USER_CTRL`, so the driver is left in Bank 0.
    ///
    /// This is a read-only diagnostic. It does not write any register and does
    /// not read interrupt or status registers.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub fn read_mag_config_snapshot(&mut self) -> Result<DmpMagConfigSnapshot, Error<I::Error>> {
        read_mag_snapshot_impl!(self, blocking)
    }
}

#[cfg(feature = "async")]
impl<I> Icm20948Driver<I>
where
    I: device_driver::AsyncRegisterInterface<AddressType = u8>,
{
    /// Read the current DMP magnetometer configuration as raw register bytes
    /// (async).
    ///
    /// Behaves exactly like the blocking `read_mag_config_snapshot`: it reads
    /// the Bank 3 magnetometer registers, then `LP_CONFIG` and `USER_CTRL` in
    /// Bank 0, leaving the driver in Bank 0. It is a read-only diagnostic and
    /// does not touch interrupt or status registers.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn read_mag_config_snapshot_async(
        &mut self,
    ) -> Result<DmpMagConfigSnapshot, Error<I::Error>> {
        read_mag_snapshot_impl!(self, asynchronous)
    }
}
