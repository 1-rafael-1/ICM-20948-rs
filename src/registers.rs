//! Register definitions for the ICM-20948
//!
//! This module contains the register definitions for all banks of the ICM-20948.
//! The ICM-20948 uses a bank-switching architecture where registers at addresses 0x00-0x7F
//! have different meanings depending on which bank is selected via `REG_BANK_SEL` (0x7F).
//!
//! ## Bank Architecture
//! - **Bank 0**: Primary configuration and sensor data
//! - **Bank 1**: Self-test and offset configuration
//! - **Bank 2**: Gyroscope and accelerometer configuration
//! - **Bank 3**: I2C master configuration for magnetometer access
//!
//! All registers that share addresses across banks use `ALLOW_ADDRESS_OVERLAP = true`.

device_driver::create_device!(
    device_name: Icm20948,
    dsl: {
        config {
            type RegisterAddressType = u8;
            type DefaultByteOrder = BE;
        }

        // ==================== BANK 0 REGISTERS ====================
        // Primary configuration and sensor data

        /// WHO_AM_I - Device ID Register (Bank 0, 0x00)
        /// Expected value: 0xEA
        register WhoAmI {
            const ADDRESS = 0x00;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Device ID (should read 0xEA)
            who_am_i: uint = 0..8,
        },

        /// USER_CTRL - User Control (Bank 0, 0x03)
        register UserCtrl {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_0: uint = 0..1,
            /// I2C master reset
            i2c_mst_rst: bool = 1,
            /// SRAM reset
            sram_rst: bool = 2,
            /// DMP reset
            dmp_rst: bool = 3,
            /// I2C interface disable
            i2c_if_dis: bool = 4,
            /// I2C master enable
            i2c_mst_en: bool = 5,
            /// FIFO enable
            fifo_en: bool = 6,
            /// DMP enable
            dmp_en: bool = 7,
        },

        /// LP_CONFIG - Low Power Configuration (Bank 0, 0x05)
        register LpConfig {
            const ADDRESS = 0x05;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_3_0: uint = 0..4,
            /// Gyroscope cycle mode
            gyro_cycle: bool = 4,
            /// Accelerometer cycle mode
            accel_cycle: bool = 5,
            /// I2C master cycle mode
            i2c_mst_cycle: bool = 6,
            reserved_7: uint = 7..8,
        },

        /// PWR_MGMT_1 - Power Management 1 (Bank 0, 0x06)
        register PwrMgmt1 {
            const ADDRESS = 0x06;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Clock source select (0=internal 20MHz, 1=auto select best, 2-7=stop clock)
            clksel: uint = 0..3,
            /// Temperature sensor disable
            temp_dis: bool = 3,
            reserved_4: uint = 4..5,
            /// Low power mode enable
            lp_en: bool = 5,
            /// Sleep mode enable
            sleep: bool = 6,
            /// Device reset
            device_reset: bool = 7,
        },

        /// PWR_MGMT_2 - Power Management 2 (Bank 0, 0x07)
        register PwrMgmt2 {
            const ADDRESS = 0x07;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Disable gyroscope Z-axis
            disable_gyro_z: bool = 0,
            /// Disable gyroscope Y-axis
            disable_gyro_y: bool = 1,
            /// Disable gyroscope X-axis
            disable_gyro_x: bool = 2,
            /// Disable accelerometer Z-axis
            disable_accel_z: bool = 3,
            /// Disable accelerometer Y-axis
            disable_accel_y: bool = 4,
            /// Disable accelerometer X-axis
            disable_accel_x: bool = 5,
            reserved_7_6: uint = 6..8,
        },

        /// INT_PIN_CFG - Interrupt Pin Configuration (Bank 0, 0x0F)
        register IntPinCfg {
            const ADDRESS = 0x0F;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_0: uint = 0..1,
            /// Bypass enable
            bypass_en: bool = 1,
            /// FSYNC interrupt mode select
            fsync_int_mode_en: bool = 2,
            /// ACTL - Active low interrupt
            actl_fsync: bool = 3,
            /// INT_ANYRD_2CLEAR - Interrupt status clear on any read
            int_anyrd_2clear: bool = 4,
            /// Latch interrupt
            int1_latch_int_en: bool = 5,
            /// INT1 open drain
            int1_open: bool = 6,
            /// INT1 active low
            int1_actl: bool = 7,
        },

        /// INT_ENABLE - Interrupt Enable (Bank 0, 0x10)
        register IntEnable {
            const ADDRESS = 0x10;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_2_0: uint = 0..3,
            /// I2C master interrupt enable
            i2c_mst_int_en: bool = 3,
            /// DMP interrupt enable
            dmp_int1_en: bool = 4,
            /// PLL ready interrupt enable
            pll_rdy_en: bool = 5,
            /// WoM (Wake on Motion) interrupt enable
            wom_int_en: bool = 6,
            reserved_7: uint = 7..8,
        },

        /// INT_ENABLE_1 - Interrupt Enable 1 (Bank 0, 0x11)
        register IntEnable1 {
            const ADDRESS = 0x11;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Raw data ready interrupt enable
            raw_data_0_rdy_en: bool = 0,
            reserved_7_1: uint = 1..8,
        },

        /// INT_ENABLE_2 - Interrupt Enable 2 (Bank 0, 0x12)
        register IntEnable2 {
            const ADDRESS = 0x12;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO overflow interrupt enable (bits 4:0 for different FIFOs)
            fifo_overflow_en: uint = 0..5,
            reserved_7_5: uint = 5..8,
        },

        /// INT_ENABLE_3 - Interrupt Enable 3 (Bank 0, 0x13)
        register IntEnable3 {
            const ADDRESS = 0x13;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO watermark interrupt enable (bits 4:0 for different FIFOs)
            fifo_wm_en: uint = 0..5,
            reserved_7_5: uint = 5..8,
        },

        /// I2C_MST_STATUS - I2C Master Status (Bank 0, 0x17)
        register I2cMstStatus {
            const ADDRESS = 0x17;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 0 NACK
            i2c_slv0_nack: bool = 0,
            /// I2C slave 1 NACK
            i2c_slv1_nack: bool = 1,
            /// I2C slave 2 NACK
            i2c_slv2_nack: bool = 2,
            /// I2C slave 3 NACK
            i2c_slv3_nack: bool = 3,
            /// I2C slave 4 NACK
            i2c_slv4_nack: bool = 4,
            /// I2C lost arbitration
            i2c_lost_arb: bool = 5,
            /// I2C slave 4 done
            i2c_slv4_done: bool = 6,
            /// Pass through mode
            pass_through: bool = 7,
        },

        /// INT_STATUS - Interrupt Status (Bank 0, 0x19)
        register IntStatus {
            const ADDRESS = 0x19;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_2_0: uint = 0..3,
            /// I2C master interrupt
            i2c_mst_int: bool = 3,
            /// DMP interrupt
            dmp_int1: bool = 4,
            /// PLL ready interrupt
            pll_rdy_int: bool = 5,
            /// WoM interrupt
            wom_int: bool = 6,
            reserved_7: uint = 7..8,
        },

        /// INT_STATUS_1 - Interrupt Status 1 (Bank 0, 0x1A)
        register IntStatus1 {
            const ADDRESS = 0x1A;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Raw data ready interrupt
            raw_data_0_rdy_int: bool = 0,
            reserved_7_1: uint = 1..8,
        },

        /// INT_STATUS_2 - Interrupt Status 2 (Bank 0, 0x1B)
        register IntStatus2 {
            const ADDRESS = 0x1B;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO overflow interrupt status (bits 4:0)
            fifo_overflow_int: uint = 0..5,
            reserved_7_5: uint = 5..8,
        },

        /// INT_STATUS_3 - Interrupt Status 3 (Bank 0, 0x1C)
        register IntStatus3 {
            const ADDRESS = 0x1C;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO watermark interrupt status (bits 4:0)
            fifo_wm_int: uint = 0..5,
            reserved_7_5: uint = 5..8,
        },

        /// SINGLE_FIFO_PRIORITY_SEL - Single FIFO Priority Select (Bank 0, 0x26)
        /// Hardware fix register for DMP operation
        /// Cybergear sets this to 0xE4 for DMP mode
        register SingleFifoPrioritySel {
            const ADDRESS = 0x26;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Single FIFO priority selection
            single_fifo_priority_sel: uint = 0..8,
        },

        /// DELAY_TIMEH - Delay Time High (Bank 0, 0x28)
        register DelayTimeh {
            const ADDRESS = 0x28;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Delay time high byte
            delay_time_h: uint = 0..8,
        },

        /// DELAY_TIMEL - Delay Time Low (Bank 0, 0x29)
        register DelayTimel {
            const ADDRESS = 0x29;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Delay time low byte
            delay_time_l: uint = 0..8,
        },

        /// ACCEL_XOUT_H - Accelerometer X-axis High Byte (Bank 0, 0x2D)
        register AccelXoutH {
            const ADDRESS = 0x2D;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer X-axis data high byte
            accel_xout_h: uint = 0..8,
        },

        /// ACCEL_XOUT_L - Accelerometer X-axis Low Byte (Bank 0, 0x2E)
        register AccelXoutL {
            const ADDRESS = 0x2E;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer X-axis data low byte
            accel_xout_l: uint = 0..8,
        },

        /// ACCEL_YOUT_H - Accelerometer Y-axis High Byte (Bank 0, 0x2F)
        register AccelYoutH {
            const ADDRESS = 0x2F;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer Y-axis data high byte
            accel_yout_h: uint = 0..8,
        },

        /// ACCEL_YOUT_L - Accelerometer Y-axis Low Byte (Bank 0, 0x30)
        register AccelYoutL {
            const ADDRESS = 0x30;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer Y-axis data low byte
            accel_yout_l: uint = 0..8,
        },

        /// ACCEL_ZOUT_H - Accelerometer Z-axis High Byte (Bank 0, 0x31)
        register AccelZoutH {
            const ADDRESS = 0x31;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer Z-axis data high byte
            accel_zout_h: uint = 0..8,
        },

        /// ACCEL_ZOUT_L - Accelerometer Z-axis Low Byte (Bank 0, 0x32)
        register AccelZoutL {
            const ADDRESS = 0x32;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer Z-axis data low byte
            accel_zout_l: uint = 0..8,
        },

        /// GYRO_XOUT_H - Gyroscope X-axis High Byte (Bank 0, 0x33)
        register GyroXoutH {
            const ADDRESS = 0x33;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope X-axis data high byte
            gyro_xout_h: uint = 0..8,
        },

        /// GYRO_XOUT_L - Gyroscope X-axis Low Byte (Bank 0, 0x34)
        register GyroXoutL {
            const ADDRESS = 0x34;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope X-axis data low byte
            gyro_xout_l: uint = 0..8,
        },

        /// GYRO_YOUT_H - Gyroscope Y-axis High Byte (Bank 0, 0x35)
        register GyroYoutH {
            const ADDRESS = 0x35;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope Y-axis data high byte
            gyro_yout_h: uint = 0..8,
        },

        /// GYRO_YOUT_L - Gyroscope Y-axis Low Byte (Bank 0, 0x36)
        register GyroYoutL {
            const ADDRESS = 0x36;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope Y-axis data low byte
            gyro_yout_l: uint = 0..8,
        },

        /// GYRO_ZOUT_H - Gyroscope Z-axis High Byte (Bank 0, 0x37)
        register GyroZoutH {
            const ADDRESS = 0x37;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope Z-axis data high byte
            gyro_zout_h: uint = 0..8,
        },

        /// GYRO_ZOUT_L - Gyroscope Z-axis Low Byte (Bank 0, 0x38)
        register GyroZoutL {
            const ADDRESS = 0x38;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope Z-axis data low byte
            gyro_zout_l: uint = 0..8,
        },

        /// TEMP_OUT_H - Temperature High Byte (Bank 0, 0x39)
        register TempOutH {
            const ADDRESS = 0x39;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Temperature data high byte
            temp_out_h: uint = 0..8,
        },

        /// TEMP_OUT_L - Temperature Low Byte (Bank 0, 0x3A)
        register TempOutL {
            const ADDRESS = 0x3A;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Temperature data low byte
            temp_out_l: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_00 - External Sensor Data 00 (Bank 0, 0x3B)
        register ExtSlvSensData00 {
            const ADDRESS = 0x3B;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 00
            ext_slv_sens_data_00: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_01 - External Sensor Data 01 (Bank 0, 0x3C)
        register ExtSlvSensData01 {
            const ADDRESS = 0x3C;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 01
            ext_slv_sens_data_01: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_02 - External Sensor Data 02 (Bank 0, 0x3D)
        register ExtSlvSensData02 {
            const ADDRESS = 0x3D;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 02
            ext_slv_sens_data_02: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_03 - External Sensor Data 03 (Bank 0, 0x3E)
        register ExtSlvSensData03 {
            const ADDRESS = 0x3E;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 03
            ext_slv_sens_data_03: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_04 - External Sensor Data 04 (Bank 0, 0x3F)
        register ExtSlvSensData04 {
            const ADDRESS = 0x3F;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 04
            ext_slv_sens_data_04: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_05 - External Sensor Data 05 (Bank 0, 0x40)
        register ExtSlvSensData05 {
            const ADDRESS = 0x40;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 05
            ext_slv_sens_data_05: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_06 - External Sensor Data 06 (Bank 0, 0x41)
        register ExtSlvSensData06 {
            const ADDRESS = 0x41;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 06
            ext_slv_sens_data_06: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_07 - External Sensor Data 07 (Bank 0, 0x42)
        register ExtSlvSensData07 {
            const ADDRESS = 0x42;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 07
            ext_slv_sens_data_07: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_08 - External Sensor Data 08 (Bank 0, 0x43)
        register ExtSlvSensData08 {
            const ADDRESS = 0x43;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 08
            ext_slv_sens_data_08: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_09 - External Sensor Data 09 (Bank 0, 0x44)
        register ExtSlvSensData09 {
            const ADDRESS = 0x44;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 09
            ext_slv_sens_data_09: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_10 - External Sensor Data 10 (Bank 0, 0x45)
        register ExtSlvSensData10 {
            const ADDRESS = 0x45;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 10
            ext_slv_sens_data_10: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_11 - External Sensor Data 11 (Bank 0, 0x46)
        register ExtSlvSensData11 {
            const ADDRESS = 0x46;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 11
            ext_slv_sens_data_11: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_12 - External Sensor Data 12 (Bank 0, 0x47)
        register ExtSlvSensData12 {
            const ADDRESS = 0x47;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 12
            ext_slv_sens_data_12: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_13 - External Sensor Data 13 (Bank 0, 0x48)
        register ExtSlvSensData13 {
            const ADDRESS = 0x48;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 13
            ext_slv_sens_data_13: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_14 - External Sensor Data 14 (Bank 0, 0x49)
        register ExtSlvSensData14 {
            const ADDRESS = 0x49;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 14
            ext_slv_sens_data_14: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_15 - External Sensor Data 15 (Bank 0, 0x4A)
        register ExtSlvSensData15 {
            const ADDRESS = 0x4A;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 15
            ext_slv_sens_data_15: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_16 - External Sensor Data 16 (Bank 0, 0x4B)
        register ExtSlvSensData16 {
            const ADDRESS = 0x4B;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 16
            ext_slv_sens_data_16: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_17 - External Sensor Data 17 (Bank 0, 0x4C)
        register ExtSlvSensData17 {
            const ADDRESS = 0x4C;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 17
            ext_slv_sens_data_17: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_18 - External Sensor Data 18 (Bank 0, 0x4D)
        register ExtSlvSensData18 {
            const ADDRESS = 0x4D;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 18
            ext_slv_sens_data_18: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_19 - External Sensor Data 19 (Bank 0, 0x4E)
        register ExtSlvSensData19 {
            const ADDRESS = 0x4E;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 19
            ext_slv_sens_data_19: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_20 - External Sensor Data 20 (Bank 0, 0x4F)
        register ExtSlvSensData20 {
            const ADDRESS = 0x4F;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 20
            ext_slv_sens_data_20: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_21 - External Sensor Data 21 (Bank 0, 0x50)
        register ExtSlvSensData21 {
            const ADDRESS = 0x50;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 21
            ext_slv_sens_data_21: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_22 - External Sensor Data 22 (Bank 0, 0x51)
        register ExtSlvSensData22 {
            const ADDRESS = 0x51;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 22
            ext_slv_sens_data_22: uint = 0..8,
        },

        /// EXT_SLV_SENS_DATA_23 - External Sensor Data 23 (Bank 0, 0x52)
        register ExtSlvSensData23 {
            const ADDRESS = 0x52;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// External sensor data byte 23
            ext_slv_sens_data_23: uint = 0..8,
        },

        /// FIFO_EN_1 - FIFO Enable 1 (Bank 0, 0x66)
        register FifoEn1 {
            const ADDRESS = 0x66;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// SLV_0 FIFO enable
            slv_0_fifo_en: bool = 0,
            /// SLV_1 FIFO enable
            slv_1_fifo_en: bool = 1,
            /// SLV_2 FIFO enable
            slv_2_fifo_en: bool = 2,
            /// SLV_3 FIFO enable
            slv_3_fifo_en: bool = 3,
            reserved_7_4: uint = 4..8,
        },

        /// FIFO_EN_2 - FIFO Enable 2 (Bank 0, 0x67)
        register FifoEn2 {
            const ADDRESS = 0x67;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Temperature FIFO enable
            temp_fifo_en: bool = 0,
            /// Gyroscope X FIFO enable
            gyro_x_fifo_en: bool = 1,
            /// Gyroscope Y FIFO enable
            gyro_y_fifo_en: bool = 2,
            /// Gyroscope Z FIFO enable
            gyro_z_fifo_en: bool = 3,
            /// Accelerometer FIFO enable
            accel_fifo_en: bool = 4,
            reserved_7_5: uint = 5..8,
        },

        /// FIFO_RST - FIFO Reset (Bank 0, 0x68)
        register FifoRst {
            const ADDRESS = 0x68;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO reset bits (one per FIFO)
            fifo_reset: uint = 0..5,
            reserved_7_5: uint = 5..8,
        },

        /// FIFO_MODE - FIFO Mode (Bank 0, 0x69)
        register FifoMode {
            const ADDRESS = 0x69;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO mode bits (one per FIFO)
            fifo_mode: uint = 0..5,
            reserved_7_5: uint = 5..8,
        },

        /// FIFO_COUNTH - FIFO Count High (Bank 0, 0x70)
        register FifoCounth {
            const ADDRESS = 0x70;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO count high byte (12:8)
            fifo_cnt_h: uint = 0..5,
            reserved_7_5: uint = 5..8,
        },

        /// FIFO_COUNTL - FIFO Count Low (Bank 0, 0x71)
        register FifoCountl {
            const ADDRESS = 0x71;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO count low byte (7:0)
            fifo_cnt_l: uint = 0..8,
        },

        /// FIFO_R_W - FIFO Read/Write (Bank 0, 0x72)
        register FifoRW {
            const ADDRESS = 0x72;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO read/write data
            fifo_r_w: uint = 0..8,
        },

        /// DATA_RDY_STATUS - Data Ready Status (Bank 0, 0x74)
        register DataRdyStatus {
            const ADDRESS = 0x74;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Raw data ready status
            raw_data_rdy: uint = 0..4,
            /// WoM interrupt status
            wof_status: uint = 4..8,
        },

        /// HW_FIX_DISABLE - Hardware Fix Disable (Bank 0, 0x75)
        /// Hardware fix register for DMP operation
        /// Cybergear sets this to 0x48 for DMP mode
        register HwFixDisable {
            const ADDRESS = 0x75;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Hardware fix disable value
            hw_fix_disable: uint = 0..8,
        },

        /// FIFO_CFG - FIFO Configuration (Bank 0, 0x76)
        register FifoCfg {
            const ADDRESS = 0x76;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FIFO configuration
            fifo_cfg: uint = 0..1,
            reserved_7_1: uint = 1..8,
        },

        /// MEM_START_ADDR - DMP Memory Start Address (Bank 0, 0x7C)
        register MemStartAddr {
            const ADDRESS = 0x7C;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// DMP memory start address
            mem_start_addr: uint = 0..8,
        },

        /// MEM_R_W - DMP Memory Read/Write (Bank 0, 0x7D)
        register MemRW {
            const ADDRESS = 0x7D;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// DMP memory read/write data
            mem_r_w: uint = 0..8,
        },

        /// MEM_BANK_SEL - DMP Memory Bank Select (Bank 0, 0x7E)
        register MemBankSel {
            const ADDRESS = 0x7E;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// DMP memory bank selection
            mem_bank_sel: uint = 0..8,
        },

        /// REG_BANK_SEL - Register Bank Selection (All Banks, 0x7F)
        register RegBankSel {
            const ADDRESS = 0x7F;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_3_0: uint = 0..4,
            /// User bank selection (0-3, shifted left by 4 bits: 0x00, 0x10, 0x20, 0x30)
            user_bank: uint = 4..6,
            reserved_7_6: uint = 6..8,
        },

        // ==================== BANK 1 REGISTERS ====================
        // Self-test and offset configuration

        /// SELF_TEST_X_GYRO (Bank 1, 0x02)
        register Bank1SelfTestXGyro {
            const ADDRESS = 0x02;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// X-axis gyroscope self-test value
            xg_st_data: uint = 0..8,
        },

        /// SELF_TEST_Y_GYRO (Bank 1, 0x03)
        register Bank1SelfTestYGyro {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Y-axis gyroscope self-test value
            yg_st_data: uint = 0..8,
        },

        /// SELF_TEST_Z_GYRO (Bank 1, 0x04)
        register Bank1SelfTestZGyro {
            const ADDRESS = 0x04;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Z-axis gyroscope self-test value
            zg_st_data: uint = 0..8,
        },

        /// SELF_TEST_X_ACCEL (Bank 1, 0x0E)
        register Bank1SelfTestXAccel {
            const ADDRESS = 0x0E;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// X-axis accelerometer self-test value
            xa_st_data: uint = 0..8,
        },

        /// SELF_TEST_Y_ACCEL (Bank 1, 0x0F)
        register Bank1SelfTestYAccel {
            const ADDRESS = 0x0F;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Y-axis accelerometer self-test value
            ya_st_data: uint = 0..8,
        },

        /// SELF_TEST_Z_ACCEL (Bank 1, 0x10)
        register Bank1SelfTestZAccel {
            const ADDRESS = 0x10;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Z-axis accelerometer self-test value
            za_st_data: uint = 0..8,
        },

        /// XA_OFFS_H (Bank 1, 0x14)
        register Bank1XaOffsH {
            const ADDRESS = 0x14;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// X-axis accelerometer offset high byte
            xa_offs_h: uint = 0..8,
        },

        /// XA_OFFS_L (Bank 1, 0x15)
        register Bank1XaOffsL {
            const ADDRESS = 0x15;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_0: uint = 0..1,
            /// X-axis accelerometer offset low 7 bits
            xa_offs_l: uint = 1..8,
        },

        /// YA_OFFS_H (Bank 1, 0x17)
        register Bank1YaOffsH {
            const ADDRESS = 0x17;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Y-axis accelerometer offset high byte
            ya_offs_h: uint = 0..8,
        },

        /// YA_OFFS_L (Bank 1, 0x18)
        register Bank1YaOffsL {
            const ADDRESS = 0x18;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_0: uint = 0..1,
            /// Y-axis accelerometer offset low 7 bits
            ya_offs_l: uint = 1..8,
        },

        /// ZA_OFFS_H (Bank 1, 0x1A)
        register Bank1ZaOffsH {
            const ADDRESS = 0x1A;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Z-axis accelerometer offset high byte
            za_offs_h: uint = 0..8,
        },

        /// ZA_OFFS_L (Bank 1, 0x1B)
        register Bank1ZaOffsL {
            const ADDRESS = 0x1B;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            reserved_0: uint = 0..1,
            /// Z-axis accelerometer offset low 7 bits
            za_offs_l: uint = 1..8,
        },

        /// TIMEBASE_CORRECTION_PLL (Bank 1, 0x28)
        register Bank1TimebaseCorrectionPll {
            const ADDRESS = 0x28;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// PLL timebase correction
            timebase_correction_pll: uint = 0..8,
        },

        // ==================== BANK 2 REGISTERS ====================
        // Gyroscope and accelerometer configuration

        /// GYRO_SMPLRT_DIV (Bank 2, 0x00)
        register Bank2GyroSmplrtDiv {
            const ADDRESS = 0x00;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope sample rate divider
            gyro_smplrt_div: uint = 0..8,
        },

        /// GYRO_CONFIG_1 (Bank 2, 0x01)
        register Bank2GyroConfig1 {
            const ADDRESS = 0x01;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope FCHOICE
            gyro_fchoice: bool = 0,
            /// Gyroscope full scale select (±250, ±500, ±1000, ±2000 dps)
            gyro_fs_sel: uint = 1..3,
            /// Gyroscope DLPF configuration
            gyro_dlpfcfg: uint = 3..6,
            reserved_7_6: uint = 6..8,
        },

        /// GYRO_CONFIG_2 (Bank 2, 0x02)
        register Bank2GyroConfig2 {
            const ADDRESS = 0x02;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Gyroscope averaging configuration
            gyro_avgcfg: uint = 0..3,
            /// Z-axis gyroscope self-test enable
            zgyro_cten: bool = 3,
            /// Y-axis gyroscope self-test enable
            ygyro_cten: bool = 4,
            /// X-axis gyroscope self-test enable
            xgyro_cten: bool = 5,
            reserved_7_6: uint = 6..8,
        },

        /// XG_OFFS_USRH (Bank 2, 0x03)
        register Bank2XgOffsUsrh {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// X-axis gyroscope offset high byte
            xg_offs_usr_h: uint = 0..8,
        },

        /// XG_OFFS_USRL (Bank 2, 0x04)
        register Bank2XgOffsUsrl {
            const ADDRESS = 0x04;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// X-axis gyroscope offset low byte
            xg_offs_usr_l: uint = 0..8,
        },

        /// YG_OFFS_USRH (Bank 2, 0x05)
        register Bank2YgOffsUsrh {
            const ADDRESS = 0x05;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Y-axis gyroscope offset high byte
            yg_offs_usr_h: uint = 0..8,
        },

        /// YG_OFFS_USRL (Bank 2, 0x06)
        register Bank2YgOffsUsrl {
            const ADDRESS = 0x06;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Y-axis gyroscope offset low byte
            yg_offs_usr_l: uint = 0..8,
        },

        /// ZG_OFFS_USRH (Bank 2, 0x07)
        register Bank2ZgOffsUsrh {
            const ADDRESS = 0x07;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Z-axis gyroscope offset high byte
            zg_offs_usr_h: uint = 0..8,
        },

        /// ZG_OFFS_USRL (Bank 2, 0x08)
        register Bank2ZgOffsUsrl {
            const ADDRESS = 0x08;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Z-axis gyroscope offset low byte
            zg_offs_usr_l: uint = 0..8,
        },

        /// ODR_ALIGN_EN (Bank 2, 0x09)
        register Bank2OdrAlignEn {
            const ADDRESS = 0x09;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// ODR alignment enable
            odr_align_en: bool = 0,
            reserved_7_1: uint = 1..8,
        },

        /// PRGM_START_ADDRH - DMP Program Start Address High (Bank 2, 0x50)
        register Bank2PrgmStartAddrh {
            const ADDRESS = 0x50;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// DMP program start address high byte
            prgm_start_addrh: uint = 0..8,
        },

        /// PRGM_START_ADDRL - DMP Program Start Address Low (Bank 2, 0x51)
        register Bank2PrgmStartAddrl {
            const ADDRESS = 0x51;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// DMP program start address low byte
            prgm_start_addrl: uint = 0..8,
        },

        /// ACCEL_SMPLRT_DIV_1 (Bank 2, 0x10)
        register Bank2AccelSmplrtDiv1 {
            const ADDRESS = 0x10;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer sample rate divider high byte
            accel_smplrt_div_1: uint = 0..4,
            reserved_7_4: uint = 4..8,
        },

        /// ACCEL_SMPLRT_DIV_2 (Bank 2, 0x11)
        register Bank2AccelSmplrtDiv2 {
            const ADDRESS = 0x11;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer sample rate divider low byte
            accel_smplrt_div_2: uint = 0..8,
        },

        /// ACCEL_INTEL_CTRL (Bank 2, 0x12)
        register Bank2AccelIntelCtrl {
            const ADDRESS = 0x12;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer intelligence enable
            accel_intel_en: bool = 0,
            /// Accelerometer intelligence mode
            accel_intel_mode_int: bool = 1,
            reserved_7_2: uint = 2..8,
        },

        /// ACCEL_WOM_THR (Bank 2, 0x13)
        register Bank2AccelWomThr {
            const ADDRESS = 0x13;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Wake-on-motion threshold
            wom_threshold: uint = 0..8,
        },

        /// ACCEL_CONFIG (Bank 2, 0x14)
        register Bank2AccelConfig {
            const ADDRESS = 0x14;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer FCHOICE
            accel_fchoice: bool = 0,
            /// Accelerometer full scale select (±2g, ±4g, ±8g, ±16g)
            accel_fs_sel: uint = 1..3,
            /// Accelerometer DLPF configuration
            accel_dlpfcfg: uint = 3..6,
            reserved_7_6: uint = 6..8,
        },

        /// ACCEL_CONFIG_2 (Bank 2, 0x15)
        register Bank2AccelConfig2 {
            const ADDRESS = 0x15;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Accelerometer averaging configuration
            dec3_cfg: uint = 0..2,
            /// Z-axis accelerometer self-test enable
            az_st_en: bool = 2,
            /// Y-axis accelerometer self-test enable
            ay_st_en: bool = 3,
            /// X-axis accelerometer self-test enable
            ax_st_en: bool = 4,
            reserved_7_5: uint = 5..8,
        },

        /// FSYNC_CONFIG (Bank 2, 0x52)
        register Bank2FsyncConfig {
            const ADDRESS = 0x52;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// FSYNC configuration
            ext_sync_set: uint = 0..4,
            /// WoF edge select
            wof_edge_int: bool = 4,
            /// WoF deglitch enable
            wof_deglitch_en: bool = 5,
            reserved_6: uint = 6..7,
            /// Delay time enable
            delay_time_en: bool = 7,
        },

        /// TEMP_CONFIG (Bank 2, 0x53)
        register Bank2TempConfig {
            const ADDRESS = 0x53;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Temperature sensor DLPF configuration
            temp_dlpfcfg: uint = 0..3,
            reserved_7_3: uint = 3..8,
        },

        /// MOD_CTRL_USR (Bank 2, 0x54)
        register Bank2ModCtrlUsr {
            const ADDRESS = 0x54;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// DMP low power enable
            reg_lp_dmp_en: bool = 0,
            reserved_7_1: uint = 1..8,
        },

        // ==================== BANK 3 REGISTERS ====================
        // I2C master configuration for magnetometer access

        /// I2C_MST_ODR_CONFIG (Bank 3, 0x00)
        register Bank3I2cMstOdrConfig {
            const ADDRESS = 0x00;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C master ODR configuration
            i2c_mst_odr_config: uint = 0..4,
            reserved_7_4: uint = 4..8,
        },

        /// I2C_MST_CTRL (Bank 3, 0x01)
        register Bank3I2cMstCtrl {
            const ADDRESS = 0x01;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C master clock speed
            i2c_mst_clk: uint = 0..4,
            /// I2C master P_NSR
            i2c_mst_p_nsr: bool = 4,
            reserved_6_5: uint = 5..7,
            /// Multi-master enable
            mult_mst_en: bool = 7,
        },

        /// I2C_MST_DELAY_CTRL (Bank 3, 0x02)
        register Bank3I2cMstDelayCtrl {
            const ADDRESS = 0x02;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 0 delay enable
            i2c_slv0_delay_en: bool = 0,
            /// I2C slave 1 delay enable
            i2c_slv1_delay_en: bool = 1,
            /// I2C slave 2 delay enable
            i2c_slv2_delay_en: bool = 2,
            /// I2C slave 3 delay enable
            i2c_slv3_delay_en: bool = 3,
            /// I2C slave 4 delay enable
            i2c_slv4_delay_en: bool = 4,
            reserved_6_5: uint = 5..7,
            /// Delay ES shadow
            delay_es_shadow: bool = 7,
        },

        /// I2C_SLV0_ADDR (Bank 3, 0x03)
        register Bank3I2cSlv0Addr {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 0 physical address
            i2c_id_0: uint = 0..7,
            /// I2C slave 0 read/write (1=read, 0=write)
            i2c_slv0_rnw: bool = 7,
        },

        /// I2C_SLV0_REG (Bank 3, 0x04)
        register Bank3I2cSlv0Reg {
            const ADDRESS = 0x04;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 0 register address
            i2c_slv0_reg: uint = 0..8,
        },

        /// I2C_SLV0_CTRL (Bank 3, 0x05)
        register Bank3I2cSlv0Ctrl {
            const ADDRESS = 0x05;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 0 number of bytes to read/write
            i2c_slv0_leng: uint = 0..4,
            /// I2C slave 0 group
            i2c_slv0_grp: bool = 4,
            /// I2C slave 0 register disable
            i2c_slv0_reg_dis: bool = 5,
            /// I2C slave 0 byte swap
            i2c_slv0_byte_sw: bool = 6,
            /// I2C slave 0 enable
            i2c_slv0_en: bool = 7,
        },

        /// I2C_SLV0_DO (Bank 3, 0x06)
        register Bank3I2cSlv0Do {
            const ADDRESS = 0x06;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 0 data out
            i2c_slv0_do: uint = 0..8,
        },

        /// I2C_SLV1_ADDR (Bank 3, 0x07)
        register Bank3I2cSlv1Addr {
            const ADDRESS = 0x07;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 1 physical address
            i2c_id_1: uint = 0..7,
            /// I2C slave 1 read/write (1=read, 0=write)
            i2c_slv1_rnw: bool = 7,
        },

        /// I2C_SLV1_REG (Bank 3, 0x08)
        register Bank3I2cSlv1Reg {
            const ADDRESS = 0x08;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 1 register address
            i2c_slv1_reg: uint = 0..8,
        },

        /// I2C_SLV1_CTRL (Bank 3, 0x09)
        register Bank3I2cSlv1Ctrl {
            const ADDRESS = 0x09;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 1 number of bytes to read/write
            i2c_slv1_leng: uint = 0..4,
            /// I2C slave 1 group
            i2c_slv1_grp: bool = 4,
            /// I2C slave 1 register disable
            i2c_slv1_reg_dis: bool = 5,
            /// I2C slave 1 byte swap
            i2c_slv1_byte_sw: bool = 6,
            /// I2C slave 1 enable
            i2c_slv1_en: bool = 7,
        },

        /// I2C_SLV1_DO (Bank 3, 0x0A)
        register Bank3I2cSlv1Do {
            const ADDRESS = 0x0A;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 1 data out
            i2c_slv1_do: uint = 0..8,
        },

        /// I2C_SLV2_ADDR (Bank 3, 0x0B)
        register Bank3I2cSlv2Addr {
            const ADDRESS = 0x0B;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 2 physical address
            i2c_id_2: uint = 0..7,
            /// I2C slave 2 read/write (1=read, 0=write)
            i2c_slv2_rnw: bool = 7,
        },

        /// I2C_SLV2_REG (Bank 3, 0x0C)
        register Bank3I2cSlv2Reg {
            const ADDRESS = 0x0C;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 2 register address
            i2c_slv2_reg: uint = 0..8,
        },

        /// I2C_SLV2_CTRL (Bank 3, 0x0D)
        register Bank3I2cSlv2Ctrl {
            const ADDRESS = 0x0D;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 2 number of bytes to read/write
            i2c_slv2_leng: uint = 0..4,
            /// I2C slave 2 group
            i2c_slv2_grp: bool = 4,
            /// I2C slave 2 register disable
            i2c_slv2_reg_dis: bool = 5,
            /// I2C slave 2 byte swap
            i2c_slv2_byte_sw: bool = 6,
            /// I2C slave 2 enable
            i2c_slv2_en: bool = 7,
        },

        /// I2C_SLV2_DO (Bank 3, 0x0E)
        register Bank3I2cSlv2Do {
            const ADDRESS = 0x0E;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 2 data out
            i2c_slv2_do: uint = 0..8,
        },

        /// I2C_SLV3_ADDR (Bank 3, 0x0F)
        register Bank3I2cSlv3Addr {
            const ADDRESS = 0x0F;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 3 physical address
            i2c_id_3: uint = 0..7,
            /// I2C slave 3 read/write (1=read, 0=write)
            i2c_slv3_rnw: bool = 7,
        },

        /// I2C_SLV3_REG (Bank 3, 0x10)
        register Bank3I2cSlv3Reg {
            const ADDRESS = 0x10;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 3 register address
            i2c_slv3_reg: uint = 0..8,
        },

        /// I2C_SLV3_CTRL (Bank 3, 0x11)
        register Bank3I2cSlv3Ctrl {
            const ADDRESS = 0x11;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 3 number of bytes to read/write
            i2c_slv3_leng: uint = 0..4,
            /// I2C slave 3 group
            i2c_slv3_grp: bool = 4,
            /// I2C slave 3 register disable
            i2c_slv3_reg_dis: bool = 5,
            /// I2C slave 3 byte swap
            i2c_slv3_byte_sw: bool = 6,
            /// I2C slave 3 enable
            i2c_slv3_en: bool = 7,
        },

        /// I2C_SLV3_DO (Bank 3, 0x12)
        register Bank3I2cSlv3Do {
            const ADDRESS = 0x12;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 3 data out
            i2c_slv3_do: uint = 0..8,
        },

        /// I2C_SLV4_ADDR (Bank 3, 0x13)
        register Bank3I2cSlv4Addr {
            const ADDRESS = 0x13;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 4 physical address
            i2c_id_4: uint = 0..7,
            /// I2C slave 4 read/write (1=read, 0=write)
            i2c_slv4_rnw: bool = 7,
        },

        /// I2C_SLV4_REG (Bank 3, 0x14)
        register Bank3I2cSlv4Reg {
            const ADDRESS = 0x14;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 4 register address
            i2c_slv4_reg: uint = 0..8,
        },

        /// I2C_SLV4_CTRL (Bank 3, 0x15)
        register Bank3I2cSlv4Ctrl {
            const ADDRESS = 0x15;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 4 delay
            i2c_mst_dly: uint = 0..5,
            /// I2C slave 4 register disable
            i2c_slv4_reg_dis: bool = 5,
            /// I2C slave 4 interrupt enable
            i2c_slv4_int_en: bool = 6,
            /// I2C slave 4 enable
            i2c_slv4_en: bool = 7,
        },

        /// I2C_SLV4_DO (Bank 3, 0x16)
        register Bank3I2cSlv4Do {
            const ADDRESS = 0x16;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 4 data out
            i2c_slv4_do: uint = 0..8,
        },

        /// I2C_SLV4_DI (Bank 3, 0x17)
        register Bank3I2cSlv4Di {
            const ADDRESS = 0x17;
            const SIZE_BITS = 8;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// I2C slave 4 data in
            i2c_slv4_di: uint = 0..8,
        }
    }
);

// Re-export commonly used types for convenience
pub use Icm20948 as RegisterDevice;

/// AK09916 Magnetometer register definitions
///
/// The AK09916 is accessed via the ICM-20948's auxiliary I2C interface.
/// It has its own separate address space and uses little-endian byte order.
pub mod magnetometer {
    device_driver::create_device!(
        device_name: Ak09916,
        dsl: {
            config {
                type RegisterAddressType = u8;
                type DefaultByteOrder = LE;
            }

            /// WIA2 - Device ID (AK09916, 0x01)
            /// Expected value: 0x09
            register Wia2 {
                const ADDRESS = 0x01;
                const SIZE_BITS = 8;

                /// Device ID (should read 0x09)
                device_id: uint = 0..8,
            },

            /// ST1 - Status 1 (AK09916, 0x10)
            register St1 {
                const ADDRESS = 0x10;
                const SIZE_BITS = 8;

                /// Data ready
                drdy: bool = 0,
                /// Data overrun
                dor: bool = 1,
                reserved_7_2: uint = 2..8,
            },

            /// HXL - X-axis Magnetic Field Low Byte (AK09916, 0x11)
            register Hxl {
                const ADDRESS = 0x11;
                const SIZE_BITS = 8;

                /// X-axis magnetic field data low byte
                hxl: uint = 0..8,
            },

            /// HXH - X-axis Magnetic Field High Byte (AK09916, 0x12)
            register Hxh {
                const ADDRESS = 0x12;
                const SIZE_BITS = 8;

                /// X-axis magnetic field data high byte
                hxh: uint = 0..8,
            },

            /// HYL - Y-axis Magnetic Field Low Byte (AK09916, 0x13)
            register Hyl {
                const ADDRESS = 0x13;
                const SIZE_BITS = 8;

                /// Y-axis magnetic field data low byte
                hyl: uint = 0..8,
            },

            /// HYH - Y-axis Magnetic Field High Byte (AK09916, 0x14)
            register Hyh {
                const ADDRESS = 0x14;
                const SIZE_BITS = 8;

                /// Y-axis magnetic field data high byte
                hyh: uint = 0..8,
            },

            /// HZL - Z-axis Magnetic Field Low Byte (AK09916, 0x15)
            register Hzl {
                const ADDRESS = 0x15;
                const SIZE_BITS = 8;

                /// Z-axis magnetic field data low byte
                hzl: uint = 0..8,
            },

            /// HZH - Z-axis Magnetic Field High Byte (AK09916, 0x16)
            register Hzh {
                const ADDRESS = 0x16;
                const SIZE_BITS = 8;

                /// Z-axis magnetic field data high byte
                hzh: uint = 0..8,
            },

            /// ST2 - Status 2 (AK09916, 0x18)
            register St2 {
                const ADDRESS = 0x18;
                const SIZE_BITS = 8;

                reserved_2_0: uint = 0..3,
                /// Magnetic sensor overflow
                hofl: bool = 3,
                reserved_7_4: uint = 4..8,
            },

            /// CNTL2 - Control 2 (AK09916, 0x31)
            register Cntl2 {
                const ADDRESS = 0x31;
                const SIZE_BITS = 8;

                /// Operation mode:
                /// 0x00 = Power-down
                /// 0x01 = Single measurement
                /// 0x02 = Continuous 10Hz
                /// 0x04 = Continuous 20Hz
                /// 0x06 = Continuous 50Hz
                /// 0x08 = Continuous 100Hz
                /// 0x10 = Self-test
                mode: uint = 0..5,
                reserved_7_5: uint = 5..8,
            },

            /// CNTL3 - Control 3 (AK09916, 0x32)
            register Cntl3 {
                const ADDRESS = 0x32;
                const SIZE_BITS = 8;

                /// Soft reset
                srst: bool = 0,
                reserved_7_1: uint = 1..8,
            }
        }
    );

    // Re-export magnetometer device
    pub use Ak09916 as MagnetometerDevice;
}
