//! Hardware Interrupt (Data Ready) Example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates using the INT1 pin to trigger data-ready interrupts
//! and reading accelerometer + gyroscope data on each interrupt.
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
//! - GND: GND
//! - AD0: GND (for address 0x68)
//! - INT1: GPIO14

#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    block::ImageDef,
    config::Config,
    gpio::{Input, Pull},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::I2C0,
};
use embassy_time::{Delay, Timer};
use icm20948::{
    sensors::{AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale},
    I2cInterface, Icm20948Driver, InterruptConfig, InterruptPinConfig,
};
use panic_probe as _;

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Bind I2C interrupts
bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting Data-Ready interrupt example (RP2350 async)...");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400kHz
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    let mut imu_int1 = Input::new(p.PIN_14, Pull::Down);

    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface).await {
        Ok(imu) => imu,
        Err(e) => {
            error!("IMU initialization failed: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("Failed to initialize ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    Timer::after_millis(50).await;

    // Configure sensors for ~100Hz
    let accel_cfg = AccelConfig {
        full_scale: AccelFullScale::G2,
        sample_rate_div: 10,
        dlpf_enable: true,
        dlpf: AccelDlpf::Hz50,
    };
    imu.configure_accelerometer(accel_cfg).await.unwrap();

    let gyro_cfg = GyroConfig {
        full_scale: GyroFullScale::Dps250,
        sample_rate_div: 10,
        dlpf_enable: true,
        dlpf: GyroDlpf::Hz51,
    };
    imu.configure_gyroscope(gyro_cfg).await.unwrap();

    // Configure interrupt pin and Data-Ready interrupt
    let int_pin_cfg = InterruptPinConfig {
        active_low: false,
        open_drain: false,
        latch_enabled: true,
        clear_on_any_read: true,
    };
    imu.configure_interrupt_pin(&int_pin_cfg).await.unwrap();

    let int_cfg = InterruptConfig::data_ready_only();
    imu.configure_interrupts(&int_cfg).await.unwrap();

    let _ = imu.read_interrupt_status().await;
    info!("Interrupt configured! Waiting for data-ready events...");

    let mut counter = 0u32;

    loop {
        imu_int1.wait_for_high().await;

        if let (Ok(accel), Ok(gyro)) = (imu.read_accelerometer().await, imu.read_gyroscope().await)
        {
            counter = counter.wrapping_add(1);

            if counter.is_multiple_of(10) {
                info!(
                    "[10Hz Sample] Accel: [{}, {}, {}] g | Gyro: [{}, {}, {}] °/s",
                    accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
                );
            }
        }
    }
}
