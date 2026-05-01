//! DMP Integration Example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates loading the DMP firmware, configuring quaternion output,
//! and reading DMP FIFO packets using the INT1 pin for interrupts.
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
use embassy_time::{Delay, Instant, Timer};
use icm20948::{dmp::DmpConfig, I2cInterface, Icm20948Driver, InterruptConfig, InterruptPinConfig};
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
    info!("Starting DMP integration example (RP2350 async)...");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400kHz
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    let mut imu_int = Input::new(p.PIN_14, Pull::Down);

    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::try_new(i2c_interface).await {
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

    let int_pin_cfg = InterruptPinConfig {
        active_low: false,
        open_drain: false,
        latch_enabled: true,
        clear_on_any_read: true,
    };
    imu.configure_interrupt_pin(&int_pin_cfg).await.unwrap();

    let mut int_cfg = InterruptConfig::fifo_batch();
    int_cfg.dmp = true;
    imu.configure_interrupts(&int_cfg).await.unwrap();

    info!("Loading DMP firmware and configuring...");
    imu.dmp_init(&mut delay).await.unwrap();
    imu.dmp_init_magnetometer(&mut delay).await.unwrap();

    let dmp_sample_rate_hz: u16 = 225;

    let dmp_config = DmpConfig::new()
        .with_quaternion_9axis(true)
        .with_host_calibrated_accel(true)
        .with_raw_accel(true)
        .with_sample_rate(dmp_sample_rate_hz);

    imu.dmp_configure(&dmp_config).await.unwrap();
    imu.dmp_enable(true).await.unwrap();
    imu.reset_fifo().await.unwrap();

    info!("Entering DMP read loop...");

    let mut sample_count = 0u32;
    let mut last_print_time = Instant::now();

    let mut vel_x = 0.0f32;
    let mut vel_y = 0.0f32;
    let mut pos_x = 0.0f32;
    let mut pos_y = 0.0f32;

    let dt = 1.0 / dmp_sample_rate_hz as f32;

    loop {
        imu_int.wait_for_high().await;

        let _status = imu.read_interrupt_status().await.unwrap();

        if let Some(packet) = imu.dmp_read_fifo().await.unwrap() {
            sample_count = sample_count.wrapping_add(1);

            let quat_opt = packet.quaternion_9axis.or(packet.quaternion_6axis);

            if let Some(quat) = quat_opt {
                let euler = quat.to_euler_angles();
                let (roll, pitch, mut yaw) = euler.to_degrees();

                if yaw < 0.0 {
                    yaw += 360.0;
                }

                if let Some(accel) = packet.host_calibrated_accel {
                    let ax_g = accel.0 as f32 / 8192.0;
                    let ay_g = accel.1 as f32 / 8192.0;

                    let gravity_x = 2.0 * (quat.x * quat.z - quat.w * quat.y);
                    let gravity_y = 2.0 * (quat.w * quat.x + quat.y * quat.z);

                    let lin_acc_x = (ax_g - gravity_x) * 9.81;
                    let lin_acc_y = (ay_g - gravity_y) * 9.81;

                    let clean_acc_x = if lin_acc_x.abs() > 0.2 {
                        lin_acc_x
                    } else {
                        0.0
                    };
                    let clean_acc_y = if lin_acc_y.abs() > 0.2 {
                        lin_acc_y
                    } else {
                        0.0
                    };

                    vel_x += clean_acc_x * dt;
                    vel_y += clean_acc_y * dt;
                    pos_x += vel_x * dt;
                    pos_y += vel_y * dt;

                    vel_x *= 0.95;
                    vel_y *= 0.95;
                }

                if sample_count.is_multiple_of(100) {
                    let now = Instant::now();
                    let diff_micros = (now - last_print_time).as_micros() as f32;
                    let fps = if diff_micros > 0.0 {
                        10.0 / (diff_micros / 1_000_000.0)
                    } else {
                        0.0
                    };
                    last_print_time = now;

                    info!(
                        "⏱️ {}Hz | 📐 Roll: {}°, Pitch: {}° | 🧭 Yaw: {}° | 🏃 PosX: {}m, PosY: {}m",
                        fps, roll, pitch, yaw, pos_x, pos_y
                    );
                }
            }
        }
    }
}
