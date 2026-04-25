//! Wake-on-Motion Example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates enabling Wake-on-Motion (WoM) and using the INT1
//! pin to wake the MCU on motion events.
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
    I2cInterface, Icm20948Driver, InterruptConfig, InterruptPinConfig,
    power::{LowPowerConfig, LowPowerRate, WomMode},
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
    info!("Starting Wake-on-Motion test (RP2350 async)...");

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

    let int_pin_cfg = InterruptPinConfig {
        active_low: false,
        open_drain: false,
        latch_enabled: true,
        clear_on_any_read: false,
    };
    imu.configure_interrupt_pin(&int_pin_cfg).await.unwrap();

    let int_cfg = InterruptConfig::wake_on_motion_only();
    imu.configure_interrupts(&int_cfg).await.unwrap();

    let lp_config = LowPowerConfig {
        accel_rate: LowPowerRate::Hz31_25,
        enable_wake_on_motion: true,
        wom_threshold: 50, // 50 * 4mg = 200mg trigger threshold
        wom_mode: WomMode::CompareCurrentSample,
    };
    imu.enter_low_power_mode(&lp_config, &mut delay)
        .await
        .unwrap();

    let _ = imu.read_interrupt_status().await;
    info!("IMU and MCU ready. Waiting for motion on INT1 (GPIO14)...");

    let mut motion_count = 0u32;

    loop {
        imu_int1.wait_for_high().await;
        let status = imu.read_interrupt_status().await.unwrap();

        if status.wake_on_motion {
            motion_count = motion_count.wrapping_add(1);
            info!("Motion detected! (Wakeup count: {})", motion_count);

            if let Ok(accel) = imu.read_accelerometer_raw().await {
                let dx = (accel.x.unsigned_abs() as u32) % 16384;
                let dy = (accel.y.unsigned_abs() as u32) % 16384;
                let dz = (accel.z.unsigned_abs() as u32) % 16384;

                let axis = if dx > dy && dx > dz {
                    "X-axis"
                } else if dy > dx && dy > dz {
                    "Y-axis"
                } else if dz > dx && dz > dy {
                    "Z-axis"
                } else {
                    "Unknown"
                };

                info!(
                    "Inferred trigger direction: {} (ΔX:{}, ΔY:{}, ΔZ:{})",
                    axis, dx, dy, dz
                );
            }

            info!("Returning to low-power wait...");
        }
    }
}
