//! Wake-on-Motion Example for ICM-20948 on Raspberry Pi Pico 2 (Blocking version)
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
use embassy_time::{Delay, Duration};
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
    info!("Starting Wake-on-Motion test (RP2350 blocking)...");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400kHz
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_13, p.PIN_12, i2c_config);

    let mut imu_int1 = Input::new(p.PIN_14, Pull::Down);

    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface) {
        Ok(imu) => imu,
        Err(e) => {
            error!("IMU initialization failed: {:?}", e);
            loop {
                embassy_time::block_for(Duration::from_millis(1000));
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay) {
        error!("Failed to initialize ICM-20948: {:?}", e);
        loop {
            embassy_time::block_for(Duration::from_millis(1000));
        }
    }

    embassy_time::block_for(Duration::from_millis(50));

    let int_pin_cfg = InterruptPinConfig {
        active_low: false,
        open_drain: false,
        latch_enabled: true,
        clear_on_any_read: false,
    };
    imu.configure_interrupt_pin(&int_pin_cfg).unwrap();

    let int_cfg = InterruptConfig::wake_on_motion_only();
    imu.configure_interrupts(&int_cfg).unwrap();

    let lp_config = LowPowerConfig {
        accel_rate: LowPowerRate::Hz31_25,
        enable_wake_on_motion: true,
        wom_threshold: 50, // 50 * 4mg = 200mg trigger threshold
        wom_mode: WomMode::CompareCurrentSample,
    };
    imu.enter_low_power_mode(&lp_config, &mut delay).unwrap();

    let _ = imu.read_interrupt_status();
    info!("IMU and MCU ready. Waiting for motion on INT1 (GPIO14)...");

    let mut motion_count = 0u32;

    loop {
        imu_int1.wait_for_high().await;
        let status = imu.read_interrupt_status().unwrap();

        if status.wake_on_motion {
            motion_count = motion_count.wrapping_add(1);
            info!("Motion detected! (Wakeup count: {})", motion_count);

            if let Ok(accel) = imu.read_accelerometer_raw() {
                let dx = (accel.0.unsigned_abs() as u32) % 16384;
                let dy = (accel.1.unsigned_abs() as u32) % 16384;
                let dz = (accel.2.unsigned_abs() as u32) % 16384;

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
