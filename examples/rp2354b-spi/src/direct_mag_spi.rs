//! Direct AK09916 magnetometer read over SPI on the RP2354B board.
//!
//! This is the first hardware-verifiable SPI path on the custom board. It talks
//! to the magnetometer through the ICM-20948's I2C master only, without loading
//! or enabling the DMP, so a dead or stale path can be separated from a DMP
//! configuration problem.
//!
//! The log line is: `raw=(x,y,z) uT=(x,y,z) heading_deg=.. ST2=ok|overflow`.
//! Raw counts are converted locally at 0.15 uT/LSB; no extra bus reads.
//!
//! Set `SPI_CONTROL_HZ` instead of `SPI_DEFAULT_HZ` below to rerun the same
//! firmware at 1 MHz when triaging signal integrity on the level-shifted bus.

#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{block::ImageDef, config::Config};
use embassy_time::{Delay, Timer};
use icm20948::{MagConfig, MagMode};
use panic_probe as _;

mod board;

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// AK09916 sensitivity: 0.15 uT per LSB.
const MAG_UT_PER_LSB: f32 = 0.15;

/// Poll interval for the headless stream (about 10 Hz).
const POLL_INTERVAL_MS: u64 = 100;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("ICM-20948 direct magnetometer read over SPI (RP2354B)");

    let mut p = embassy_rp::init(Config::default());

    // Signal-integrity control: swap `SPI_DEFAULT_HZ` for `SPI_CONTROL_HZ`.
    let mut imu = board::init_imu_or_idle(&mut p, board::SPI_DEFAULT_HZ).await;

    let mut delay = Delay;
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    if imu.init_magnetometer(mag_config, &mut delay).await.is_err() {
        error!("AK09916 magnetometer initialization failed");
        board::idle_on_error("Check the ICM-20948's internal I2C master wiring and mag power")
            .await;
    }

    info!("AK09916 in continuous 100 Hz mode; streaming at ~10 Hz");
    Timer::after_millis(POLL_INTERVAL_MS).await;

    loop {
        match imu.read_magnetometer_raw().await {
            Ok((x, y, z)) => {
                let x_ut = f32::from(x) * MAG_UT_PER_LSB;
                let y_ut = f32::from(y) * MAG_UT_PER_LSB;
                let z_ut = f32::from(z) * MAG_UT_PER_LSB;

                let mut heading_deg = libm::atan2f(y_ut, x_ut) * (180.0 / core::f32::consts::PI);
                if heading_deg < 0.0 {
                    heading_deg += 360.0;
                }

                info!(
                    "raw=({},{},{}) uT=({},{},{}) heading_deg={} ST2=ok",
                    x, y, z, x_ut, y_ut, z_ut, heading_deg
                );
            }
            Err(icm20948::Error::Magnetometer) => {
                // Either ST2 magnetic overflow or the magnetometer was never
                // initialized; a successful init above leaves overflow.
                error!("raw=(-,-,-) uT=(-,-,-) heading_deg=0 ST2=overflow");
            }
            Err(_) => {
                error!("Magnetometer read failed on the SPI bus; check wiring and clock");
            }
        }

        Timer::after_millis(POLL_INTERVAL_MS).await;
    }
}
