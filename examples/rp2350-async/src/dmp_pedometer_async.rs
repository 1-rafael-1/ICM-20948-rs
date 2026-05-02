//! Pedometer Example for ICM-20948 on Raspberry Pi Pico 2 (Async version)
//!
//! This example demonstrates the ICM-20948 DMP pedometer in async mode. It enables
//! pedometer-fused 6-axis quaternion output (`PQuat6`) and step-event detection.
//! On each interrupt, orientation data is read via `dmp_read_fifo()` and, when a
//! step event is present, the cumulative step count is retrieved via a separate
//! `dmp_read_step_count()` SRAM transaction (the two-call pattern).
//!
//! ## Two Outputs
//!
//! 1. **`pedometer_quaternion` (PQuat6)** — A 6-axis (accel + gyro) quaternion computed
//!    by the DMP's pedestrian-optimised fusion algorithm. Unlike the standard 6-axis
//!    quaternion, its output rate is cadence-locked: the DMP only emits orientation
//!    packets while pedestrian motion is active. During quiet periods you still receive
//!    FIFO interrupts at 56 Hz, but `pedometer_quaternion` will be `None`.
//!
//! 2. **`pedometer_timestamp` / step count (two-call pattern)** — When the DMP detects
//!    a footfall it stamps the FIFO packet with an internal cycle counter value
//!    (`pedometer_timestamp`). On the same event, a second call to
//!    `dmp_read_step_count()` reads the cumulative step total directly from DMP SRAM.
//!    The two calls together give you both *when* the step happened (relative to other
//!    steps) and *how many* steps have occurred in total.
//!
//! ## Hardware Connections (I2C0)
//!
//! | Signal | GPIO  |
//! |--------|-------|
//! | SDA    | 12    |
//! | SCL    | 13    |
//! | INT1   | 14    |
//! | VCC    | 3.3V  |
//! | GND    | GND   |
//! | AD0    | GND → I2C address 0x68 |

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
use icm20948::{dmp::DmpConfig, I2cInterface, Icm20948Driver, InterruptConfig, InterruptPinConfig};
use panic_probe as _;

/// Firmware image type required by the RP2350 bootloader.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Bind the I2C0 hardware interrupt to the Embassy async driver.
bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting pedometer example (RP2350 async)...");

    let p = embassy_rp::init(Config::default());

    // Configure I2C at 400 kHz (fast mode).
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    // INT1 pin: active-high interrupt from the ICM-20948.
    let mut imu_int = Input::new(p.PIN_14, Pull::Down);

    // Initialise the ICM-20948 driver.
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::try_new(i2c_interface).await {
        Ok(imu) => imu,
        Err(e) => {
            error!("IMU initialisation failed: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("Failed to initialise ICM-20948: {:?}", e);
        loop {
            Timer::after_millis(1000).await;
        }
    }

    // Brief settling time after init.
    Timer::after_millis(50).await;

    // Configure INT1 pin: active-high, push-pull, latched, cleared on any read.
    let int_pin_cfg = InterruptPinConfig {
        active_low: false,
        open_drain: false,
        latch_enabled: true,
        clear_on_any_read: true,
    };
    imu.configure_interrupt_pin(&int_pin_cfg).await.unwrap();

    // Enable DMP interrupt (plus FIFO overflow so we can detect and recover from it).
    let mut int_cfg = InterruptConfig::fifo_batch();
    int_cfg.dmp = true;
    imu.configure_interrupts(&int_cfg).await.unwrap();

    info!("Loading DMP firmware and configuring for pedometer mode...");

    // Pedometer-fused 6-axis mode does NOT need magnetometer initialisation.
    imu.dmp_init(&mut delay).await.unwrap();

    // -------------------------------------------------------------------------
    // DmpConfig notes:
    //
    // * `pedometer_six_axis()` enables PQuat6 (FIFO output of the pedometer
    //   orientation fusion algorithm).
    // * `with_step_detector()` adds step-event timestamps to FIFO packets so we
    //   know exactly which packet corresponds to a footfall.
    // * 56 Hz is one of the three validated rates for the DMP pedestrian algorithm
    //   (the others are 112 Hz and 225 Hz). The calibration constants written during
    //   dmp_configure() — ACCEL_ONLY_GAIN, ACCEL_ALPHA_VAR, ACCEL_A_VAR — are
    //   derived for these exact rates. Using a different rate requires re-deriving
    //   those constants; step detection accuracy degrades otherwise.
    // -------------------------------------------------------------------------
    let dmp_config = DmpConfig::pedometer_six_axis()
        .with_step_detector()
        .with_sample_rate(56);

    imu.dmp_configure(&dmp_config).await.unwrap();
    // Reset the FIFO *before* enabling the DMP so that any data accumulated
    // during configuration is discarded. This matches the InvenSense eMD SDK
    // and SparkFun reference order.
    imu.reset_fifo().await.unwrap();
    imu.dmp_enable(true).await.unwrap();

    info!("Entering pedometer read loop...");

    // -------------------------------------------------------------------------
    // Key concepts for interpreting pedometer output:
    //
    // `pedometer_timestamp`  is a DMP *internal cycle counter*, NOT wall-clock
    //   time. Consecutive timestamps are useful for cadence calculation:
    //     cadence_hz ≈ DMP_RATE / (ts_current - ts_previous)
    //   Do NOT assume any fixed relationship to real seconds.
    //
    // `dmp_read_step_count()` reads cumulative steps from DMP SRAM via a
    //   dedicated I2C transaction (separate from the FIFO read). Only call it
    //   when `pedometer_timestamp` is `Some`, i.e. when a step was actually
    //   detected, to avoid unnecessary bus traffic between steps.
    //
    // PQuat6 (`pedometer_quaternion`) is cadence-locked: the DMP suppresses
    //   orientation output when pedestrian motion is absent. You will receive
    //   FIFO interrupts at 56 Hz regardless, but `pedometer_quaternion` will
    //   be `None` while the device is stationary.
    // -------------------------------------------------------------------------

    let mut packet_count = 0u32;

    loop {
        // Wait for INT1 to go high (DMP FIFO watermark / step event).
        imu_int.wait_for_high().await;

        // Reading interrupt status clears the latched INT1 pin.
        let _status = imu.read_interrupt_status().await.unwrap();

        if let Some(packet) = imu.dmp_read_fifo().await.unwrap() {
            packet_count = packet_count.wrapping_add(1);

            // --- PQuat6: pedestrian-fused orientation ---
            //
            // Present in FIFO packets only while the pedestrian algorithm detects
            // motion. Convert the quaternion to Euler angles for human-readable logging.
            if let Some(quat) = packet.pedometer_quaternion {
                let euler = quat.to_euler_angles();
                let (roll, pitch, mut yaw) = euler.to_degrees();

                // Normalise yaw to [0°, 360°).
                if yaw < 0.0 {
                    yaw += 360.0;
                }

                info!(
                    "Orientation (PQuat6) — Roll: {}° Pitch: {}° Yaw: {}°",
                    roll, pitch, yaw
                );
            }

            // --- Step event: DMP cycle timestamp + cumulative step count ---
            //
            // `pedometer_timestamp` is only `Some` when the DMP stamped this
            // packet with a step event. Read the step counter only in that case
            // to avoid an extra I2C transaction on every non-step packet.
            if let Some(ts) = packet.pedometer_timestamp {
                match imu.dmp_read_step_count().await {
                    Ok(steps) => {
                        info!(
                            "Step detected! DMP cycle timestamp: {} | Total steps: {}",
                            ts, steps
                        );
                    }
                    Err(e) => {
                        error!("Step count read failed: {:?}", e);
                    }
                }
            }

            // Periodic heartbeat so there is visible output even when stationary.
            if packet_count.is_multiple_of(50) {
                info!("[pedometer] Packets received: {}", packet_count);
            }
        }
    }
}
