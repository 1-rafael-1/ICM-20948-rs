//! AHRS Euler Angles Example with ICM-20948
//!
//! This example demonstrates sensor fusion using the AHRS crate with Madgwick filtering
//! to compute Euler angles (roll, pitch, yaw) for orientation tracking.
//!
//! Features:
//! - Madgwick filter with beta=0.1 (good vibration/noise rejection)
//! - 100 Hz update rate for smooth orientation tracking
//! - 9-axis sensor fusion with tilt-compensated compass heading
//! - Rotation detection (yaw changes)
//! - EMA filtering for smooth output
//! - Clean, condensed output format
//!
//! Hardware connections (I2C0):
//! - SDA: GPIO12
//! - SCL: GPIO13
//! - VCC: 3.3V
//! - GND: GND
//! - AD0: GND (for address 0x68)

#![no_std]
#![no_main]

use ahrs::{Ahrs, Madgwick};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    block::ImageDef,
    config::Config,
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::I2C0,
};
use embassy_time::{Delay, Duration, Ticker};
use icm20948::{
    AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale, I2cInterface,
    Icm20948Driver, MagConfig, MagMode,
};
use nalgebra::{UnitQuaternion, Vector3};
use panic_probe as _;

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Bind I2C interrupts
bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

/// Simple exponential moving average filter for smoothing sensor data
struct EmaFilter {
    value: f32,
    alpha: f32, // Smoothing factor (0-1, higher = less smoothing)
}

impl EmaFilter {
    fn new(alpha: f32) -> Self {
        Self {
            value: 0.0,
            alpha: alpha.clamp(0.0, 1.0),
        }
    }

    fn update(&mut self, new_value: f32) -> f32 {
        self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value;
        self.value
    }
}

/// Euler angles in degrees
#[derive(Debug, Clone, Copy)]
struct EulerAngles {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

impl EulerAngles {
    /// Convert nalgebra UnitQuaternion to Euler angles (in degrees)
    fn from_quaternion(q: &UnitQuaternion<f32>) -> Self {
        // Extract quaternion components (w, x, y, z)
        let quat = q.quaternion();
        let w = quat.w;
        let x = quat.i;
        let y = quat.j;
        let z = quat.k;

        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = libm::atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (w * y - z * x);
        let pitch = if libm::fabsf(sinp) >= 1.0 {
            libm::copysignf(core::f32::consts::FRAC_PI_2, sinp) // Use 90 degrees if out of range
        } else {
            libm::asinf(sinp)
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        let yaw = libm::atan2f(siny_cosp, cosy_cosp);

        // Convert to degrees
        Self {
            roll: roll * 180.0 / core::f32::consts::PI,
            pitch: pitch * 180.0 / core::f32::consts::PI,
            yaw: yaw * 180.0 / core::f32::consts::PI,
        }
    }

    /// Normalize yaw to 0-360 degrees
    fn normalize_yaw(&mut self) {
        while self.yaw < 0.0 {
            self.yaw += 360.0;
        }
        while self.yaw >= 360.0 {
            self.yaw -= 360.0;
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("ICM-20948 AHRS Euler Angles for Tracked Robot (Async)");

    let p = embassy_rp::init(Config::default());

    // Configure I2C with 400kHz frequency
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);

    info!("Initializing ICM-20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut imu = match Icm20948Driver::new(i2c_interface).await {
        Ok(imu) => {
            info!("ICM-20948 detected!");
            imu
        }
        Err(e) => {
            error!("Failed to detect ICM-20948: {:?}", e);
            loop {
                embassy_time::Timer::after_millis(1000).await;
            }
        }
    };

    // Initialize the device
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay).await {
        error!("Failed to initialize ICM-20948: {:?}", e);
        loop {
            embassy_time::Timer::after_millis(1000).await;
        }
    }

    embassy_time::Timer::after_millis(100).await;
    info!("ICM-20948 initialized successfully!");

    // Configure accelerometer: ±4g range (for handling bumps/shocks), 111 Hz DLPF
    // Higher range and lower DLPF to filter vibrations
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G4,
        dlpf: AccelDlpf::Hz111, // Good vibration filtering
        dlpf_enable: true,
        sample_rate_div: 19, // ~55 Hz sample rate (1125 Hz / (1 + 19))
    };

    if let Err(e) = imu.configure_accelerometer(accel_config).await {
        error!("Failed to configure accelerometer: {:?}", e);
    } else {
        info!("Accelerometer: ±4g, 111Hz DLPF, ~55Hz sample rate");
    }

    // Configure gyroscope: ±500°/s range (for fast movements), 51 Hz DLPF
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps500, // Captures fast rotations
        dlpf: GyroDlpf::Hz51,              // Good vibration filtering
        dlpf_enable: true,
        sample_rate_div: 19, // ~55 Hz sample rate (1125 Hz / (1 + 19))
    };

    if let Err(e) = imu.configure_gyroscope(gyro_config).await {
        error!("Failed to configure gyroscope: {:?}", e);
    } else {
        info!("Gyroscope: ±500°/s, 51Hz DLPF, ~55Hz sample rate");
    }

    // Configure magnetometer: Continuous 100Hz mode for compass heading
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    if let Err(e) = imu.init_magnetometer(mag_config, &mut delay).await {
        error!("Failed to initialize magnetometer: {:?}", e);
    } else {
        info!("Magnetometer: Continuous 100Hz");
    }

    embassy_time::Timer::after_millis(100).await;
    info!("All sensors configured!");
    info!("");

    // Initialize Madgwick filter
    // Beta = 0.1 is good for noisy environments (brushed motors create vibrations)
    // Higher beta = faster convergence but more noise sensitivity
    // Lower beta = slower convergence but better noise rejection
    const SAMPLE_RATE_HZ: f32 = 100.0;
    const BETA: f32 = 0.1; // Good for filtering motor vibrations
    let mut madgwick = Madgwick::new(1.0 / SAMPLE_RATE_HZ, BETA);

    info!("Madgwick filter initialized:");
    info!("  Sample rate: {} Hz", SAMPLE_RATE_HZ);
    info!("  Beta: {} (vibration-resistant)", BETA);
    info!("");

    // Create EMA filters for additional smoothing (helps with vibration noise)
    let mut roll_filter = EmaFilter::new(0.3); // More aggressive smoothing
    let mut pitch_filter = EmaFilter::new(0.3);
    let mut yaw_filter = EmaFilter::new(0.5); // Less smoothing for responsive heading

    info!("Calibrating... Keep device stationary for 3 seconds!");
    embassy_time::Timer::after_millis(3000).await;

    info!("Starting AHRS processing at {} Hz", SAMPLE_RATE_HZ);
    info!("");

    let mut sample_count: u32 = 0;
    let mut ticker = Ticker::every(Duration::from_hz(SAMPLE_RATE_HZ as u64));

    // Gyro bias estimation (simple calibration)
    let mut gyro_bias_x = 0.0f32;
    let mut gyro_bias_y = 0.0f32;
    let mut gyro_bias_z = 0.0f32;
    let mut calibration_samples = 0;
    const CALIBRATION_COUNT: u32 = 50;

    loop {
        ticker.next().await;
        sample_count += 1;

        // Read accelerometer (in g) - convert to Vector3
        let accel = match imu.read_accelerometer().await {
            Ok(data) => Vector3::new(data.x, data.y, data.z),
            Err(_) => continue,
        };

        // Read gyroscope (in degrees/second) - convert to Vector3
        let gyro_raw = match imu.read_gyroscope().await {
            Ok(data) => Vector3::new(data.x, data.y, data.z),
            Err(_) => continue,
        };

        // Read magnetometer (in μT) - convert to Vector3
        let mag = match imu.read_magnetometer().await {
            Ok(data) => Vector3::new(data.x, data.y, data.z),
            Err(_) => continue,
        };

        // Calibration phase: collect gyro bias
        if calibration_samples < CALIBRATION_COUNT {
            gyro_bias_x += gyro_raw[0];
            gyro_bias_y += gyro_raw[1];
            gyro_bias_z += gyro_raw[2];
            calibration_samples += 1;

            if calibration_samples == CALIBRATION_COUNT {
                gyro_bias_x /= CALIBRATION_COUNT as f32;
                gyro_bias_y /= CALIBRATION_COUNT as f32;
                gyro_bias_z /= CALIBRATION_COUNT as f32;
                info!("Calibration complete!");
                info!(
                    "Gyro bias: X={} Y={} Z={}",
                    gyro_bias_x, gyro_bias_y, gyro_bias_z
                );
                info!("");
            }
            continue;
        }

        // Apply gyro bias correction
        let gyro = Vector3::new(
            gyro_raw[0] - gyro_bias_x,
            gyro_raw[1] - gyro_bias_y,
            gyro_raw[2] - gyro_bias_z,
        );

        // Convert gyroscope from degrees/s to radians/s for AHRS
        let gyro_rad = Vector3::new(
            gyro[0] * core::f32::consts::PI / 180.0,
            gyro[1] * core::f32::consts::PI / 180.0,
            gyro[2] * core::f32::consts::PI / 180.0,
        );

        // Update Madgwick filter with 9-axis data
        // Note: AHRS crate expects (gyro, accel, mag) in radians/s, g, and μT
        // Try to update with magnetometer for absolute heading if available and reasonable
        let mag_magnitude = mag.norm();
        let result = if mag_magnitude > 10.0 && mag_magnitude < 200.0 {
            // Magnetometer reading seems valid - use 9-axis fusion
            madgwick.update(&gyro_rad, &accel, &mag)
        } else {
            // Magnetometer not available or unreliable - use 6-axis IMU only
            madgwick.update_imu(&gyro_rad, &accel)
        };

        if let Ok(quat) = result {
            // Convert quaternion to Euler angles
            let mut euler = EulerAngles::from_quaternion(quat);
            euler.normalize_yaw();

            // Apply EMA filtering for smoother output (reduces jitter from motors)
            let roll = roll_filter.update(euler.roll);
            let pitch = pitch_filter.update(euler.pitch);
            let yaw = yaw_filter.update(euler.yaw);

            // Sensor magnitudes for display
            let accel_magnitude = accel.norm();
            let gyro_magnitude = gyro.norm();

            // ROTATION DETECTION:
            //
            // Problem: When you tilt the device, gyro sees rotation on X/Y axes
            // This shouldn't be classified as "TURNING" - that should be yaw rotation only
            //
            // Solution: Check only Z-axis gyro (yaw rotation)
            // - Tilting forward/back: gyro X changes (pitch)
            // - Tilting left/right: gyro Y changes (roll)
            // - Spinning/rotating: gyro Z changes (yaw) ← This is "TURNING"

            // Rotation detected = rotation around Z axis (yaw changes)
            // Threshold 3°/s catches deliberate rotation while ignoring drift and small movements
            let is_rotating = gyro[2].abs() > 3.0;

            // Cardinal direction
            let cardinal = if !(22.5..337.5).contains(&yaw) {
                "N "
            } else if (22.5..67.5).contains(&yaw) {
                "NE"
            } else if (67.5..112.5).contains(&yaw) {
                "E "
            } else if (112.5..157.5).contains(&yaw) {
                "SE"
            } else if (157.5..202.5).contains(&yaw) {
                "S "
            } else if (202.5..247.5).contains(&yaw) {
                "SW"
            } else if (247.5..292.5).contains(&yaw) {
                "W "
            } else {
                "NW"
            };

            // Status: show rotation detection
            let status = if is_rotating { "ROTATING" } else { "STEADY  " };

            // Tilt warning
            let tilt = if roll.abs() > 15.0 || pitch.abs() > 15.0 {
                "TILT!"
            } else {
                ""
            };

            // Output every 10th sample (5 Hz display rate) - clean and readable
            if sample_count.is_multiple_of(10) {
                info!("");
                info!("STATUS: {} {}", status, tilt);
                info!("Orient: R={} P={} Y={} ({})", roll, pitch, yaw, cardinal);
                info!(
                    "Motion: Accel={} Gyro={} Mag={}",
                    accel_magnitude, gyro_magnitude, mag_magnitude
                );
            }
        } else {
            // Filter convergence issue
            if sample_count.is_multiple_of(50) {
                warn!("AHRS filter convergence issue at sample {}", sample_count);
            }
        }
    }
}
