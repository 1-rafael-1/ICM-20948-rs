#![no_std]
#![no_main]

use defmt::info;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    spi::{Config as SpiConfig, Spi},
};
use embassy_time::{Delay, Instant};
use embedded_hal_bus::spi::ExclusiveDevice;
use icm20948::{Icm20948Driver, InterruptConfig, InterruptPinConfig, SpiInterface, dmp::DmpConfig};

bind_interrupts!(struct SpiIrqs {
    DMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
    DMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH4>;
    EXTI3 =>  embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Starting Data-Ready interrupt data collection test...");

    let mut spi_config = SpiConfig::default();
    // spi_config.frequency = Hertz(5_000_000);
    spi_config.mode = embassy_stm32::spi::MODE_0;

    let spi_bus = Spi::new(
        p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH5, p.DMA1_CH4, SpiIrqs, spi_config,
    );
    let cs_pin = Output::new(p.PA2, Level::High, Speed::VeryHigh);
    let spi_device = ExclusiveDevice::new(spi_bus, cs_pin, Delay).unwrap();

    let mut imu_int = ExtiInput::new(p.PA3, p.EXTI3, Pull::Down, SpiIrqs);

    let mut imu = Icm20948Driver::new(SpiInterface::new(spi_device))
        .await
        .unwrap();

    imu.init(&mut Delay).await.unwrap();

    imu.enable_spi_mode().await.unwrap();

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

    info!("Loading DMP Firmware and configuring...");
    imu.dmp_init(&mut Delay).await.unwrap();
    imu.dmp_init_magnetometer(&mut Delay).await.unwrap();

    let dmp_config = DmpConfig::new()
        .with_quaternion_9axis(true)
        .with_host_calibrated_accel(true)
        .with_raw_accel(true)
        .with_sample_rate(225);

    imu.dmp_configure(&dmp_config).await.unwrap();

    imu.dmp_enable(true).await.unwrap();
    imu.reset_fifo().await.unwrap();

    info!("Entering 6/9-Axis DMP read loop...");

    let mut sample_count = 0u32;
    let mut last_print_time = Instant::now();

    let mut vel_x = 0.0f32;
    let mut vel_y = 0.0f32;
    let mut pos_x = 0.0f32;
    let mut pos_y = 0.0f32;

    let dt = 1.0 / 56.0;

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
                    // let az_g = accel.2 as f32 / 8192.0;

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

                if sample_count % 10 == 0 {
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
