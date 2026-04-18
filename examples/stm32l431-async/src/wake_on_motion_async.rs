#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _; // defmt transport
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use icm20948::{
    Icm20948Driver, InterruptConfig, InterruptPinConfig, SpiInterface,
    power::{LowPowerConfig, LowPowerRate, WomMode},
};

bind_interrupts!(struct SpiIrqs {
    DMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
    DMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH4>;
    EXTI3 =>  embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Starting Wake-on-Motion test...");

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);
    spi_config.mode = embassy_stm32::spi::MODE_0;

    let spi_bus = Spi::new(
        p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH5, p.DMA1_CH4, SpiIrqs, spi_config,
    );

    let cs_pin = Output::new(p.PA2, Level::High, Speed::VeryHigh);
    let spi_device = ExclusiveDevice::new(spi_bus, cs_pin, Delay).unwrap();

    let mut imu_int1 = ExtiInput::new(p.PA3, p.EXTI3, Pull::Down, SpiIrqs);

    let mut imu = match Icm20948Driver::new(SpiInterface::new(spi_device)).await {
        Ok(driver) => driver,
        Err(e) => {
            error!("IMU initialization failed: {:?}", e);
            return;
        }
    };

    let mut delay = Delay;
    imu.init(&mut delay).await.unwrap();
    imu.enable_spi_mode().await.unwrap();
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
    info!("IMU and MCU have entered deep sleep! Waiting for motion...");

    let mut motion_count = 0;

    loop {
        imu_int1.wait_for_high().await;
        let status = imu.read_interrupt_status().await.unwrap();

        if status.wake_on_motion {
            motion_count += 1;
            info!("Motion detected! (Wakeup count: {})", motion_count);

            if let Ok(accel) = imu.read_accelerometer_raw().await {
                let dx = (accel.x.unsigned_abs() as u32) % 16384;
                let dy = (accel.y.unsigned_abs() as u32) % 16384;
                let dz = (accel.z.unsigned_abs() as u32) % 16384;

                let mut axis = "Unknown";
                if dx > dy && dx > dz {
                    axis = "X-axis";
                } else if dy > dx && dy > dz {
                    axis = "Y-axis";
                } else if dz > dx && dz > dy {
                    axis = "Z-axis";
                }

                info!(
                    "Software inferred trigger source -> Primary force direction: {} (ΔX:{}, ΔY:{}, ΔZ:{})",
                    axis, dx, dy, dz
                );
            }

            info!("Calming down, returning to sleep...");
        }
    }
}
