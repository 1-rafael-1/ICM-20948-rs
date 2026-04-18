#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
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
    sensors::{AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale},
};

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

    let mut imu_int1 = ExtiInput::new(p.PA3, p.EXTI3, Pull::Down, SpiIrqs);

    let mut imu = Icm20948Driver::new(SpiInterface::new(spi_device))
        .await
        .unwrap();
    let mut delay = Delay;

    imu.init(&mut delay).await.unwrap();
    imu.enable_spi_mode().await.unwrap();
    Timer::after_millis(50).await;

    // 1. Configure sensors (100Hz)
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

    // 2. Configure pins and Data-Ready interrupt
    let int_pin_cfg = InterruptPinConfig {
        active_low: false,
        open_drain: false,
        latch_enabled: true,
        clear_on_any_read: true, // Automatically clear the latched interrupt pin on any IMU register read (e.g., INT_STATUS or sensor data registers)
    };
    imu.configure_interrupt_pin(&int_pin_cfg).await.unwrap();

    // Enable Data Ready interrupt
    let int_cfg = InterruptConfig::data_ready_only();
    imu.configure_interrupts(&int_cfg).await.unwrap();

    let _ = imu.read_interrupt_status().await; // Clear historical interrupt status
    info!("Interrupt configured! The MCU will be awakened by hardware at a 100Hz frequency...");

    let mut counter = 0;

    loop {
        // Block and wait for hardware data, achieving 0 polling overhead
        imu_int1.wait_for_high().await;

        // Fetch the freshly generated data
        if let (Ok(accel), Ok(gyro)) = (imu.read_accelerometer().await, imu.read_gyroscope().await)
        {
            counter += 1;

            // Reduce print frequency to prevent terminal flooding (100Hz -> 10Hz)
            if counter % 10 == 0 {
                info!(
                    "[10Hz Sample] Accel: [{}, {}, {}] g | Gyro: [{}, {}, {}] °/s",
                    accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
                );
            }
        }
    }
