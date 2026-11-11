//! DMP six-axis quaternion over SPI on the RP2354B board.
//!
//! This is the control for the nine-axis reproduction harness. It loads the
//! same DMP firmware and runs the same FIFO drain path, but configures 6-axis
//! fusion (accelerometer + gyroscope) and never initializes the magnetometer.
//! If this example produces a stable `quat6` stream while the nine-axis example
//! falls back or freezes its magnetometer fields, the DMP and FIFO path are
//! healthy and the fault is in the magnetometer lifecycle.
//!
//! The ~10 Hz summary line reports the packet header, the fusion mode
//! (`quat6`/`quat9`/`none`), the Euler angles, the configured and actual DMP
//! output rate, the divider, the poll interval, the FIFO count, how many
//! packets this poll drained, and cumulative overflow and malformed counters.
//! Optional raw accel/gyro values are logged on indented follow-up lines only
//! when the packet carries them. The magnetometer, heading and compass fields
//! are deliberately absent.
//!
//! ## HIL procedure (rotate 90 degrees)
//!
//! 1. Flash and attach RTT (`cargo run --bin dmp_six_axis_spi --features dmp
//!    --release`) and confirm a steady summary with `fusion=quat6`.
//! 2. Note `yaw_deg` with the board flat and stationary.
//! 3. Rotate the board **90 degrees about the vertical (Z) axis** and hold it
//!    still.
//! 4. `yaw_deg` must move by about 90 degrees. There are no `dmp_compass`,
//!    `heading_accuracy` or `compass_accuracy` fields by design: this is the
//!    control without the magnetometer.
//! 5. Compare against `dmp_nine_axis_spi`: if yaw still tracks here, the DMP
//!    and FIFO path work, and a stale nine-axis magnetometer is the fault.
//!
//! Set `SPI_CONTROL_HZ` instead of `SPI_DEFAULT_HZ` below to rerun the same
//! firmware at 1 MHz when triaging signal integrity on the level-shifted bus.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{block::ImageDef, config::Config};
use embassy_time::{Delay, Timer};
use icm20948::{
    dmp::{DmpConfig, DmpData, DmpFifoReader, DmpSampleRate},
    Error,
};
use panic_probe as _;

use crate::report::{classify, PacketKind};

mod board;
mod report;

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// FIFO poll interval.
///
/// At the configured 112.5 Hz DMP output a packet arrives every 8.9 ms, so a
/// 4 ms poll always drains the FIFO before it can back up.
const POLL_INTERVAL_MS: u64 = 4;

/// Log one summary per this many completed packets (~10 Hz at 112.5 Hz).
const LOG_EVERY_PACKETS: u32 = 11;

/// Most recently observed packet fields, kept for the periodic summary.
struct Summary {
    kind: PacketKind,
    roll_deg: f32,
    pitch_deg: f32,
    yaw_deg: f32,
    raw_accel: Option<(i16, i16, i16)>,
    raw_gyro: Option<(i16, i16, i16)>,
}

impl Summary {
    const fn new() -> Self {
        Self {
            kind: PacketKind::None,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
            raw_accel: None,
            raw_gyro: None,
        }
    }

    /// Fold one packet into the summary state.
    ///
    /// Euler angles come from whichever quaternion is present; the raw fields
    /// are replaced on every packet so that a field vanishing from the stream
    /// is visible in the next summary.
    fn update(&mut self, packet: &DmpData, kind: PacketKind) {
        self.kind = kind;

        if let Some(quat) = packet.quaternion_6axis.or(packet.quaternion_9axis) {
            let (roll, pitch, yaw) = quat.to_euler_angles().to_degrees();
            self.roll_deg = roll;
            self.pitch_deg = pitch;
            self.yaw_deg = yaw;
        }

        self.raw_accel = packet.raw_accel;
        self.raw_gyro = packet.raw_gyro;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("ICM-20948 DMP six-axis control over SPI (RP2354B)");

    let mut p = embassy_rp::init(Config::default());

    // Signal-integrity control: swap `SPI_DEFAULT_HZ` for `SPI_CONTROL_HZ`.
    let mut imu = board::init_imu_or_idle(&mut p, board::SPI_DEFAULT_HZ).await;

    let mut delay = Delay;

    info!("Loading DMP firmware...");
    if imu.dmp_init(&mut delay).await.is_err() {
        board::idle_on_error("DMP firmware load failed (dmp_init)").await;
    }

    // Control: the magnetometer is never initialized and never used.

    // Validated mid-rate: DMP ODR register 0 (maximum); packet rate 112.5 Hz.
    let dmp_config = DmpConfig::six_axis()
        .with_raw_accel()
        .with_raw_gyro()
        .with_sample_rate(DmpSampleRate::Hz112);

    info!(
        "DMP rate: nominal={} Hz actual={} Hz divider={}",
        dmp_config.sample_rate.nominal_hz(),
        dmp_config.actual_output_hz(),
        dmp_config.dmp_output_divider()
    );
    info!(
        "Poll interval: {} ms; summary every {} packets (~10 Hz)",
        POLL_INTERVAL_MS, LOG_EVERY_PACKETS
    );

    if imu.dmp_configure(&dmp_config).await.is_err() {
        board::idle_on_error("DMP configuration failed (dmp_configure)").await;
    }

    if imu.dmp_enable(true).await.is_err() {
        board::idle_on_error("DMP enable failed (dmp_enable)").await;
    }

    if imu.reset_fifo().await.is_err() {
        board::idle_on_error("DMP FIFO reset failed (reset_fifo)").await;
    }

    info!("DMP running; streaming 6-axis packets over RTT, no prompts");

    let mut reader = DmpFifoReader::new();
    let mut summary = Summary::new();

    let mut packets_total: u32 = 0;
    let mut quat9_packets: u32 = 0;
    let mut quat6_packets: u32 = 0;
    let mut overflow_events: u32 = 0;
    let mut malformed_events: u32 = 0;
    let mut packets_since_log: u32 = 0;

    loop {
        // Observe the FIFO before draining so backlog is visible in the log.
        let fifo_count = match imu.read_fifo_count().await {
            Ok(count) => count,
            Err(_) => {
                board::idle_on_error("FIFO count read failed; check the SPI bus and clock").await;
            }
        };

        // Reading INT_STATUS_2 clears the overflow flag, so resynchronise the
        // FIFO and the reader here instead of relying on the next packet read.
        match imu.fifo_overflow_status().await {
            Ok(status) if status.any_overflow() => {
                overflow_events = overflow_events.wrapping_add(1);
                reader.clear();
                if imu.reset_fifo().await.is_err() {
                    board::idle_on_error("FIFO reset after overflow failed; check the SPI bus")
                        .await;
                }
            }
            Ok(_) => {}
            Err(_) => {
                board::idle_on_error(
                    "FIFO overflow status read failed; check the SPI bus and clock",
                )
                .await;
            }
        }

        // Drain every complete packet this poll, not just one per slow poll.
        let mut drained: u32 = 0;
        loop {
            match reader.read_packet_async(&mut imu).await {
                Ok(Some(packet)) => {
                    drained = drained.wrapping_add(1);
                    packets_total = packets_total.wrapping_add(1);
                    packets_since_log = packets_since_log.wrapping_add(1);

                    let kind = classify(&packet);
                    match kind {
                        PacketKind::Quat9 => quat9_packets = quat9_packets.wrapping_add(1),
                        PacketKind::Quat6 => quat6_packets = quat6_packets.wrapping_add(1),
                        _ => {}
                    }

                    summary.update(&packet, kind);
                }
                Ok(None) => break,
                Err(Error::FifoOverflow) => {
                    overflow_events = overflow_events.wrapping_add(1);
                }
                Err(Error::MalformedDmpPacket) => {
                    malformed_events = malformed_events.wrapping_add(1);
                }
                Err(_) => {
                    board::idle_on_error("DMP FIFO read failed; check the SPI bus and clock").await;
                }
            }
        }

        if packets_since_log >= LOG_EVERY_PACKETS {
            packets_since_log = 0;

            let header = reader.last_packet_info().map_or(0, |meta| meta.header);

            info!(
                "dmp hdr=0x{:04X} fusion={} euler_deg=({},{},{}) fifo={} drained={} overflow={} malformed={} rate_cfg_hz={} rate_actual_hz={} divider={} poll_ms={} packets={} quat9_packets={} quat6_packets={}",
                header,
                summary.kind.fusion_name(),
                summary.roll_deg,
                summary.pitch_deg,
                summary.yaw_deg,
                fifo_count,
                drained,
                overflow_events,
                malformed_events,
                dmp_config.sample_rate.nominal_hz(),
                dmp_config.actual_output_hz(),
                dmp_config.dmp_output_divider(),
                POLL_INTERVAL_MS,
                packets_total,
                quat9_packets,
                quat6_packets
            );

            if let Some((x, y, z)) = summary.raw_accel {
                info!("  raw_accel=({},{},{})", x, y, z);
            }
            if let Some((x, y, z)) = summary.raw_gyro {
                info!("  raw_gyro=({},{},{})", x, y, z);
            }
        }

        Timer::after_millis(POLL_INTERVAL_MS).await;
    }
}
