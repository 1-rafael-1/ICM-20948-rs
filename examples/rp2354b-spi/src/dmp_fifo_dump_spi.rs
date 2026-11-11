//! Raw DMP FIFO dump over SPI on the RP2354B board.
//!
//! This is the packet-level evidence harness for the stale-magnetometer
//! investigation. It configures the same 9-axis DMP stream as
//! `dmp_nine_axis_spi` (raw accel/gyro/mag at the validated 112 Hz rate) but
//! reports what the framing layer actually saw, byte for byte, instead of
//! interpreting the payload:
//!
//! - `fifo` — the FIFO count read *before* draining this poll;
//! - `raw_header` — the first four bytes of the completed packet;
//! - `primary` — the decoded 16-bit primary packet header;
//! - `secondary` — the decoded 16-bit secondary header, or `none`;
//! - `size` — the packet size the reader computed from the header;
//! - `parse` — which fused/raw record the parser found in the packet;
//! - `len` — the number of bytes in the completed packet;
//! - `overflow` / `malformed` — cumulative framing-error counters.
//!
//! The first four raw bytes are the primary header (2 bytes) followed by
//! either the secondary header (2 bytes, when the primary `HEADER2_BIT` is
//! set) or the first two payload bytes (when it is clear). Comparing
//! `raw_header` with `primary`/`secondary` is how a framing defect is told
//! apart from a magnetometer fallback: a stable `primary`, a sane `size` and a
//! `parse` that flips between `quat9` and `quat6` is a fusion problem, while an
//! implausible `primary` or a `parse=none`/`malformed` churn is a framing or
//! bus problem.
//!
//! Every packet is framed by [`DmpFifoReader`], so a packet split across FIFO
//! reads is retained. FIFO overflow and malformed headers are counted and
//! logged, and the drain resumes on the next poll.
//!
//! ## HIL procedure (rotate 90 degrees)
//!
//! 1. Flash and attach RTT (`cargo run --bin dmp_fifo_dump_spi --features dmp
//!    --release`).
//! 2. Confirm a steady dump with a constant `primary`, an alternating
//!    `secondary` presence, `parse=quat9` and `overflow=0 malformed=0`.
//! 3. Rotate the board **90 degrees about the vertical (Z) axis** and hold it
//!    still.
//! 4. `primary`, `secondary`, `size` and `parse` must not change character;
//!    only the payload does.
//! 5. If `parse` falls back to `quat6` while the headers stay stable, the FIFO
//!    framing is healthy and the fault is the DMP magnetometer path.
//!
//! Set `SPI_CONTROL_HZ` instead of `SPI_DEFAULT_HZ` below to rerun the same
//! firmware at 1 MHz when triaging signal integrity on the level-shifted bus.

#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{block::ImageDef, config::Config};
use embassy_time::{Delay, Timer};
use icm20948::{
    dmp::{DmpConfig, DmpData, DmpFifoReader, DmpPacketInfo, DmpSampleRate},
    Error,
};
use panic_probe as _;

mod board;
mod report;

use crate::report::{classify, PacketKind};

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// FIFO poll interval.
///
/// At the configured 112.5 Hz DMP output a packet arrives every 8.9 ms, so a
/// 4 ms poll always drains the FIFO before it can back up.
const POLL_INTERVAL_MS: u64 = 4;

/// Dump one packet per this many completed packets (~10 Hz at 112.5 Hz).
const LOG_EVERY_PACKETS: u32 = 11;

/// Secondary header wrapper so the log prints `0x....` when present and
/// `none` when the primary `HEADER2_BIT` is clear.
struct SecondaryHeader(Option<u16>);

impl defmt::Format for SecondaryHeader {
    fn format(&self, f: defmt::Formatter) {
        match self.0 {
            Some(header2) => defmt::write!(f, "0x{:04X}", header2),
            None => defmt::write!(f, "none"),
        }
    }
}

/// Cumulative packet and framing-error counters for the dump line.
#[derive(Clone, Copy)]
struct Counters {
    packets_total: u32,
    overflow: u32,
    malformed: u32,
}

/// Log one decimated dump line for a completed packet.
fn log_packet(
    packet: &DmpData,
    info: DmpPacketInfo,
    bytes: &[u8],
    fifo_count: u16,
    drained: u32,
    counters: Counters,
) {
    // The reader always completes at least the 4-byte header block, but keep
    // the log well-defined if that ever changes.
    let mut raw = [0u8; 4];
    let raw_len = core::cmp::min(raw.len(), bytes.len());
    raw[..raw_len].copy_from_slice(&bytes[..raw_len]);

    info!(
        "dmp-dump fifo={} raw_header=0x{:02x}{:02x}{:02x}{:02x} primary=0x{:04X} secondary={} size={} parse={} len={} drained={} overflow={} malformed={} packets={}",
        fifo_count,
        raw[0],
        raw[1],
        raw[2],
        raw[3],
        info.header,
        SecondaryHeader(info.header2),
        info.size as u32,
        {
            let kind: PacketKind = classify(packet);
            kind.name()
        },
        bytes.len() as u32,
        drained,
        counters.overflow,
        counters.malformed,
        counters.packets_total
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("ICM-20948 DMP raw FIFO dump over SPI (RP2354B)");

    let mut p = embassy_rp::init(Config::default());

    // Signal-integrity control: swap `SPI_DEFAULT_HZ` for `SPI_CONTROL_HZ`.
    let mut imu = board::init_imu_or_idle(&mut p, board::SPI_DEFAULT_HZ).await;

    let mut delay = Delay;

    info!("Loading DMP firmware...");
    if imu.dmp_init(&mut delay).await.is_err() {
        board::idle_on_error("DMP firmware load failed (dmp_init)").await;
    }

    info!("Initializing the DMP magnetometer path (AK09916)...");
    if imu.dmp_init_magnetometer(&mut delay).await.is_err() {
        board::idle_on_error(
            "DMP magnetometer initialization failed (dmp_init_magnetometer); check the AK09916 and mag power",
        )
        .await;
    }

    // DMP ODR register 0 (maximum); packet rate 112.5 Hz.
    let dmp_config = DmpConfig::nine_axis()
        .with_raw_accel()
        .with_raw_gyro()
        .with_dmp_compass()
        .with_sample_rate(DmpSampleRate::Hz112);

    info!(
        "DMP rate: nominal={} Hz actual={} Hz divider={}",
        dmp_config.sample_rate.nominal_hz(),
        dmp_config.actual_output_hz(),
        dmp_config.dmp_output_divider()
    );
    info!(
        "Poll interval: {} ms; dump every {} packets (~10 Hz)",
        POLL_INTERVAL_MS, LOG_EVERY_PACKETS
    );
    info!("raw_header = primary header + (secondary header | first two payload bytes)");

    if imu.dmp_configure(&dmp_config).await.is_err() {
        board::idle_on_error("DMP configuration failed (dmp_configure)").await;
    }

    if imu.dmp_enable(true).await.is_err() {
        board::idle_on_error("DMP enable failed (dmp_enable)").await;
    }

    if imu.reset_fifo().await.is_err() {
        board::idle_on_error("DMP FIFO reset failed (reset_fifo)").await;
    }

    info!("DMP running; dumping the raw FIFO stream over RTT, no prompts");

    let mut reader = DmpFifoReader::new();

    let mut packets_total: u32 = 0;
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

        // Drain every complete packet this poll, not just one per slow poll.
        let mut drained: u32 = 0;
        loop {
            match reader.read_packet_async(&mut imu).await {
                Ok(Some(packet)) => {
                    drained = drained.wrapping_add(1);
                    packets_total = packets_total.wrapping_add(1);
                    packets_since_log = packets_since_log.wrapping_add(1);

                    if packets_since_log >= LOG_EVERY_PACKETS {
                        packets_since_log = 0;
                        if let Some(info) = reader.last_packet_info() {
                            log_packet(
                                &packet,
                                info,
                                reader.last_packet_bytes(),
                                fifo_count,
                                drained,
                                Counters {
                                    packets_total,
                                    overflow: overflow_events,
                                    malformed: malformed_events,
                                },
                            );
                        }
                    }
                }
                Ok(None) => break,
                // The reader has already reset its state and the device FIFO;
                // report the framing event and resume on the next poll.
                Err(Error::FifoOverflow) => {
                    overflow_events = overflow_events.wrapping_add(1);
                    warn!(
                        "dmp-dump event: FIFO overflow, resynchronized; overflow={} malformed={} fifo={}",
                        overflow_events, malformed_events, fifo_count
                    );
                    break;
                }
                Err(Error::MalformedDmpPacket) => {
                    malformed_events = malformed_events.wrapping_add(1);
                    let rejected = reader.last_rejected_header_bytes();
                    let mut rejected_raw = [0u8; 4];
                    let rejected_len = core::cmp::min(rejected_raw.len(), rejected.len());
                    rejected_raw[..rejected_len].copy_from_slice(&rejected[..rejected_len]);
                    warn!(
                        "dmp-dump event: malformed packet header, resynchronized; rejected=0x{:02x}{:02x}{:02x}{:02x} rejected_len={} overflow={} malformed={} fifo={}",
                        rejected_raw[0],
                        rejected_raw[1],
                        rejected_raw[2],
                        rejected_raw[3],
                        rejected_len as u32,
                        overflow_events,
                        malformed_events,
                        fifo_count
                    );
                    break;
                }
                Err(_) => {
                    board::idle_on_error("DMP FIFO read failed; check the SPI bus and clock").await;
                }
            }
        }

        Timer::after_millis(POLL_INTERVAL_MS).await;
    }
}
