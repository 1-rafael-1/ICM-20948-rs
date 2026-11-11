//! DMP FIFO reader framing tests (blocking driver)
//!
//! These tests exercise the public `DmpFifoReader` through the mock driver.
//! Byte accounting is asserted against the mock FIFO stream so that no byte can
//! be silently lost or duplicated.

use crate::common::test_utils::create_mock_driver;
use icm20948::Bank;
use icm20948::Error;
use icm20948::dmp::config::{DmpPacketHeader, DmpPacketHeader2, DmpPacketSize};
use icm20948::dmp::fifo_reader::DmpFifoReader;
use icm20948::dmp::parser::DmpParser;

/// Build a hardware-order packet with a footer.
///
/// `header2` must only be `Some` when `header` has `HEADER2_BIT` set.
fn build_packet(header: u16, header2: Option<u16>, payload: &[u8]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(payload.len() + 6);
    packet.extend_from_slice(&header.to_be_bytes());
    if let Some(header2) = header2 {
        packet.extend_from_slice(&header2.to_be_bytes());
    }
    packet.extend_from_slice(payload);
    packet.extend_from_slice(&[0u8, 0u8]);
    packet
}

/// Build a QUAT9 packet without a secondary header.
fn quat9_packet(heading_accuracy: u16) -> Vec<u8> {
    let mut payload = vec![0u8; DmpPacketSize::QUAT9];
    payload[12..14].copy_from_slice(&heading_accuracy.to_be_bytes());
    build_packet(DmpPacketHeader::QUAT9_BIT, None, &payload)
}

fn expected_size(header: u16, header2: Option<u16>) -> usize {
    DmpParser::calculate_packet_size(header, header2)
}

#[test]
fn complete_packet_yields_data_and_consumes_exact_bytes() {
    let (mut driver, interface) = create_mock_driver();
    let packet = quat9_packet(42);
    let size = expected_size(DmpPacketHeader::QUAT9_BIT, None);
    assert_eq!(packet.len(), size);

    interface.set_fifo_data(&packet);

    let mut reader = DmpFifoReader::new();
    let data = reader
        .read_packet(&mut driver)
        .expect("read_packet failed")
        .expect("expected a complete packet");

    assert!(data.quaternion_9axis.is_some());
    assert_eq!(data.heading_accuracy, Some(42.0));
    assert_eq!(interface.fifo_bytes_consumed(), size);
    assert_eq!(interface.fifo_remaining(), 0);
    assert_eq!(reader.last_packet_bytes(), &packet[..]);
    assert_eq!(
        reader.last_packet_info(),
        Some(icm20948::dmp::DmpPacketInfo {
            header: DmpPacketHeader::QUAT9_BIT,
            header2: None,
            size,
        })
    );
}

#[test]
fn split_at_header_block_is_retained() {
    let (mut driver, interface) = create_mock_driver();
    let packet = quat9_packet(1);
    let size = packet.len();

    interface.set_fifo_data(&packet[..4]);

    let mut reader = DmpFifoReader::new();
    assert!(
        reader
            .read_packet(&mut driver)
            .expect("read_packet failed")
            .is_none()
    );
    assert_eq!(interface.fifo_bytes_consumed(), 4);
    assert_eq!(interface.fifo_remaining(), 0);

    interface.append_fifo_data(&packet[4..]);
    let data = reader
        .read_packet(&mut driver)
        .expect("read_packet failed")
        .expect("expected a complete packet");

    assert!(data.quaternion_9axis.is_some());
    assert_eq!(interface.fifo_bytes_consumed(), size);
    assert_eq!(interface.fifo_remaining(), 0);
}

#[test]
fn split_in_middle_of_payload_is_retained() {
    let (mut driver, interface) = create_mock_driver();
    let packet = quat9_packet(7);
    let size = packet.len();

    interface.set_fifo_data(&packet[..10]);

    let mut reader = DmpFifoReader::new();
    assert!(
        reader
            .read_packet(&mut driver)
            .expect("read_packet failed")
            .is_none()
    );
    assert_eq!(interface.fifo_bytes_consumed(), 10);
    assert_eq!(interface.fifo_remaining(), 0);

    interface.append_fifo_data(&packet[10..]);
    let data = reader
        .read_packet(&mut driver)
        .expect("read_packet failed")
        .expect("expected a complete packet");

    assert!(data.quaternion_9axis.is_some());
    assert_eq!(data.heading_accuracy, Some(7.0));
    assert_eq!(interface.fifo_bytes_consumed(), size);
    assert_eq!(interface.fifo_remaining(), 0);
}

#[test]
fn multiple_buffered_packets_are_drained_once() {
    let (mut driver, interface) = create_mock_driver();
    let first = quat9_packet(11);
    let second = quat9_packet(22);
    let mut stream = first.clone();
    stream.extend_from_slice(&second);
    interface.set_fifo_data(&stream);

    let mut reader = DmpFifoReader::new();
    let mut accuracies = Vec::new();
    let delivered = reader
        .read_packets(&mut driver, |data| {
            accuracies.push(data.heading_accuracy.expect("heading accuracy"));
        })
        .expect("read_packets failed");

    assert_eq!(delivered, 2);
    assert_eq!(accuracies, vec![11.0, 22.0]);
    assert_eq!(interface.fifo_bytes_consumed(), stream.len());
    assert_eq!(interface.fifo_remaining(), 0);
}

#[test]
fn secondary_header_with_and_without_accuracy_words() {
    let (mut driver, interface) = create_mock_driver();

    let header = DmpPacketHeader::QUAT9_BIT | DmpPacketHeader::HEADER2_BIT;
    let header2 = DmpPacketHeader2::ACCEL_ACCURACY_BIT
        | DmpPacketHeader2::GYRO_ACCURACY_BIT
        | DmpPacketHeader2::COMPASS_ACCURACY_BIT;

    let mut with_payload = vec![0u8; DmpPacketSize::QUAT9 + 6];
    with_payload[14..16].copy_from_slice(&1u16.to_be_bytes());
    with_payload[16..18].copy_from_slice(&2u16.to_be_bytes());
    with_payload[18..20].copy_from_slice(&3u16.to_be_bytes());
    let with_accuracy = build_packet(header, Some(header2), &with_payload);

    let without_payload = vec![0u8; DmpPacketSize::QUAT9];
    let without_accuracy = build_packet(header, Some(0), &without_payload);

    let with_size = expected_size(header, Some(header2));
    let without_size = expected_size(header, Some(0));
    assert_eq!(with_accuracy.len(), with_size);
    assert_eq!(without_accuracy.len(), without_size);
    assert_ne!(with_size, without_size);

    let mut stream = with_accuracy.clone();
    stream.extend_from_slice(&without_accuracy);
    interface.set_fifo_data(&stream);

    let mut reader = DmpFifoReader::new();

    let first = reader
        .read_packet(&mut driver)
        .expect("read_packet failed")
        .expect("expected packet with accuracy words");
    assert_eq!(first.accel_accuracy, Some(1));
    assert_eq!(first.gyro_accuracy, Some(2));
    assert_eq!(first.compass_accuracy, Some(3));
    assert_eq!(
        reader.last_packet_info().expect("packet info").header2,
        Some(header2)
    );
    assert_eq!(
        reader.last_packet_info().expect("packet info").size,
        with_size
    );
    assert_eq!(reader.last_packet_bytes(), &with_accuracy[..]);

    let second = reader
        .read_packet(&mut driver)
        .expect("read_packet failed")
        .expect("expected packet without accuracy words");
    assert_eq!(second.accel_accuracy, None);
    assert_eq!(second.gyro_accuracy, None);
    assert_eq!(second.compass_accuracy, None);
    assert_eq!(
        reader.last_packet_info().expect("packet info").header2,
        Some(0)
    );
    assert_eq!(
        reader.last_packet_info().expect("packet info").size,
        without_size
    );

    assert_eq!(interface.fifo_bytes_consumed(), stream.len());
    assert_eq!(interface.fifo_remaining(), 0);
}

#[test]
fn malformed_header_resets_reader_and_fifo() {
    let (mut driver, interface) = create_mock_driver();
    // Invalid header (unknown low bits) followed by extra bytes.
    interface.set_fifo_data(&[0xFF, 0xFF, 0x11, 0x22, 0x33]);

    let mut reader = DmpFifoReader::new();
    let error = reader
        .read_packet(&mut driver)
        .expect_err("malformed header must fail");

    assert!(matches!(error, Error::MalformedDmpPacket));
    assert!(reader.last_packet_info().is_none());
    assert_eq!(interface.fifo_remaining(), 0, "FIFO must be reset");

    // Reader state was cleared; an empty FIFO now yields no packet.
    assert!(
        reader
            .read_packet(&mut driver)
            .expect("empty read failed")
            .is_none()
    );
}

#[test]
fn malformed_header_exposes_rejected_bytes() {
    let (mut driver, interface) = create_mock_driver();
    interface.set_fifo_data(&[0xFF, 0xFF, 0x11, 0x22, 0x33]);

    let mut reader = DmpFifoReader::new();
    let error = reader
        .read_packet(&mut driver)
        .expect_err("malformed header must fail");

    assert!(matches!(error, Error::MalformedDmpPacket));
    assert!(reader.last_packet_info().is_none());
    assert_eq!(
        reader.last_rejected_header_bytes(),
        &[0xFF, 0xFF, 0x11, 0x22],
        "the rejected header block must be retained"
    );

    // `clear` explicitly discards the rejected bytes as well.
    reader.clear();
    assert!(reader.last_rejected_header_bytes().is_empty());
}

#[test]
fn partial_next_packet_invalidates_last_packet_diagnostics() {
    let (mut driver, interface) = create_mock_driver();
    let packet = quat9_packet(42);
    let next = quat9_packet(43);
    let size = packet.len();

    interface.set_fifo_data(&packet);

    let mut reader = DmpFifoReader::new();
    let data = reader
        .read_packet(&mut driver)
        .expect("read_packet failed")
        .expect("expected a complete packet");
    assert!(data.quaternion_9axis.is_some());
    assert_eq!(reader.last_packet_bytes(), &packet[..]);
    assert_eq!(reader.last_packet_info().map(|info| info.size), Some(size));

    // Only the first four bytes of the next packet are available, so the
    // reader retains them and returns no packet. The previous packet's
    // diagnostics must be invalidated because `buffer[0..]` now holds the new
    // packet's prefix.
    interface.append_fifo_data(&next[..4]);
    assert!(
        reader
            .read_packet(&mut driver)
            .expect("read_packet failed")
            .is_none()
    );
    assert!(reader.last_packet_info().is_none());
    assert!(reader.last_packet_bytes().is_empty());
}

#[test]
fn fifo_overflow_is_reported_and_reset() {
    let (mut driver, interface) = create_mock_driver();
    interface.set_register(Bank::Bank0, 0x1B, 0x01);
    interface.set_fifo_data(&quat9_packet(1));

    let mut reader = DmpFifoReader::new();
    let error = reader
        .read_packet(&mut driver)
        .expect_err("overflow must fail");

    assert!(matches!(error, Error::FifoOverflow));
    assert_eq!(interface.fifo_remaining(), 0, "FIFO must be reset");
}

#[test]
fn overflow_reset_failure_keeps_resync_required_and_next_call_recovers() {
    let (mut driver, interface) = create_mock_driver();
    interface.set_register(Bank::Bank0, 0x1B, 0x01);

    // Stale bytes that must never be framed after the failed reset.
    let stale = quat9_packet(1);
    interface.set_fifo_data(&stale);

    let mut reader = DmpFifoReader::new();

    // Fail the FIFO_RST write; the driver is already on Bank 0 after the
    // overflow-status read, so the next write is the reset itself.
    interface.fail_next_write();
    let error = reader
        .read_packet(&mut driver)
        .expect_err("failed FIFO reset must surface as a bus error");
    assert!(matches!(error, Error::Bus(_)));

    // The reset never reached the device, so the stale bytes are still queued.
    assert_eq!(interface.fifo_remaining(), stale.len());

    // The next call must retry the reset instead of framing the stale bytes,
    // then report that no packet is available.
    assert!(
        reader
            .read_packet(&mut driver)
            .expect("recovery read failed")
            .is_none(),
        "recovery must not return data from stale bytes"
    );
    assert_eq!(
        interface.fifo_remaining(),
        0,
        "the retried reset must flush the device FIFO"
    );
}

#[test]
fn unsupported_secondary_header_bits_reject_packet_and_reset_fifo() {
    let (mut driver, interface) = create_mock_driver();

    let header = DmpPacketHeader::QUAT9_BIT | DmpPacketHeader::HEADER2_BIT;
    let header2 = DmpPacketHeader2::ACTIVITY_RECOG_BIT;
    let payload = vec![0u8; DmpPacketSize::QUAT9];
    let packet = build_packet(header, Some(header2), &payload);
    interface.set_fifo_data(&packet);

    let mut reader = DmpFifoReader::new();
    let error = reader
        .read_packet(&mut driver)
        .expect_err("unsupported secondary header bits must fail");

    assert!(matches!(error, Error::MalformedDmpPacket));
    assert!(reader.last_packet_info().is_none());
    assert_eq!(
        reader.last_rejected_header_bytes(),
        &packet[..4],
        "the rejected header block must be retained"
    );
    assert_eq!(
        interface.fifo_remaining(),
        0,
        "the FIFO must be reset so the unaccounted payload cannot be reframed"
    );

    // Framing and resync state are clean: an empty FIFO yields no packet.
    assert!(
        reader
            .read_packet(&mut driver)
            .expect("empty read failed")
            .is_none()
    );
}

#[test]
fn drain_all_callback_delivers_three_packets_then_stops() {
    let (mut driver, interface) = create_mock_driver();
    let mut stream = Vec::new();
    for accuracy in [5u16, 6, 7] {
        stream.extend_from_slice(&quat9_packet(accuracy));
    }
    interface.set_fifo_data(&stream);

    let mut reader = DmpFifoReader::new();
    let mut delivered = 0;
    let count = reader
        .read_packets(&mut driver, |_data| delivered += 1)
        .expect("read_packets failed");

    assert_eq!(count, 3);
    assert_eq!(delivered, 3);
    assert_eq!(interface.fifo_bytes_consumed(), stream.len());
    assert_eq!(interface.fifo_remaining(), 0);

    assert!(
        reader
            .read_packet(&mut driver)
            .expect("empty read failed")
            .is_none()
    );
}
