//! Caller-owned DMP FIFO packet framing
//!
//! [`DmpFifoReader`] retains bytes that belong to an incomplete DMP packet
//! between polls. It yields complete [`DmpData`] packets and never consumes a
//! header block without either completing the packet or retaining the partial
//! bytes. The reader owns its buffer; the driver carries no per-instance packet
//! buffer.
//!
//! ## Usage
//!
//! ```ignore
//! # use icm20948::{Icm20948Driver, dmp::DmpFifoReader};
//! # let mut driver: Icm20948Driver<_> = todo!();
//! let mut reader = DmpFifoReader::new();
//! loop {
//!     if let Some(data) = reader.read_packet(&mut driver)? {
//!         // Use the complete packet.
//!     }
//! }
//! ```

use crate::Error;
use crate::Icm20948Driver;
use crate::dmp::DmpData;
use crate::dmp::config::DmpPacketSize;

/// Fixed primary header plus optional secondary header block.
const HEADER_BLOCK: usize = DmpPacketSize::HEADER + DmpPacketSize::HEADER2;

/// Maximum number of leading bytes retained from a rejected packet.
const REJECTED_HEADER_MAX: usize = HEADER_BLOCK;

/// Await an expression only in the async expansion of [`read_packet_impl`].
macro_rules! maybe_await {
    ($expr:expr, blocking) => {
        $expr
    };
    ($expr:expr, asynchronous) => {
        $expr.await
    };
}

/// Single implementation of the framing state machine.
///
/// `$mode` selects the only difference between the blocking and async paths:
/// whether device I/O is awaited. Every framing rule below is defined once.
macro_rules! read_packet_impl {
    ($reader:ident, $driver:ident, $mode:ident) => {{
        use crate::dmp::config::DmpPacketHeader;
        use crate::dmp::parser::DmpParser;

        // A FIFO overflow invalidates any retained bytes.
        if maybe_await!($driver.fifo_overflow_status(), $mode)?.any_overflow() {
            $reader.state.clear();
            maybe_await!($driver.fifo_reset(), $mode)?;
            return Err(Error::FifoOverflow);
        }

        // Ensure the 4-byte header block is buffered. Never request more bytes
        // than the FIFO count reports.
        if $reader.state.len < HEADER_BLOCK {
            let available = maybe_await!($driver.read_fifo_count(), $mode)? as usize;
            let want = core::cmp::min(HEADER_BLOCK - $reader.state.len, available);
            if want > 0 {
                let start = $reader.state.len;
                // Bytes written at offset 0 start a new packet, so any previous
                // packet's diagnostics no longer describe `buffer[0..]`.
                if start == 0 {
                    $reader.state.last_len = 0;
                    $reader.state.last_info = None;
                }
                let end = start + want;
                maybe_await!(
                    $driver.read_fifo_raw(&mut $reader.state.buffer[start..end]),
                    $mode
                )?;
                $reader.state.len = end;
            }
            if $reader.state.len < HEADER_BLOCK {
                return Ok(None);
            }
        }

        let header = u16::from_be_bytes([$reader.state.buffer[0], $reader.state.buffer[1]]);
        let parser = DmpParser::new();
        if !parser.validate_header(header) {
            $reader.state.reject_header();
            $reader.state.reset_framing();
            maybe_await!($driver.fifo_reset(), $mode)?;
            return Err(Error::MalformedDmpPacket);
        }

        // When HEADER2 is absent, bytes [2..4] are the first payload bytes and
        // are already at the offset the parser expects.
        let header2 = if header & DmpPacketHeader::HEADER2_BIT != 0 {
            Some(u16::from_be_bytes([
                $reader.state.buffer[2],
                $reader.state.buffer[3],
            ]))
        } else {
            None
        };

        let expected = DmpParser::calculate_packet_size(header, header2);
        if expected > DmpPacketSize::MAX_PACKET_SIZE {
            $reader.state.reject_header();
            $reader.state.reset_framing();
            maybe_await!($driver.fifo_reset(), $mode)?;
            return Err(Error::MalformedDmpPacket);
        }

        // Complete the packet, retaining everything if it is still short.
        if $reader.state.len < expected {
            let available = maybe_await!($driver.read_fifo_count(), $mode)? as usize;
            let want = core::cmp::min(expected - $reader.state.len, available);
            if want > 0 {
                let start = $reader.state.len;
                let end = start + want;
                maybe_await!(
                    $driver.read_fifo_raw(&mut $reader.state.buffer[start..end]),
                    $mode
                )?;
                $reader.state.len = end;
            }
            if $reader.state.len < expected {
                return Ok(None);
            }
        }

        match parser.parse_packet(&$reader.state.buffer[..expected]) {
            Some((mut data, _)) => {
                if let Some(raw) = data.raw_accel {
                    let (x, y, z) = $driver
                        .accelerometer_calibration()
                        .apply(raw.0, raw.1, raw.2);
                    data.host_calibrated_accel = Some((x, y, z));
                }
                $reader.state.last_info = Some(DmpPacketInfo {
                    header,
                    header2,
                    size: expected,
                });
                $reader.state.last_len = expected;
                $reader.state.len = 0;
                Ok(Some(data))
            }
            None => {
                $reader.state.reject_header();
                $reader.state.reset_framing();
                maybe_await!($driver.fifo_reset(), $mode)?;
                Err(Error::MalformedDmpPacket)
            }
        }
    }};
}

/// Metadata for the most recently completed DMP packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmpPacketInfo {
    /// Primary 16-bit packet header.
    pub header: u16,
    /// Secondary 16-bit header, present when the primary `HEADER2_BIT` is set.
    pub header2: Option<u16>,
    /// Exact size of the completed packet in bytes, including header and footer.
    pub size: usize,
}

/// Retained framing state for a [`DmpFifoReader`].
///
/// All framing decisions live here so the blocking and async entry points
/// share one contract. `buffer[..len]` holds bytes of the packet currently
/// being assembled. `buffer[..last_len]` holds the last complete packet for
/// diagnostics.
struct DmpFifoState {
    buffer: [u8; DmpPacketSize::MAX_PACKET_SIZE],
    len: usize,
    last_len: usize,
    last_info: Option<DmpPacketInfo>,
    rejected: [u8; REJECTED_HEADER_MAX],
    rejected_len: usize,
}

impl DmpFifoState {
    const fn new() -> Self {
        Self {
            buffer: [0u8; DmpPacketSize::MAX_PACKET_SIZE],
            len: 0,
            last_len: 0,
            last_info: None,
            rejected: [0u8; REJECTED_HEADER_MAX],
            rejected_len: 0,
        }
    }

    /// Reset packet framing (and diagnostics) without touching the rejected
    /// header bytes.
    const fn reset_framing(&mut self) {
        self.len = 0;
        self.last_len = 0;
        self.last_info = None;
    }

    /// Reset all host-side state, including the rejected header bytes.
    const fn clear(&mut self) {
        self.reset_framing();
        self.rejected_len = 0;
    }

    /// Retain the first header-block bytes of the packet that just failed.
    fn reject_header(&mut self) {
        self.rejected[..HEADER_BLOCK].copy_from_slice(&self.buffer[..HEADER_BLOCK]);
        self.rejected_len = HEADER_BLOCK;
    }
}

/// Caller-owned DMP FIFO reader.
///
/// The reader retains partial DMP packet bytes between polls, validates packet
/// headers, and yields complete [`DmpData`] packets. Create one reader per DMP
/// stream and reuse it for the lifetime of the stream.
pub struct DmpFifoReader {
    state: DmpFifoState,
}

impl DmpFifoReader {
    /// Create a new, empty reader.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: DmpFifoState::new(),
        }
    }

    /// Discard any retained partial-packet bytes.
    ///
    /// This performs no device I/O; it only clears host-side state. It also
    /// clears [`last_rejected_header_bytes`](Self::last_rejected_header_bytes).
    pub const fn clear(&mut self) {
        self.state.clear();
    }

    /// Return the bytes of the most recently completed packet.
    ///
    /// Returns an empty slice if no packet has been completed since the last
    /// [`clear`](Self::clear).
    #[must_use]
    pub fn last_packet_bytes(&self) -> &[u8] {
        let end = self.state.last_len;
        &self.state.buffer[..end]
    }

    /// Return metadata for the most recently completed packet.
    ///
    /// Returns `None` if no packet has been completed since the last
    /// [`clear`](Self::clear).
    #[must_use]
    pub const fn last_packet_info(&self) -> Option<DmpPacketInfo> {
        self.state.last_info
    }

    /// Return the first bytes of the most recently rejected (malformed) packet.
    ///
    /// Up to the 4-byte primary/secondary header block is retained after an
    /// invalid header or a parse failure, so the raw bytes that caused
    /// [`Error::MalformedDmpPacket`] can be inspected after the error.
    ///
    /// Returns an empty slice if no packet has been rejected, or after
    /// [`clear`](Self::clear).
    #[must_use]
    pub fn last_rejected_header_bytes(&self) -> &[u8] {
        &self.state.rejected[..self.state.rejected_len]
    }

    /// Read and frame the next complete DMP packet.
    ///
    /// Returns `Ok(None)` when the FIFO does not currently contain a complete
    /// packet. Any bytes already read are retained for the next call.
    ///
    /// Returns [`Error::FifoOverflow`] after resetting the reader and device
    /// FIFO when an overflow is reported, and [`Error::MalformedDmpPacket`]
    /// after resetting the reader and device FIFO when a header is invalid or
    /// the payload cannot be parsed.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails, or on FIFO
    /// overflow or a malformed packet.
    #[cfg(not(feature = "async"))]
    pub fn read_packet<I>(
        &mut self,
        driver: &mut Icm20948Driver<I>,
    ) -> Result<Option<DmpData>, Error<I::Error>>
    where
        I: device_driver::RegisterInterface<AddressType = u8>,
    {
        read_packet_impl!(self, driver, blocking)
    }

    /// Read and frame the next complete DMP packet (async).
    ///
    /// Behaves exactly like `read_packet`, including the same retention,
    /// validation, and reset rules.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails, or on FIFO
    /// overflow or a malformed packet.
    #[cfg(feature = "async")]
    pub async fn read_packet_async<I>(
        &mut self,
        driver: &mut Icm20948Driver<I>,
    ) -> Result<Option<DmpData>, Error<I::Error>>
    where
        I: device_driver::AsyncRegisterInterface<AddressType = u8>,
    {
        read_packet_impl!(self, driver, asynchronous)
    }

    /// Drain every complete packet currently available through `callback`.
    ///
    /// Returns the number of packets delivered. Stops when the FIFO no longer
    /// holds a complete packet; partial bytes remain retained for the next
    /// call. The callback runs once per packet with no allocation.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails, or on FIFO
    /// overflow or a malformed packet.
    #[cfg(not(feature = "async"))]
    pub fn read_packets<I, F>(
        &mut self,
        driver: &mut Icm20948Driver<I>,
        mut callback: F,
    ) -> Result<usize, Error<I::Error>>
    where
        I: device_driver::RegisterInterface<AddressType = u8>,
        F: FnMut(DmpData),
    {
        let mut delivered = 0;
        while let Some(data) = self.read_packet(driver)? {
            callback(data);
            delivered += 1;
        }
        Ok(delivered)
    }

    /// Drain every complete packet currently available through `callback`
    /// (async).
    ///
    /// Behaves exactly like `read_packets`.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails, or on FIFO
    /// overflow or a malformed packet.
    #[cfg(feature = "async")]
    pub async fn read_packets_async<I, F>(
        &mut self,
        driver: &mut Icm20948Driver<I>,
        mut callback: F,
    ) -> Result<usize, Error<I::Error>>
    where
        I: device_driver::AsyncRegisterInterface<AddressType = u8>,
        F: FnMut(DmpData),
    {
        let mut delivered = 0;
        while let Some(data) = self.read_packet_async(driver).await? {
            callback(data);
            delivered += 1;
        }
        Ok(delivered)
    }
}

impl Default for DmpFifoReader {
    fn default() -> Self {
        Self::new()
    }
}
