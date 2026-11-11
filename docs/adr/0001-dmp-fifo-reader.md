# ADR 0001: DMP FIFO reader framing

## Status

Accepted.

## Context

The DMP writes variable-length packets into the FIFO. A packet begins with a
2-byte header, optionally followed by a 2-byte secondary header, payload fields
in hardware order, and a 2-byte footer. The previous `dmp_read_fifo` path reads
a fixed 4-byte header block first. If the payload has not arrived yet, it
returns no packet and discards the header block. On the next poll the payload
bytes are interpreted as a new header: the stream has desynchronised and every
subsequent packet can be misread or dropped.

The fundamental problem is that packet boundaries are lost whenever the host
consumes bytes that do not form a complete packet. Reading more than the FIFO
count allows, or waiting for a maximum-size packet, does not solve it: the
count is authoritative and packets are variable length.

## Decision

Add a public, caller-owned `DmpFifoReader` that retains partial DMP packet bytes
between polls.

- The reader owns a buffer sized for one maximum-size DMP packet. The driver
  carries no per-instance packet buffer, so there is no added cost for drivers
  that never use the DMP.
- The framing state machine is defined once and shared by the blocking and
  async entry points; only the device I/O is awaited differently.
- The reader reads only bytes reported by the FIFO count. If a packet is
  incomplete it retains the bytes and returns `Ok(None)`. It never consumes
  bytes that belong to an incomplete packet without retaining them.
- The primary header is validated. An invalid header yields
  `Error::MalformedDmpPacket`, and both the reader state and the device FIFO
  are reset. A FIFO overflow yields `Error::FifoOverflow` with the same resets.
- `read_packet` yields one packet, and `read_packets` drains all complete
  packets through a no-allocation callback.
- The old `dmp_read_fifo` remains for the migration window and is removed by a
  later ticket rather than deprecated. Update 2026-11-11: ticket 06 removed
  `dmp_read_fifo` and `dmp_read_quaternion` for 0.3.0; the migration window is
  complete.

## Consequences

- Packet boundaries survive a split across any number of polls, and a partial
  packet costs at most one maximum-size packet of caller-owned memory.
- Callers must keep one reader per DMP stream and must not share it between
  streams.
- A partially received packet is delivered on a later poll, adding up to one
  poll of latency rather than losing data.
- Corruption is visible as an error and a reset instead of being silently
  resynchronised.

## Rejected alternatives

- **Reset on incomplete packet.** This is the current defect: it loses a valid
  header and can desynchronise the stream, so it cannot satisfy the no-loss
  requirement.
- **Wait for a maximum-size packet before reading.** Small packets never reach
  the maximum size, so polling stalls; reading a maximum-size block also
  over-reads into the next packet and cannot distinguish packet boundaries.
- **Per-driver buffer.** It adds a maximum-packet-sized cost to every driver
  instance, including drivers that never enable the DMP, which is unacceptable
  for small MCUs.
- **Deprecate the old API without a reader.** The old contract cannot be made
  safe without retained reader state, so deprecation would leave a
  desync-prone path available. The old API is removed instead.
