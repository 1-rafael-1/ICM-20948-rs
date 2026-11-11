//! Shared DMP packet classification for the RP2354B SPI examples.
//!
//! The nine-axis, six-axis and FIFO dump binaries all need to name which
//! record a completed DMP packet carries. Keeping that vocabulary here means
//! the examples cannot drift from one another.

use icm20948::dmp::DmpData;

/// Highest-priority record found in a completed DMP packet.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PacketKind {
    /// Native 9-axis quaternion: magnetometer is part of the solution.
    Quat9,
    /// 6-axis quaternion: the magnetometer dropped out of the solution.
    Quat6,
    /// Geomagnetic rotation vector.
    Geomag,
    /// Pedometer quaternion.
    PQuat6,
    /// No fused record, but at least one raw sensor record.
    RawOnly,
    /// Nothing the parser recognises.
    None,
}

impl PacketKind {
    /// Stable log label for this packet kind.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Quat9 => "quat9",
            Self::Quat6 => "quat6",
            Self::Geomag => "geomag",
            Self::PQuat6 => "pquat6",
            Self::RawOnly => "raw-only",
            Self::None => "none",
        }
    }

    /// Label for the nine/six-axis `fusion=` field.
    ///
    /// Only the two fused quaternions count as a fusion result. Any other
    /// record is reported as `none` so the summary never claims a 9-axis
    /// quaternion the DMP did not produce.
    ///
    /// Used by the nine/six-axis binaries; the dump binary uses [`name`](Self::name).
    #[allow(dead_code)]
    pub const fn fusion_name(self) -> &'static str {
        match self {
            Self::Quat9 | Self::Quat6 => self.name(),
            Self::Geomag | Self::PQuat6 | Self::RawOnly | Self::None => "none",
        }
    }
}

/// Classify a packet by the highest-priority record it carries.
pub fn classify(packet: &DmpData) -> PacketKind {
    if packet.quaternion_9axis.is_some() {
        PacketKind::Quat9
    } else if packet.quaternion_6axis.is_some() {
        PacketKind::Quat6
    } else if packet.geomag_rotation_vector.is_some() {
        PacketKind::Geomag
    } else if packet.pedometer_quaternion.is_some() {
        PacketKind::PQuat6
    } else if packet.raw_accel.is_some()
        || packet.raw_gyro.is_some()
        || packet.dmp_compass.is_some()
    {
        PacketKind::RawOnly
    } else {
        PacketKind::None
    }
}
