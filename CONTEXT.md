# ICM-20948 Driver

The `icm20948-rs` crate is a platform-agnostic Rust driver for the ICM-20948
9-axis IMU, including its AK09916 magnetometer and DMP.

## Language

**DMP output rate**:
The rate at which the DMP writes DMP packets into the FIFO; 56.25, 112.5, or
225 Hz for the validated sample rates. The DMP ODR register is written as 0
(maximum), so this rate equals the configured sensor ODR bucket; the "distinct"
contract is conceptual, not a different number.
_Avoid_: sample rate, ODR

**Sensor sample rate**:
The hardware gyroscope or accelerometer output data rate feeding the DMP; it is
distinct from the DMP output rate.
_Avoid_: DMP rate, packet rate

**Validated sample rate**:
One of the three DMP configurations (nominal 56, 112, 225 Hz) for which the
driver has calibration parameters; other rates are not part of the public
contract.
_Avoid_: arbitrary rate, custom rate

**Direct magnetometer read**:
Reading AK09916 data through the ICM-20948 I2C master without involving the DMP.
_Avoid_: raw mag read, passthrough

**DMP raw compass**:
The 16-bit compass field the DMP includes in a DMP packet; it is not evidence
that the magnetometer is fresh.
_Code_: `DmpData::dmp_compass` field, `DmpConfig::dmp_compass` flag, and the
`with_dmp_compass()` builder.
_Avoid_: raw mag, direct read

**DMP packet**:
A variable-length FIFO record with a header, an optional secondary header,
payload fields in hardware order, and a footer.
_Avoid_: frame

**FIFO desync**:
Loss of DMP packet boundaries when the host consumes bytes that do not form a
complete packet.
_Avoid_: corruption, misalignment

**DMP FIFO reader**:
The host-side state object that retains partial DMP packet bytes and yields
complete packets.
_Avoid_: buffer, parser

**Fusion mode**:
The DMP algorithm selection; SixAxis fuses accelerometer and gyroscope, NineAxis
adds the magnetometer.
_Avoid_: Axis6/Axis9 when referring to the driver API

**Heading accuracy**:
The per-packet accuracy word accompanying a 9-axis quaternion; it is not a
freshness proof.
_Avoid_: compass accuracy

**Compass accuracy**:
The optional secondary-header accuracy field for the compass; it is absent when
the DMP does not emit it.
_Avoid_: heading accuracy
