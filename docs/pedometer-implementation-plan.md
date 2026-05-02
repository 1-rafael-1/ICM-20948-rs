# Pedometer Implementation Plan

## Background

The ICM-20948 DMP firmware supports two related but distinct pedometer features:

- **PQuat6** (`DmpFusionMode::PedometerSixAxis`) — a 16-bit compact quaternion output driven
  by the DMP's pedestrian-motion algorithm (cadence-locked, slower than the standard QUAT6).
- **Step Detector** (`with_step_detector()`) — a 4-byte DMP-cycle timestamp written to the
  FIFO on every detected step event.
- **Step Counter** (`with_step_counter()`) — cumulative step count accumulated in DMP SRAM at
  address `PEDSTD_STEPCTR` (not in the FIFO). Must be polled separately.

Reference: [SparkFun ICM-20948 Arduino Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)
— specifically `src/util/ICM_20948_DMP.h` and `src/util/ICM_20948_C.c`.

---

## What Is Already Implemented

| Component | File | Notes |
|---|---|---|
| `DmpFusionMode::PedometerSixAxis` + `DmpConfig::pedometer_six_axis()` | `src/dmp/mod.rs` | ✅ |
| `with_step_detector()` / `with_step_counter()` builder methods | `src/dmp/mod.rs` | ✅ |
| `DmpFeatures::STEP_DETECTOR`, `STEP_COUNTER`, `QUATERNION_P6AXIS` | `src/dmp/config.rs` | ✅ |
| `DmpControl1Flags::STEP_DETECTOR` (0x0010) and `PQUAT6` (0x0200) | `src/dmp/config.rs` | ✅ |
| `DmpMotionEventControl::PEDOMETER_INTERRUPT` (0x2000) set by `as_motion_event()` | `src/dmp/config.rs` | ✅ |
| `DmpPacketHeader::STEP_BIT` / `PQUAT6_BIT` | `src/dmp/config.rs` | ✅ |
| `DmpPacketSize::PEDOMETER` (4 bytes) / `PQUAT6` (6 bytes) | `src/dmp/config.rs` | ✅ |
| `DmpOdrRegisters::PQUAT6` + ODR written in `get_init_sequence()` | `src/dmp/config.rs` | ✅ |
| FIFO parser: `STEP_BIT` → 4-byte big-endian `pedometer_timestamp` | `src/dmp/parser.rs` | ✅ |
| FIFO parser: `PQUAT6_BIT` → `parse_pquat6()` → Q14 fixed-point → `Quaternion` | `src/dmp/parser.rs` | ✅ |
| `DmpData::pedometer_timestamp: Option<u32>` | `src/dmp/mod.rs` | ✅ |
| `DmpData::pedometer_quaternion: Option<Quaternion>` | `src/dmp/mod.rs` | ✅ |
| `read_dmp_memory()` / `read_dmp_memory_async()` in driver | `src/device.rs` | ✅ |

---

## Implementation Phases

### Phase 1 — Correctness Fix + Memory Addresses ✅ DONE (97/97 tests passing)

**Files:** `src/dmp/config.rs`

#### 1a — Fix `as_data_ready()` for `QUATERNION_P6AXIS`

`QUATERNION_P6AXIS` (PQuat6) is an accel + gyro fusion but was absent from the ACCEL and
GYRO conditions in `DmpFeatures::as_data_ready()`. Without these bits in `DATA_RDY_STATUS`
the DMP never receives sensor triggers and produces no PQuat6 output.

Add `Self::QUATERNION_P6AXIS` alongside `Self::QUATERNION_6AXIS` in both the ACCEL and GYRO
conditions inside `as_data_ready()`.

#### 1b — Fix `as_motion_event()` for `QUATERNION_P6AXIS`

Same issue: `QUATERNION_P6AXIS` was missing from the `ACCEL_CALIBR` and `GYRO_CALIBR`
conditions in `as_motion_event()`, so the internal calibration engines do not run for PQuat6.

Add `Self::QUATERNION_P6AXIS` alongside `Self::QUATERNION_6AXIS` in both the `ACCEL_CALIBR`
and `GYRO_CALIBR` conditions inside `as_motion_event()`.

#### 1c — Add pedometer DMP memory address constants

Add four new `pub const` items to `DmpMemoryAddresses` (from `ICM_20948_DMP.h`):

| Constant | Value | Purpose |
|---|---|---|
| `PEDSTD_STEPCTR` | `54 * 16 = 0x0360` | 4-byte cumulative step counter in DMP SRAM |
| `PEDSTD_STEPCTR2` | `58 * 16 + 8 = 0x03A8` | Extended high-word step counter |
| `STPDET_TIMESTAMP` | `18 * 16 + 8 = 0x0128` | Last step event DMP timestamp |
| `PEDSTEP_IND` | `19 * 16 + 4 = 0x0134` | Step indicator bits |

#### 1d — Add / update unit tests

- `test_pquat6_data_ready_includes_accel_and_gyro` — asserts `as_data_ready()` sets both
  `ACCEL` and `GYRO` for `pedometer_six_axis()` config.
- `test_pquat6_motion_event_includes_calibration` — asserts `as_motion_event()` sets
  `ACCEL_CALIBR` and `GYRO_CALIBR` for `pedometer_six_axis()` config.
- `test_pedometer_step_detector_data_ready` — asserts step detector config sets ACCEL in
  data_ready and PEDOMETER_INTERRUPT in motion event.
- `test_pedometer_memory_addresses` — asserts the four new address constants equal their
  expected values.

---

### Phase 2 — New `dmp_read_step_count()` API ✅ DONE (97/97 tests passing)

**Files:** `src/device.rs`

Add two new `#[cfg(feature = "dmp")]` methods — one in each of the sync and async `impl`
blocks — placed immediately after the existing `read_dmp_memory` / `read_dmp_memory_async`
methods:

```rust
/// Read the cumulative step count from DMP SRAM.
///
/// The DMP accumulates a step count at `PEDSTD_STEPCTR` (address 0x0360).
/// This value is NOT delivered through the FIFO; it must be polled via this
/// method whenever a step event is detected (i.e. when `pedometer_timestamp`
/// is `Some` in a `DmpData` packet).
///
/// Returns the total number of steps counted since the last DMP reset.
pub fn dmp_read_step_count(&mut self) -> Result<u32, Error<I::Error>> {
    use crate::dmp::config::DmpMemoryAddresses;
    let mut buf = [0u8; 4];
    self.read_dmp_memory(DmpMemoryAddresses::PEDSTD_STEPCTR, &mut buf)?;
    Ok(u32::from_be_bytes(buf))
}

// (async version mirrors the above using read_dmp_memory_async)
```

**Design note:** step count is polled explicitly rather than auto-populated inside
`dmp_read_fifo()` to avoid the extra DMP SRAM read on every FIFO call. The caller checks
whether `DmpData::pedometer_timestamp.is_some()` and polls accordingly.

---

### Phase 3 — Pedometer Example ✅ DONE (builds clean for thumbv8m.main-none-eabihf, 97/97 tests passing)

**Files:**
- `examples/rp2350-async/src/pedometer_async.rs` (new)
- `examples/rp2350-async/Cargo.toml` (add `[[example]]` entry)

The example will:

1. Initialise the driver and load DMP firmware (no magnetometer needed).
2. Configure DMP with `DmpConfig::pedometer_six_axis().with_step_detector()` at 56 Hz.
3. Enable interrupt on INT1 (GPIO14) and wait for DMP packets.
4. On each packet:
   - If `pedometer_quaternion` is `Some`, log orientation (roll/pitch/yaw).
   - If `pedometer_timestamp` is `Some`, call `dmp_read_step_count()` and log the total.
5. Include a comment block explaining the two-call pattern and the meaning of the DMP
   cycle timestamp vs wall-clock time.

---

### Phase 4 — Docs + Additional Tests ✅ DONE (99/99 tests passing)

**Files:** `src/dmp/config.rs`, `src/dmp/mod.rs`

- Update module-level doc comment in `src/dmp/mod.rs` to include a pedometer usage example
  showing `pedometer_six_axis().with_step_detector()` and the `dmp_read_step_count()` pattern.
- Expand doc comment on `DmpFusionMode::PedometerSixAxis` to explain PQuat6 vs QUAT6.
- Expand doc comment on `DmpConfig::with_step_detector()` / `with_step_counter()` to
  document the two-call pattern.
- Add integration-level test (under `tests/` if mock supports it, otherwise as a lib test)
  verifying packet-size calculation for a pedometer config.

---

## Key Reference Values (from SparkFun `ICM_20948_DMP.h`)

| Symbol | Value | Description |
|---|---|---|
| `DMP_header_bitmap_Step_Detector` | `0x0010` | FIFO header bit for step event |
| `DMP_header_bitmap_PQuat6` | `0x0200` | FIFO header bit for PQuat6 |
| `DMP_Data_Output_Control_1_Step_Detector` | `0x0010` | DATA_OUT_CTL1 step detector bit |
| `DMP_Data_Output_Control_1_PQuat6` | `0x0200` | DATA_OUT_CTL1 PQuat6 bit |
| `DMP_Motion_Event_Control_Pedometer_Interrupt` | `0x2000` | MOTION_EVENT_CTL pedometer bit |
| `icm_20948_DMP_Step_Detector_Bytes` | 4 | FIFO bytes per step event (big-endian u32 DMP cycles) |
| `icm_20948_DMP_PQuat6_Bytes` | 6 | FIFO bytes per PQuat6 packet (3 × i16 Q14 big-endian) |
| `PEDSTD_STEPCTR` | `54*16 = 0x0360` | DMP SRAM address for step count |
| `ODR_PQUAT6` | `10*16+4 = 0x00A4` | PQuat6 ODR register |
| `ODR_CNTR_PQUAT6` | `8*16+4 = 0x0084` | PQuat6 ODR counter register |

---

## Review Findings Resolution

A full senior-developer review was conducted after all four phases completed.
All findings from `docs/pedometer-review.md` have been addressed:

### 🔴 Required changes — all resolved

| ID | Finding | Resolution |
|---|---|---|
| R1 | `pedometer_timestamp` doc said "timestamp/count" — "count" is wrong | Fixed: field doc replaced with accurate multi-paragraph explanation distinguishing DMP cycle-counter from step total |
| R2 | `dmp_read_step_count()` did not document counter persistence or lack of reset | Fixed: `# Counter lifecycle` section added to both sync and async docs, with baseline-subtract pattern |
| R3 | Torn-read risk in `dmp_read_step_count()` undocumented | Fixed: `# Caveats — shared SRAM, non-atomic read` section added to both sync and async docs |

### 🟡 Suggestions — all applied

| ID | Finding | Resolution |
|---|---|---|
| S1 | `with_step_counter()` / `with_step_detector()` set the same hardware bit | Fixed: `#[deprecated]` added to `with_step_counter()` pointing to `with_step_detector()` |
| S2 | `QUATERNION_P6AXIS` absent from HEADER2 condition with no comment | Fixed: 13-line comment added explaining the intentional omission |
| S3 | `read_register(0x7D, 8, buffer)` passes misleading `size_bits` literal | Fixed: changed to `0` with explanatory comment in both sync and async |
| S4 | Chinese comment in `parser.rs` `parse_pquat6()` | Fixed: replaced with English |
| S5 | No test for `pedometer_six_axis()` alone (HEADER2 regression guard) | Fixed: `test_packet_size_pedometer_six_axis_only` added |
| S6 | FIFO reset after DMP enable (wrong order vs SparkFun reference) | Fixed: swapped to `reset_fifo()` → `dmp_enable(true)` with rationale comment |
| S7 | "Maximum" vs "validated rate" for 56 Hz | Fixed: comment updated to explain calibrated-constant constraint |
| S8 | `PEDSTD_STEPCTR2` defined but never read, no explanation | Fixed: doc expanded explaining u32 overflow impossibility |
| S9 | Two-call pattern can return count one step ahead | Fixed: `# Note on timing` added to both sync and async docs |
| S10 | `DmpData` has no `step_count` field, design rationale undocumented | Fixed: struct-level doc updated with `# Why there is no step_count field` section |

**Final state: 100/100 tests passing, pedometer example builds clean for `thumbv8m.main-none-eabihf`.**

---

## Notes

- **Byte ordering in `parse_pquat6`**: SparkFun uses a `{1,0, 3,2, 5,4}` reorder table to
  convert big-endian FIFO pairs into little-endian `int16_t` C storage. The Rust implementation
  using `i16::from_be_bytes([data[0], data[1]])` achieves the identical numeric result.
- **Step detector vs step counter**: Both set `DmpControl1Flags::STEP_DETECTOR` in
  `DATA_OUT_CTL1`. The FIFO payload is always a 4-byte DMP cycle timestamp. The cumulative
  count lives in DMP SRAM and must be read via `dmp_read_step_count()`.
- **PQuat6 cadence**: PQuat6 output rate is derived from the DMP's pedometer cadence, not a
  fixed Hz rate. The ODR register sets a maximum; actual output depends on motion.
