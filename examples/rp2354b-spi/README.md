# ICM-20948 SPI Examples for RP2354B

Async SPI examples for the ICM-20948 on the custom RP2354B robot board, built
with the Embassy framework.

This workspace targets the RP235x **B** variant (`rp235xb` HAL feature, 2 MiB
flash / 512 KiB RAM memory layout, `thumbv8m.main-none-eabihf`, probe-rs
`RP235x` chip). It is deliberately independent of the other example
directories.

## Board profile

- MCU: RP2354B (RP235x, B variant)
- IMU: ICM-20948 (AK09916 magnetometer behind the ICM-20948's I2C master)
- Bus: SPI0, mode 0 (data captured on the first clock transition, clock idles
  low), CS driven as a plain GPIO
- Default clock: **7 MHz**, which reproduces the robot's level-shifted bus
- Control clock: **1 MHz**, for signal-integrity triage

All pin, DMA and clock constants live in `src/board.rs`; the binaries never
duplicate them.

`board::init_imu_or_idle` wraps `board::init_imu`, which brings the IMU up with
`Icm20948Driver::init_spi`: the soft reset clears `USER_CTRL.I2C_IF_DIS`, so SPI
mode is restored with a write-only access before any read. The legacy `init()`
polling path is for I2C only and times out over SPI.

### Pin map

| ICM-20948 / bus | RP2354B pin | `board.rs` constant |
| --------------- | ----------- | ------------------- |
| SCK             | GP18        | `SPI0_SCK_PIN`      |
| MOSI (SDI)      | GP19        | `SPI0_MOSI_PIN`     |
| MISO (SDO)      | GP20        | `SPI0_MISO_PIN`     |
| CS              | GP21        | `SPI0_CS_PIN`       |
| VCC             | 3V3         | —                   |
| GND             | GND         | —                   |
| SPI TX DMA      | DMA channel 1 | `DMA_CH_TX`       |
| SPI RX DMA      | DMA channel 2 | `DMA_CH_RX`       |

Both DMA channels share `DMA_IRQ_0`.

## Building and flashing

```bash
# Debug build
cargo build --bin direct_mag_spi

# Release build
cargo build --bin direct_mag_spi --release

# Flash and stream defmt/RTT logs (probe-rs, SWD, RP235x chip)
cargo run --bin direct_mag_spi --release

# The DMP binaries are feature-gated
cargo build --bin dmp_nine_axis_spi --features dmp
cargo build --bin dmp_six_axis_spi --features dmp
cargo build --bin dmp_fifo_dump_spi --features dmp
```

`DEFMT_LOG` defaults to `info` in `.cargo/config.toml`.

### 7 MHz default and 1 MHz control

`src/direct_mag_spi.rs` passes `board::SPI_DEFAULT_HZ` (7 MHz) to
`board::init_imu_or_idle`. To check whether a failure is signal integrity rather
than protocol behaviour, change that one argument to `board::SPI_CONTROL_HZ`
(1 MHz) and rebuild. Nothing else moves.

## Binaries

### `direct_mag_spi`

Direct AK09916 read: no DMP firmware is loaded and the DMP is never enabled.
It configures the magnetometer for continuous 100 Hz measurement and streams
at roughly 10 Hz:

```
raw=(x,y,z) uT=(x,y,z) heading_deg=NN.N ST2=ok
```

- `raw` are the signed 16-bit AK09916 counts.
- `uT` are microteslas, converted from the raw counts locally at
  **0.15 uT/LSB** (no extra bus reads).
- `heading_deg` is `atan2(y, x)` in degrees, normalized to 0..360, assuming a
  level sensor.
- `ST2=overflow` means the ST2 magnetic-overflow bit was set for that sample;
  the sample is dropped and the loop continues.

The binary is headless: it streams over RTT with no interactive prompts.

### `dmp_nine_axis_spi`

Reproduction harness for the stale-magnetometer symptom. It loads the DMP
firmware, initializes the magnetometer through `dmp_init_magnetometer`, and
runs 9-axis fusion with raw accel/gyro/mag output:

```bash
cargo build --bin dmp_nine_axis_spi --features dmp            # debug
cargo build --bin dmp_nine_axis_spi --features dmp --release  # release
cargo run   --bin dmp_nine_axis_spi --features dmp --release  # flash + RTT
```

Configuration and polling:

- `DmpConfig::nine_axis()` with raw accel, raw gyro and the DMP raw compass enabled.
- Validated rate `DmpSampleRate::Hz112`: nominal 112 Hz, **actual 112.5 Hz**.
  The DMP ODR register is written to its maximum (0), so the packet rate equals
  the selected sensor-rate bucket (56.25/112.5/225 Hz) and is not divided a
  second time. The configured rate, actual rate and ODR value are logged at
  startup and in every summary line.
- FIFO poll interval **4 ms**, faster than the 8.9 ms packet period, so the
  FIFO is drained rather than accumulated. A 100 Hz consumer that reads one
  packet per poll would back up at 112.5 Hz; drain all packets or poll at
  >= 112.5 Hz.
- Every complete packet is framed by `DmpFifoReader`, so a packet split across
  FIFO reads is retained instead of being lost. One poll drains all complete
  packets, not one packet per poll.
- A summary is logged every 11 completed packets, about 10 Hz.

#### Magnetometer lifecycle diagnostic

The `mag-reinit-diagnostic` feature turns the binary into an evidence harness
for the stale-magnetometer lifecycle defect. It logs the Bank 3 magnetometer
configuration snapshot at three points:

- `mag_cfg before-enable` — after `dmp_configure`, before `dmp_enable`.
- `mag_cfg after-enable` — immediately after `dmp_enable` succeeds.
- `mag_cfg after-reinit` — after re-running `dmp_init_magnetometer` once the
  DMP is already enabled.

The snapshot reports the raw `I2C_SLV0_ADDR/REG/CTRL`, `I2C_SLV1_ADDR/REG/DO/CTRL`,
`I2C_MST_ODR_CONFIG`, `LP_CONFIG` and `USER_CTRL` bytes, so a field that the
enable path rewrote is visible in the log. It is read-only and does not touch
interrupt or status registers.

The diagnostic re-init is **not** a fix: it only re-applies the host
configuration after enable, without re-running `dmp_configure`, to test whether
that restores the DMP raw compass. Diff the `before-enable`, `after-enable` and
`after-reinit` lines and paste them into the analysis ticket.

```bash
cargo build --bin dmp_nine_axis_spi --features "dmp mag-reinit-diagnostic"
```

Summary line:

```
dmp hdr=0x.... fusion=quat9|quat6|none euler_deg=(roll,pitch,yaw) fifo=N drained=N overflow=N malformed=N rate_cfg_hz=112 rate_actual_hz=112.5 divider=0 poll_ms=4 packets=N quat9_packets=N quat6_packets=N
```

Optional fields are logged on indented follow-up lines only when the packet
carries them: `raw_accel=(x,y,z)`, `raw_gyro=(x,y,z)`, `dmp_compass=(x,y,z)`,
`heading_accuracy=...` and `compass_accuracy=...`. `fusion=quat6` (or a
`dmp_compass` line that stops changing while the board rotates) is the DMP
falling back without the magnetometer.

The binary is headless: it streams over RTT with no interactive prompts.

### `dmp_six_axis_spi`

Control for the nine-axis harness. It uses the same DMP firmware, the same
`DmpFifoReader` drain path, the same 4 ms poll and the same 112 Hz validated
rate, but configures `DmpConfig::six_axis()` with raw accel and raw gyro and
**never initializes the magnetometer**:

```bash
cargo build --bin dmp_six_axis_spi --features dmp            # debug
cargo build --bin dmp_six_axis_spi --features dmp --release  # release
cargo run   --bin dmp_six_axis_spi --features dmp --release  # flash + RTT
```

Its summary line has the same fields as the nine-axis example except
`dmp_compass`, `heading_accuracy` and `compass_accuracy`. A stable `fusion=quat6`
stream here proves the DMP quaternion and FIFO paths work without the
magnetometer.

### `dmp_fifo_dump_spi`

Raw byte-stream dump. It configures the same 9-axis stream as
`dmp_nine_axis_spi` (raw accel/gyro/mag at 112 Hz / 112.5 Hz actual) but logs
what the framing layer actually saw rather than interpreting the payload. It is
the packet-level evidence used to separate a framing defect from a DMP
magnetometer fallback:

```bash
cargo build --bin dmp_fifo_dump_spi --features dmp            # debug
cargo build --bin dmp_fifo_dump_spi --features dmp --release  # release
cargo run   --bin dmp_fifo_dump_spi --features dmp --release  # flash + RTT
```

Dump line (one per 11 completed packets, about 10 Hz; the poll interval is
4 ms):

```
dmp-dump fifo=N raw_header=0x.... primary=0x.... secondary=0x....|none size=N parse=quat9|quat6|geomag|pquat6|raw-only|none len=N drained=N overflow=N malformed=N packets=N
```

How to read it:

- `fifo` is the FIFO count read before draining that poll; a value that keeps
  growing is a poll-rate backlog, not a magnetometer fault.
- `raw_header` is the first four bytes of the completed packet: the primary
  header followed by **either** the secondary header (when the primary
  `HEADER2_BIT` is set) **or** the first two payload bytes (when it is clear).
- `primary` and `secondary` are those header bytes decoded as big-endian 16-bit
  values; `secondary=none` means `HEADER2_BIT` was clear.
- `size` is the packet size computed from the header, and `len` is the number of
  bytes in the completed packet. They must match.
- `parse` is the highest-priority record the parser found: `quat9`, `quat6`,
  `geomag`, `pquat6`, `raw-only` or `none`.
- `overflow` and `malformed` are cumulative counters. Each event also logs an
  immediate `dmp-dump event:` line; the reader has already reset its state and
  the device FIFO by then.

A stable `primary`, sane `size`/`len` and a `parse` that flips between `quat9`
and `quat6` means the framing is healthy and the magnetometer path is the
problem. An implausible `primary`, a `size`/`len` mismatch, or repeated
`malformed` events points at framing or bus integrity instead (rerun at
`SPI_CONTROL_HZ`).

The binary is headless: it streams over RTT with no interactive prompts.

## HIL procedure (rotate 90 degrees)

1. Flash `direct_mag_spi` and attach RTT.
2. Confirm the log shows the pin map and the rotate-90 hint, then a steady
   stream of `ST2=ok` lines.
3. Note `heading_deg` with the board flat and stationary.
4. Rotate the board **90 degrees about the vertical (Z) axis** and hold it
   still.
5. `heading_deg` must move by approximately 90 degrees in the direction of the
   rotation, and `uT` must stay in the same magnitude range (a few tens of
   microteslas).
6. Rotate another 90 degrees and confirm another ~90 degree step; a full turn
   should return near the starting heading.

Interpreting the result:

- **Heading moves smoothly, uT is plausible** — direct magnetometer path is
  healthy at the current clock.
- **`ST2=overflow` repeatedly, or uT pinned at an extreme** — suspect the
  magnetometer configuration or a saturated field, not the SPI link.
- **Reads fail or values are garbage** — rerun at `SPI_CONTROL_HZ` (1 MHz) to
  separate signal integrity from protocol behaviour.

## HIL procedure (DMP nine-axis vs six-axis control)

1. Flash `dmp_six_axis_spi --features dmp` and attach RTT. Confirm the log
   shows the pin map, the configured 112/112.5 Hz rate, `poll_ms=4`, and a
   steady `fusion=quat6` summary with a plausible `drained` count. There must
   be no `dmp_compass`, `heading_accuracy` or `compass_accuracy` lines.
2. Flash `dmp_nine_axis_spi --features dmp` and attach RTT. Confirm a steady
   `fusion=quat9` summary plus `dmp_compass`, `heading_accuracy` and
   `compass_accuracy` lines.
3. Note `yaw_deg` and `dmp_compass` with the board flat and stationary.
4. Rotate the board **90 degrees about the vertical (Z) axis** and hold it
   still.
5. In the nine-axis log, `yaw_deg` must move by approximately 90 degrees and
   `dmp_compass` must change with it while `heading_accuracy` and
   `compass_accuracy` stay populated. Rotate another 90 degrees and confirm
   another step.
6. Compare the two logs:
   - **Six-axis tracks yaw, nine-axis yaw tracks and `dmp_compass` changes** —
     DMP and FIFO paths are healthy at the current clock.
   - **Six-axis tracks yaw, but nine-axis falls back to `fusion=quat6` or its
     `dmp_compass` stops changing** — stale DMP magnetometer path reproduced;
     this is the fault the harness exists to capture.
   - **Both examples show stagnant Euler angles or a growing `fifo` backlog** —
     the DMP output rate and the poll interval are mismatched, or the SPI link
     is failing; rerun at `SPI_CONTROL_HZ` (1 MHz).

No HIL log is committed to the repository; paste the relevant summary lines
into the analysis ticket.

## Features

| Feature                  | Purpose                                              |
| ------------------------ | ---------------------------------------------------- |
| `dmp`                    | Enable the driver's DMP support (`dmp_*` binaries)    |
| `dmp-lzss`               | `dmp` plus the compressed DMP firmware path          |
| `mag-reinit-diagnostic`  | Re-run `dmp_init_magnetometer` after enable and log the Bank 3 snapshot (nine-axis example) |

`direct_mag_spi` needs no feature; it uses only the direct magnetometer path.
`dmp_nine_axis_spi`, `dmp_six_axis_spi` and `dmp_fifo_dump_spi` declare
`required-features = ["dmp"]`, so they are only built when the `dmp` feature is
enabled.

## Troubleshooting

- **Device not detected** — check 3V3/GND, SCK/MOSI/MISO/CS wiring, and that
  CS idles high.
- **Bus errors at 7 MHz** — rerun with `SPI_CONTROL_HZ`; long or unshielded
  jumpers on a level-shifted bus are the usual cause.
- **Build errors** — install the target with
  `rustup target add thumbv8m.main-none-eabihf` and `cargo install probe-rs-tools`.

## License

MIT
