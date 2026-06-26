# Pedometer Implementation Review

**Reviewed files:** `src/dmp/config.rs`, `src/dmp/mod.rs`, `src/dmp/parser.rs`,
`src/device.rs`, `examples/rp2350-async/src/pedometer_async.rs`,
`examples/rp2350-async/Cargo.toml`

**Test baseline:** `cargo test --lib --features dmp` → **99 passed, 0 failed**

---

## Summary

**Verdict: Needs minor-to-moderate changes.**

The core mechanics are sound: the PEDSTD_STEPCTR address is correct, the
big-endian decode is correct, both `#[cfg(feature = "dmp")]` guards are in
place, and the Phase 1 PQuat6 trigger-mask fixes are correct and match the
SparkFun/InvenSense reference. The example is well-commented and demonstrates
the intended two-call pattern clearly.

However, there are several documentation gaps that will mislead junior
developers (one of the field-level doc comments is actively wrong), a public API
with two methods that set *exactly the same hardware bit* without any visible
differentiation, and a handful of embedded-specific hazards (torn read, interrupt
latency, counter persistence) that need to be documented so that readers learn
the right mental model.

None of the issues are "the hardware will explode" severity, but several will
cause bugs in production code written by developers who read only the doc
comments rather than the module-level prose.

---

## 🟢 What Is Done Well

1. **PEDSTD\_STEPCTR address is correct.** `54 * 16 = 0x0360` matches the
   SparkFun `ICM_20948_DMP.h` constant exactly. The formula is preserved in
   source rather than being collapsed to the raw hex, which makes the source of
   the magic number auditable at a glance.

2. **Big-endian decoding is correct.** `u32::from_be_bytes(buf)` is the right
   call. The DMP SRAM stores multi-byte values big-endian, consistent with the
   rest of the ICM-20948's register format.

3. **Phase 1 PQuat6 trigger-mask fixes are correct.** Adding
   `QUATERNION_P6AXIS` to the `ACCEL` + `GYRO` clauses in `as_data_ready()`,
   and to the `ACCEL_CALIBR` + `GYRO_CALIBR` clauses in `as_motion_event()`,
   matches the SparkFun/InvenSense reference: the DMP pedestrian 6-axis fusion
   algorithm consumes both sensors and needs both calibration engines running.
   The two new unit tests (`test_pquat6_data_ready_includes_accel_and_gyro` and
   `test_pquat6_motion_event_includes_calibration`) pin this behaviour correctly.

4. **`#[cfg(feature = "dmp")]` guards are present on every new function.**
   Both `read_dmp_memory` (L1467 sync / L6469 async) and `dmp_read_step_count`
   carry the attribute. No DMP-specific code leaks into non-DMP builds.

5. **Both sync and async variants are provided and structurally identical.** The
   duplication is unavoidable given the Embassy async model. The implementations
   are symmetric and easy to compare.

6. **Packet size arithmetic is correct.** Working through the test case manually:
   `Header(2) + Header2-word(2) + PQuat6(6) + Step-timestamp(4) + Footer(2) = 16`.
   The test hardcodes `16` and the comment shows the derivation, so any future
   regression will be caught immediately. The step-detector-only test (10 bytes)
   is also correct.

7. **The DMP cycle counter nature of `pedometer_timestamp` is well-explained at
   module level.** The module doc, the `PedometerSixAxis` enum variant doc, and
   the example all consistently use the phrase "DMP cycle counter, not
   wall-clock time" and explain how to compute cadence. This is one of the most
   common misunderstandings when porting pedometer code, and the docs address it
   proactively.

8. **Latched interrupt configuration is correctly used.** `latch_enabled: true`
   keeps INT1 high until a register read occurs, so the async executor cannot
   miss the edge even if it is temporarily busy with other tasks. This is the
   correct setting for the Embassy `wait_for_high()` pattern.

9. **The example uses `match` on `dmp_read_step_count()`** rather than
   `.unwrap()`, so it handles I2C errors gracefully and logs them. This is good
   practice and a good example for junior developers to follow.

10. **`[[bin]]` entry in `Cargo.toml` correctly gates on `required-features =
    ["dmp"]`**, preventing accidental compilation attempts when the DMP feature
    is not enabled.

---

## 🟡 Suggestions (non-blocking)

### S1 — `with_step_detector()` and `with_step_counter()` set the exact same hardware bit

**Location:** `src/dmp/config.rs`, `as_control1()` ~L550, `as_motion_event()`
~L649, and the two builder methods ~L315 and ~L334

**Issue:** Both `STEP_DETECTOR` and `STEP_COUNTER` map to the single
`DmpControl1Flags::STEP_DETECTOR` bit, the single `PEDOMETER_INTERRUPT` bit in
`MOTION_EVENT_CTL`, and the single `ACCEL` bit in `DATA_RDY_STATUS`. There is
no hardware difference at all between calling `with_step_detector()` and calling
`with_step_counter()`. Despite this, the builder exposes them as two distinct
choices and neither deprecates the other.

**Why it matters:** A user reading the API will naturally assume the two methods
produce different DMP behavior. When they discover they are identical, they lose
trust in the API, and junior developers often conclude that they must be missing
something and add *both* calls "to be safe", which is noise.

The `DmpData` struct also reinforces the confusion: it has a `pedometer_timestamp`
field (set by the STEP_BIT) but no `step_count` field. A user who calls
`with_step_counter()` expecting to find a count somewhere in `DmpData` will be
confused when they cannot find one.

**Suggested fix:** Deprecate `with_step_counter()` in favour of
`with_step_detector()`, add a clear `/// Deprecated` note, and move the
explanation of why there are two names (historical API compatibility) into the
deprecation notice. Alternatively, remove one entirely if the API is not yet
stable.

```ICM-20948-rs/src/dmp/config.rs#L334-337
    /// Enable the pedometer step counter.
    ///
    /// # Deprecated
    ///
    /// This method sets the same `STEP_DETECTOR` hardware bit as
    /// [`with_step_detector()`](Self::with_step_detector). Prefer
    /// `with_step_detector()`. This alias is retained for source compatibility
    /// and will be removed in a future version.
    #[deprecated(since = "0.x.0", note = "use `with_step_detector()` instead")]
    pub const fn with_step_counter(mut self) -> Self {
        self.step_counter = true;
        self
    }
```

---

### S2 — `QUATERNION_P6AXIS` is absent from the `HEADER2` enable condition with no explanatory comment

**Location:** `src/dmp/config.rs`, `as_control1()` ~L554

**Issue:** The HEADER2 condition enables the secondary-header word when
`QUATERNION_6AXIS`, `QUATERNION_9AXIS`, or `GEOMAG_ROTATION_VECTOR` are active,
but NOT when `QUATERNION_P6AXIS` is active (unless a step detector is also
enabled). This is intentionally correct — the lightweight PQuat6 output format
does not include accuracy bytes (DATA_OUT_CTL2 is always empty for a
PQuat6-only config), so HEADER2 is unnecessary — but to a reader the asymmetry
looks like an omission.

**Why it matters:** A junior developer looking at this code, especially one who
has just seen that QUAT6 always forces HEADER2, will add `QUATERNION_P6AXIS` to
the list "fixing the bug", which would break the packet size calculation and the
parser for pure-PQuat6 configurations.

**Suggested fix:** Add a comment.

```ICM-20948-rs/src/dmp/config.rs#L554-558
        // HEADER2 is required when accuracy bytes (DATA_OUT_CTL2) are in use,
        // when any step event is active (step indicator lives in HEADER2 word),
        // or when the standard fusion outputs (QUAT6/QUAT9/GEOMAG) are enabled,
        // because those modes always accompany accuracy reporting in the reference
        // firmware. PQUAT6 is intentionally excluded: it is a lightweight
        // 6-byte output that has no associated accuracy fields, so adding HEADER2
        // for it would waste 2 bytes per packet with nothing to put in them.
```

---

### S3 — `read_register(0x7D, 8, buffer)` passes a misleading `size_bits` literal

**Location:** `src/device.rs` ~L1507 (sync) and ~L6505 (async)

**Issue:** The `RegisterInterface` trait's `read_register` signature takes a
`size_bits: u32` parameter that is ignored by both the I2C and SPI
implementations (`let _ = size_bits;`). The call site passes the literal `8`,
which is the width of a single-byte register in bits. When `buffer` is 4 bytes
long (the step-count read), a reader might believe only 1 byte (8 bits) will be
transferred — the opposite of what happens.

**Why it matters:** This is a pre-existing pattern in the codebase, but the new
`dmp_read_step_count()` function is the first place where the mismatch between
the literal (`8` bits = 1 byte) and the actual transfer (`buffer.len()` bytes)
is directly observable in the same function. A developer debugging a "why am I
only getting one byte" bug will look at this call first.

**Suggested fix:** Either add a comment at the call site, or define a symbolic
constant for the ignored value.

```ICM-20948-rs/src/device.rs#L1507-1509
        // `size_bits` is ignored by both the I2C and SPI implementations;
        // the actual byte count is determined by `buffer.len()`.  Pass 0 to
        // make it obvious this argument carries no information.
        self.device.interface.read_register(0x7D, 0, buffer)?;
```

---

### S4 — A comment in `parser.rs` is in Chinese

**Location:** `src/dmp/parser.rs`, `parse_pquat6()` ~L388

**Issue:** The comment `// PQUAT6 提取 3 × 16-bit values (big-endian)` is in
Chinese. The rest of the codebase is entirely in English.

**Suggested fix:** Replace with `// PQUAT6: extract 3 × 16-bit signed integers
(big-endian, Q14 format)`.

---

### S5 — No test for `pedometer_six_axis()` alone (no step detector)

**Location:** `src/dmp/config.rs` test module

**Issue:** The new packet-size tests only cover two configurations:
`pedometer_six_axis() + with_step_detector()` (16 bytes) and
`DmpConfig::new() + with_step_detector()` (10 bytes). There is no test for
`pedometer_six_axis()` alone.

**Why it matters:** For `pedometer_six_axis()` alone the expected packet is
`Header(2) + PQuat6(6) + Footer(2) = 10 bytes`, with HEADER2 correctly absent.
If someone later "fixes" the HEADER2 condition to include `QUATERNION_P6AXIS`
(see S2), this test would catch the resulting packet size regression.

**Suggested fix:**

```ICM-20948-rs/src/dmp/config.rs#L1670-1680
    #[test]
    fn test_packet_size_pedometer_six_axis_only() {
        // PQuat6 without step detector: HEADER2 is not needed because there
        // are no accuracy bytes to report.
        // Header (2) + PQuat6 (6) + Footer (2) = 10 bytes
        let config = DmpConfig::pedometer_six_axis();
        let size = config.packet_size();
        assert_eq!(size, 10,
            "pedometer_six_axis alone should be 10 bytes, got {}", size);
        let features = config.get_active_features();
        let c1 = features.as_control1();
        assert!(!c1.contains(DmpControl1Flags::HEADER2),
            "HEADER2 must NOT be set for PQUAT6 without step detector");
    }
```

---

### S6 — FIFO reset happens *after* DMP enable; SparkFun reference does it before

**Location:** `examples/rp2350-async/src/pedometer_async.rs` ~L131

**Issue:** The example calls `dmp_enable(true)` and then `reset_fifo()`. The
SparkFun reference (and InvenSense eMD SDK) always reset the FIFO *before*
enabling the DMP so that no packets from the pre-DMP mode appear in the queue.

**Why it matters:** With the current order, the DMP starts producing packets for
the ~1 µs window between `dmp_enable(true)` and `reset_fifo()`. In practice
those packets are flushed by the reset, so the effect is harmless. But the order
is pedagogically wrong for junior developers who may copy this pattern to other
drivers where the timing window is larger.

**Suggested fix:** Swap the two calls.

```ICM-20948-rs/examples/rp2350-async/src/pedometer_async.rs#L129-133
    // Reset FIFO *before* enabling the DMP so that no pre-DMP packets appear
    // in the queue.  InvenSense eMD SDK and SparkFun both use this order.
    imu.reset_fifo().await.unwrap();
    imu.dmp_enable(true).await.unwrap();
```

---

### S7 — "Maximum" vs "optimal" for 56 Hz

**Location:** `examples/rp2350-async/src/pedometer_async.rs` comment ~L120

**Issue:** The comment reads "56 Hz is the recommended maximum for the
pedestrian algorithm". The word *maximum* implies that higher rates are possible
but useless. The actual constraint is that 56 Hz is the *calibrated* rate: the
`ACCEL_ONLY_GAIN`, `ACCEL_ALPHA_VAR`, and `ACCEL_A_VAR` constants baked into
the DMP calibration sequence have been validated at 56 Hz (and 112 Hz and 225
Hz). Running at a different rate with uncalibrated coefficients degrades step
detection accuracy — it does not simply waste power.

**Suggested fix:** Replace "maximum" with "optimal/validated":

```ICM-20948-rs/examples/rp2350-async/src/pedometer_async.rs#L120-122
    // * 56 Hz is the validated rate for the pedestrian algorithm; the
    //   calibration constants (ACCEL_ONLY_GAIN etc.) are tuned for this rate.
    //   Using a different rate requires re-deriving those constants.
```

---

### S8 — `PEDSTD_STEPCTR2` is defined but never read; this should be explained

**Location:** `src/dmp/config.rs`, `DmpMemoryAddresses::PEDSTD_STEPCTR2` ~L757

**Issue:** The constant is defined with a comment claiming it is the
"extended (high-word) step counter", suggesting that a complete step count
requires reading both words. Yet `dmp_read_step_count()` only reads
`PEDSTD_STEPCTR` (4 bytes, u32). A developer seeing the `STEPCTR2` constant
may believe the step count is silently truncated.

**Why it matters:** At walking cadence it takes roughly 136 years of continuous
walking to overflow a u32 (4 294 967 295 steps at 1 step/sec). Practically, the
DMP is reset every power cycle, so STEPCTR2 will never be non-zero. But without
a comment explaining this, developers will ask "am I losing the high word?".

**Suggested fix:** Add an explanatory comment to both the constant and
`dmp_read_step_count()`.

```ICM-20948-rs/src/dmp/config.rs#L757-762
    /// DMP SRAM address of the extended (high-word) step counter (4 bytes).
    ///
    /// This word would only become non-zero after ~4 billion steps, which is
    /// physically impossible in a single DMP session (power cycling resets
    /// the DMP and both counter words).  `dmp_read_step_count()` reads only
    /// `PEDSTD_STEPCTR` (the low word) for this reason.
    pub const PEDSTD_STEPCTR2: u16 = 58 * 16 + 8; // 0x03A8
```

---

### S9 — The two-call pattern can return a count that is one step ahead of the timestamp

**Location:** `src/device.rs` `dmp_read_step_count()` doc, and
`examples/rp2350-async/src/pedometer_async.rs` ~L158

**Issue:** The two-call pattern is:

```ICM-20948-rs/examples/rp2350-async/src/pedometer_async.rs#L158-165
    if let Some(ts) = packet.pedometer_timestamp {
        match imu.dmp_read_step_count().await {
            Ok(steps) => { ... }
```

Between the moment `dmp_read_fifo()` captures the FIFO packet and the moment
`dmp_read_step_count()` reads DMP SRAM, a second step can occur. When it does,
`steps` will reflect the *new* total (e.g., 3) while `ts` corresponds to the
*previous* step (step 2). In other words, the count and the timestamp can be
one step out of sync.

**Why it matters:** At brisk walking pace (~2 steps/sec) and a 400 kHz I2C
bus, the I2C read takes roughly 80 µs. The probability of a step landing in that
window is 80 µs / 500 ms ≈ 0.016 % per step. Over a 30-minute walk that is
~3 600 steps, so on average ~0.6 events. In a long-running fitness application
this adds up to a real (if minor) discrepancy.

**Suggested fix:** Add a sentence to the `dmp_read_step_count()` doc.

```ICM-20948-rs/src/device.rs#L1519-1522
    /// # Note on timing
    ///
    /// `dmp_read_step_count()` reads DMP SRAM in a separate I2C transaction
    /// from `dmp_read_fifo()`.  If a second step occurs between the two calls,
    /// the returned count will be one ahead of the step described by the FIFO
    /// timestamp.  For most applications this is negligible.
```

---

### S10 — `DmpData` has no `step_count` field; the design decision is not documented

**Location:** `src/dmp/mod.rs`, `DmpData` struct ~L454

**Issue:** `pedometer_timestamp` lives in `DmpData` because it arrives from the
FIFO as part of the packet. `step_count` does *not* live in `DmpData` because
it requires a separate SRAM read. This is a deliberate and correct design
choice, but it is not documented anywhere.

**Why it matters:** A junior developer who reads "step counter" in the
`with_step_counter()` doc, looks at `DmpData` for a `step_count` field, does
not find one, and then searches for another way to get it will waste significant
time. A short doc note on `DmpData` would short-circuit that search.

**Suggested fix:** Add a comment near the end of the `DmpData` struct.

```ICM-20948-rs/src/dmp/mod.rs#L503-506
    /// DMP cycle counter value at the moment a step was detected.
    ///
    /// This is **not** a wall-clock timestamp and **not** the cumulative step
    /// total.  To get the total step count, call `dmp_read_step_count()` after
    /// observing a `Some` value here.  The step count lives in DMP SRAM, not
    /// in the FIFO, and cannot be embedded in `DmpData` without a second I2C
    /// transaction.
    pub pedometer_timestamp: Option<u32>,
```

---

## 🔴 Issues Requiring Changes

### R1 — `pedometer_timestamp` field doc says "timestamp/count" — "count" is wrong

**Location:** `src/dmp/mod.rs`, `DmpData` ~L505

**Issue:** The field-level doc comment is:
```ICM-20948-rs/src/dmp/mod.rs#L505-506
    /// Pedometer step detector timestamp/count
    pub pedometer_timestamp: Option<u32>,
```

The word "count" is incorrect. This field holds a DMP cycle counter — a 32-bit
value that increments with every DMP clock tick and is frozen at the moment a
step is detected. It is *not* the cumulative step total. The step total must be
obtained by calling `dmp_read_step_count()` separately.

**Why it matters:** A junior developer reading only the doc comment will
interpret this field as the step count. They will write code like:

```ICM-20948-rs/docs/pedometer-review.md#L1-1
// WRONG: this is the DMP cycle counter, not the step count
let total_steps = data.pedometer_timestamp.unwrap_or(0);
```

That code compiles silently, produces wildly incorrect step totals, and is very
hard to debug because the values look plausible at low step counts.

**Suggested fix:** The field comment must be corrected and then the suggestion in
S10 can replace it entirely. At a minimum:

```ICM-20948-rs/src/dmp/mod.rs#L505-506
    /// DMP internal cycle-counter value captured at the moment this step
    /// was detected.  This is NOT the cumulative step count; call
    /// `dmp_read_step_count()` to get the total.
    pub pedometer_timestamp: Option<u32>,
```

---

### R2 — `dmp_read_step_count()` does not document that the counter is monotonically increasing and never auto-resets

**Location:** `src/device.rs`, `dmp_read_step_count()` doc ~L1519

**Issue:** The documentation says the function returns "the total number of
steps counted since the last DMP reset", but:
- There is no `dmp_reset_step_count()` function.
- There is no way to reset the counter without calling `dmp_init()` and
  reloading the entire firmware.
- The counter persists across calls to `dmp_enable(false)` / `dmp_enable(true)`.

A developer building a run-tracking app that wants to start the counter at zero
for each workout session will search for a reset function, not find one, and then
try to simulate it by storing an offset and subtracting — which works, but only
if they know that is the intended approach.

**Why it matters:** The current documentation implies the counter can be reset
"at the last DMP reset", without explaining what a DMP reset entails (full
firmware reload + reconfiguration, O(50ms), multiple I2C transactions). Calling
`dmp_init()` to reset a step counter mid-run is not a trivial operation.

**Suggested fix:** Add to the `dmp_read_step_count()` doc:

```ICM-20948-rs/src/device.rs#L1524-1535
    /// # Counter lifecycle
    ///
    /// The counter starts at zero after firmware load (`dmp_init()`) and
    /// increments monotonically.  There is **no** `dmp_reset_step_count()`
    /// function; the only way to reset the counter is to reinitialise the DMP
    /// entirely.
    ///
    /// **Implication:** If your application needs per-session step counts,
    /// capture the counter value at the start of the session and subtract:
    ///
    /// ```ignore
    /// let baseline = driver.dmp_read_step_count()?;
    /// // ... later ...
    /// let session_steps = driver.dmp_read_step_count()? - baseline;
    /// ```
    ///
    /// Toggling `dmp_enable(false)` / `dmp_enable(true)` does **not** reset
    /// the counter.
```

---

### R3 — Torn read in `dmp_read_step_count()` is undocumented

**Location:** `src/device.rs`, `read_dmp_memory()` and `dmp_read_step_count()`,
~L1479

**Issue:** Reading four bytes from `PEDSTD_STEPCTR` requires three separate I2C
transactions:

1. Write the DMP memory bank to `MEM_BANK_SEL`
2. Write the byte offset to `MEM_START_ADDR`
3. Burst-read 4 bytes from `MEM_R_W` (register 0x7D)

Between transactions 2 and 3, the DMP continues running its firmware. If a
step occurs in that window, the DMP will update the 4-byte counter inside that
window. Because the ICM-20948's DMP SRAM is shared between the DMP processor
and the host I2C bus with no hardware arbitration, the host could read bytes
0–1 from the *old* value and bytes 2–3 from the *new* value (or vice versa),
producing a corrupted 32-bit result.

**Why it matters:** At 400 kHz I2C the bank-select and address-write
transactions take approximately 50 µs each, and the data read takes another
80 µs. The total window is ~180 µs. At 2 steps/second the probability of a tear
is ~0.036 % per call. In a 30-minute walk (~3 600 step events) that is roughly
1.3 potential tears. For a fitness counter where an off-by-one is inconsequential
this might be acceptable, but for safety-related counting (e.g., rehabilitation
protocols) it is not. More importantly, junior developers who copy this pattern
to other DMP SRAM reads (e.g., reading calibration data) may encounter the issue
in contexts where the probability is far higher.

**Suggested fix:** Add a `# Safety` / `# Caveats` note. No code change is
needed unless a mitigation is desired (double-read with comparison is the
standard embedded pattern when hardware arbitration is absent).

```ICM-20948-rs/src/device.rs#L1518-1525
    /// # Caveats — shared SRAM, non-atomic read
    ///
    /// The DMP firmware and the host share the same SRAM bus with no hardware
    /// arbitration.  Reading the 4-byte counter requires three I2C transactions
    /// (bank select, address write, data burst).  In the ~180 µs window between
    /// the address write and the data read, the DMP can increment the counter,
    /// producing a torn value.  At normal walking cadence (~2 steps/sec) the
    /// probability per call is ~0.04 %.  Applications that require exact counts
    /// can mitigate by reading the counter twice and accepting the result only
    /// when both reads agree.
```

---

## 📚 Educational Notes for the Developer

### DMP SRAM access is not atomic

The ICM-20948 DMP is a small embedded processor that shares its internal RAM
with the host via the I2C/SPI slave interface. There is no mutex, no
copy-on-read buffer, and no "freeze counter while host reads" mechanism.

When you write to `MEM_BANK_SEL` and `MEM_START_ADDR` and then read from
`MEM_R_W`, you are directly peeking into the DMP's running memory. The DMP
firmware continues to execute during those transactions. For values that change
slowly (calibration matrices, sensor scale factors) this is harmless. For values
that change at the rate of sensor output (step counter, calibrated sensor data)
there is a real but small chance of reading a value that was partially updated
between transactions.

The standard mitigation in embedded systems is to read the value twice and
accept only when both reads agree:

```ICM-20948-rs/docs/pedometer-review.md#L1-1
loop {
    let a = driver.dmp_read_step_count()?;
    let b = driver.dmp_read_step_count()?;
    if a == b { break a; }
}
```

For a step counter this level of rigour is probably not warranted, but knowing
the pattern exists is important when you encounter registers that change faster
(e.g., gyroscope calibration bias updated at 1 kHz).

---

### Interrupt latency in an async executor is fine here, but only because of latching

`imu_int.wait_for_high().await` is an *edge-triggered* wait in Embassy's GPIO
driver. If the INT1 pin goes high and then comes back low before the executor
polls the future, a pure edge-detect implementation would miss the interrupt.

The reason this code is safe is that `latch_enabled: true` holds INT1 high
until a register read occurs. The sequence becomes:

1. DMP fires interrupt → INT1 goes high and **stays high**.
2. Executor eventually runs → `wait_for_high()` sees the already-high level → future resolves.
3. `read_interrupt_status()` is called → INT1 goes low.
4. DMP fires again → INT1 goes high again → captured on next iteration.

Remove `latch_enabled: true` and you have a race: if the executor is busy for
longer than one sample period (17 ms at 56 Hz), the pin could go high, be
cleared by the next I2C read, go high again — and the first interrupt's data
is still in the FIFO but the edge was missed. Always use latching for
interrupt-driven DMP reads with an async executor.

---

### `clear_on_any_read` has a wider blast radius than it looks

The `INT_ANYRD_2CLEAR` bit in `INT_PIN_CFG` is what `clear_on_any_read: true`
controls. When this bit is set, *any* read from *any* register over I2C clears
the interrupt status register and releases the latched INT1 pin.

This means every call inside your interrupt handler — `dmp_read_fifo()`,
`dmp_read_step_count()`, even a bank-select write followed by a register read
— will clear the interrupt. In this example that is fine because
`read_interrupt_status()` clears it deliberately before the other calls. But if
you add a register read *before* `read_interrupt_status()` (e.g., checking
`who_am_i()` for debug), you will clear the interrupt early without saving its
status bits.

For stricter code, prefer `clear_on_any_read: false` and clear the interrupt
only by explicitly reading `INT_STATUS`. This makes the clearing point visible
in the control flow rather than implicit in every I2C read.

---

### The DMP cycle counter is not wall-clock time — but it is proportional

`pedometer_timestamp` holds a 32-bit value that increments at the DMP's internal
clock rate (nominally the gyroscope output rate, 1100 Hz / (GYRO_SR_DIV + 1)).
At 56 Hz output, `GYRO_SR_DIV = 19`, so the DMP clock runs at
`1100 / (19 + 1) = 55 Hz`. This means consecutive step timestamps differ by
approximately `55 * step_period_seconds` ticks.

To convert a difference of `Δts` ticks to milliseconds:
`Δt_ms ≈ Δts * 1000 / 55`

This conversion is only an approximation because the crystal oscillator has a
±2 % tolerance. For cadence calculation (steps per minute) use the ratio of two
consecutive timestamps; the absolute value is meaningless.

Also note: the 32-bit counter will wrap around after `2^32 / 55 ≈ 78 million
seconds ≈ 903 days` of continuous DMP operation. In practice this never happens
(power cycles reset the DMP), but code that computes `ts_current - ts_previous`
using unsigned arithmetic will handle the wrap correctly automatically.

---

### Why `pedometer_six_axis()` alone might produce no FIFO output

The DMP pedestrian algorithm generates PQuat6 packets only while it detects
"pedestrian motion" — it is intentionally cadence-locked and will suppress output
during quiet periods. When `pedometer_six_axis()` is used *without*
`with_step_detector()`, the `PEDOMETER_INTERRUPT` bit in `MOTION_EVENT_CTL` is
not set. Based on the InvenSense register descriptions, this bit controls whether
a step event generates a host interrupt, *not* whether the pedestrian algorithm
runs. So PQuat6 output should still be generated.

However, this has not been verified against the DMP firmware binary. The
SparkFun reference and the InvenSense eMD SDK always enable pedometer output
alongside the step interrupt. If you find that `pedometer_six_axis()` alone
produces no FIFO packets, add `with_step_detector()` — this enables
`PEDOMETER_INTERRUPT` and is the combination tested in hardware.

The documentation on `DmpFusionMode::PedometerSixAxis` uses the phrase "use
this mode together with `with_step_detector()`". This should be made more
explicit: state whether `with_step_detector()` is *required* for PQuat6 output
or merely *recommended* for step timestamp collection.

---

### Why does QUAT6 always get HEADER2 but PQUAT6 does not?

This is intentional and correct, but the code contains no comment explaining it.

HEADER2 is the 2-byte secondary header word that tells the parser which accuracy
fields are present later in the packet. The standard fusion outputs (QUAT6,
QUAT9, GEOMAG) are almost always paired with accuracy reporting
(`with_host_calibrated_accel()` → `ACCEL_ACCURACY`, etc.), and the SparkFun
reference firmware always emits HEADER2 for those modes.

PQuat6 is a lightweight output (6 bytes vs 12 for QUAT6) designed for low-power
pedestrian use. It has no associated accuracy field in `DATA_OUT_CTL2`. Adding a
HEADER2 word when no accuracy bits can possibly be set would waste 2 bytes per
packet. The code correctly omits it, and the parser correctly handles PQuat6
packets without HEADER2.

The asymmetry is real: `DmpConfig::six_axis()` alone produces an 18-byte packet
(with a 0x0000 HEADER2 word), while `DmpConfig::pedometer_six_axis()` alone
produces a 10-byte packet (no HEADER2 word). Both are correct. A comment in the
`as_control1()` function would make this immediately clear to any future
maintainer (see S2).
