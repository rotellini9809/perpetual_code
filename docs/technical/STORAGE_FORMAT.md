# Storage map and binary formats

This is the canonical on-flash specification. Runtime ownership/recovery belongs
in [Storage](STORAGE.md); commands belong in [Recording](../testing/RECORDING.md).
The fixed product is W25Q128JV, 16 MiB, 4096-byte sectors and 256-byte pages.

All integers are explicit little-endian; floats are IEEE-754 binary32. CRC is CRC-32/ISO-HDLC
(reflected EDB88320, initial/final XOR FFFFFFFF); incremental CRC retains the unfinalized state
until the final XOR. Unspecified bytes are FF. Native pointers, size_t and raw C++ struct layout
are never serialized. Shared `BinaryEncoding.h` implements these primitives. Records use
separate body/commit programming units, each programmed once between erases. Per-record rules
below define the covered bytes; CRC alone does not establish extent ownership.

## Memory map (layout v2)

Goals: every small record is written beside the previous one and the old copy is erased only
afterwards (no power cut can lose a setting); every large region is 64 KiB aligned (every erase is
one command); flight space grows while flying and the oldest flights are overwritten.

| 64 KiB block | Address | Size | Contents | Erase unit |
|---|---|---|---|---|
| 0 | 0x000000 | 32 KiB | Device journal, bank A | 32 KiB block (0x52) |
| 0 | 0x008000 | 32 KiB | Device journal, bank B | 32 KiB block (0x52) |
| 1 | 0x010000 | 64 KiB | Flight directory, bank A | 64 KiB block (0xD8) |
| 2 | 0x020000 | 64 KiB | Flight directory, bank B | 64 KiB block (0xD8) |
| 3..255 | 0x030000 | 253 x 64 KiB | Flight ring (15.81 MiB) | 64 KiB block (0xD8) |

Nothing else exists: no separate calibration, identity or settings sectors and no reserved gaps.
The map is fixed in `include/storage/FlashLayout.h` (compile-time checks for alignment and
disjoint regions); constructors accept only FlashDevice and alternate layouts are not supported.
`flights format` erases only blocks 1..255 (directory and ring); block 0 is erased only by its own
journal compaction or by the diagnostic ERASE ALL. A device with the previous layout (directory at
0x005000) mounts as `Directory`/`Corrupt` and needs `flights format`; its old calibration, rules
and settings are not read and must be saved again.

## Device journal v1 (block 0)

One typed, append-only journal for everything that describes the device
(`DeviceJournal`). For each record type the valid record with the highest sequence number wins.
Every operation rescans the bank, so the configuration and calibration stores never disagree.

Each 32 KiB bank: page 0 is the bank header, page 1 its commit page, pages 2..127 hold records.

| Bank header offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x4A564450 ("PDVJ") |
| 4 | 2 | Journal version 1 |
| 8 | 8 | Bank generation, nonzero; the valid header with the highest generation is active |
| 16 | 8 | Next record sequence after the records copied into this bank |
| 252 | 4 | CRC-32 of bytes 0..251 |

Two valid headers with the same generation are `Corrupt`. A record is a header page,
`ceil(length / 256)` payload pages and a commit page, written in that order, each programmed
once and read back:

| Record header offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x52564450 ("PDVR") |
| 4 | 1 | Record type (below) |
| 5 | 1 | Type version 1 |
| 6 | 1 | Flags, 0 for new writes: bit 0 damaged after, bit 1 corrupt after, bit 2 placeholder (compaction only) |
| 8 | 8 | Sequence number, strictly increasing across the journal |
| 16 | 4 | Payload length in bytes, 0..4096 |
| 20 | 4 | CRC-32 of the payload |
| 252 | 4 | CRC-32 of bytes 0..251 |

The commit page holds magic 0x43564450 ("PDVC"), the header CRC and its complement at offsets
0, 4, 8; the rest is FF. Bank commit pages use the same layout with the bank header CRC.

| Type | Record | Payload |
|---|---|---|
| 1 | IMU calibration | 42 bytes: approval (u8: 0 required, 1 verified, 2 in use, 3 invalid), identity (u8: 0x49 factory identity coefficients, FF custom), revision (u32), accel bias XYZ (3 x float32, m/s^2), accel scale XYZ (3 x float32), last verification residuals: corrected \|a\| minus g on +X, +Y, +Z (3 x float32 m/s^2, NaN for a face not measured; all NaN for a new calibration). A verification, passed or failed, writes its residuals with the approval it leads to. |
| 2 | Orientation | 1 byte: up axis +X, -X, +Y, -Y, +Z, -Z = 0..5 |
| 3 | Rules | configuration codec version (u16, 3), then the [codec v3 package](#configuration-codec-v3) (at most 4094 bytes) |
| 4 | IMU identity revoked | empty; its presence revokes the factory identity |
| 5 | State machine | 5 bytes: bistage (u8 0/1), minimum COAST 1 velocity (float32 m/s) |
| 6 | Airbrakes | 1 byte: enabled 0/1 |

Each setting is its own type, so damage to one never resets another. A missing type means "not
set": no rules (`Missing`), orientation unconfigured (physical arm blocked with
ORIENTATION_NOT_READY), normal state machine, airbrakes disabled, no calibration.

**Reading.** Pick the active bank, then walk pages from 2. A header with valid magic and CRC owns
its declared extent: with a matching payload CRC, type version and commit it is a record; any
other page that is not erased is skipped one page at a time and marks every type as damaged. The
append position follows the last non-erased page.

**Torn versus damaged.** A record without its commit never completed (a power cut while
writing): it is ignored and the previous record of that type stays in force, but the type is
reported as *damaged after*. A record whose commit is present but whose payload no longer
matches was complete and was damaged later: the type is *corrupt after*. Settings and rules with
a corrupt newer record report `Corrupt` (run readiness blocked until saved again) rather than
silently falling back to an older value. For the IMU calibration, any damage after the newest
record turns the approval into Invalid (recalibration required) without revoking the identity:
a torn write is not a failed verification.

**Writing.** The first write on a device whose block 0 has no valid bank erases bank A and
commits generation 1. A new record is appended at the append position, then read back through the
normal scan: it must be the newest valid record of its type. If it does not fit, the journal
compacts first. IMU approval changes (verified, in use at arming, verified again after disarm)
are new type-1 records with the same coefficients.

**Compaction.** Erase the inactive bank (32 KiB), copy the newest valid record of every kept type
into it with its original sequence and its damage flags (byte 6), then write the bank header with
generation + 1 and its commit. A type that has damage but no valid copy is kept as an empty
placeholder record (flag bit 2) carrying that damage. Until the commit the old bank stays active;
after it, the new one is. A bank holds 126 record pages: a rules save uses at most 18, other saves
3, so compaction happens every few tens of saves, only while disarmed and idle.

**Factory reset** (`config reset`) is a compaction that keeps only types 1 and 4: rules, orientation
and settings are gone; calibration and the identity revocation survive. The revocation is erased
only by ERASE ALL.

## Configuration codec v3

The rules package stored in journal type 3 and frozen in every flight header. All metadata is
checked before bounded decoding. Payload field order:

1. Rule count (u8).
2. Four pyro pulse durations (u32 milliseconds each).
3. Four servo records, five bytes each: calibrated (u8 strictly 0/1), initial pulse (u16),
   final pulse (u16). The common prefix is 37 bytes; rules begin immediately afterward.
4. Each active rule: ID (u32), name (48 bytes, NUL-terminated), rule delay
   (u32 milliseconds), condition count (u8), assignment count (u8). The exporter uses
   `delay_ms` for the rule delay.
5. That rule's conditions, each 19 bytes: kind (u8), comparison (u8), phase mask
   (u8), value (f32), upper bound (f32), elapsed milliseconds (u32), rule reference
   (u32).
6. That rule's assignments, each 6 bytes: output kind (u8), one-based channel (u8),
   additional channel delay (u32 milliseconds). Activation waits for rule delay plus channel
   delay. This reuses the existing word without changing the format/version; the previous
   firmware wrote zero here. The offline downloader exports this field as `delay_ms` from
   each frozen recording snapshot.

7. Continuous airbrake rule: servo channel mask (u8, bits 0..3), condition count (u8),
   followed by that many conditions with the same 19-byte encoding. Mask zero disables
   control; enabled rules require 1..8 AND conditions. See [Airbrakes](AIRBRAKES.md).

Kind/comparison/phase encodings follow the explicit current declarations in
`RecoveryConfiguration.h`; changing their numeric meanings requires a format version change.
Tagged irrelevant fields are retained, but never evaluated by the configuration store. Extra
payload bytes and excessive counts are rejected.

Current maximum structural payload is 2,255 bytes (all arrays at capacity, before global
channel-ownership validation). The largest semantically possible plan is smaller because there
are only eight uniquely assignable physical channels. A compile-time check keeps it within one
journal record (4094 bytes after the version prefix). The flight header embeds this same
encoded plan.

Condition-kind codes are Phase=0, ApogeeDetected=1, AltitudeComparison=2, AltitudeRange=3,
AbsoluteVelocityComparison=4, AbsoluteVelocityRange=5, TimeSinceLaunch=6, TimeSinceApogee=7,
TimeSinceRule=8. Comparison codes are Less=0, Greater=1, LessEqual=2, GreaterEqual=3. Output
kind is Pyro=0, Servo=1.

## Flight directory v2 (blocks 1..2)

Each 64 KiB bank has 128 slots of 512 bytes: a 256-byte body and a separate 256-byte commit page.
The body is written and read back first, the commit page last. The body CRC includes reserved FF
bytes; the commit binds that CRC and its complement.

| Body offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x52494450 |
| 4 | 2 | Version 2 |
| 6 | 1 | Kind: 0 bank header, 1 flight update |
| 8 | 8 | Bank generation, starting at 1 |
| 16 | 4 | Slot index |
| 24 | 8 | Flight sequence (header: sequence high-water mark) |
| 32 | 2 | First ring block 0..252 (header: baseline entry count, u32) |
| 34 | 2 | Owned ring blocks, 1..253 |
| 36 | 4 | Committed logical bytes from the flight start |
| 40 | 1 | Flight status |
| 252 | 4 | CRC-32/ISO-HDLC of body bytes 0..251 |

Header bodies bind the layout with five u32 values at offsets 40, 44, 48, 52, 56: bank A start,
bank B start, bank bytes, ring start (0x030000) and ring block count (253); a mismatch is
`Corrupt` and is never autoformatted. Commit pages store magic 0x54494D43 at offset 0, body CRC
at 4 and its complement at 8; the rest is FF.

Status values: 1 Recording, 2 Completed, 3 Interrupted, 4 StoppedFull, 5 Invalidated, 6 Corrupt.
Slot 0 is the bank header. Its baseline count is the number of copied retained entries; its saved
high-water mark prevents sequence reuse after deletion. Generation and recording sequence are
nonzero u64 values; exhaustion fails without wrapping. The RAM ordering of entries is not their
stable identity. See [transactions](STORAGE.md#directory-transactions).

At most 32 retained flights (MaxEntries); placement invalidates the oldest beyond that. One
flight writes at most 65 updates: start, growths, close, and one invalidation for each older
flight it overwrites. Arming compacts the directory first (erase the other 64 KiB bank, copy at
most 32 entries) when fewer than 65 slots are free; a compacted bank leaves at least 95, so
compaction never happens during a flight. Overlap checks are circular over the ring; an extent
longer than the ring is rejected at mount without writes.

## Flight ring (blocks 3..255)

A flight owns `n` consecutive ring blocks starting at block `b`, wrapping: blocks
`b, b+1, ..., b+n-1` modulo 253. Logical byte `o` of the flight lives at
`0x030000 + ((b * 65536 + o) mod (253 * 65536))`. The flight header (recording header, starts
table, summary; at most 8 KiB) occupies the start of block `b`; v4 sample blocks follow at
logical offset `headerBytes`. 2 KiB sample blocks never straddle a 64 KiB block, and each stores
its physical address.

- **Placement.** A new flight starts at the block after the newest retained flight (block 0 on
  an empty directory).
- **Arming.** Own and erase a base of 8 blocks (512 KiB, about 30 s at 476 Hz) before the flight
  starts. Measured on the board: 8 erases in about 1.2 s; preparation about 1.5 s.
- **Growth.** When less than 128 KiB (about 7.5 s at 476 Hz) of erased space remains ahead, own
  8 more blocks: write the directory update first, then erase them one 64 KiB block at a time
  (asynchronous erase command and status polling on core 1). While an erase is busy nothing else
  touches the flash: sealed 2 KiB blocks wait in a RAM ring of 16 (28 KiB, about 2 s at 476 Hz,
  the W25Q128JV worst-case 64 KiB erase time) and are written once the erase completes.
- **Overwrite.** Blocks needed by growth or placement that belong to older flights are freed by
  invalidating those flights, oldest first, whatever their status (including Corrupt), before
  any erase. A flight is invalidated whole even if only part of it is needed.
- **Full.** The current flight stops with StoppedFull only when its next block would be its own
  first block, after the whole ring is used (about 16 minutes at 476 Hz, see [recording time](#size-and-recording-time)).

### Logging rate by phase

PAD keeps the [pre-launch history](#pre-launch-history) (last 0.5 s at full rate, 10 Hz for the
10 s before). BOOST, COAST and DESCENT are logged at the full IMU rate. LANDED is logged at 10 Hz
(one sample per 100 ms with the newest barometer reading attached, decimation GAPs flagged
deliberate) for the 15 s landing window, after which the run closes. State changes between kept
samples appear on the next kept sample; output start times stay exact in the header starts table.

## Recording header v8

`plan(config, limits, layout)` validates the complete plan and computes layout without flash
access. It reserves exactly one slot per assigned output, ordered by rule order and assignment
order, up to all eight physical channels. There are no slots for unassigned channels. Duplicate
ownership is rejected by configuration validation. `create()` accepts only the start of a ring
block, checks the complete header reservation is erased, and never overwrites existing bytes. The caller must already own the reserved extent; erased space
is not proof that it is free. Allocation and explicit reuse belong to FlightRecorder.

The header reservation is sector-aligned; `FlightSampleStore` starts at logical offset
`layout.headerBytes` of the same flight extent. The table is sized
before any samples are written and is never extended later. The small three-output fixture needs
4 KiB; an eight-output header needs 8 KiB. The exact size also depends on the encoded
configuration.

Header regions are:

| Relative address | Contents |
|---|---|
| 0 | 256-byte descriptor |
| 256 | Separate descriptor/configuration commit page |
| 512 | Configuration codec-v3 payload, padded to a 256-byte boundary with FF |
| `tableOffset` | One 512-byte output slot for each assigned channel |
| after table | Always-reserved 512-byte final summary |
| after reserved slots | Erased padding through the next 4 KiB boundary |
| `headerBytes` | Beginning of separately stored sample blocks |

Descriptor fields:

| Offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x52444850 |
| 4 | 2 | Header version 8 |
| 6 | 2 | Descriptor bytes 256 |
| 8 | 4 | Total header reservation bytes |
| 12 | 4 | Configuration offset 512 |
| 16 | 4 | Actual configuration payload bytes |
| 20 | 2 | Configuration codec version 3 |
| 22 | 1 | Sensor mode: 0 hardware, 1 simulation |
| 23 | 1 | Output mode: 0 physical, 1 virtual |
| 24 | 1 | Frame: 0 sensor axes, 1 logged calibrated body |
| 25 | 1 | Sample block format version 4 |
| 26 | 1 | Assigned-output slot count |
| 28 | 4 | Output-table offset |
| 32 | 2 | Slot bytes 512 |
| 36 | 4 | Physical header address, binding relocated metadata |
| 40 | 4 | Sample-data offset, equal to header reservation bytes |
| 44 | 4 | Summary offset, exactly after output slots |
| 64 | count x 8 | Output mappings: uint32 rule, uint8 kind, uint8 channel, FF x 2 |
| 160 | 64 | Per-flight calibration and state-machine snapshot; [fields below](#per-flight-calibration-snapshot-header-v8) |
| 252 | 4 | Immutable CRC |

Unspecified bytes are FF. Output kind is 0 pyro or 1 servo; channels are one-based. The
immutable CRC is CRC-32/ISO-HDLC over descriptor bytes 0..251 followed by the padded
configuration payload. Mutable table/summary bytes are excluded. The commit page contains magic
0x43444850, immutable CRC and its uint32 complement at offsets 0,4,8; remaining bytes are FF.
Payload pages are programmed/readback-verified first, then the descriptor, then its commit. Each
page is programmed once between erases. Creation is acknowledged only after decoding through the
normal read path.

The header omits a flight ID, firmware ID, launch pressure, ranges and nominal rate. The
sequence remains directory/block bookkeeping. The airbrake law version is frozen separately. Current header v8 always reserves the final
summary; v1..v7/unknown headers are rejected without migration. Layout equations are:

```text
tableOffset   = 512 + align256(configBytes)
summaryOffset = tableOffset + slotCount * 512
headerBytes   = align4096(summaryOffset + 512)
sampleStart   = ringAddress(flightStart, headerBytes)
```

## Output-start slot v1

| Slot-body offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x54534850 |
| 4 | 2 | Version 1 |
| 6 | 2 | Slot index |
| 8 | 4 | Physical header address |
| 12 | 4 | Immutable header CRC |
| 16 | 4 | Rule identifier |
| 20 | 1 | Output kind |
| 21 | 1 | Channel |
| 24 | 4 | Actual start timestamp in milliseconds |
| 252 | 4 | CRC-32/ISO-HDLC over body bytes 0..251 |

All unspecified bytes are FF. Slot commit magic is 0x43534850, followed by its body CRC and
complement; remaining bytes are FF. Binding to the header CRC, address, slot and immutable
rule/channel mapping detects stale/relocated data. The coordinator must still invalidate
retained ownership before erasing or reusing a flight; CRCs do not replace that ownership
protocol.

Slot state is Empty=0, Pending=1 (RAM only), Recorded=2, Invalid=3. Timestamp zero and
UINT32_MAX are valid. Empty/Invalid never prove the actuator did not operate. An individual
damaged slot does not invalidate other slots or immutable metadata.

## Samples and blocks v4

Replaces v3 together with recording header v8 (descriptor byte 25 = 4). There is no reader for
v3 or header v7: old recordings report `UnsupportedVersion`; `flights format` clears them.
`FlightLogCodec` encodes on the storage worker (core 1); `FlightLogDecoder.h` is the C++
reference decoder used by host tests and `tools/replay/flight_log.py` the PC decoder.

### Goals

One record per IMU sample at the full FIFO rate (480 Hz nominal, 476.3 Hz on the current board)
with raw sensor values, full-rate estimates, barometer only when fresh, and state only when it
changes. The board stores and transfers raw blocks; only the PC decodes and calibrates.

### Block

A block is 2048 bytes = 8 pages: pages 0..6 are the 1792-byte body, page 7 the commit page. Every
page is programmed once between erases, body before commit. Blocks are contiguous after the
header reservation at logical offsets of the flight extent (wrapping with the ring); 32 blocks
fill one 64 KiB block and a block never straddles two.

| Body offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x4B425350 |
| 4 | 2 | Block version 4 |
| 6 | 2 | Body bytes 1792 |
| 8 | 8 | Nonzero recording sequence, bound to directory ownership |
| 16 | 4 | Zero-based block index |
| 20 | 4 | Physical block address |
| 24 | 8 | Time of the first FRAME, us from the run origin |
| 32 | 4 | IMU sequence number of the first FRAME |
| 36 | 2 | FRAME count, 1..114 |
| 38 | 2 | Payload bytes used, 1..1724 |
| 40 | 1 | Phase in effect before the first record (same mapping as v3) |
| 41 | 1 | Abort reason in effect before the first record |
| 42 | 2 | Battery mV in effect before the first record; FFFF unavailable |
| 44 | 2 | Airbrake flags in effect before the first record |
| 46 | 1 | Airbrake failed-condition bitmask |
| 47 | 1 | Airbrake channel mask (bits 0..3) |
| 48 | 8 | Airbrake openings ch 1..4, u16 0..10000; FFFF unassigned |
| 56 | 8 | Reserved FF |
| 64 | 1724 | Records, then FF to the end of the payload |
| 1788 | 4 | CRC-32/ISO-HDLC over body bytes 0..1787 |

Commit page: magic 0x4D435350 at offset 0, body CRC at 4, its uint32 complement at 8; remaining
bytes FF (unchanged from v3). A block is committed once those 12 bytes follow the body. The
writer programs the 7 body pages with `programErased()` (each 64 KiB block is erased just
before use, at arming or by growth; the driver verifies each page), then the commit page, then reads the 2 KiB back
once and compares it end to end. A write failure is never retried. Bytes 40..55 make every block decodable on its own: a decoder
starts each block from this state snapshot, so a damaged earlier block never corrupts a later one.
Recovery and `flights info` count samples from offset 36 without decoding records; recovery
checks that each block header starts after the previous block's samples. The saved sample count
(summary) is the number of FRAMEs. "Full" means a sample was refused, not that the last block
of the extent is in use.

### Records

Each record starts with a tag byte: bits 7..5 type, bits 4..0 flags. Undefined flag bits must be
zero. Tag 0xFF ends the payload (erased flash). Records appear in time order. One IMU sample
produces one group: optional STATUS, BATTERY, AIRBRAKE, GAP, TIME and BARO records, then its
FRAME. Auxiliary records belong to the FRAME that follows them. A group (at most 63 bytes) is
never split across blocks.

| Type | Record | Bytes | Fields after the tag |
|---|---|---|---|
| 1 | FRAME | 15 or 28 | u16 dt; i16 ax, ay, az, gx, gy, gz; then, with flag bit 0, the estimates below |
| 2 | BARO | 8 | u16 age us (FRAME time minus barometer sample time); u24 pressure in 1/64 Pa; i16 temperature in 0.01 C |
| 3 | STATUS | 3 | u8 phase; u8 abort reason |
| 4 | BATTERY | 3 | u16 mV, FFFF unavailable |
| 5 | AIRBRAKE | 5 + 2n | u16 flags; u8 failed conditions; u8 channel mask; u16 opening for each set mask bit, ascending |
| 6 | GAP | 3 | u16 count of IMU samples not stored before the next FRAME (>= 1) |
| 7 | TIME | 5 | u32 dt for the next FRAME, used when dt > 65534 us; tag must be exactly 0xE0 |

FRAME flags: bit 0 estimates present; bit 1 pre-launch history (written from the pad buffer);
bit 2 IMU invalid (the six values are then 0; -32768 remains a valid saturated code).
GAP flags: bit 0 deliberate decimation (pad history, LANDED at 10 Hz), clear for real loss (queue full, FIFO
overrun, invalid acquisition).

FRAME `dt` is the time since the previous FRAME of the same block, 0 for the first one (whose
time is body offset 24). `dt = FFFF` means "take dt from the preceding TIME record". The IMU
sequence number of each FRAME is the previous one + 1 + the counts of GAP records in between.
The first FRAME of a block takes its time and sequence from the header; a GAP before it is
informational (the decoder compares with the previous block). A gap beyond 65535 samples or a
time step beyond 32 bits starts a new block instead.

FRAME estimates, 13 bytes (each field's minimum value means invalid):

| Field | Encoding | Range |
|---|---|---|
| Altitude | i24, cm | +/- 83.8 km |
| Vertical velocity | i16, 0.02 m/s | +/- 655 m/s |
| Vertical acceleration | i16, 0.01 m/s^2 | +/- 327 m/s^2 (33 g) |
| Roll, pitch, yaw | i16 each, 0.01 degree | +/- 327 degrees |

### IMU values

The six i16 values are, in order, accelerometer X/Y/Z then gyroscope X/Y/Z. Their meaning
depends on the header frame convention (descriptor byte 24):

- **0, sensor axes (hardware):** the unmodified LSM6DSV32X output codes from the FIFO, in sensor
  axes. Saturation is the code itself (-32768 or 32767). Firmware conversion, which decoders
  must reproduce: accel m/s^2 = code x 0.976 x 9.80665 / 1000; gyro rad/s = code x 70 /
  1000 x pi / 180.
- **1, calibrated body (simulation/HIL):** body-axis values already calibrated by the source,
  quantized with the same two LSBs, rounded to nearest and clamped to the i16 range.

To rebuild calibrated BODY values from sensor-axis codes, the decoder applies the per-flight
snapshot (descriptor bytes 160..223) exactly as the firmware pipeline does:
`accel = (a - accelBias) x accelScale`, `gyro = g - gyroBias` per sensor axis, then
`body[i] = sign x sensor[|m| - 1]` with `m = MountingAxes[upAxis][i]` and `MountingAxes =
{1,2,3}, {-1,2,-3}, {2,-1,3}, {-2,1,3}, {3,2,-1}, {-3,2,1}` for upAxis 0..5.

### Pre-launch history

While the phase is PAD nothing reaches flash. Core 1 keeps the last 0.5 s of groups at full rate
and, before that, one group every 100 ms for 10 s (the latest barometer and state travel with
each kept group). On the first non-PAD frame it encodes the 10 Hz history, then the full-rate
tail, both with FRAME flag bit 1 and decimation GAPs, then continues live. A run that ends in
PAD is discarded as today for hardware; simulation/HIL write their buffer at close. Writing the
buffer (about 11 KB) takes about 40 ms of core 1 while live frames wait in the 128-entry queue
(about 270 ms at 476 Hz). On the board, the Hoppino replay (200 Hz, 8.2 s of PAD) keeps 175
history samples and, with LANDED at 10 Hz, 4603 of 6600 overall (82 blocks); the boost-time
write is the longest worker job (about 75 ms, queue peak 17 of 128).

### Size and recording time

At 476 Hz with valid estimates: FRAME 28 B x 476 + BARO 8 B x 100 = 14.1 KB/s of records; with
84 % payload efficiency (1724 of 2048 bytes) about 16.8 KB/s of flash. Measured on the board:
about 55 frames per block and 7.8 ms of worker time per block (write + read-back), i.e. about 7 %
of core 1 at 476 Hz (v3: about 33 %; v3 held about 19 s per MiB).

**Recording time per flight: about 16 minutes at the full IMU rate**, counted from launch. This
was measured on the board on 20 November 2026: flight 40 filled the whole ring with 464015
full-rate samples (974 s at 476.2 Hz), 31 growths and 16.58 MB
([Validation](../testing/VALIDATION.md#physical-arming-on-the-bench---19-november-2026)).

| Space | Size | Full-rate time at 476 Hz |
|---|---|---|
| Base reserved at arming (8 blocks) | 512 KiB | about 30 s |
| Each growth (8 blocks) | 512 KiB | about 30 s more |
| Growth threshold (erased space left ahead) | 128 KiB | about 7.5 s |
| RAM ring while an erase runs (16 x 2 KiB) | 28 KiB of payload | about 2 s |
| Whole flight ring (253 blocks) | 15.81 MiB | about 16 minutes |

The time does not depend on how many flights are stored: a new flight overwrites the oldest ones
when it needs their blocks, so every flight can use the whole ring (and a 16-minute flight leaves
no older flight behind). PAD costs almost nothing: only the pre-launch history (about 11 KB) is
written. LANDED at 10 Hz for the 15 s landing window costs about 150 samples.

When the ring is full, later samples are dropped and counted (`StorageFull`). The flight itself
carries on without storage: phases, rules and outputs still run, output starts are still recorded
in their own reserved space, and the flight closes as StoppedFull at the end of the run. Flight 40
went on for another 208 s after the ring filled, and its DESCENT servo start was recorded.

### Transfer

`flights info` reports `sample_transport: "raw-v4"`, `block_body_bytes: 1792` and
`block_chunk_bytes: 448`. `flights block <sequence> <index> <offset>` returns `bytes` of the
validated body starting at `offset` as base64 (`data`), with the block's `frames`; four chunks
make one body. The board validates header, CRC and commit before sending; the PC decoder checks
everything again and rejects any damaged byte. The Hoppino flight (82 blocks) downloads in
about 3 s over USB.

### CSV export (PC decoder)

One row per FRAME, in order:

```text
t_us,imu_seq,pad_history,gap_before,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,pressure,temp,z_est,vz_est,az_est,roll,pitch,yaw,state,abort_reason,battery_v,airbrake_1_pct,airbrake_2_pct,airbrake_3_pct,airbrake_4_pct,airbrake_flags,airbrake_failed_conditions,airbrake_channel_mask
```

`raw_*` are the stored codes (empty for calibrated-body flights); `accel*`/`gyro*` are calibrated
BODY m/s^2 and rad/s. `pressure` (hPa) and `temp` (C) are filled only on rows whose group had a
BARO record. State, battery and airbrake columns carry the last value forward. Invalid or
missing values are empty cells. `gap_before` is the number of samples not stored before the row.

## Final summary v1

The final summary uses its own 256-byte descriptor and separate 256-byte commit page, each
programmed once between erases.

| Descriptor offset | Width | Value |
|---|---|---|
| 0 | 4 | Magic 0x4D535250 |
| 4 | 2 | Summary version 1 |
| 6 | 2 | Slot bytes 512 |
| 8 | 8 | Stable flight sequence |
| 16 | 4 | Physical header address |
| 20 | 4 | Immutable header CRC |
| 24 | 4 | Verified saved sample count |
| 28 | 4 | Dropped sample count |
| 32 | 4 | Sample queue peak |
| 36 | 4 | Sample queue capacity |
| 40 | 1 | End reason: 1 Completed, 2 Stopped, 3 Disconnected, 4 StorageFull |
| 41..251 | 211 | Reserved FF |
| 252 | 4 | CRC-32/ISO-HDLC of bytes 0..251 |

Commit page offsets 0,4,8 contain magic 0x43535250, descriptor CRC and its uint32 complement.
Remaining bytes are FF. Identity/address/header-CRC binding prevents attaching a summary to
another flight or a reused extent. Readback verifies both pages. There is no rewrite/retry of an
ambiguous partially programmed summary.

Reason codes are Completed=1, Stopped=2, Disconnected=3, StorageFull=4. Readers report Missing,
Incomplete, Valid, Corrupt or Unsupported; only Valid exposes counters/reason/version. A valid
summary must match the scanned sample count. There is no Legacy state. A valid summary with an
unclosed directory entry does not turn that entry into Completed after a restart.

## Compatibility

Component versions are independent: device journal v1, flight directory v2, configuration
codec v3, recording header v8, sample blocks v4, start slots and summary v1. Only configuration codec v3 is read and written;
older packages and snapshots are rejected. There is no conversion or legacy decoder.
No automatic erase or migration writes occur at boot; the previous layout is not read. `flights info` reports `config_version:3`
and `flights config` exports that codec.
The frozen whole-image FNV64 regression value is
`0xf00e48afbf5b4dc9` (header v8, sample blocks v4; the fixture uses zero channel delays).

## Per-flight calibration snapshot (header v8)

Descriptor bytes 160..223 store the immutable startup calibration, covered by the same
CRC and commit as the configuration snapshot. Old header versions are unsupported;
use explicit `flights format` to start with the new format. The rule codec is v3.

| Descriptor offset | Width | Meaning |
|---|---|---|
| 160 | 1 | Flags: bit 0 snapshot available, bit 1 sensor-axis corrections applied, bit 2 gyro calibrated at physical arming |
| 161 | 1 | Noseward axis enum 0..5, or FF for already calibrated BODY input |
| 162 | 2 | Reserved FF |
| 164 | 4 | Barometric reference altitude, float32 metres |
| 168 | 12 | Gyroscope biases X/Y/Z in sensor coordinates, float32 rad/s |
| 180 | 12 | Accelerometer biases X/Y/Z in sensor coordinates, float32 m/s^2 |
| 192 | 1 | Bistage selected: 0 normal, 1 bistage |
| 193 | 3 | Reserved FF |
| 196 | 4 | Minimum COAST 1 velocity, float32 m/s |
| 200 | 4 | Compiled airbrake law version, u32 |
| 204 | 1 | Airbrakes enabled at startup, strictly 0/1 |
| 205 | 3 | Reserved FF |
| 208 | 12 | XYZ accelerometer scales, float32; FF for BODY input |
| 220 | 4 | Device accelerometer calibration revision, u32; FF for BODY input |

Unavailable snapshots use flags 0 and FF in all remaining snapshot bytes. When input is
already calibrated BODY data, axis and bias fields remain FF and export as null, rather
than inventing a physical calibration. The finite barometric reference is still saved.
All remaining descriptor padding stays FF. `flights calibration <sequence>` exports this
snapshot separately to keep replies bounded; the downloader stores it under `calibration`
in the flight JSON sidecar. No current device settings are consulted when displaying it.
Offset 200 freezes `AirbrakeLawVersion` and 204 the saved airbrake enable flag; the immutable
configuration snapshot includes the airbrake channel mask and AND conditions. Airbrakes do not
allocate once-only output-start slots; their continuous state belongs to sample records.
