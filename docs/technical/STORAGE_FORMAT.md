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

## Fixed memory map

| Region | Begin | End inclusive | Size |
|---|---|---|---|
| Accelerometer calibration, reserved only | 0x000000 | 0x000FFF | 4 KiB |
| Configuration reservation (rules + mounting) | 0x001000 | 0x004FFF | 16 KiB |
| Directory A | 0x005000 | 0x00CFFF | 32 KiB |
| Directory B | 0x00D000 | 0x014FFF | 32 KiB |
| Flight headers and sample extents | 0x015000 | 0xFFFFFF | 0xFEB000 bytes |

The W25Q128JV capacity, geometry and region map are fixed in `include/storage/FlashLayout.h`.
Constructors accept only FlashDevice; alternate runtime layouts are not supported. Compile-time
assertions check alignment and disjoint regions. Persisted directory headers must match this
same map before mount. Directory operations erase/program only the two directory banks. Every
allocation must be contiguous and end at or before DataEnd. Overlap checks use linear intervals;
older wrapped metadata is rejected without writes.

The calibration reservation is for three sensor-axis accelerometer biases (m/s^2), defaulting
to zero when no calibration is available. No record format or reader/writer is implemented yet;
the future interface will supply calibration. The current gyro bias lives in arming-session RAM; a historical copy is saved in each
flight header and is not reused for another arming. Mounting uses an explicitly saved signed axis selector; see
[navigation](NAVIGATION.md#calibration-and-mounting) for its separate device-setting policy.

## Configuration package

| Region-relative address | Contents |
|---|---|
| 0x000..0x0FF | Header page |
| 0x100..0x1FF | Completion page, programmed last |
| 0x200 onward | Payload, padded to 256-byte pages |

Header: magic 0x47464350 at 0 (u32), version 2 at 4 (u16), payload byte length at 8 (u32),
payload CRC at 12 (u32), header CRC over bytes 0..251 at 252 (u32). Completion page: magic
0x54494D43 at 0, header CRC at 4, payload CRC at 8 and payload length at 12 (all u32). All
metadata is checked before bounded decoding.

Payload field order:

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

Kind/comparison/phase encodings follow the explicit current declarations in
`RecoveryConfiguration.h`; changing their numeric meanings requires a format version change.
Tagged irrelevant fields are retained, but never evaluated by the configuration store. Extra
payload bytes and excessive counts are rejected.

Current maximum structural payload is 2,101 bytes (all arrays at capacity, before global
channel-ownership validation). The largest semantically possible plan is smaller because there
are only eight uniquely assignable physical channels. The codec buffer is 4,096 bytes, guarded
by a compile-time single-sector size check; the reserved region remains 16 KiB. This
configuration format is separate from the flight-log/header format, which embeds this same
encoded plan.

Condition-kind codes are Phase=0, ApogeeDetected=1, AltitudeComparison=2, AltitudeRange=3,
AbsoluteVelocityComparison=4, AbsoluteVelocityRange=5, TimeSinceLaunch=6, TimeSinceApogee=7,
TimeSinceRule=8. Comparison codes are Less=0, Greater=1, LessEqual=2, GreaterEqual=3. Output
kind is Pyro=0, Servo=1.

## Directory journal

Each bank has 64 slots of 512 bytes: a 256-byte body and separate 256-byte commit. The body CRC
includes reserved FF bytes; the commit binds that CRC and its complement.

| Body offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x52494450 |
| 4 | 2 | Version 1 |
| 6 | 1 | Kind: 0 bank header, 1 flight update |
| 8 | 8 | Bank generation, starting at 1 |
| 16 | 4 | Slot index |
| 24 | 8 | Flight sequence, or sequence high-water mark in header |
| 32 | 4 | Flight start, or baseline entry count in header |
| 36 | 4 | Committed logical byte length, flight updates only |
| 40 | 4 | Allocated erase-aligned extent length, flight updates only |
| 44 | 1 | Flight status, flight updates only |
| 252 | 4 | CRC-32/ISO-HDLC of body bytes 0..251 |

Header bodies additionally bind the layout using five uint32 values at offsets 40,44,48,52,56: A
start, B start, bank size, data begin and exclusive data end. Commit pages store magic
0x54494D43 at offset 0, body CRC at 4 and its complement at 8; the rest is FF. Body is
written/readback-verified first; commit page last.

Status values: 1 Recording, 2 Completed, 3 Interrupted, 4 StoppedFull, 5 Invalidated, 6 Corrupt.
Existing status values and the normal format-1 encoding are unchanged; firmware must understand
value 6 to mount a directory containing quarantined flights.

Slot zero is the bank header. Its baseline count is the number of copied retained entries; its
saved high-water mark prevents sequence reuse after deletion. Generation and recording sequence
are nonzero uint64 values; exhaustion fails without wrapping. Each bank has 64 slots; MaxEntries
is 48 retained flights.
The RAM ordering of entries is not their stable identity. See [transactions](STORAGE.md#directory-transactions).

## Recording header v4

`plan(config, limits, layout)` validates the complete plan and computes layout without flash
access. It reserves exactly one slot per assigned output, ordered by rule order and assignment
order, up to all eight physical channels. There are no slots for unassigned channels. Duplicate
ownership is rejected by configuration validation. `create()` accepts only sector-aligned
locations inside the flight-data region, checks the complete header reservation is erased, and
never overwrites existing bytes. The caller must already own the reserved extent; erased space
is not proof that it is free. Allocation and explicit reuse belong to FlightRecorder.

The header reservation is sector-aligned, allowing the existing `FlightSampleStore` to start at
`flightStart + layout.headerBytes` in a separately owned sample extent. The table is sized
before any samples are written and is never extended later. The small three-output fixture needs
4 KiB; an eight-output header needs 8 KiB. The exact size also depends on the encoded
configuration.

Header regions are:

| Relative address | Contents |
|---|---|
| 0 | 256-byte descriptor |
| 256 | Separate descriptor/configuration commit page |
| 512 | Configuration codec-v2 payload, padded to a 256-byte boundary with FF |
| `tableOffset` | One 512-byte output slot for each assigned channel |
| after table | Always-reserved 512-byte final summary |
| after reserved slots | Erased padding through the next 4 KiB boundary |
| `headerBytes` | Beginning of separately stored sample blocks |

Descriptor fields:

| Offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x52444850 |
| 4 | 2 | Header version 4 |
| 6 | 2 | Descriptor bytes 256 |
| 8 | 4 | Total header reservation bytes |
| 12 | 4 | Configuration offset 512 |
| 16 | 4 | Actual configuration payload bytes |
| 20 | 2 | Configuration codec version 2 |
| 22 | 1 | Sensor mode: 0 hardware, 1 simulation |
| 23 | 1 | Output mode: 0 physical, 1 virtual |
| 24 | 1 | Frame: 0 sensor axes, 1 logged calibrated body |
| 25 | 1 | Sample block format version 2 |
| 26 | 1 | Assigned-output slot count |
| 28 | 4 | Output-table offset |
| 32 | 2 | Slot bytes 512 |
| 36 | 4 | Physical header address, binding relocated metadata |
| 40 | 4 | Sample-data offset, equal to header reservation bytes |
| 44 | 4 | Summary offset, exactly after output slots |
| 64 | count x 8 | Output mappings: uint32 rule, uint8 kind, uint8 channel, FF x 2 |
| 160 | 40 | Per-flight calibration and state-machine snapshot; [fields below](#per-flight-calibration-snapshot-header-v4) |
| 252 | 4 | Immutable CRC |

Unspecified bytes are FF. Output kind is 0 pyro or 1 servo; channels are one-based. The
immutable CRC is CRC-32/ISO-HDLC over descriptor bytes 0..251 followed by the padded
configuration payload. Mutable table/summary bytes are excluded. The commit page contains magic
0x43444850, immutable CRC and its uint32 complement at offsets 0,4,8; remaining bytes are FF.
Payload pages are programmed/readback-verified first, then the descriptor, then its commit. Each
page is programmed once between erases. Creation is acknowledged only after decoding through the
normal read path.

The header omits a flight ID, firmware ID, launch pressure, ranges and nominal rate. The
sequence remains directory/block bookkeeping. Current header v4 always reserves the final
summary; v1/v2/unknown headers are rejected without migration. Layout equations are:

```text
tableOffset   = 512 + align256(configBytes)
summaryOffset = tableOffset + slotCount * 512
headerBytes   = align4096(summaryOffset + 512)
sampleStart   = flightStart + headerBytes
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

## Samples and blocks v2

`FlightSample` preserves the agreed CSV order:

```text
t_ms,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,pressure,temp,z_est,vz_est,roll,pitch,yaw,az_est,state,battery_v
```

Samples have 64 bytes: uint32 little-endian milliseconds at offset 0, fourteen IEEE-754 binary32
values at offsets 4..59 in the listed order, phase uint8 at 60, battery uint16 millivolts at
61..62 and abort reason at 63 (0 none, 1 velocity median, 2 tilt). Battery FFFF means
unavailable; zero means a measured 0 V. The record remains 64 bytes, but block version 2 and
recording-header version 4 are required; old recordings are rejected, not migrated.
Timestamp zero is valid; accepted timestamps strictly increase without uint32 rollover.
Phase mapping is 0 PAD, 1 BOOST/BOOST 1, 2 COAST/COAST 1, 3 DESCENT, 4 LANDED,
5 BOOST 2, 6 COAST 2, 7 ABORT. The frozen machine setting disambiguates stage-one labels.
Unknown phases
are rejected. All finite float values are preserved, including heights above 327.67 m.
Missing/nonfinite measurements encode canonical quiet NaN 0x7FC00000, rather than zero or a
saturated measurement. No precision-reducing quantization is applied. The source convention
(sensor axes or calibrated body) belongs to the FlightHeaderStore metadata; this low-level
component does not infer it.

Output starts use the independently committed FlightHeaderStore table, as required by the flash
design. There are no output events, deployment flags, magnetometer fields or rule-trigger
records in samples.

| Fields | Stored/exported unit |
|---|---|
| t_ms | Milliseconds from the run origin |
| accelX/Y/Z, az_est | m/s^2 |
| gyroX/Y/Z | rad/s |
| pressure | hPa |
| temp | C |
| z_est | Launch-relative metres |
| vz_est | m/s |
| roll, pitch, yaw | Degrees |
| state | Numeric phase |
| battery_v | Stored as integer millivolts, exported in volts; FFFF exports null/CSV nan |
| abort_reason | Integer 0 none, 1 velocity, 2 tilt; retained through the terminal ABORT phase |

Battery acquisition is real board telemetry at 5 Hz, independently of the navigation sensor
source. Samples hold the latest acquisition, including during onboard simulation/HIL; Hoppino
contains no fabricated battery trace. This adds no sample/block space or extra flash writes.

Each 1024-byte block has three 256-byte body pages and a separate 256-byte commit page. Every
page is programmed once between erases. Integers are little-endian.

| Body offset | Width | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x4B425350 |
| 4 | 2 | Block version 2 |
| 6 | 2 | Sample size 64 |
| 8 | 8 | Nonzero recording sequence, bound to directory ownership |
| 16 | 4 | Zero-based block index |
| 20 | 2 | Sample count 1..11 |
| 22 | 2 | Reserved FF |
| 24 | 4 | First sample ordinal within recording |
| 28 | 4 | Physical block address |
| 32 | count x 64 | Encoded samples |
| after samples | remaining | FF padding |
| 764 | 4 | CRC-32/ISO-HDLC over body bytes 0..763 |

Commit page: magic 0x4D435350 at offset 0, body CRC at 4, its uint32 complement at 8; remaining
bytes are FF. Body pages are programmed and readback-verified before commit. The committed block
is decoded and checked before advancing writer counters. A full block uses 704 payload bytes for
11 samples: about 93.1 physical bytes per sample including block overhead. Explicit partial
flushes cost more space. Four blocks fit one erase sector. This is the current uncompressed
encoding.

The logging adapter converts pressure Pa to hPa and Euler radians to degrees. The source-frame
convention is frozen in the header; it is not inferred from the sample. Final partial blocks
contain their actual count. Four blocks fit one sector; 1 MiB holds 1024 blocks / 11,264
samples, before header reservation.

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

Component versions are independent: configuration codec is v2 and recording header is v4;
sample blocks are v2; directory/start/summary remain v1. Only configuration codec v2 is read and written;
older packages and snapshots are rejected. There is no conversion or legacy decoder.
No automatic erase or migration writes occur at boot. `flights info` reports `config_version:2`
and `flights config` exports that codec.
Directory status values are Recording=1, Completed=2, Interrupted=3,
StoppedFull=4, Invalidated=5, Corrupt=6. Firmware that only accepts 1..5 cannot mount a bank
containing Corrupt and must not autoformat it. The frozen whole-image FNV64 regression value is
`0x3000346afdd6427f` (current fixture uses zero channel delays).

## Per-flight calibration snapshot (header v4)

Descriptor bytes 160..199 store the immutable startup calibration, covered by the same
CRC and commit as the configuration snapshot. Old header versions are unsupported;
use explicit `flights format` to start with the new format. The rule codec remains v2.

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

Unavailable snapshots use flags 0 and FF in all remaining snapshot bytes. When input is
already calibrated BODY data, axis and bias fields remain FF and export as null, rather
than inventing a physical calibration. The finite barometric reference is still saved.
All remaining descriptor padding stays FF. `flights calibration <sequence>` exports this
snapshot separately to keep replies bounded; the downloader stores it under `calibration`
in the flight JSON sidecar. No current device settings are consulted when displaying it.

## Board mounting record

The second sector of the configuration reservation (0x002000) contains the orientation,
independent of the unchanged v2 rule package at 0x001000. Its first 256-byte page stores
magic 0x544E524F at 0 (u32), version 1 at 4 (u8), signed axis enum at 5 (u8:
+X, -X, +Y, -Y, +Z, -Z = 0..5), and CRC32 of bytes 0..251 at 252 (u32).
Other bytes remain FF. Saving erases only this sector and programs one verified page.
An erased page means orientation is unconfigured; missing/invalid records or read errors block physical arm with ORIENTATION_NOT_READY until an explicit orientation is saved.
Factory reset clears the rule package and erases the orientation sector; formatting flights preserves this sector.

## State-machine device record

The third sector of the configuration reservation (0x003000) stores mode independently
of rules and orientation. Its first page uses magic 0x4D534650 at offset 0 (u32), version 1
at 4 (u8), bistage 0/1 at 5, minimum COAST 1 velocity at 8 (float32 m/s), and CRC32 over
0..251 at 252 (u32). Reserved bytes are FF. Missing/erased record defaults to normal and 0;
invalid records prevent run readiness until rewritten. Saves erase/program/read back this
sector. Factory reset erases it; flight-memory format preserves it. No calibration
reservation or retained flight extent is touched by a mode change.
