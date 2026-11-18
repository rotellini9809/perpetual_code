# Storage behavior and recovery

`FlightRecorder` coordinates a synchronous directory/header/sample/summary stack. MAIN wraps it
with `QueuedRecorder`, which exposes the same producer contract
without flash calls from the sensor path. [Formats](STORAGE_FORMAT.md) owns the
map/offsets; [Recording](../testing/RECORDING.md) owns commands and timing usage.

## Driver and verification

`FlashLogger` operates the fixed W25Q128 at the board's 18.75 MHz SPI setting (FLASH_TEST `c`: error-free to 37.5 MHz, 75 MHz fails). Busy waits poll status every 10 us for the first 5 ms (page programs take ~0.4 ms), then every 1 ms. Flight sample blocks use `programErased()`, which skips the pre-write NeedsErase read because preparation erased and verified the whole extent; post-write verification is unchanged. Measured with the onboard Hoppino replay recording (200 Hz): worker 8.3 ms per 1 KiB block (was 21.8 ms at 4 MHz with 1 ms sleeps), about 44% of core 1 at 476 Hz. Preparation erases the new extent with `eraseRange()` (64 KiB block erases where aligned, 4 KiB elsewhere, no read-back); `FlightHeaderStore::create` and `FlightSampleStore::begin` then verify the whole extent erased exactly once. 1 MiB arming went from 7.9 s to 3.3 s and the worker to 6.3 ms per block (about 33% of core 1 at 476 Hz) after also replacing the bitwise CRC with a nibble table and removing fixed microsecond delays around flash transactions. With log format v4 ([2 KiB blocks of tagged records](STORAGE_FORMAT.md#samples-and-blocks-v4)) the worker needs about 7.8 ms per block including one end-to-end read-back, about 7% of core 1 at 476 Hz; PAD samples stay in the RAM pre-launch history and reach flash only at the first non-PAD sample. Initialize wakes the
chip, requires three matching supported JEDEC reads and waits for ready. Read/program/erase
reject invalid arguments, bounds and alignment. Program preflights the entire range for
forbidden 0-to-1 changes, splits at page boundaries, checks write-enable and verifies each
written chunk. Erase verifies all 4096 bytes. Multi-page writes are not atomic; failure may
leave a programmed prefix.

Busy polling uses wrap-safe milliseconds: 20 ms program completion / 2000 ms erase and readiness
waits. Timeout clears driver readiness; `flights recover` does not reinitialize the driver.
Reboot is the current application route to reinitialization; in-place timeout recovery was
explicitly deferred. JEDEC/SR/legacy-ID reads diagnose communication/identity, not memory
integrity. There is no whole-chip erase command.

`FlashLoggerAdapter` maps typed errors; `ProfiledFlashDevice` adds outer operation,
address/length and RAM statistics. `VerifiedPage` performs a shared program/readback comparison,
allocating nothing and never retrying or erasing. Stores preserve their commit ordering; sample
commits retain full-block validation. Driver and store readbacks have separate contracts and
have not been removed as a speculative speedup.

## Preparation and allocation

Boot reads settings and, after successful flash initialization, calls `initialize()` to mount
existing flights and reconcile interrupted recordings before accepting commands/runs. Only a never-used
flash (directory missing, both banks erased) is prepared at boot through `initializeBlank()`; used or
damaged storage is never formatted, and failed mounting leaves recording unready with diagnostics in
`flights status/failure`.
`flights recover` can repeat mounting manually; `initializeBlank()` (`flights init`) formats only
if both complete directory banks are erased.
Unknown non-erased metadata fails closed. Settings/calibration are outside the recording banks.
Erased data is not proof of free ownership.

Begin validates the frozen plan/modes and positive sector-aligned sample budget, then finds a
free contiguous header+sample extent. Retained Interrupted, Completed, StoppedFull and Corrupt
flights all occupy space. There are 48 retained identities; NoSpace, DirectoryFull and
SequenceExhausted are distinct. There is no active growth, wrapped extent, erase-ahead,
automatic oldest deletion or periodic `checkpoint()`.

The transaction is: commit a new Recording reservation (committed length zero), erase only that
extent, create/verify its header, preflight the sample area, then publish the header-ready
length in the directory. Only afterward accept samples. A zero-length reservation cannot
authorize stale bytes left before erasure.

## Producer and worker contract

`append()` accepts samples in RAM. An eleven-sample native staging buffer flushes only in worker
service; append itself performs no I/O. `recordStart()` preserves the first actual logical
command-start timestamp against its immutable mapping; an identical duplicate is idempotent and
a conflicting time/unknown mapping fails. The worker services starts before samples; a full RAM block is written by the append that overflows it.
Dedicated start capacity remains usable when sample capacity fills. Every durable block is read
back and decoded before saved counters advance. Last partial blocks flush at finish.

MAIN has a 128-slot sample FIFO (about 270 ms at 476 Hz) and separate eight-start FIFO. Both are fixed SPSC queues with
release/acquire indices, no allocation/overwrite, and quiescent-only reset. Full queue returns
Backpressure; full sample extent returns Full. HIL counts producer rejects and worker background
drops separately and reports their sum. Saved means verified persistence, not successful
enqueue. Queue size cannot fix insufficient sustained throughput; the 1 MHz measurements
demonstrated that.

Physical arming submits preparation to core 1 with `beginAsync`: Preparing -> Prepared -> Running.
Core 0 keeps servicing USB/Bluetooth and accepts disarm while erase/verification runs.
Prepared is an acknowledgment barrier: only then does core 0 adopt metadata and start the flight
epoch/outputs. Cancellation stays Closing until core 1 finishes its in-progress preparation
and invalidates the unused reservation. A preparation fault never arms the board.
The synchronous `begin` entry remains available for HIL/simulation and host fixtures.
Only core 1 touches native stores/profile buffers during asynchronous ownership. Core 0 keeps an immutable mapping/identity snapshot and
atomic progress; detailed native inspection is BUSY. Core 1 has a separate 8 KiB stack and event
wake/sleep. One physical flash owner also serializes settings and downloads; this is not
arbitrary concurrent SPI access.

## Asynchronous close and abort

The lifecycle owner cancels future starts before calling `finish(true, summary)`; false
producerClosed is rejected. For QueuedRecorder, success means request accepted. Closing drains
accepted samples/starts, flushes the partial block, commits summary, then publishes Completed or
StoppedFull. The caller supplies drop count/end reason; the worker supplies actual queue
peak/capacity/background drops, and the recorder supplies verified saved count. Full capacity
overrides the reason to StorageFull.

Orderly hardware-mode closure uses the runtime's actual BOOST history: without BOOST,
`finish(..., RecordingRetention::Discard)` commits Invalidated directly, skips queued/RAM
samples and start records, and leaves data sectors for later reuse. The directory length and
extent are preserved in that transition. A full queue or sample extent cannot hide a computed
BOOST or cause a launched flight to be discarded. Simulation/HIL retain PAD-only logs for
analysis. Processing failures retain their diagnostic recording; abort/storage faults and
reset recovery retain uncertain interrupted logs. Cancelled asynchronous preparations invalidate their unused reservation on the worker;
preparation faults retain uncertain metadata for recovery.

Close and abort return without waiting. `recording_busy` stays true until worker release; a
stopped sensor run may still be closing. Final status acquires that acknowledgment before
reading terminal counters. Settings mutations, new runs, downloads, recovery and detailed timing
remain excluded until release. Python clients wait on the PC and then check errors/counts. A
late persistence failure must not be reported as successful recording just because close was
accepted.

Abort requests RAM abandonment at the worker's next cancellation check; it does not forcibly
interrupt an executing flash operation. Native abort performs no flash writes and preserves the
durable Recording entry. No progress deadline or forced ownership transfer was added: a stalled
worker still owns storage, while core 0 remains free. Idle preparation/settings/scans may still
block core 0.

## Recovery and corruption

Mount never resumes navigation, outputs, pending rules or append writes. For each Recording
entry, zero committed bytes becomes Interrupted without reading old data. Otherwise verify the
header and scan blocks in sequence, checking identity, physical address, ordinal and increasing
time. Include valid blocks beyond the published header length. Stop at the first
erased/incomplete unpublished tail. Persist the verified prefix as Interrupted, retaining its
full allocated extent. Recovery is repeatable if the final directory update itself is
interrupted.

A committed block/header that fails structural/integrity checks is quarantined as directory
status Corrupt (6), preserving its original extent and published length. The immutable header
and sample bytes are not rewritten. Healthy history remains downloadable and new recordings may
use other free extents. The sticky flag survives reboot/compaction; it neither repairs samples
nor releases space. `inspect()`/`readBlock()` may also persist a flag when detecting
closed-flight damage. Known Corrupt flights return Corrupt without another flag write. There is
no full scan of every closed flight at boot; corruption is reported when detected.

Ordinary incomplete tails stay Interrupted, not Corrupt. Invalid output slots and summary states
are independent and need not hide healthy samples. Read I/O errors, unsupported header versions,
damaged directory metadata and failure to commit a flag fail the operation instead of
authorizing quarantine/reuse. No shortened published length hides committed damage. A valid
summary with an unclosed directory entry remains interrupted. Uncertain pre-BOOST logs are
retained by recovery; the orderly hardware-mode PAD-discard policy is not applied during recovery
or to development HIL.

## Directory transactions

Each 32 KiB bank has 64 body/commit slots. Append updates, skip/consume torn slots, and never
reprogram them. Committed bad bodies fail closed. Mutation failure invalidates the mount: a
reported error might already have committed.

Before appends consume the two reserved slots, compact into the inactive bank: erase it, copy
retained entries, then commit its generation+1 header last. Mount requires the whole baseline
and selects the complete compatible highest generation; equal generations are ambiguous. Old
bank remains until a later compaction. Compaction drops only tombstones and preserves the
durable uint64 high-water mark, including an empty baseline; deleted identities cannot return
under old sequence IDs.

Recording can grow monotonically and close, or be invalidated at orderly pre-launch closure.
Invalidation preserves its published length and extent. Corrupt conversion preserves length/ extent and
cannot revert. Invalidated never returns to retained state. All retained extents must be
disjoint and contiguous, including CRC-valid metadata; unsupported wrapped metadata is rejected
without treating its region as free.

## Explicit deletion and inspection

`flights format` explicitly formats both flight directory banks while disarmed and idle. All flights become inaccessible and their storage is reusable; this is a logical format, not secure data erasure. Configuration and calibration regions are preserved. It also works when the directory cannot be mounted.

User deletion is explicit, disarmed and inactive; the lifecycle also discards unflown hardware
recordings during orderly closure. Commit Invalidated before space becomes
allocatable. It does not erase flight sectors; later allocation reserves the new owner before
erasing reused data. A surviving tombstone makes repeated delete idempotent; after compaction,
the old ID returns StaleSelection. Retained identity capacity is reusable rather than limited to
48 lifetime recordings. Corrupt flights remain occupied until explicit deletion; no new user
deletion UI or automatic cleanup of retained/corrupt history was introduced.

Select by stable sequence, not mutable list index. Valid inspection scans every published block;
individual reads check adjacent ordinal/time continuity. Failures leave caller outputs
unchanged. Empty interrupted reservations return EmptyRecording. Historical configuration never
replaces applied settings. FlightCommands caches closed metadata/block selection;
recover/delete/read failure invalidate it. Corruption detected on inspection can therefore write
the directory under idle/disarmed ownership.

## Failure diagnostics and limits

`RecordingFailure` records recorder error, component/cause and typed FlashFailure
(read/program/erase, address, bytes). The first fatal snapshot survives later failed API calls
and unrelated successful reads until explicit initialization/recovery. Rejected arguments do not
latch a permanent fatal fault. Boolean-only mock devices yield Unknown driver cause. Detailed
snapshots/profile buffers require acknowledgment.

Corruption counts/flags are exposed by idle status/recovery/list; clients warn and the list
labels CORRUPT. Counts from an interrupted run are unavailable unless a valid matching final
summary was committed. Counters/timing/failure snapshots are otherwise RAM-only. Diagnostics and
format versions are detailed in the linked
format/recording guides. [Validation](../testing/VALIDATION.md) separates host
fault-model evidence from measured hardware behavior and unresolved electrical tests.

