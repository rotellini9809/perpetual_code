# Recording, download and timing

MAIN records normal physical flight, sensor HIL and autonomous simulation on W25Q128JV,
exporting sample CSV plus frozen metadata JSON. HIL is virtual; simulation defaults to virtual
and explicitly supports physical outputs. Normal HIL without `--record` performs no
flight-storage writes. Valid saved settings and no draft are required; close Serial Monitor
before Python opens the board's port (`COMx` below).

## Initialize and recover

On genuinely blank storage, upload MAIN and send once in Serial Monitor:

```text
flights init
```

Expect `ready:true`. Existing valid metadata is mounted/recovered and retained; both complete
banks must be erased to authorize blank initialization. Unknown non-erased contents fail,
without autoformat. Boot automatically mounts existing recording storage and reconciles interrupted
recordings. Clients may still issue `flights recover` to repeat that operation; it does not reinitialize
a driver disabled by timeout. The latter currently needs a restart.

Do not use the directory diagnostic as an initializer on retained data: it mutates
metadata with only a RAM backup. See [diagnostic regions](README.md#manual-firmware-modes).
Corrupt flights are retained/flagged and their space remains reserved; clients warn after
recovery and `--list` labels CORRUPT. Healthy history stays accessible and new runs can use
other free space. Ordinary incomplete tails recover as Interrupted.

## Explicit flight-memory format

`flights format` intentionally removes all recorded flights and makes their space reusable,
including when old/unsupported headers prevent recovery. It preserves configuration,
orientation and the accelerometer-calibration reservation. Run it only while disarmed/idle.
This is a logical format, not secure erasure; see [storage behavior](../technical/STORAGE.md#explicit-deletion-and-inspection).

## Record

```powershell
python tools/replay/replay_sensors_usb.py --port COMx --record
python tools/replay/run_simulation_usb.py --port COMx --record
```

Onboard Hoppino records like a physical flight: 512 KiB erased at preparation (about 1.5 s),
growing in 512 KiB steps while recording, the oldest flights overwritten when the ring is full.
USB Hoppino HIL keeps an explicit sample budget (`--record-bytes
<positive_multiple_of_4096>`, 1 MiB by default) inside the same growth scheme. HIL also accepts `--input`, `--pressure-unit hpa|pa` and `--output`. Temperature is
sent from Hoppino, or NaN when absent. The live USB CSV and later flash export are separate
artifacts.

For the Hoppino excerpt, expect `6600 saved, 0 dropped; storage None` when the worker keeps up.
`None` is the error name for success, not a missing storage object. Output starts depend on the
applied plan. These are test workflows, not flight settings.

`hil stop [completed]` stops processing/cancels pending starts and submits close. Poll `hil
status` until `recording_busy:false`, then check `recording_error` and final counts. `sim
status` uses the same busy field. Config mutations/new runs, download/recovery and detailed
timing stay BUSY during closure, including after HIL disconnect. Both recording clients perform
that wait on the PC. Successful enqueue/accepted close is not proof of completed persistence.

## List, download and explicit deletion

```powershell
python tools/replay/download_flight.py --port COMx --list
python tools/replay/download_flight.py --port COMx --sequence 1
python tools/replay/download_flight.py --port COMx --delete <sequence>
```

Lists are newest first by persistent uint64 sequence: `flights list 0` is the newest retained
flight, including a corrupt one if it is the latest. Reboots and deletion preserve the sequence
high-water mark; new recordings never reuse deleted identities. No separate latest-flight flag
or header rewrite is needed. The app displays this order and preserves a selection by sequence
when refreshed; with no surviving selection it selects the first row.
Select the printed stable sequence, not a changing list index. Default unique files under `.pio`
are `flight_<sequence>_<uuid>.csv` and sibling `.json`. `--output path.csv` requires both
destinations to be unused/non-overlapping. Incomplete transfers keep partial CSV and do not
publish complete metadata. A Corrupt flight cannot be exported as healthy; another healthy
flight still can.

CSV columns are defined once in [Storage formats](../technical/STORAGE_FORMAT.md#samples-and-blocks-v4).
The metadata contains sequence/status/sample count, sensor/output/frame modes, frozen
configuration with rule names and codec hex, output-start states/timestamps, and
recording_summary or its unavailable state, and the frozen `calibration` snapshot retrieved with
`flights calibration <sequence>`. Its schema is [defined once in Storage formats](../technical/STORAGE_FORMAT.md#per-flight-calibration-snapshot-header-v8). Historical names/actions come from that flight, even
if applied settings change. Null numeric values decode as NaN. An empty/invalid output slot is
not proof the physical output never operated.

Deletion is deliberate, idle/disarmed and permanent at the directory level. It durably
invalidates before reuse; physical data erasure happens on later allocation.
Settings/calibration/other retained flights remain protected. There is no automatic deletion or
new user UI; the existing technical command also
handles Corrupt entries. See [storage lifecycle](../technical/STORAGE.md).

## Current USB commands

```text
flights status
flights init
flights recover
flights format
flights list <zero_based_retained_index>
flights info <uint64_sequence>
flights config <uint64_sequence> <byte_offset>
flights block <uint64_sequence> <block_index> <byte_offset>
flights summary <uint64_sequence>
flights calibration <uint64_sequence>
flights failure
flights delete <uint64_sequence>
flights timing
flights timing reset
```

Sequences are decimal JSON strings to preserve uint64 precision. Configuration chunks carry at
most 128 bytes as hex; block chunks carry at most 448 bytes of a raw v4 block body as base64
(four chunks per 1792-byte body), decoded only on the PC by `tools/replay/flight_log.py`. Numeric
modes/start states/status values are in [the format specification](../technical/STORAGE_FORMAT.md).
Idle status/recovery expose `corrupt_flights`, and list entries include `corrupt`. `hil
status/stop` expose requested/active/busy recording, sequence, durable saved/ dropped samples,
pending/queue capacity, fullness and independent storage error. Saved counts lag buffered
samples. While the phase is PAD nothing is written: the last 0.5 s at full rate and 10 Hz for the
10 s before stay in RAM and are written when the first non-PAD sample arrives (or at close for
simulation/HIL), so fewer samples are saved than processed. Full sample storage drops further samples while starts remain recordable. Logging
faults do not stop HIL navigation.

`flights failure` exposes error/component/cause and typed flash cause/operation/ address/bytes.
Operation codes are 0 none, 1 read, 2 program, 3 erase. Detailed native/profile state is
unavailable while the worker owns it. Reads/recovery and mutation policy remain
disarmed/inactive; status uses safe live snapshots.

## Timing

```powershell
python tools/replay/replay_sensors_usb.py --port COMx --record --timing
python tools/replay/replay_sensors_usb.py --port COMx --record --realtime
```

The client resets flash counters after mounting/settings inspection, records, waits for close
and writes an exclusive sibling `.timing.json`. `--realtime` also adds
send_late_us/round_trip_us to the live CSV; without `--record`, timing can measure write-free
HIL. Autonomous simulation writes its own unique JSON with status/timing/events and optional
flash counters.

| Metric | Included work |
|---|---|
| prepare_us | Recorder preparation: owned extent erase/verification, immutable header and initial directory checkpoint |
| finish_us | Close request cost; queued firmware returns before the worker drain/commit. The direct synchronous sink includes finalization |
| compute | Source consumption, preprocessing, navigation and rule evaluation |
| storage | Sample conversion, start notifications and sink submission; MAIN enqueues work without flash access |
| step | Computation plus storage submission on the producer core |
| worker | Core 1 iteration handling starts/sample and any flash service, including finalization |
| read | Calls through FlashDevice::read |
| program | Calls through FlashDevice::program, including driver preflight reads, busy waits and verification |
| erase | Calls through FlashDevice::eraseSector, including busy waits and full-sector verification |

These are nested measurements. Do not add flash-operation totals to step or prepare/finish
times. Driver-internal reads are already inside program/erase; the read category contains
separate storage-component read calls. Operation counts describe API calls, which need not equal
physical page commands. Failures and attempted bytes are counted even when the driver returns
false.

Step timing excludes USB reception/transmission, command parsing, acquisition metadata
construction in `sample()`, JSON formatting and host delays. It is not the complete periodic
hardware control loop. `over_budget` counts accepted steps whose measured duration exceeds
`nominal_period_us`, the integer period derived from the supplied nominal HIL rate. USB replay
waits for each reply; `--realtime` additionally schedules absolute send times and records
lateness rather than shifting later deadlines. Untimed queued recording uses explicit PC flow
control. A nonzero over-budget count does not by itself prove lost samples. PC/USB pacing is
distinct from an autonomous hardware-timer sensor source.

Per-operation maxima are observations, not worst-case device guarantees. Timings include the
small instrumentation overhead and interrupt delays within each scope. Subtraction handles one
`micros()` rollover; an individual operation must remain shorter than the uint32 microsecond
interval. Totals accumulate in uint64.

All statistic arrays are `[count,total_us,max_us]`; mean is total/count when nonzero. `hil
timing` is safe during HIL; detailed `flights timing` requires worker release. `flights timing
reset` is idle/disarmed and changes RAM only. HIL counters reset on begin; flash counters
include every profiled access since reset (including later downloads/settings), so inspect
before unrelated storage work. Reboot loses these RAM counters. `timing_valid:false` means no
clock was supplied (host harness), not zero hardware latency; an unprofiled target returns
TIMING_UNAVAILABLE.

Simulation `cycle` adds source generation and phase/start bookkeeping to processing; it excludes
idle polls, USB, preparation and final drain. `missed_ticks` counts skipped source slots;
`max_lateness_us` is arrival after the original next deadline before removing skipped slots.
`finish_us` measures queued close submission now; historical pre-asynchronous reports measured
complete drain instead. Observed
metrics and the 1 MHz/4 MHz comparison live in [Validation](VALIDATION.md).
