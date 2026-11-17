# Shared USB/Bluetooth command service

MAIN exposes USB serial at 115200. Commands end with LF or CRLF; the monitor's send_on_enter
filter sends a line on Enter. There are no unsolicited debug prints.

The same service/handlers also run on the JDY-34 transparent link through hardware UART1,
MCU TX GP4 / RX GP5, **230400 8N1**. USB is checked every loop, independently of Bluetooth.
There is no wait for either connection and no periodic USB throttle. Each endpoint has its
own framing, retry cache and pending reply. Existing PC runners can use the outgoing Windows
SPP port (COM11 on this PC) instead of COM9; their PC-side baud setting does not change the
module-to-MCU UART rate. BLE terminal operation is not yet verified.

`bluetooth status` returns `ready`, `connected`, `error`, UART `baud`, cumulative MCU `uart_rx_overflows`,
the module `name` read at startup, the last `rename` result (`None`, `PENDING`, `ACTIVE`, `OK` or a failure
code), `configured` (a factory module was set up this power-up) and `stage` (the current startup step, or the
one that failed); it is also available over USB when radio startup fails. Boot nonblockingly queries AT/version, single-link mode 1, notifications enabled,
the SPP name and the eight-channel connection snapshot. PWRC stays low during these queries and high
for transparent traffic. A module that answers at 230400 is never reconfigured at boot. A module that
is silent at 230400 but answers at the factory 9600 as a JDY-34 with no radio link is set up once per
power-up (names `Perpetual`/`Perpetual_BLE`, `BTMODE1`, `ENLOG1`, `BAUDA`), restarted and then checked
like any other module. The module sometimes ignores a request (the first after a quiet period, or one shortly after a restart),
so queries are repeated: the startup `AT` for about 4 s, other queries 3 times. Writes and resets are
sent once. Incompatible settings or replies still missing leave Bluetooth unavailable; USB and the flight
loop keep running. A module restart notification (`+START=OK`) repeats this startup, also after a failed one.

`bluetooth name <suffix|default>` renames the module to `Perpetual-<suffix>` (1-4 letters, digits or `-`)
or back to `Perpetual`; the BLE name is the same plus `_BLE`, within the module's 18-byte limit. It is
accepted from USB or Bluetooth while disarmed and idle. After the accepting reply is sent, MAIN writes
and reads back both names with PWRC low, then restarts the module (`AT+RESET`) because names only apply
after a restart: a Bluetooth link drops for a few seconds. Errors: `INVALID_NAME`, `BLUETOOTH_NOT_READY`,
`BUSY`; a write that cannot be read back reports `NAME_NOT_VERIFIED` in `rename` and the link recovers.

`ping`, `info`, `status`, `sensors`, `help` have readable replies without IDs. Prefix any request with
`@<uint32>` for a single compact LF-terminated JSON reply, for example `@42 status`.
Configuration groups, `capabilities`, `preflight`, `arm`, `disarm` and `ds` return JSON even
without IDs.

The flash is initialized by the reusable FlashLogger driver. Boot tries to load the single
configuration copy; it never autoformats unexpected data or installs default recovery rules.
`status` reports configuration validity/store error, flash-driver availability, actual
`arming_state`, `flight_mode`, `output_mode`, `outputs_active`, computed phase, `gyro_calibration_skipped` and sample progress. Simulation
can report armed without enabling physical channels. Phase is null while unavailable. Sensor
flags reflect actual probes. `battery_v` is the latest real 5 Hz board voltage in volts, or null
before acquisition/on ADC saturation; it does not add an arming threshold.
`power_mode` is `active` or `sleeping`; it is independent of arming and output mode.
`power_error` reports the last hardware transition failure, or `None` after success.
`status` reports observations only; readiness/reasons come exclusively
from `preflight`, avoiding the former hardcoded blocker list. Boot automatically mounts/reconciles
existing flights; `hil_recording_ready` describes initialized recording storage. `flights recover`
repeats recovery manually; genuinely blank storage still requires explicit `flights init`.
`capabilities` alone exposes `physical_actions` and `physical_simulation` as fixed integration capabilities,
not permission to arm. Outputs stay inactive at boot; `servo arm`, accepted physical flight and
explicit `sim begin physical` can command GPIO/PWM.

`sensors` adds raw sensor-axis vectors `accel_mps2` and `gyro_radps`, each `[x,y,z]`, plus
`imu_sample_valid`, `imu_sample_age_ms` and board `uptime_ms`. Vectors are null when unavailable, stale (>100 ms)
or non-finite; these observations never change arming readiness or the navigation pipeline.


## Command reference

| Group | Canonical instructions |
|---|---|
| ping, info, status, sensors, help, capabilities | This document; `help` lists the supported groups |
| bluetooth status, control status | Radio health and transport ownership below |
| sleep, wake | Peripheral power saving below; ordinary operational commands also wake automatically |
| preflight [physical\|simulation], arm, disarm / ds, servo arm [channel], servo release [channel] | Shared arming/readiness below |
| config, rule, condition, output, pyro, servo | [Configuration/edit grammar](CONFIGURATION.md) |
| replay, hil, sim | [Simulation/replay commands](../testing/SIMULATION.md) |
| flights | [Recording/download/diagnostics](../testing/RECORDING.md) |

Physical channel tests, monitoring and receiver-negotiated binary transfer are not implemented.
See [remaining work](../README.md#remaining-work-and-documentation-gaps). Current exports use
bounded JSON chunks decoded to CSV on the PC.

## Power saving

`sleep` suspends detected sensors and external flash, stops battery sampling, releases servo
PWM/preparation and disables pyros. MCU, USB and Bluetooth remain active. Automatic entry uses
the same path after **five minutes of inactivity**, even with USB/Bluetooth connected.
`wake` reinitializes flash and probes the real sensors again, preserving configuration and
flights. Both explicit commands return `power_mode` and are idempotent.

Configuration inspection/edits, flight-storage requests, `sensors`, `preflight`, arming/disarming
and run control count as activity and wake automatically when necessary. After successful wake,
the original request executes once through its normal handler/readiness checks. Failed wake
returns the hardware error without executing that request. Wake itself never prepares servo
PWM, arms or formats storage; missing/faulty sensors remain physical-preflight blockers.

Passive requests are `ping`, `info`, `status`, `help`, `capabilities`, radio/control status and
run status/events/timing. They neither reset inactivity nor wake peripherals. Thus app heartbeats
and background status polling keep the link usable without preventing power saving. Live sensor
details repeatedly request `sensors` and keep the board awake; the Dashboard uses passive status
for availability instead. There are no app sleep/wake buttons.

Sleep returns BUSY during arming/runs, drafts, servo preview, recording/finalization, pending
responses or a bulk flight transfer. The transfer guard lasts five seconds after the last
successful `flights config`/`flights read`/`flights packed` chunk, including a finished or cancelled download.
Transport ownership can instead return CONTROL_BUSY. No deferred sleep is scheduled.

Active arming/runs, drafts, previews and recording/finalization restart the idle interval until
they end. A prepared servo only holding PWM does not prevent eventual sleep: entry releases it,
and wake does not restore preparation. Hardware failures report SENSOR_SLEEP_FAILED,
FLASH_SLEEP_FAILED or FLASH_WAKE_FAILED. Automatic suspension failures retry only after another
five-minute idle interval, with the error visible in status; flash timeout/recovery policy is
unchanged. See [hardware](../HARDWARE.md#status-led-and-power-saving) for LED behavior and limits.

## Arming and readiness

`imu status` passively reports the IMU procedure and saved calibration state.
`imu begin`, `imu verify`, `imu capture <signed-axis>` and `imu cancel` control it;
see [configuration](CONFIGURATION.md#device-orientation-and-factory-reset). A procedure
reserves the run/configuration lock and transport ownership, blocks sleep and other
runs, and never enables flight outputs. The current procedure survives a transport
disconnect; reconnect can inspect/cancel it. The 180-second timeout releases ownership.
Status is passive; acquisition controls wake the board.

Preflight includes `imu_calibration_state` (`absent`, `required`, `verified`) and
`imu_calibration_reason` (`MISSING`, `NOT_VERIFIED`, `AFTER_BOOST`, `FLIGHT_OR_RESET`,
`None`). Saved approval is consumed only by physical flight. Physical arming remains
blocked until a current revision is verified, alongside existing hardware checks.

The [state-machine reference](state_macines.md#5-ciclo-di-armamento--macchina-separata-implementata)
owns operational transition functions; this section defines command effects and replies.

```text
preflight
preflight simulation
arm
disarm
ds
```

Bare `preflight` selects physical mode. It returns `ok:true` with a `ready` flag, all blocking
`reasons`, `failed_pyro_mask`, `unprepared_servo_mask`, `unconfigured_servo_mask` and `prepared_servo_mask`; bit 0 means channel 1. Only configured
pyros are read for continuity, sequentially. It never prepares/fires outputs or initializes
recording. Busy checks run before hardware inspection. Simulation preflight checks common
configuration/ownership only; it skips physical readiness and does not request recording.

`arm` requests physical mode (120 Hz nominal, 1 MiB sample budget). On the present board it
returns `ok:false`, normally `IMU_UNAVAILABLE`, with the complete blocker list. Depending on the
current state this also includes `STORAGE_NOT_READY`, `CONTINUITY_FAULT`, `SERVOS_NOT_PREPARED`,
`SERVOS_NOT_CONFIGURED` (an assigned servo has no saved endpoints),
and `ORIENTATION_NOT_READY` (no explicitly saved mounting).
Mount/recovery failures remain visible in `flights status` and `flights failure`. Boot prepares only a never-used
flash (no directory, both banks erased), like `flights init`; it never formats used or damaged storage.
MAIN does not provide a way to override physical activation readiness. A rejected preflight
does not reserve a flight or write flash. Positive physical calibration/start transitions are
currently tested on PC with fake sensor/GPIO adapters and the production physical executor.

`servo arm` prepares all assigned servos; `servo arm <channel>` prepares one assigned channel.
It uses applied endpoints, rejects missing configuration or active drafts/runs, and reports the
prepared mask. PWM remains active after success; disconnect or a rejected Arm does not release it.
For disarmed editor positioning, `servo preview <channel> <pwm>` holds a bounded PWM without
saving settings or marking the servo prepared; `servo release <channel>` releases that preview.
Previews block arming/runs and are released when their controlling transport disconnects.
`servo release` without a channel releases held PWM while idle/disarmed; idle `disarm`/`ds`
does not release preparation. Normal Arm always selects physical outputs,
collects gyro bias, rechecks readiness and prepares recording before starting the flight epoch.
After landing's 15-second window it cancels future starts, finishes active pulses, releases PWM
and requests asynchronous recording close. Held preparation does not block idle settings or HIL/estimate replay. Virtual onboard simulation
releases it before starting; physical simulation adopts valid preparation and prepares missing
assigned channels through the same executor.

Simulation still starts with `sim begin [physical] [record]` or `hil begin ...`: the same controller goes
directly to Armed, with zero gyro calibration samples. `disarm`/`ds` target the active physical,
onboard-simulation or HIL controller; they cancel an arming attempt or stop at PAD/LANDED.
Airborne attempts return `DISARM_PROHIBITED_IN_FLIGHT`. A separate active estimate replay
returns BUSY and is stopped through `replay stop`. Virtual `sim stop`/`hil stop` still allow
explicit debug cancellation at any phase. Recording close remains asynchronous; inspect
`recording_busy` before starting another operation.

Arming replies also expose `boost_reached`, `recording_busy`, `recording_discarded` and
`recording_error`. Orderly physical PAD disarm requests log discard; `recording_discarded` becomes
true only after its durable directory invalidation, never merely on close submission. Logs that
reached BOOST are retained even if their BOOST samples were dropped. Development simulation/HIL
also retain PAD-only recordings. The current board's IMU fault blocks normal physical arming;
physical output execution itself is integrated, including opt-in physical simulation.

## Sensor diagnostics

After uploading MAIN, send `sensors` for a readable reply or `@42 sensors` for JSON. It reports
IMU/barometer initialization, chip IDs, last driver errors, barometer sample validity/age,
pressure in Pa and temperature in C. `initialized` means probes were attempted, not that both
succeeded. `imu_ready` and `baro_ready` report initialization; sample validity is separate.

The current board has faulty IMU soldering; barometer-only USB acquisition was verified on
19 October 2026. Barometer data never substitutes for an IMU flight frame. MAIN
polls hardware only while no replay/simulation is active. During a run the diagnostic sample
ages; values become null with `baro_sample_valid:false` after 100 ms, or after a read failure.
`status` identifies sensor availability; `preflight` reports the actual blockers. Hardware
frames require fresh IMU data, while output pulse deadlines are serviced even without a new frame.

## Framing and retry rules

Input is accumulated across reads and dispatched only on LF; CR is permitted immediately before
LF. Whitespace-only lines are ignored. Maximum line length is 128 bytes before LF, including
optional ID/CR. Oversized lines are discarded through LF; their suffix is never executed.
Invalid controls/NULs, malformed IDs and extra arguments are rejected. The current subset
accepts ASCII command/name bytes. Full UTF-8 argument decoding is future work.

With the command router installed, IDs span 0..4294967295 and must increase within each
connection. A one-result cache retains the latest numbered request/reply: an exact repeated
request returns that reply without executing again; different content with the same ID returns
ID_CONFLICT. Older IDs return STALE_REQUEST. Payload comparison includes original whitespace/CR.
Once the uint32 range is exhausted, start a new session; no wrap comparison is implied.
Unnumbered commands have no retry guarantee and must not be automatically resent.

USB disconnect/DTR changes discard its partial input/cache and any draft it owns. Applied
settings stay loaded. An accepted save is synchronous and may finish even if the host
disconnects; reconnect and query state before retrying. Owner disconnect stops injected HIL,
cancels pending virtual starts and finalizes its optional log. No cross-reboot exactly-once promise or
authentication is provided. USB and Bluetooth have independent ID sequences; a new connection
resets only its own cache/parser. Lost replies must be resolved by querying current state,
including across a transport change.

Each endpoint drains one 1024-byte pending response using available transmit capacity, up to
64 bytes per loop. Arduino-Pico UART exposes writable capacity as 0/1, so radio TX advances a
byte at a time without waiting. USB input pauses during its pending reply; radio RX continues
to recognize link notifications but additional application requests during a reply are dropped.
Receive work is limited to 64 bytes per endpoint per loop. Send one command and wait for its
**complete** response. Host flooding is not guaranteed lossless. Detailed config inspection is
paged by rule/condition/output to keep each reply bounded. JSON strings escape names; non-finite
values render as null with validity flags rather than invalid JSON numbers. The UART
has a theoretical 23040 bytes/s payload ceiling at 230400 8N1, before radio and
command/response overhead; USB remains preferable for the fastest downloads.

## Control ownership and connection checks

`control status` returns `transport` (`usb`/`bluetooth`) and `owner` (`none`/`usb`/`bluetooth`).
The connection opening a draft or starting a run owns its mutations. The other connection can
inspect configuration/status/events/timing, but editing, feeding/stopping a run, arming/disarming
or storage maintenance returns `CONTROL_BUSY`. Ownership clears once the work is idle. A
disconnected observer does not affect the owner's work.

Owner disconnect discards its draft and stops injected HIL/estimate replay, finalizing any log.
Autonomous simulation and physical arming/flight survive; another connection may explicitly
take control. USB DTR transitions and JDY-34 connection/disconnection notifications invalidate
that endpoint's partial input, retry cache and pending response. The notification filter also
recognizes events appended directly after unfinished/oversized commands. UART overflow discards
through LF and invalidates the radio session. This is application coordination, not authentication
or a guarantee about bytes already transmitted before link loss.

For the future GUI, opening a COM port means connecting; a valid `info` response and recent
`ping` replies establish application connectivity. Use a bounded reply deadline and show stale
connections separately. Windows pairing/port existence and the board's radio-link flag alone
do not establish that the app is communicating. No automatic heartbeat producer is added to
firmware. Name changes remain in the isolated diagnostic/`NameController`; MAIN does not yet
offer a radio rename command.

## PC client and connection lifecycle

[`tools/device_client.py`](../../tools/device_client.py) is the common Python implementation.
Replay, sensor HIL, onboard simulation and download entry points use `DeviceConnection`;
raw protocol tests borrow its synchronous `Client`. No runner imports transport from the
estimate-replay script anymore. Real serial ports open with bounded reads/writes and USB DTR
enabled; there is no input-buffer flush that could hide an unexpected packet.

`DeviceConnection` owns one worker thread and a FIFO of at most 32 queued requests. It validates
`info` (Perpetual/protocol 1), then `control status` to identify USB or Bluetooth. Only after
both replies does it become connected. Opening the OS port alone is insufficient. Each command
must contain printable ASCII/tabs, no supplied ID or line ending. The API reserves the full
12-byte uint32-ID prefix within the board's 128-byte line limit.

| State | Meaning |
|---|---|
| `disconnected` | No active session |
| `connecting` | Port opening and handshake in progress |
| `connected` | Handshake completed, with recent valid replies |
| `waiting` | A user command is in progress with its own deadline |
| `disconnecting` | Worker cancellation/port cleanup in progress |
| `unresponsive` | An established session failed or has no sufficiently recent reply |
| `error` | Port opening or handshake failed |

`snapshot()` returns state, port/transport, pending command, last-reply age, device info and the
last error. Any valid command reply proves liveness. With no work queued, two seconds without
a reply schedules `ping`; its default deadline is three seconds and it must return PONG.
Heartbeats never interleave with a pending response or long flash operation. Such work remains
`waiting` until its explicit command deadline (existing storage commands can allow 600 s).
An in-progress operation is not proof of current radio liveness; reply age remains visible.

`CommandError` carries firmware `code` and the complete response, including blocker/owner
details. A normal rejection such as BUSY/CONTROL_BUSY leaves the connection usable. A timeout,
IO error, malformed/oversized/extra packet, wrong envelope/command/ID, ID_CONFLICT or STALE_REQUEST
invalidates the session. The worker closes the port and rejects unsent queued work. It never
retries the ambiguous command or reconnects automatically. After explicit reconnect, query
device/run/configuration state before deciding another action.

Scripts can use `with DeviceConnection(port) as device:` and blocking `device.request(...)`.
The Qt prototype uses the asynchronous `connect()`/request API, `snapshot()`, and
`disconnect(wait=False)` without running serial IO on its UI thread. Future callbacks can execute
on the worker; marshal their results to the UI. Calling blocking `request()` on the worker
is rejected. Queued Futures can be cancelled before transmission; after transmission the
operation may already have taken effect. A Future wait timeout does not cancel the command.
The reply timeout starts when the request is executed, excluding FIFO wait time.

Disconnect completion is another Future. Blocking script cleanup waits up to three seconds;
an OS Bluetooth open that itself stalls may finish later, after which the worker closes the
port without sending commands. The UI waits asynchronously before reconnecting; see [Desktop](DESKTOP.md).

```powershell
python tools/device_client.py --port COM11 --watch     # Ctrl+C to close
python tools/device_client.py --port COM9 --watch 10  # Bounded observation
```

These commands handshake and request status; `--command "config show"` selects another
single request. No firmware changes, settings writes or radio reconfiguration are required.

## Current scheduling limit

Rule Apply performs bounded flash operations synchronously on core 0 while disarmed/idle,
and reports success only after verification; USB is temporarily unserviced during it. The
router excludes these mutations during runs. Idle preparation/scans still need measured
latency qualification; this is not a hard real-time scheduling guarantee. HIL frame processing now enqueues sample/start data for the core 1 worker.
Preparation still waits for storage while disarmed. Close/abort submit a request and return
immediately; poll `recording_busy` until release before using flash. Bulk flash scans and erases
can take seconds; development clients wait without automatic retry. Detailed flash timing is
BUSY until the worker releases ownership; HIL timing and atomic progress remain readable.
Preflight/arming commands report shared lifecycle state. Onboard simulation can explicitly
select physical outputs; normal flight uses the same executor after real readiness checks. Manual
channel tests remain unavailable.

### Compact flight download

Firmware codec: `communication/PackedFlightCodec.h`; the flights handler owns
selection, guards and transfer lifetime. Shared recording status fragments live in
`communication/RecordingReply.h`; routing policy lives in `communication/CommandPolicy.h`.

The app downloads `flights packed <sequence> <block> <offset>`. Each JSON reply
contains sequence, block, offset, first sample index, block_count, count, crc32,
and data (strict Base64). At most 9 samples and 448 encoded bytes fit each reply;
no larger serial buffers or unsolicited streaming are required. The old `read`
command remains available for diagnostics. App and firmware must both be updated.
The current format is sample v3/header v7; older flight memory needs an explicit
`flights format`. See [Airbrakes](AIRBRAKES.md) for the coordinated format change.

Payload encoding: start with nineteen zero 32-bit words for each reply. For each
sample, read a little-endian uint32 mask; for each set bit in ascending order,
read a little-endian uint32 and XOR it with the previous word at that index.
Unchanged words retain their previous value. The reconstructed 76 bytes use the
sample-v3 layout (bits 0..18; higher mask bits are rejected). CRC-32/ISO-HDLC covers the encoded bytes before Base64; the
client validates length, mask boundaries, checksum, sample ordering and identity.
This preserves all stored float32 bits and missing values without decimal rounding.
Cancellation still consumes the current reply before stopping and removes partial
export files. Bluetooth's board-to-module UART is now 230400 baud; compression reduces
traffic in addition to this baud increase. The PC SPP port's baud is independent.

### JDY-34 UART configuration and recovery

MAIN expects a module configured to 230400 baud (AT+BAUDA on verified JDY-34
SPP-BLE V1.52, [manual](../datasheets/JDY-34%20High%20Speed%20Dual-mode%20Bluetooth%20User%20Manual-V1.3.pdf)).
This is a persistent module setting, not a Windows COM setting. A new board needs no manual step:
MAIN sets up a factory module at 9600 by itself (see `bluetooth status` above). Modules left at
other rates (for example 600000) are not touched and still need the diagnostic below.
USB remains available regardless of UART baud.

BLUETOOTH_NAME_TEST starts its local UART at 9600. Via USB, use `baud 230400` to
reach a migrated module, `gate low`, and read `at AT+VERSION` / `at AT+BAUD`.
`module baud 600000`, `module baud 230400` or `module baud 9600` verifies the known module version and
that all eight radio channels are disconnected, writes the explicit setting,
switches the local UART and reads back the baud. Do not repeat a setter merely
because an acknowledgment was lost: query at the new and old rates first.
Reflash MAIN after diagnostics. A factory reset of flight configuration does not
change the separate Bluetooth module setting.


The JDY-34 V1.52 manual does not specify the internal UART-to-radio buffer size
or expose an occupancy query. Its statement that UART writes have no byte-count
limit is not a buffer capacity guarantee. Our 256-byte UART FIFO setting belongs
to the MCU. `uart_rx_overflows` counts MCU receive overflow events only; it cannot
observe overflow inside the radio module. Bounded replies and one outstanding
request limit queued data, while checksums detect damaged received payloads;
neither establishes long-range losslessness without testing.

A bench trial at 600000 baud with MTU=0 completed two 5001-sample SPP downloads
in 51.594 and 51.938 seconds; both CSV hashes matched an independent USB download
and MCU RX overflows stayed zero. The earlier 230400 baseline took 53.42 seconds.
This small difference does not establish a useful baud-related speedup, and does
not qualify range/interference behavior. MAIN and the module were restored to
230400; protocol/round-trip overhead should be measured before raising baud again.

Airbrake configuration commands share ordinary draft ownership and mutation guards;
see [Airbrakes](AIRBRAKES.md#configuration-commands) for grammar and continuous semantics.

`config airbrakes [on/off]` reads or persists the independent enable setting.
`airbrake status` is a passive read that does not wake hardware or acquire control;
its bounded reply includes enabled, active, physical, channel_mask, flags,
failed_conditions and four nullable opening_pct values. No sensor acquisition is
performed by that read. Other airbrake commands retain normal draft ownership.
