# USB and Bluetooth protocol

The board accepts the same text commands over USB and over the JDY-34 Bluetooth module, and
both links can be used at the same time. This page defines the transport, the request and reply
rules, connection ownership and the commands that are not owned by another page. The PC side is
implemented once, in [`tools/device_client.py`](../../tools/device_client.py).

## Transports

| Link | Details |
|---|---|
| USB | Native RP2350 USB CDC; the PC opens it at 115200 baud |
| Bluetooth | JDY-34 transparent link on hardware UART1 (MCU TX GP4, RX GP5) at **230400 8N1**. On Windows, use the outgoing SPP COM port of the paired module; its PC-side baud setting does not matter |

Each link has its own command service: framing, retry cache and pending reply. The main loop
services both on every pass and never waits for either. The board sends nothing unsolicited.

## Requests

- A request is one line ending in LF; CR right before LF is accepted. Maximum length: **128 bytes**,
  including an optional ID and CR. Longer lines are discarded up to the next LF and never executed.
- Arguments are ASCII. Control characters, malformed IDs and extra arguments are rejected.
- Prefix a request with `@<id>` (`0` to `4294967295`) to get a single compact JSON reply, for
  example `@42 status`. Without an ID, `ping`, `info`, `status`, `sensors` and `help` reply in
  readable text; other commands reply in JSON anyway.

**IDs and retries.** IDs must increase within a connection. The board keeps the last numbered
request and its reply: sending exactly the same request again returns the cached reply without
executing it twice; different content with the same ID returns `ID_CONFLICT`, and an older ID
returns `STALE_REQUEST`. USB and Bluetooth count IDs separately, and a new connection starts its
own sequence. Unnumbered commands have no retry protection and must never be resent automatically.
After a lost reply, query the current state before deciding what to do.

## Replies and flow control

Each link holds at most one pending reply of up to **1024 bytes** and transmits it a bounded
amount per loop. Send one command and wait for its complete reply before the next: while a reply
is pending, further requests on that link are not processed. Large inspections are paged (for
example rule by rule) so every reply fits. Names are escaped in JSON, and non-finite numbers are
sent as `null` with a validity flag.

Bluetooth throughput is limited by the 230400-baud UART (about 23 kB/s before any overhead), so
USB is preferable for large downloads.

## Ownership

`control status` returns the requesting `transport` (`usb` or `bluetooth`) and the current `owner`
(`none`, `usb` or `bluetooth`). The link that opens a draft or starts a run owns it until the work
is idle. The other link can inspect configuration, status and events, but edits, run control,
arming and storage maintenance return `CONTROL_BUSY`.

When the owner disconnects, its draft is discarded and PC-fed runs (sensor HIL, estimate replay)
stop and finalize their log. Autonomous simulation and physical flight keep running, and another
link can take control. A USB DTR change or a JDY-34 connect/disconnect notification clears that
link's partial input, retry cache and pending reply. This is coordination between links, not
authentication.

## Connecting from a PC

`DeviceConnection` in `device_client.py` owns one worker thread, the serial port and a FIFO of up
to 32 requests. A session counts as connected only after `info` identifies a Perpetual board and
`control status` identifies the link; opening the port or pairing in Windows is not enough.

| State | Meaning |
|---|---|
| `disconnected` | No session |
| `connecting` | Opening the port and running the handshake |
| `connected` | Handshake done, recent valid replies |
| `waiting` | A command is in progress with its own deadline |
| `disconnecting` | Closing the port |
| `unresponsive` | An established session stopped replying |
| `error` | Opening or handshake failed |

Any valid reply proves the link is alive. With nothing queued, two seconds of silence send a
`ping` (3 s deadline). Long operations such as flash commands use their own deadlines (up to
600 s) and are never interleaved with heartbeats.

A firmware rejection (`CommandError`, for example `BUSY` or `CONTROL_BUSY`) leaves the session
usable. A timeout, IO error, malformed or unexpected reply, `ID_CONFLICT` or `STALE_REQUEST`
closes it: queued requests are rejected, and the ambiguous command is never retried and the link
never reconnected automatically.

Scripts use `with DeviceConnection(port) as device:` and blocking `device.request(...)`. The
desktop app uses the asynchronous API so serial IO never runs on the UI thread. From a terminal:

```powershell
python tools/device_client.py --port COM9 --watch          # watch the link until Ctrl+C
python tools/device_client.py --port COM9 --command "config show"
```

## Command reference

`help` lists every command group. Each group is defined on one page:

| Commands | Defined in |
|---|---|
| `ping`, `info`, `status`, `sensors`, `capabilities`, `control status` | [This page](#board-state) |
| `sleep`, `wake` | [This page](#power-saving) |
| `preflight`, `arm`, `disarm`/`ds`, `servo arm`, `servo preview`, `servo release` | [This page](#arming-and-outputs) |
| `bluetooth status`, `bluetooth name` | [This page](#bluetooth-module) |
| `imu status/begin/verify/capture/cancel` | [Navigation](NAVIGATION.md#calibration-and-mounting) |
| `config …`, `rule`, `condition`, `output`, `pyro`, `servo` (settings) | [Configuration](CONFIGURATION.md) |
| `config machine` | [State machines](STATE_MACHINES.md#machine-selection) |
| `config airbrakes`, `airbrake status` and airbrake editing | [Airbrakes](AIRBRAKES.md#configuration-commands) |
| `replay`, `hil`, `sim` | [Simulation and replay](../testing/SIMULATION.md) |
| `flights …` | [Recording and download](../testing/RECORDING.md) |

There are no commands to fire a single channel directly.

## Board state

| Command | Reply |
|---|---|
| `ping` | `PONG` |
| `info` | Board and protocol identification |
| `status` | Observations: configuration validity, `arming_state`, `flight_mode`, `output_mode`, `outputs_active`, phase, sensor and storage availability, gyro collection progress, `battery_v`, `power_mode`, `power_error` |
| `sensors` | IMU and barometer state, chip IDs, last errors, pressure, temperature, sensor-axis `accel_mps2` and `gyro_radps`, sample validity and age, `uptime_ms` |
| `capabilities` | Fixed limits and features: maximum rules, conditions and outputs, servo PWM range, pyro pulse and enable settling times, supported physical actions |

`status` only reports what the board observes; whether it may arm comes only from `preflight`.
`battery_v` is telemetry and never blocks arming. Sensor values are `null` when unavailable,
older than 100 ms or not finite. While a run is active, idle sensor polling pauses and the
`sensors` sample ages.

## Power saving

`sleep` puts the sensors and flash to sleep, stops battery sampling, releases servo PWM and
disables the pyros; USB, Bluetooth and the MCU stay on. The board does the same by itself after
**five minutes** without operational activity, even with links connected. `wake` restarts the
flash and probes the sensors again; configuration and flights are kept. Both return `power_mode`
and can be repeated safely.

- **Operational** commands (configuration inspection and edits, `flights`, `sensors`, `preflight`,
  arming and run control) count as activity and wake the board first, then run normally. If the
  wake fails, the command is not run and the hardware error is returned.
- **Passive** commands (`ping`, `info`, `status`, `help`, `capabilities`, `control status`,
  `bluetooth status`, `airbrake status` and run status/events/timing) never wake the board or
  reset the timer, so heartbeats and background polling do not prevent sleep.

`sleep` returns `BUSY` during runs, arming, drafts, servo previews, recording or within five
seconds of a flight download reply. Wake never restores servo preparation. Hardware failures are
reported as `SENSOR_SLEEP_FAILED`, `FLASH_SLEEP_FAILED` or `FLASH_WAKE_FAILED`. The LED and
hardware side are described in [Hardware](../HARDWARE.md#status-led-and-power-saving).

## Arming and outputs

The transitions are defined in [State machines](STATE_MACHINES.md#arming-machine); this section
covers the commands.

**`preflight`** (physical, the default) or **`preflight simulation`** returns `ready` and every
blocking reason, plus `failed_pyro_mask`, `prepared_servo_mask`, `unprepared_servo_mask` and
`unconfigured_servo_mask` (bit 0 is channel 1), and the IMU calibration state. It reads the
continuity of the pyros used by the applied rules, one at a time, and never fires, prepares or
records anything. Simulation preflight checks only configuration and ownership.

Physical blockers include `IMU_UNAVAILABLE`, `BAROMETER_UNAVAILABLE`, `STORAGE_NOT_READY`,
`CONTINUITY_FAULT`, `SERVOS_NOT_PREPARED`, `SERVOS_NOT_CONFIGURED`, `ORIENTATION_NOT_READY` and
an IMU calibration that is missing or needs verification. There is no way to override them.

**`arm`** starts a physical flight (480 Hz IMU, 1 MiB of recording space). It fails with the same
blocker list if `preflight` is not ready. A rejected arm reserves nothing and writes nothing.

**`disarm`** (or `ds`) stops the active run: physical flight, on-board simulation or sensor HIL.
It cancels an arming in progress and is accepted at PAD, LANDED and ABORT; in powered or coasting
flight it returns `DISARM_PROHIBITED_IN_FLIGHT`. Disarming at PAD discards that flight's log;
logs that reached BOOST are always kept. Recording closes in the background: check
`recording_busy` before starting another operation. Fifteen seconds after landing the run stops
new activations, lets active pulses finish, releases PWM and closes the recording.

**Servo commands** (all require a disarmed, idle board and no draft):

| Command | Effect |
|---|---|
| `servo arm [channel]` | Move the assigned servos (or one) to their initial position and hold the PWM. Required before physical ARM |
| `servo preview <channel> <pwm>` | Hold a PWM value for the servo editor, without saving or preparing. Released when its link disconnects |
| `servo release [channel]` | Release held PWM |

Preparation survives a disconnect, a rejected ARM and an idle DISARM; it does not block other idle
operations. A virtual simulation releases it before starting; a physical simulation uses it and
prepares any missing assigned servo itself.

## Bluetooth module

`bluetooth status` returns `ready`, `connected`, `error`, the UART `baud`, the MCU's
`uart_rx_overflows` count, the module `name`, the last `rename` result, `configured` (a factory
module was set up at this boot) and `stage` (the current or failed startup step). It also works
over USB when the radio failed to start.

At boot the firmware queries the module without blocking: version, single-link mode, connection
notifications, name and connection state. A module answering at 230400 is never reconfigured. A
factory module answering at 9600 with no active link is set up once (names `Perpetual` /
`Perpetual_BLE`, `BTMODE1`, `ENLOG1`, `BAUDA`), restarted and checked. Queries that the module
ignores are repeated (the first `AT` for about 4 s, others three times); writes and resets are
sent once. If the module is incompatible, Bluetooth stays unavailable while USB and the flight
loop continue. A module restart notification (`+START=OK`) repeats the startup.

`bluetooth name <suffix|default>` renames the module to `Perpetual-<suffix>` (1 to 4 letters,
digits or `-`) or back to `Perpetual`; the BLE name adds `_BLE`. It is accepted over either link
while disarmed and idle. After replying, the firmware writes and reads back both names and
restarts the module, so a Bluetooth link drops for a few seconds. Errors: `INVALID_NAME`,
`BLUETOOTH_NOT_READY`, `BUSY`; a name that cannot be read back is reported as `NAME_NOT_VERIFIED`
in `rename`.

A module left at another baud rate can be recovered with the `BLUETOOTH_NAME_TEST` firmware; see
the [test guide](../testing/README.md). Module settings are independent of a factory reset of the
flight configuration. The module's internal buffer size is not documented, so
`uart_rx_overflows` only counts overflows inside the MCU. See the
[JDY-34 manual](../datasheets/JDY-34%20High%20Speed%20Dual-mode%20Bluetooth%20User%20Manual-V1.3.pdf).
