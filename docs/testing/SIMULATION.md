# Simulation and replay

Modes default to **virtual outputs**; onboard simulation optionally drives physical outputs.
Host sensor HIL and onboard Hoppino simulation share
`flight::FlightRuntime` for source polling, SensorPipeline/navigation/detector/rules and optional
recording. They also use the same ArmingController as physical operation: simulation starts
immediately without real sensor checks or gyro calibration. Physical preflight/calibration are
implemented; normal flight uses the same output executor after explicit servo preparation and
real sensor checks. The current board's IMU fault remains a blocker.
Estimate replay intentionally bypasses navigation and remains an independent rule reference.

## Choose the input

| Tool | Runs where | Injected data | What it verifies |
|---|---|---|---|
| `run_hoppino_replay.ps1` | PC | Recorded estimates/phase with its own fixture plan | Independent rule/event reference |
| `replay_navigation.py` | PC | Recorded BODY IMU/pressure | Navigation reference; stored estimates are comparison-only |
| `replay_hoppino_usb.py` | Board via USB | Recorded estimates/phase, board's applied plan | Rule/protocol path without sensors/navigation |
| `replay_sensors_usb.py` | Board via USB | BODY IMU/pressure, board's applied plan | Shared navigation/rules; optional recording |
| `run_simulation_usb.py` | Board's clock | Embedded Hoppino BODY IMU/pressure | Autonomous scheduling/shared pipeline; optional recording |

Default Hoppino source: [`datasets/hoppino_VOLO_UFFICIALE.csv`](../../datasets/hoppino_VOLO_UFFICIALE.csv),
is the 2..35-second excerpt of Recovery System's official flight log. All replay runners and
recorded integration tests use these 6600 samples. The first retained row, originally 2003 ms,
is timestamp zero; the last, originally 34998 ms, is now 32995 ms. Sample intervals, missing
values, measurement/estimate values and columns are preserved: BODY acceleration is m/s²,
gyro rad/s and pressure hPa. This excerpt retains about 8.2 seconds of PAD and 2.8 seconds after detected LANDED,
including the complete ascent and descent. The untouched original is stored alongside it as
`datasets/hoppino_VOLO_UFFICIALE_original.csv`; `python tools/replay/trim_hoppino.py`
rebuilds the crop without changing any measurement fields. The recorded replay regression
compares rotations against the full-log replay (less than 0.1 degree difference after launch).
Euler roll/yaw remain ill-conditioned near pitch -90 degrees; these are not absolute heading measurements.
Recorded navigation/phase columns are references; sensor replay recomputes them onboard or on PC.
USB tools need pyserial (`python -m pip install pyserial` if absent). Upload
MAIN, use a valid applied plan with no active draft, and close Serial Monitor before opening
COM9. Change `--port` for another device. Outputs are uniquely created under `.pio`; `--output`
must not overwrite a file. Fixtures are test settings, not flight recommendations.

The same commands/runners also work over the outgoing Bluetooth SPP port (`--port COM11` on
this PC). The JDY-34 UART is currently 9600 baud: full recorded HIL/download is much slower
there, so USB is preferred for bulk traffic. Onboard simulation sends only control/status,
making it a useful Bluetooth test. See [shared transport ownership](../technical/USB.md#control-ownership-and-connection-checks).

## PC references

```powershell
powershell -ExecutionPolicy Bypass -File tools/replay/run_hoppino_replay.ps1
python tools/replay/replay_navigation.py
```

The PowerShell runner compiles the real validator/rule engine, using apogee -> Pyro 1
immediately and Servo 1 after 250 ms, plus descent below 30 m confirmed 100 ms -> Pyro 2.
Optional `-InputCsv`/`-OutputCsv` select files. It replays recorded estimates, not sensor
physics, and does not change trajectory after output commands. The navigation runner computes
from IMU/pressure/timestamps; recorded estimates,
Euler angles and state cannot influence calculations. See [Validation](VALIDATION.md#navigation-reference)
for the distinct estimate/sensor reference times.

## USB sensor HIL

```powershell
python tools/replay/replay_sensors_usb.py --port COM9
```

Options include `--input`, `--pressure-unit hpa|pa`, `--output`, `--record`,
`--timing` and `--realtime`. [Recording](RECORDING.md) explains storage options.
Hoppino IMU was logged after arming-bias and mounting transforms, so `logged_body` avoids
applying them twice. Pressure hPa converts once to Pa. PC initialization uses the initial
one-second mean pressure altitude and median period because historical arming metadata is
absent; this is not a new physical calibration.

```text
hil begin <reference_pressure_altitude_m> <nominal_imu_hz> logged_body [record <sample_bytes>]
hil sample <t_ms> <ax> <ay> <az> <gx> <gy> <gz> <pressure_pa> [temperature_c]
hil status
hil timing
hil stop [completed]
```

Begin freezes the plan/source. Timestamps are normalized strictly increasing uint32 milliseconds
after the first sample; USB delays do not advance logical flight time. Sample units are m/s^2,
rad/s and Pa; optional temperature C only feeds logging. The mailbox cannot grow or overwrite
unread data. Bad syntax, overflow and repeated/backward times are rejected before processing.
Explicit nan/inf represent failed measurements: invalid IMU freezes navigation/resets required
confirmations while already-triggered actions remain pending.

Replies contain computed phase/z/vz/bias/vertical acceleration, validity and fresh barometer
flag, trigger IDs and starts `[rule_id,kind,channel,actual_start_ms]`. All eight starts fit a
bounded reply; nonfinite results are JSON null. The client uses float32 round-trip text and
validates the 128-byte wire limit before connecting, without silently reducing precision.
Current USB accepts only logged BODY; the typed pipeline can also process validated raw
SensorAxes from a future provider.

## Autonomous onboard profile

```powershell
python tools/replay/run_simulation_usb.py --port COM9
python tools/replay/run_simulation_usb.py --port COM9 --record
```

```text
sim begin [physical] [record]
sim status
sim timing
sim events
sim stop
```

The app's **Uscite fisiche** checkbox defaults off and is fixed for the run. `sim begin physical`
uses the same navigation/rules/runtime with the board output executor: firmware checks configured
servo endpoints, assigned pyro continuity and output availability before moving anything.
It then commands assigned servos to their initial positions before starting the profile clock;
rules command final positions and the PWM remains attached until simulation shutdown.
Real IMU/barometer readiness is deliberately skipped.
Pyros share a 10 ms driver-enable settle, then each requested channel gets one fixed 1000 ms
pulse; concurrent channels have independent deadlines. No retry sequence is implemented.
`output_mode` is `PHYSICAL` or `VIRTUAL` in status/events and recorded headers. Physical start
times represent servo command/pulse-high timing, not proof of motion or ignition. Stop/completion
immediately disables pyros, releases assigned servo PWM and cancels pending starts before flash
finalization. Client disconnect leaves an accepted autonomous run running. The Python replay
script above intentionally remains virtual-only. Normal physical flight always selects real outputs,
regardless of this simulation setting, and requires explicit servo preparation. Virtual onboard simulation releases held servo preparation before starting; physical
simulation reuses valid preparation. Idle preparation does not block HIL/estimate replay.

The board reads the official Hoppino sensor log from a const table in program flash, one
sample at a time without a RAM dataset copy or external recording-flash allocation. The
previous synthetic profile is replaced, with no profile selector. There are 6600 readings over
32995 ms; timestamps are normalized to the first row. Real recorded gyro/acceleration,
temperature, pressure and missing values are retained. Pressure is converted once to Pa;
stored estimates, Euler angles, magnetometer and phase never enter the table. Reference altitude
and nominal 200 Hz match PC sensor HIL's initial one-second pressure mean and median period.
Navigation, detection, rules, outputs and recording run through the shared flight runtime.

After editing the source CSV, regenerate and verify the checked-in table before building MAIN:

```powershell
python tools/replay/generate_hoppino_data.py
python tools/replay/generate_hoppino_data.py --check
```

`src/simulation/data/HoppinoData.inc` includes the source SHA256. The recorded replay suite
checks freshness and compares the embedded source/pipeline with CSV navigation on every frame.

Core 0 polls absolute micros deadlines before USB. It emits one latest due frame, counts missed
slots and never catches up in a burst or shifts later deadlines. Unsigned subtraction handles
rollover for this bounded run. This is cooperative scheduling, not an ISR/hard-real-time
guarantee. A full virtual replay has 6600 frames; check `samples + missed_ticks == 6600`. Phase/start
arrays are bounded to five/eight.

The 1 MiB record budget is prepared before the profile clock begins. Physical outputs retain the
shared landing-window shutdown policy, which may complete before the last dataset timestamp.
At completion or manual
stop, cancel future starts and request asynchronous storage close. `active:false` ends sensor
generation; only `recording_busy:false` confirms flash release. The client waits for both and
checks errors/counts/deadlines before success. Completed JSON reports are exclusive and retained
when validation fails.

## USB estimate replay

```powershell
python tools/replay/replay_hoppino_usb.py --port COM9
```

```text
replay begin
replay sample <t_ms> <legacy_state> <altitude_m> <vertical_velocity_mps>
replay status
replay stop
```

This does not run Kalman/detection or write flight logs. It publishes supplied estimates to
rules using the board's applied plan, not the PC runner's fixture. Legacy states map 0->PAD,
1->BOOST, 2->COAST, 3/4->DESCENT, 5->LANDED; never cast the old enum to the current numeric
phase. Each accepted row publishes fresh estimates, nan/inf marks invalid inputs, and gaps over
100 ms reset relevant untriggered confirmations. The one-rule USB example starts Pyro 1 near
12551 ms; sensor HIL computes DESCENT near 12553 ms. Different plans naturally differ.

## Shared lifecycle and timing limits

One run owns the plan/navigation; competing runs and configuration mutations are BUSY.
Inspection remains available. No new rule triggers after LANDED; previously scheduled starts may
run during the 15-second development window. EOF/stop cancels the rest without inventing
samples. Starts occur at the first evaluation at/after their deadline, not an interpolated or
rule-trigger timestamp.

Host-fed runs end when their owning USB/Bluetooth connection disconnects. Autonomous simulation continues disconnected; reconnect to
inspect/stop. The client sends one command per reply and never automatically retries timed-out
samples. Failures attempt stop and retain a partial CSV; if stop is not acknowledged, disconnect
the owning HIL connection. A new connection resets its own
ID caching, not a cross-reboot exactly-once guarantee. See [USB](../technical/USB.md).

`compute_us` covers source consumption/preprocessing/navigation/rules, excluding
USB/parsing/formatting/storage. `--realtime` uses PC absolute recorded deadlines, records host
lateness/round trip and enables timing export. Untimed queued recording uses PC flow control;
real-time replay omits throttling so overflow remains visible. Neither USB timing nor a single
profile proves worst-case live deadlines.
Use [Recording](RECORDING.md#timing) for full timing scopes and
[Validation](VALIDATION.md) for actual observed phases and measurements.

## Bistage simulation

Save `config machine bistage <minimum_velocity_mps>` or use Settings, then start the same
`sim begin [physical] [record]` command. The board automatically chooses a 25-second synthetic
BODY sensor source at 200 Hz: PAD to 5 s, first burn 5..6 s, coast 6..7 s, second burn 7..8 s,
then ballistic coast/descent/landing. Source readings, not preassigned phases, feed shared
navigation. `sim status` reports source `bistage_synthetic`, actual duration/sample count,
bistage, abort reason, current median and tilt. Thresholds above the first burnout speed
exercise velocity ABORT. Save `config machine normal` to restore the unchanged Hoppino replay.
Host tests check all seven successful phases, latched velocity/tilt ABORT, frozen settings,
physical pulse completion and cancellation of future starts. These are synthetic/host tests,
not physical flight qualification.
