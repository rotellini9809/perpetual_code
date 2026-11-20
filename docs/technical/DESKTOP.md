# Desktop app internals

`perpetual_app/` is the Qt (PySide6) desktop app for Windows. This page is for people changing
the app. How to use it is in the [app guide](../APP_GUIDE.md); the commands it sends are defined in
[USB/Bluetooth](USB.md).

## Principles

- **The board decides.** Arming readiness, configuration validity, calibration acceptance and
  the arming state always come from firmware replies. Python never duplicates those checks and
  never shows an optimistic state before the board acknowledges it.
- **Nothing is saved implicitly.** Rule edits stay in a local draft until Validate and Apply;
  each device setting has its own Save. There is no auto-save and no save-all.
- **No fake data.** Missing measurements stay missing in live views and plots. The only
  exception is the illustrative battery trace of simulated recordings, which is display-only.
- **Behavior never depends on displayed text.** Use codes, job kinds and flags; texts are
  translated.

## Running, testing and packaging

From the repository root:

```powershell
.\.venv\Scripts\python.exe perpetual_app/main.py                 # normal start
.\.venv\Scripts\python.exe perpetual_app/main.py --port COMx     # connect to one port only
.\.venv\Scripts\python.exe perpetual_app/main.py --arm-preview   # ARM/DISARM hold animation, no board
```

`--smoke-test <folder>` (hidden) runs offscreen, saves a screenshot of each page and
`result.json`, then exits; add `--port` to capture it connected.

Tests live in `test/integration/desktop_*.py`. Each area runs on its own:

```powershell
python tools/run_tests.py --suite desktop               # whole Qt suite
python tools/run_tests.py --area translations           # one area (python test/integration/desktop_test.py --list)
python tools/run_tests.py --changed                     # only what uncommitted files affect
```

New tests go in the matching area module and are registered in `desktop_test.py`; build state
with `session()` from `desktop_support.py` instead of relying on earlier areas.
`tools/profile_desktop.py --flight <CSV>` measures page and plot timing offline.

`perpetual_app/build.ps1 [-Zip] [-SkipInstall]` builds the portable app with PyInstaller into
`.pio/desktop_dist/Perpetual/` (and `Perpetual-windows-x64.zip` with `-Zip`), copying
[PORTABLE.txt](../../perpetual_app/PORTABLE.txt) as its `README.txt`. `package.py` adds the app
source, dependency notices and build versions to that folder. Dependencies are pinned in
[requirements.txt](../../perpetual_app/requirements.txt).

## Structure

| File | Responsibility |
|---|---|
| `main.py` | Entry point, palette, language load, smoke test and relaunch after a language change |
| `window.py` | Window shell, sidebar, connection bar, Test and simulation and Console pages |
| `controller.py` | App state, operations, background polling and applying firmware replies |
| `connection.py`, `ports.py` | Port discovery and ranking, connect/close lifecycle |
| `dashboard_page.py`, `arm_button.py` | Dashboard and the hold-to-ARM button |
| `configuration_page.py`, `dialogs.py`, `configuration.py` | Rule and channel editing, editors, pure configuration model |
| `airbrakes.py`, `airbrake_dialog.py` | Airbrake presentation and editor |
| `imu_calibration_page.py` | Six-face IMU calibration and verification |
| `settings_page.py`, `orientation.py` | Device settings, board orientation preview and Euler display convention |
| `sensor_details.py` | Live sensor window with a 60-second history |
| `recorded_flights_page.py`, `flight_data.py`, `flight_plot.py` | Flight list, download, CSV loading and plots |
| `i18n.py`, `translations_it.py` | English UI text and the Italian catalog |
| `widgets.py`, `assets/style.qss` | Shared widgets and the stylesheet |

The serial protocol, request ordering and heartbeat belong to the shared
[`tools/device_client.py`](../../tools/device_client.py); flight download uses
[`tools/replay/download_flight.py`](../../tools/replay/download_flight.py), which decodes raw log
blocks with [`tools/replay/flight_log.py`](../../tools/replay/flight_log.py) (packaged with the
app). Widgets never own serial IO.

## Controller and jobs

Every board operation is a job run on a single worker thread, so requests never interleave.
A job has a kind (`operation`, `poll`, `servo_preview`, `download`, `simulation_start`) that
drives behavior; its name is only a label. Foreground operations show progress and block other
operations; background polls never overwrite the feedback of the user's last operation.
Results are applied in `_complete` and the `_apply_*` helpers; `_command_effects` and `_read`
decide what is read back after a command.

Background polling runs only while no job is active, one request at a time:

| What | When | Interval |
|---|---|---|
| `status`, `control status` | Always | 4 s |
| `sim status` | Simulation running or its page open | 1.2 s |
| `imu status` | Configuration open or a calibration running | 0.75 s |
| `sensors` | Live sensor window open | 0.25 s USB, 1 s Bluetooth |

`airbrake status` is not polled: the airbrakes do not move on the pad, so the Dashboard shows only
configuration hints, and the recorded flight shows what they did. It is still read after airbrake
setting changes. Every board reply is prepared on the flight loop's core, so polling stays sparse
while armed (see [Validation](../testing/VALIDATION.md)).

`preflight` is read again after wake, after a run ends and after a calibration ends, so cached
readiness never outlives a state change.

## Connection

Connect searches native RP2350 USB CDC ports first, then paired outgoing Bluetooth SPP ports
whose name looks like a Perpetual or JDY-34 module. The last verified identity of each transport
is preferred; if several equal candidates remain, the user chooses. The app never pairs devices.

At startup the app makes one quiet automatic attempt: its failures are not shown. Manual
attempts, errors after a successful handshake and link loss are shown. There is no periodic
retry, no reconnect after loss and no replay of a command whose outcome is unknown. A failed
session cleans itself up, so Connect works again without a Disconnect. The only automatic
reconnection is after a Bluetooth rename, when the module restarts on purpose.

## Behavior by page

**Dashboard.** ARM always requests a normal physical flight and is enabled only when `preflight`
reports `ready:true`; Recheck refreshes it. ARM and DISARM need a continuous 1.5-second hold;
releasing, leaving the button, losing focus or navigating away cancels it, and one gesture sends
one request. Pyro continuity (`pyro continuity`) is informational and never approves ARM. Servo
preparation lists only applied assignments; an unconfigured servo turns its action into a link to
the servo editor. POWER SAVE ACTIVE appears during firmware sleep; there are no sleep/wake controls.

**Configuration.** Editors change a local `Plan`. Validate sends the draft to the board's RAM
draft and checks it; Apply restages, validates and saves it, then reads it back. A draft owned by
another connection is read-only and never discarded silently. Rule IDs are internal and stable;
users see and reference rules by name. The servo editor previews positions with
`servo preview` at most 10 Hz, one request in flight, keeping only the latest value; closing it
releases the servo. Confirm saves only that servo: the applied plan with this servo changed goes
through the usual staging and `config apply`, and the local draft keeps the other unsaved edits. It
refuses when a draft validated on the board is pending, or when the board changed since it was read.

**Test and simulation.** One run at a time. Physical outputs is unticked by default and fixed
while running; the actual mode always comes from the firmware reply. A virtual run first releases
any held servo preparation. If a physical run is rejected by preflight, an "Arming blocked" link
opens the Dashboard and runs Recheck.

**Recorded flights.** The board sends raw [flight log v4](STORAGE_FORMAT.md#samples-and-blocks-v4)
blocks (`flights block`, four 448-byte chunks per block); only the PC decodes them. Downloads write
a CSV and a JSON file and replace existing files only when the transfer completes. The CSV has one
row per IMU sample with `t_us` time, `imu_seq`, `pad_history` and `gap_before`, the raw sensor
codes (`raw_*`, empty for simulated BODY input) and calibrated BODY `accel*`/`gyro*` rebuilt with
the frozen calibration in the JSON, using the firmware's float32 arithmetic. Pressure and
temperature are filled only on rows with a new barometer reading; state, battery and airbrake
columns carry their last value forward; missing values are empty cells. `flight_data.py` requires
`t_us`, `z_est` and `vz_est`, so v3 CSVs (`t_ms`) are not opened. Plots share the time axis,
show phases as bands and output starts from the recorded metadata as markers, and are decimated to about 1200 points for display without
changing the files. Gyro rates are converted from rad/s to °/s only for display. The orientation
plot uses `orientation.py`: stored Z-up ZYX Euler angles are re-expressed in an X-up reference,
so the rocket standing nose-up reads zero.

**Settings.** Orientation, state machine and airbrake enable are saved one by one and verified by
the board. Factory reset and Format memory ask for confirmation and are blocked during runs. A
Bluetooth rename restarts the module; the app reconnects once to the same port and checks the name.

**Sensor window.** Opens on demand, keeps 60 seconds of board timestamps, pauses on disconnect
and clears on reconnect or reboot.

## Language

English is the text written in the code; Italian comes from `translations_it.py`. Wrap every
user-facing string in `t('English text')` at display time, never at import, and pass values as
placeholders. Error messages, exports and firmware reasons stay in English and are not wrapped.
The `translations` test area fails on missing, unused or mismatched entries. The language is saved
in QSettings and applied after a restart.
