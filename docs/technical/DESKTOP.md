# Qt desktop application

`perpetual_app/` is the Windows Qt application. It uses the shared
[PC connection API](USB.md#pc-client-and-connection-lifecycle); widgets never own serial IO.

## Build, run and verify

During development, launch from the repository root:

```powershell
& .pio/perpetual_app_venv/Scripts/python.exe perpetual_app/main.py
python tools/run_tests.py --suite desktop
```

The app environment isolates the pinned dependencies in `perpetual_app/requirements.txt`.
A different Python may install those requirements and launch the same entry point.
`--port COM9` selects one explicit connection; normal startup makes one quiet automatic
USB/Bluetooth attempt. Smoke tests without `--port` stay offline:

```powershell
& .pio/perpetual_app_venv/Scripts/python.exe perpetual_app/main.py --smoke-test .pio/desktop_smoke
```

Portable distribution is deferred during routine development. When explicitly requested:

```powershell
powershell -ExecutionPolicy Bypass -File perpetual_app/build.ps1
& .pio/perpetual_app_venv/Scripts/python.exe perpetual_app/package.py
```

Build prepares the app environment and PyInstaller folder; `-SkipInstall` reuses dependencies.
Packaging produces `.pio/desktop_dist/Perpetual-Windows.zip`, including source and dependency
notices. Extract the whole ZIP and keep `_internal` beside `Perpetual.exe`; Python is bundled.
Neither build nor packaging uploads firmware or changes PlatformIO's Python.

## Six views

| View | User-facing behavior |
|---|---|
| Dashboard | Applied configuration, sensor availability/details, firmware preflight blockers, ARM/DISARM, configured pyro continuity and assigned servo preparation |
| Configuration | Local rule/condition/output editing, channel endpoints and Valida/Applica/Scarta |
| Test and simulation | Embedded Hoppino replay, optional physical outputs and recording, progress and events; duration comes from firmware |
| Recorded flights | Recover/list, CSV plus JSON export, offline inspection and five linked-time plots |
| Impostazioni | Board orientation, saved normal/bistage selection, factory reset and flight-memory format |
| Console | Manual shared-protocol requests and bounded request/reply log |

## Connection and presentation

Connetti searches native RP2350 USB CDC first, then paired outgoing Bluetooth SPP ports.
`ports.py` filters unrelated candidates, all-zero incoming SPP records and uses the last verified
identity within a transport; USB still has priority. Ambiguous equal-priority candidates require
a choice. Pair Bluetooth once in Windows; BLE/GATT is not implemented. Transport/port are
read-only labels; Porta manuale exposes a hidden non-editable selector.

Search/cleanup disables Connetti and shows progress without a cancel-search button. Only the
initial automatic attempt hides connection failures. Manual errors and established link loss
remain visible. Cleanup finishes before retry; no periodic reconnect, transport switching after
loss or replay of uncertain commands occurs. An OS Bluetooth open may delay cleanup.

The connection header stays visible with DISARMED/ARMING/ARMED/CLOSING driven by firmware replies.
The light palette, checkbox indicators and popup colors are explicit, independent of Windows dark
mode. Official branding stays in `assets/`; styles are in `assets/style.qss`.

## Dashboard and servo actions

ARM and DISARM share one red button: solid when enabled, faded when disabled. Firmware physical
preflight controls ARM readiness; simulation results never replace physical readiness.
Ricontrolla refreshes blockers. Missing saved orientation is shown as a blocker.

Applied assignments alone expose servo/pyro rows. Names stay black; preparation/continuity
statuses are green or red. Prepara selezionati is green and enabled only if a selected channel
needs preparation; already prepared channels are skipped. An assigned servo without endpoints
changes the action to the red Vai alla configurazione, leading to its editor/apply workflow.
Rilascia servo is a separate red action, enabled when held PWM is reported; it releases all held
servo PWM independently of selection. Idle DISARM does not release preparation.
Command semantics and run-time exclusions are defined in [USB](USB.md#arming-and-readiness) and
[configuration](CONFIGURATION.md#editing-and-inspection), not inferred by Python.

The pyro check reads assigned channels with `pyro continuity`; its result does not approve ARM.
No direct pyro-firing/test button is provided. Prepared PWM alone does not block idle settings.

Dashboard sensors show availability only. POWER SAVE ACTIVE appears beside the heading during
firmware sleep. The app retains last-awake availability labels while asleep, clears cached
preflight/continuity on power transitions and uses fresh observations after wake. Display caches
do not change actual readiness. Power entry/wake is firmware-owned; no app sleep/wake controls.

## Configuration and servo editor

Dialog acceptance edits a local `Plan`, distinct from applied settings. Valida compiles/checks
local input before replacing this connection's remote RAM draft. Applica restages/validates,
requires applied/persisted/verified replies and rereads the plan. Scarta discards owned remote
and local drafts. Rereading the baseline before staging and checking the reserved clone prevents
silent overwrites. A foreign draft is read-only; it is never silently discarded.

Rule IDs stay internal/stable; names and referenced-rule selectors are user-facing. Duplicate
names are disambiguated. Missing references must be repaired. Rule delay appears below the
condition list and before outputs; numeric editors show units outside input boxes.
Rule timing, allowed conditions and saved formats are canonical in [configuration](CONFIGURATION.md).

Only assigned servo rows show Configura/Modifica. The editor has prominent centered PWM values,
sliders and +/-1/10 controls bounded by capabilities. Each endpoint offers Applica and Live;
only one Live is active, green, with other movement buttons disabled. Preview requests run at
most 10 Hz, one in flight, replacing intermediate pending values. Closing releases the preview;
Conferma changes only the draft and does not certify preparation or persist settings.

Local drafts survive disconnect; closing prompts before losing them. Console edits may require
Ricarica dalla scheda. An unsupported optional orientation command cannot discard the entire
initial configuration sync: only the orientation panel requests a firmware update.

## Settings

Orientation uses `orientation.py` and the user's original board image in
`assets/board-orientation-reference.jpg`: +X means connectors above and USB below. -X rotates
180 degrees, +Y/-Y rotate +/-90 degrees (USB left/right). Z views remain labelled +X reference.
Saving requires firmware verification. Factory reset leaves the selector at an em dash;
physical ARM remains blocked until an explicit axis is saved. Mapping and persistence live in
[navigation](NAVIGATION.md#calibration-and-mounting) and [configuration](CONFIGURATION.md#device-orientation-and-factory-reset).

Macchina a stati shows a black normal/bistage diagram, a red ABORT branch and the checkbox
below. The speed field has an external m/s label and is faded/disabled when unchecked.
Salva macchina a stati writes/verifies the selected firmware mode; the saved mode is reported
separately from local edits. Runs block edits; factory reset clears bistage selection. The
simulation uses Hoppino in normal mode and a sensor-derived two-BOOST synthetic profile in
bistage mode, according to firmware replies. [State machines](state_macines.md) owns the exact
median/tilt and ABORT transitions.

Factory reset and Formatta memoria require explicit confirmation. Their different effects are
[configuration reset](CONFIGURATION.md#device-orientation-and-factory-reset) and
[logical flight-memory format](STORAGE.md#explicit-deletion-and-inspection). Runs block them;
held preparation alone does not. Formatting additionally requires no pending draft.

## Live and recorded plots

Leggi dettagli sensori opens a separate nonmodal `sensor_details.py` window: acceleration XYZ,
angular velocity XYZ, pressure, and temperature smoothed over five seconds. Fresh invalid
readings display unavailable immediately. History uses board timestamps, grows to 60 seconds,
then retains the last minute. Disconnect pauses it; reconnect/reboot clears it; uptime rollover
is handled. Missing readings are never replaced with fake data.

Sensor polling targets 4 Hz USB or 1 Hz Bluetooth only while the detail window is open.
Status/control poll about every four seconds, active/visible simulation about every 1.2 seconds;
foreground operations pause polling. Rates stretch with reply time, particularly at 9600 baud.

Recorded flights show altitude; velocity/vertical acceleration with separate units and aligned
zero ticks; gyro XYZ converted from CSV rad/s to degrees/s for display; roll/pitch/yaw in degrees;
and battery voltage. Missing values break curves. Decimation keeps about 1200 displayed
points without changing original files; phase transitions are extracted before decimation.
Actuator markers use saved actual-start metadata. Independent checkboxes control markers/state
bands only. Hover highlights the nearest displayed sample. Orientation wraps break lines at
jumps of at least 180 degrees; isolated points remain inspectable.

Battery voltage is in V from `battery_v`. Files without that field have no measured battery curve. The battery plot is labelled `Tensione batteria` in both modes. For simulated recordings
(`sensor_mode=1`) it displays only an illustrative 2S profile from 8.4 to 8.2 V;
real recordings (`sensor_mode=0`) display the measured voltage. The display-only simulated
profile does not estimate charge or endurance and does not modify CSV telemetry.
The final details include `Tipo di volo`: `Simulato`, `Reale`, or `Non disponibile` when
metadata is missing. The persisted sensor mode determines the type, independently of
physical/virtual outputs; unknown types never receive a synthetic battery curve.
Recorded CSV gyro values stay in rad/s and are converted to °/s only for preview/hover.
These are BODY angular-velocity components, not derivatives of the displayed Euler angles.
The original Hoppino logger stores gyro rad/s multiplied by 1000 and exports them divided by
1000; the display applies only `180/pi`. In the current excerpt, peak absolute X/Y/Z rates
are 24.672/22.873/11.383 rad/s, or 1413.6/1310.5/652.2 degrees/s. These peaks already exist
in the source recording; unit consistency does not establish sensor accuracy. Live sensor
details intentionally retain rad/s, explicitly labelled.

`orientation.py` converts the stored Z-up Euler degrees into an X-up reference for the
orientation plot. With the standard active BODY-to-reference convention,
`R = Rz(yaw) Ry(pitch) Rx(roll)` and `R_display = Ry(+90 degrees) R`.
This maps NAV +Z to display +X, NAV +Y to display +Y and NAV +X to display -Z.
ZYX Euler angles are extracted again from the composed matrix: adding 90 degrees to pitch
alone is incorrect for general orientations. The upright reference `R=Ry(-90 degrees)`
displays zero roll/pitch/yaw. Rotations around the upright BODY X appear as roll.
Zero includes a chosen transverse orientation; vertical alignment alone cannot determine
absolute heading. This is a fixed reference conversion, not first-sample zeroing.
At the new pitch +/-90-degree singularity, roll is set to zero and yaw retains the observable
combined rotation. Missing any input Euler component leaves all three display angles missing.
The conversion precedes decimation; CSVs, firmware navigation and BODY gyro axes remain unchanged.
It does not itself initialize the estimator. Physical arming now initializes attitude from
the averaged gravity vector collected alongside gyro bias; logged replay retains its original
startup behavior.


The final details table includes distinct rules activated, saved rule conditions/output delays,
gyro biases, barometric reference, accelerometer biases, orientation and sensor/output mode.
Values come only from frozen flight metadata; simulated BODY data shows non-applied biases.
The [header snapshot specification](STORAGE_FORMAT.md#per-flight-calibration-snapshot-header-v4)
defines persisted values. No legacy log conversion is implemented.

Download prompts for unused paths, shows sample progress/estimated remaining time, then the
saved path or a precise failure. During transfer Annulla waits for the current reply, removes
only files created by that export and leaves stored flight/connection intact. Empty but valid
exports still report their saved files even if no plot can be drawn. The exporter protocol,
partial-file policy and metadata are owned by [Recording](../testing/RECORDING.md).

## Source ownership and verification

`window.py` owns the shell/views; `controller.py` operations and acknowledged state;
`connection.py` discovery/connect/close; `ports.py` port ranking; `configuration_page.py` draft
presentation; `configuration.py` translation; `dialogs.py` editors; `orientation.py` mounting;
`flight_plot.py` recorded plotting; `widgets.py` shared UI helpers.

Qt queued signals publish worker results; stale-session generations are ignored. One operation
executor sequences multi-request jobs; the transport owns serial IO. Close/disconnect await
cleanup asynchronously. Ctrl+C uses the same close path rather than raising a GUI exception.

The desktop suite uses the production C++ PC endpoint and fake peripherals, never COM ports.
It checks draft conflicts, responsiveness, six views, preparation/release, orientation reset,
recording/export and stale results. These checks are not physical output qualification; see
[Validation](../testing/VALIDATION.md#remaining-work-and-limits) for hardware limits.
