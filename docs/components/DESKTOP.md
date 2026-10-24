# Qt desktop prototype

The Windows app lives in `perpetual_app/`, reusing `tools/device_client.py`. Extract the complete
portable ZIP and open **Perpetual.exe**, keeping `_internal` beside it. Python is bundled.
End-user instructions are in `LEGGIMI.txt`; application source and dependency notices are included.

## Build, run and verify

From the project root on Windows:

The dedicated environment is named `perpetual_app_venv`. It isolates GUI dependencies from
PlatformIO and other projects; a separate environment is recommended, but not required.
To use a local Python instead, install `perpetual_app/requirements.txt` with that interpreter's
`-m pip install -r` and launch `perpetual_app/main.py` with the same interpreter.

```powershell
powershell -ExecutionPolicy Bypass -File perpetual_app/build.ps1
& .pio/desktop_dist/Perpetual/Perpetual.exe
& .pio/perpetual_app_venv/Scripts/python.exe perpetual_app/main.py
python tools/run_tests.py --suite desktop
python tools/run_tests.py --full --desktop
```

During development, launch from source with the Python command above. Portable ZIP distribution
is deferred until the user is satisfied with the app; do not regenerate archives for routine changes.
Build creates an isolated `.pio/perpetual_app_venv`, installs pinned requirements and builds the windowed
PyInstaller folder, without creating a ZIP. `-SkipInstall` reuses installed dependencies.
When explicitly requested, `& .pio/perpetual_app_venv/Scripts/python.exe perpetual_app/package.py`
packages the built folder as `.pio/desktop_dist/Perpetual-Windows.zip`.
It does not change PlatformIO's Python or upload firmware. Qt remains
in separate DLLs. Normal launch attempts USB/Bluetooth connection once after showing the window,
without periodic retries. Only that background startup attempt hides connection failures and
leaves the app disconnected; manual attempts and errors after a successful handshake remain visible.
`--port COM9` replaces this with one explicit-port attempt. Smoke tests
without `--port` stay offline. After failure or a later disconnect, reconnect manually.
Pressing **Connetti** searches native RP2350 USB CDC candidates first, then paired outgoing
Bluetooth SPP candidates. **USB · COM9** / **Bluetooth · COM11** are read-only transport/port
labels, not fixed COM numbers. The button stays **Connetti**, disabled/faded during search and
cleanup, with **Connessione in corso…** progress; there is no search-cancel button. A successful
Perpetual/protocol/transport handshake enables **Disconnetti**. Normal requests in progress
still show Connesso; the operation line describes work/reply age separately.

`ports.py` reads pySerial metadata, filters the current USB VID/PID and outgoing Bluetooth SPP
remote-address records, and excludes Windows' all-zero incoming SPP port. Cached paired-device
names filter known unrelated Bluetooth devices when available. The last verified hardware
identity is preferred within its transport, while USB always has priority. Multiple candidates
in the same transport without a remembered identity require a choice rather than opening a
random board. A hidden-by-default **Porta manuale** option exposes a non-editable port list and
refresh button; `--port` also retains explicit CLI selection. Windows pairing is done once
outside the app. BLE/GATT is not implemented.

Every failed attempt finishes closing before the next candidate opens. If all attempts fail,
or an established link is lost, the app cleans up the session and restores **Connetti** without
requiring a manual disconnect. Unsaved local edits remain; commands are never replayed and a
lost established session is not automatically switched/reconnected. Closing the window stops
the search and invalidates late results. An OS Bluetooth open can still delay cleanup until
Windows returns; that existing worker limitation is not a handshake success.

## Five views

| View | Implemented |
|---|---|
| Dashboard | Configuration card, one sensor-availability card with live details, exact firmware preflight blockers and arm/disarm, compact pyro check card above servo preparation |
| Configuration | Four pyro/four servo settings, local rule/condition/output dialogs, expandable rules, Valida/Applica/Scarta footer |
| Test and simulation | Onboard Hoppino sensor replay (47.164 s, no profile selector), unchecked-by-default Uscite fisiche, opt-in recording, acknowledged start/stop and phase/output events |
| Recorded flights | Explicit recover/list, CSV/JSON download through the existing exporter, offline CSV opening, altitude plot and saved rule/output-start inspection |
| Console | Manual shared-protocol commands and bounded request/reply log |

Connection, transport and a large centered DISARMED/ARMED state stay visible in one header row;
operation feedback stays below the page. Firmware
replies change displayed state; pressing Arm never changes the UI to Armed by itself.
The board authorizes physical arming. The app displays its preflight and failed-arm reasons,
without duplicating hardware checks in Python. Dashboard **Arm** always requests physical flight;
board preflight blockers make it red/disabled, and `ready:true` is required to enable it.
Use **Ricontrolla** after resolving blockers. Simulation reports do not replace physical preflight.
The IMU solder fault remains a real sensor blocker. The dashboard has no flight-phase card or
duplicate sensor-count card. Preparation lists only servos assigned in the applied configuration;
unsaved local edits cannot expose new hardware controls. Removing an applied assignment clears its
hidden selection. **Verifica continuità pyro** sends the dedicated read-only `pyro continuity`
command for assigned channels. Its result is separate from preflight and never enables arming.
Preflight and Arm always measure continuity again through the same hardware reader.
No Python arming checks or firing command are added. The button is disabled while armed,
busy or editing an unapplied configuration; reconnect/configuration reload clears its result.
Select servo channels and use **Prepara selezionati**
to command their saved initial positions; reported preparation enables Disarm even before flight.
Selecting an assigned servo with unset applied endpoints turns that button red, labelled
**Vai alla configurazione**. It opens Channels and the first missing servo's editor rather than
sending any preparation commands. If that servo already has a configured local draft, it leads
to Channels to apply the draft first. Firmware remains authoritative for physical readiness.
Failed arming and link loss keep prepared PWM; Disarm releases it. Manual direct channel tests
remain unavailable. Physical simulation prepares its assigned servos automatically;
its checkbox is locked during start/run and the board validates outputs without real sensor checks.
Editing servo endpoints
changes settings only, without moving/calibrating hardware. Default simulation is virtual and
does not record. Firmware controls five-minute inactivity sleep and automatic operational-command
wake, even while the app stays connected. The Dashboard gets sensor availability from passive
status; only the live detail window polls sensor measurements. On sleep, cached preflight and
manual continuity results are cleared and the existing readiness card shows power saving.
The larger POWER SAVE ACTIVE badge appears at the right of the Dashboard heading only while
sleeping. Sensor labels retain the last awake availability during sleep, without changing raw
status/readiness; new observations after wake replace that display cache, and reconnect clears it.
Ricontrolla or another operational request wakes through firmware and refreshes readiness.
No sleep/wake buttons or independent Python arming checks are added. Console still accepts
the diagnostic `sleep`/`wake` commands.

## Configuration transactions

The last applied snapshot and local `Plan` are distinct. Dialog acceptance only edits local RAM.
Only servo rows assigned to a rule in the current configuration draft show **Configura** or
**Modifica** in Channels; unassigned rows remain marked free. The position editor shows PWM values
with synchronized sliders and ±1/±10 controls, bounded by firmware capabilities. Unset positions
start at the midpoint of the allowed range. Confirming creates a configured local servo draft;
it still needs Valida/Applica. Editing never moves hardware. Servo data contains only the configured
flag and initial/final PWM; the executor commands endpoints directly.
Rule IDs are internal and assigned automatically; editors and references display rule names.
Outputs select a kind, channel and additional channel delay (default zero). The common rule delay starts
once all conditions first become true and is not reset by later condition changes. Pyro timing
is a fixed firmware profile, automatically set when assigning a channel, with no duration editor.
Rules with assigned, unset servos can be validated and saved. The configuration page marks
them as needing setup; physical preflight/Arm reports `SERVOS_NOT_CONFIGURED` until configured.
Virtual simulation can still run. This behavior requires the matching updated MAIN firmware.
Duplicate channel
ownership is explained with the conflicting channel and rule name before any board mutation.
Update firmware together with the app: `rule delay` / `delay_ms` replace the confirmation setting;
`output assign` accepts the channel delay, added to the common rule delay; output editors and
configuration detail rows show it in milliseconds. Old firmware produces a clear update message.
Renaming preserves identity. References use a selection of existing rules, excluding the rule
being edited; missing references must be explicitly repaired. IDs are retained in the protocol
and exported data, and are never renumbered when rules are reordered or renamed.
Valida checks/compiles all input before starting or replacing **this connection's** firmware RAM
draft and invoking its validator; it never saves flash. Applica repeats staging/validation,
requires applied/persisted/verified acknowledgments, then reads the new plan before replacing
the applied baseline. It asks for explicit save confirmation. Scarta discards owned remote and
local edits. A foreign draft is never silently discarded.

The applied baseline is reread before staging and its clone checked again after reservation,
preventing silent overwrite of an externally changed plan. Referencing rules are removed first;
all IDs are added before their conditions. Firmware remains the semantic validator for cycles,
channel ownership and numeric/endpoint limits. Inspection automatically selecting a firmware
draft is labelled/read-only rather than presented as applied. Local changes remain in memory
through disconnect, but app exit prompts before losing them. No extra device-calibration or
Bluetooth-name persistence is introduced. Console edits may need an explicit configuration reload.

## Ownership and responsiveness

`configuration.py` is a pure model/translator; `controller.py` owns app state, operations and
operation-generation guards. `connection.py` owns search/connect/close with one lifecycle phase
and guards against stale connection callbacks; `ports.py` detects and orders candidates.
`configuration_page.py` owns rule/channel editing and the draft footer; `window.py` owns the
shell and remaining pages, with editors in `dialogs.py` and shared widgets in `widgets.py`.
`assets/style.qss` contains the Qt stylesheet, loaded on startup and bundled with existing assets.
One operation executor sequences multi-request work and discovery; `DeviceConnection` owns serial IO. Qt queued
signals publish on the UI thread. Old-session results are ignored. Close/disconnect await
cleanup asynchronously; an OS Bluetooth open that stalls can keep cleanup pending.
Ctrl+C in the launching terminal requests the same asynchronous window close rather than
opening an error dialog for KeyboardInterrupt. Repeated interrupts keep cleanup running;
the previous SIGINT handler is restored when the Qt event loop exits.

Only one poll is outstanding, with at most one foreground action behind it. Foreground work
pauses polling. Status/control are read about every four seconds, active/visible simulation
about every 1.2 seconds, dashboard sensors about every ten. Rates stretch with reply time,
especially at the module's 9600 baud; this is not high-rate telemetry. New phases/start counts
request events. Simulation shows numeric metrics and events; full plots are in recorded flights.

The Dashboard sensor card shows availability only. **Leggi dettagli sensori** opens a separate
nonmodal window with acceleration XYZ, angular-rate XYZ and pressure plots; temperature is a
numeric value at the top right, smoothed over the last five seconds of valid readings; invalid
current samples display a dash immediately. Sensor availability is green/red (unknown is neutral).
While open, sensor polls target 4 Hz on USB and 1 Hz on Bluetooth,
still yielding to foreground operations and other polls. Closing restores the normal slow poll.
`sensor_details.py` owns the history and Qt-rendered plots without an extra chart dependency.
The time axis grows from the first board `uptime_ms` to 60 seconds, then retains only the last
minute. Invalid samples break curves, disconnect pauses acquisition, and reconnect/reboot clears
the history; uint32 uptime rollover remains continuous. No sensor data or timestamps are fabricated
when missing; an older firmware without timestamps prompts an update.

CSV plots are decimated to about 1200 points; original CSVs remain intact. Existing download
files are never silently overwritten; interrupted downloads may leave partial CSV data.
Recorded flights show five plots: altitude; vertical velocity and acceleration with separate
m/s and m/s² axes and aligned zero ticks; gyroscope X/Y/Z in °/s; roll/pitch/yaw in degrees;
battery voltage in V from `battery_v`. Files without that field have no battery curve.
Recorded CSV gyro values stay in rad/s and are converted to °/s only for preview/hover.
All share recorded timestamps. `flight_plot.py`
owns these curves; missing values break them rather than inserting zeroes. Two independent
checkboxes show actuator start markers and flight-state background bands, enabled initially.
State transitions come from the full CSV before decimation; actuator events come from the
companion JSON. These switches change only the view, never firmware or actuator settings.
In flight details, each saved rule expands into its AND conditions followed by assigned outputs
(`Uscita · PYRO 1`, `Uscita · SERVO 1`). Output values show each channel's configured delay from
the frozen flight snapshot; missing historical fields are shown as unavailable. The adjacent
`Regole azionate` row counts distinct saved rules with at least one matching, recorded output
start, once per rule (virtual or physical). It reports unavailable when metadata is absent;
interrupted recordings may contain fewer committed start records than actually occurred.
Actual recorded start times remain on the plot markers. There is no separate output-activation section.
Hovering near a curve highlights the nearest displayed sample and shows its recorded time,
value and unit in a tooltip; actuator labels sit to the left of their event lines.
The hovered curve becomes thicker while other curves and their legend entries fade;
moving away restores the normal appearance.
Numeric axes always include a labelled zero. Orientation uses −180°…180° with 90° ticks;
angular jumps of at least 180° break the curve, including exact half-turns near Euler
singularities. Disconnected angular samples remain visible as points with hover values.
After confirming a selected flight's save path, a sample-based progress bar appears next to
**Scarica selezionato**. The remaining time is written inside the bar beside the percentage,
using average measured sample-transfer speed
after the first second, excluding initial metadata preparation; before that it shows calculation
progress, and after the last sample it shows finalization. Updates continue at least every half
second when sample chunks arrive, even within the same percentage. Successful CSV/JSON export
replaces the bar and estimate with selectable **Downloaded at …**
and the full path. Cancelling the file dialog starts nothing; failures/disconnects show their
outcome without a success message. A saved flight with no plottable samples still reports its
downloaded path and explains that the preview is unavailable.
During transfer, **Scarica selezionato** becomes a red **Annulla** button. Cancellation waits
for the current reply, sends no further download requests, removes only files created by that
export, and shows **Download annullato**. The connection and stored flight remain usable;
the button is disabled as **Annullamento…** while the pending reply finishes.

## Tests and limits

The optional desktop suite uses the production PC C++ endpoint for configuration transactions,
conflicts/dependency order and flash-format recording/download. Qt tests cover responsive waits,
acknowledged state, uint32 IDs, draft separation, all five views and stale-result isolation.
These tests never open COM ports. Packaged startup/rendering can also be checked with:

```powershell
& .pio/desktop_dist/Perpetual/Perpetual.exe --smoke-test .pio/desktop_exe_smoke
```

Real observations are in [Validation](../testing/VALIDATION.md). A separate Windows machine still
needs qualification. Calibration/direct test procedures, richer
graphs, firmware updates and additional power modes remain future work.
