# Perpetual app: rules for contributors and coding agents

Design decisions for the Qt desktop app in this folder. Follow them when changing the app; behavior
that the firmware owns is described in the [technical docs](../docs/README.md), app internals in
[Desktop](../docs/technical/DESKTOP.md).

## Contents

- [Working on the app](#working-on-the-app)
- [General principles](#general-principles)
- [Connection](#connection)
- [Dashboard](#dashboard)
- [Configuration](#configuration)
- [Test and simulation](#test-and-simulation)
- [Recorded flights](#recorded-flights)
- [Settings](#settings)
- [Language](#language)
- [File map](#file-map)

## Working on the app

- **Launch from source** during development: `.venv/Scripts/python.exe perpetual_app/main.py` from
  the repository root. Do not regenerate the portable ZIP unless packaging is explicitly requested;
  distribution is deferred until the app is satisfactory. The stylesheet is loaded at startup and
  is included by the existing assets packaging.
- **Scope.** Read and search the relevant component first; widen the scope only for real
  dependencies. Keep the existing compact Python style unless an expression clearly benefits from
  expansion. Extract further pages only when that makes their ownership clearer.
- **Tests while iterating.** Run the affected desktop area with `--area NAME`, or `--quick` for
  firmware/host-only edits; `--plan` shows the selection without running. Use `--suite desktop` for
  the whole Qt suite, add `clients` for transport/protocol changes and the affected C++ suites for
  firmware semantics. For style-only changes, use a source preview; do not rebuild firmware or
  portable artifacts.
- **Final check.** Run `python tools/run_tests.py --changed` once at the end: it runs the suites and
  desktop areas affected by uncommitted files (anything not provably unaffected still runs) and
  lists files no host test covers. Do not repeat unchanged suites after it.
- **Writing tests.** Add them to the matching area module (`test/integration/desktop_*.py`,
  registered in `desktop_test.py`). Each area must pass alone: create state with `session()` from
  `desktop_support.py` rather than relying on earlier areas.
- **Performance.** Use `tools/profile_desktop.py --flight <CSV>` with the app venv: it runs offline
  and reports first-visit and repeated page timings. Profile connected separately for
  simulation/download stalls. FlightPlot geometry caches must invalidate on new data, curve changes
  and resize, while preserving gaps, hover and angular wraps.
- **Documentation.** Keep it proportional: update the [file map](#file-map) for structural changes
  and the canonical component docs for behavior, without duplicating descriptions or adding
  per-edit reports.
- **Branding.** The official artwork is in `assets/`. Preserve the folded P / infinity symbol as the
  defining brand mark: the full circular logo with the name for branding, the symbol alone for the
  app icon. Keep the original images intact.

## General principles

- **The firmware is the source of truth.** Arming state, run mode, preparation, readiness and
  settings shown as active always come from acknowledged firmware replies, never from optimistic
  UI state or checkbox values. Local drafts never activate anything.
- **Firmware owns physical checks.** Do not duplicate sensor or actuator checks in Python.
- **Never fake data.** Unavailable hardware stays unavailable; do not substitute measurements.
- **Behavior never depends on displayed text**: use codes, job kinds or flags.
- **Nothing saves automatically.** Use the generic close confirmation for unsaved rules or
  settings, with no list of changed groups, navigation dots, save-all action or automatic save.
  SIGINT/Ctrl+C skips that confirmation and follows the normal asynchronous cleanup.
- **One configuration codec (v3)** across firmware, app and exporters. Format changes use an
  explicit fresh start, not legacy decoding or hidden compatibility settings.
- **Power saving.** The firmware sleeps after five minutes without operational activity, even with
  the app connected. Do not add sleep/wake buttons. Background polling stays passive; operational
  commands wake the board through the firmware. Clear cached preflight and continuity on power
  transitions and rely on fresh firmware replies after wake.

## Connection

- Normal Connect searches USB first, then paired outgoing Bluetooth SPP. Show transport and port as
  read-only text. While searching or closing, keep Connect disabled and show progress; there is no
  search-cancel button. Failed or lost sessions clean up automatically, so retrying needs no manual
  Disconnect. Manual port selection is non-editable and hidden by default.
- At startup the app makes one automatic connection attempt. Its failures stay silent in the UI and
  console and leave the app disconnected. Manual attempts, errors after a successful handshake and
  later link loss remain visible. No periodic retry and no reconnect on loss; manual Connect stays
  available. An explicit `--port` replaces the automatic attempt; smoke tests without `--port` stay
  offline.
- With several compatible boards, the chooser shows each paired Bluetooth name.
- The connection bar has one main row: link and port on the left, a large centered uppercase
  DISARMED (green) / ARMED (red), connection controls on the right. No second row of
  mode/output/reply-age badges. Unknown or disconnected arming state stays neutral; connection
  progress uses the existing feedback area.

## Dashboard

**Layout.** Two aligned rows. Above, the rule summary and the sensors share the same height. Below,
preflight fills the height beside one shared actuator card, which holds the Pyro check and Servo
preparation subsections separated by a subtle divider, each with its commands.

**Summary.** The board-reported Bluetooth name is the large title, also over USB, with an em dash
when disconnected or unavailable. Bistage/Airbrakes badges sit on the right, in ochre, only for the
corresponding acknowledged, ready and enabled setting; hide disabled or unknown badges. Below, in
normal text, only the applied-rule count: no rule names, no Configuration title, no large validity
indicator. Configuration failures belong in the preflight list, with the firmware reason and a link
to Configuration or Settings. The name and rule count stay grouped above the ARM/DISARM toggle.

**ARM / DISARM.**
- One prominent centered toggle with the same large dimensions in both states: ARM green (solid
  when enabled, faded when disabled), DISARM red. The actual state always comes from the board.
- ARM always requests a normal physical flight. It is enabled only with `ready:true`; the board's
  physical preflight blockers disable it, and Recheck refreshes them.
- Both actions require a continuous 1.5 s hold (mouse, Space or Enter) with an animated fill. Early
  release, pointer exit, focus loss, navigation, disable/hide or a stalled UI cancel the hold. Send
  one request per gesture, and consume its release across the ARM→DISARM state change.
- `main.py --arm-preview` opens a standalone enabled animation demo that toggles after each
  completed hold, without a controller or hardware.

**Preflight.** Shows `controller.REASONS` and `CONFIG_ERRORS` through `t()`: it is status, not an
error message. Simulation rejection reasons must never replace the physical preflight.

**Sensors.** The Dashboard shows availability only, from `status`. Measurements are acquired only
in the separate live detail window: acceleration XYZ, angular rate XYZ and pressure plots, with the
temperature as a number at the top right. History grows to 60 s of board timestamps, then drops
older samples. Poll faster only while that window is open.

**Power save.** The POWER SAVE ACTIVE badge sits at the right of the Dashboard title row and is
hidden outside sleep. During sleep, keep the last awake sensor availability labels; after wake use
fresh firmware observations. This display cache never changes sensor data or preflight checks, and
is cleared for a new connection.

**Pyro and servos.**
- The manual continuity button uses `pyro continuity`; its result is separate from the physical
  preflight, which always rechecks before arming.
- Preparation shows only applied servo assignments. Servo rows show channel names and preparation
  state, never opening percentages or bars, also during runs.
- Normal flight requires explicit `servo arm [channel]` preparation. A separate **Release servos**
  button beside **Prepare selected** sends `servo release` to release held PWM while disarmed. An
  idle Disarm does not release preparation, and held preparation does not block other idle
  operations.
- Assigned servos may be saved without setup; the firmware blocks physical arming until they are
  configured. Selecting an unconfigured servo turns preparation into a red **Go to configuration**
  action that opens its editor.

## Configuration

**Rules.**
- Rule IDs are internal, allocated automatically and immutable in editors. Show names and select
  referenced rules by name; keep the stable IDs in firmware communication and saved data.
- In Configuration → Rules, selecting a rule highlights the whole group (heading, conditions and
  outputs) subtly. Clicking any child selects its parent rule; keyboard navigation moves between
  rules. Keep row separators and expansion arrows visible, and keep the selected rule across local
  edits.
- The rule time is a delay after the conditions first become true, not a requirement that they
  stay true. Each assigned output has its own channel delay (default zero); activation waits for
  their sum. The pyro pulse length is firmware policy, not a GUI setting.
- Condition editors (rules and airbrakes) offer second-stage phases only when the acknowledged
  board setting enables bistage. "All in-flight phases" is mask 14 in normal mode and 110 in
  bistage. Preserve existing masks when opening an editor; never silently rewrite saved conditions.

**Servos.**
- A servo configuration holds only the configured flag and the initial/final PWM.
- Servo rows offer Configure/Edit. The position editor shows a centered PWM label with a prominent
  number, adjusted by sliders and ±1/±10 steps. Each endpoint has **Apply** and **Live**; only one
  Live can be active, highlighted green, and the other movement buttons are disabled meanwhile.
- Preview sends `servo preview` while disarmed and idle, at most 10 Hz, one request in flight, only
  the latest pending value. Closing the editor releases the preview; it never certifies flight
  preparation or writes flash.
- **Confirm** saves only that servo to the board (the applied plan with this servo changed); other
  unsaved edits stay in the local draft. A draft already validated on the board must be applied or
  discarded first.

**Airbrakes.** The continuous AND gate belongs to Configuration → Rules. Servos reserved for
airbrakes cannot belong to ordinary rules. Endpoint captions are Closed 0% / Open 100%. Helpers
live in `airbrakes.py`, the editor in `airbrake_dialog.py`.

## Test and simulation

- The firmware chooses the profile: the embedded Hoppino flight in normal mode, a 25 s
  sensor-derived two-boost synthetic profile in bistage. There is no profile selector; the machine
  is selected in Settings. Read source, duration, progress and actual mode from firmware replies.
- Options: **Record to memory**, then **Physical outputs** (unchecked by default, fixed while
  starting or running). Send `sim begin [physical] [record]`.
- A physical simulation prepares the assigned servos itself; both modes share OutputExecutor. A
  virtual simulation releases held servo preparation automatically while disarmed and idle, then
  starts without physical outputs.
- Show **Arming blocked / Go to checks** only after the firmware's preflight rejects a physical
  simulation start; the action opens the Dashboard and runs Recheck. Keep this hidden otherwise,
  and keep simulation rejection separate from real-flight readiness.

## Recorded flights

- Plots include `battery_v` in volts, labelled Battery voltage in both modes. It is real 5 Hz board
  telemetry, also in simulation; missing data stays unavailable. Simulation recordings show only an
  illustrative 2S trace (8.4 to 8.2 V), display-only, never written to exports.
- The details show Flight type from the saved `sensor_mode`, or Not available when missing.
- The airbrake card sits directly below vertical velocity/acceleration and appears only when that
  flight's saved enable flag is true. It shows the commanded opening and the saved AND conditions,
  without a transition table, from the recorded metadata rather than current device settings.
  Airbrake control state and opening percentages appear only here: the Dashboard neither polls nor
  shows them, because the airbrakes do not move on the pad.

## Settings

- Orientation, state machine and airbrake enable are saved individually. Local setting drafts live
  in app state; activation and runs use only values acknowledged by the board.
- Saved/unsaved status is communicated only by the Save/Saved button. Keep the label beside it
  hidden except for errors.
- While Settings is open, the bottom feedback area stays empty in normal use, including save
  confirmations and progress; it shows only errors. Other pages keep normal feedback.
- Cards share the `settingsTitle` style and heading alignment; no category headings such as
  Configuration/Storage. The orientation heading sits at the card's top left above its image, with
  ROCKET NOSE and its arrow above the image. The short transition note below the flight diagram is
  shared by normal and bistage; ABORT details stay in the bistage box.
- **Airbrake enable** is an independent saved setting, off after reset, with a warning that a
  compiled control law is required (the default law commands zero). The Dashboard shows only
  configuration hints (disabled in Settings, no dedicated servo) and shares preparation/release.
- **Bluetooth name**: "Perpetual" by default, or "Perpetual-" plus up to 4 letters, digits or `-`;
  BLE adds "_BLE". Rename over USB or Bluetooth. Over Bluetooth the expected link drop is shown as a
  restart, not an error; the app reconnects once to the same port and verifies the name. This is the
  only automatic reconnection in the app.

## Language

- English is the default and the text written in the code; Italian comes from `translations_it.py`.
- Wrap every user-facing text in `t('English text')` at display time, never at import. Pass values
  as placeholders (`t('{count} rules applied', count=n)`) and add the Italian entry; the
  `desktop/translations` area fails on missing, unused or mismatched entries.
- Texts kept in tables (e.g. `STATES`, airbrake `REASONS`) stay English and are translated with
  `t(value)` when shown; list new tables in `desktop_i18n.table_keys()`.
- Error messages always stay English and are never wrapped in `t()`: exceptions, `state.error`
  (including firmware reasons inside refusals) and IMU procedure errors.
- Numbers use a decimal point in every language (`use_decimal_point()`). Exported CSV/JSON stay
  English.
- The language is chosen in Settings → Language and confirmed with **Apply**, enabled only when the
  choice differs from the saved one. It applies after a restart: the window saves it in QSettings and
  offers to restart, then `main.py` relaunches after the normal close.
- Behavior test areas run in Italian (`start_app('it')`), so their assertions also check the
  catalog; `desktop/translations` checks the English default.

## File map

| Change | Start here |
|---|---|
| Colours, selection, scrollbars, spacing | `assets/style.qss` |
| USB/Bluetooth search, choice, connect/close lifecycle | `connection.py`; detection/order in `ports.py` |
| App state, operations, firmware replies and polling | `controller.py` |
| Applying operation results to app state | `controller.py`: `_complete` and the `_apply_*` helpers; job kinds (`POLL`, `DOWNLOAD`…) drive behavior, job names are labels only |
| What is re-read after a command | `controller.py`: `_command_effects` and `_read` |
| Window shell, connection bar, Simulation and Console views | `window.py`; shared widgets in `widgets.py` |
| Hold-to-ARM gesture, animation and isolated preview | `arm_button.py`; `main.py --arm-preview` |
| Dashboard: configuration summary, sensors, preflight, pyro/servo preparation, ARM | `dashboard_page.py`; navigation to other pages via its signals in `window.py` |
| Live sensor detail window and 60-second history/plots | `sensor_details.py`; polling in `controller.py` |
| Configuration view, rule/channel editing, draft footer | `configuration_page.py`; editors in `dialogs.py` |
| Pure configuration model and firmware draft transactions | `configuration.py` |
| Six-face IMU calibration and saved verification | `imu_calibration_page.py`; board picture from `board_preview` in `orientation.py`; operations/polling in `controller.py`; Dashboard preflight row in `dashboard_page.py` |
| Airbrake presentation and editor | `airbrakes.py`, `airbrake_dialog.py` |
| Recorded flight curves, actuator markers and phase overlays | `flight_plot.py`; CSV loading in `flight_data.py`, view in `recorded_flights_page.py` |
| Euler display convention | `orientation.py`: ZYX in an X-up reference; stored NAV Z-up angles and BODY gyro axes stay unchanged |
| Device settings, bistage and maintenance | `settings_page.py`; board operations in `controller.py` |
| Board orientation preview and selector | `orientation.py` (`board_preview` is shared with IMU calibration), embedded in `settings_page.py`; persistence in `controller.py` |
| Bluetooth name card, rename and the one reconnection after a radio restart | `settings_page.py`; `rename_bluetooth` and `_bluetooth_restart` in `controller.py`; firmware in `Jdy34Link` |
| UI language and Italian catalog | `i18n.py`, `translations_it.py`; switch in `settings_page.py`/`window.py`; relaunch in `main.py` |
