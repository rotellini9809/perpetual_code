# Perpetual app preferences

- During development, launch the Qt app from source. Do not regenerate the portable ZIP unless the user explicitly requests packaging; distribution is deferred until the app is satisfactory.
- The official artwork is in `assets/`. Preserve the folded P / infinity symbol as the defining brand mark. Use the full circular logo with the name for branding and the symbol alone for the app icon; keep original images intact.
- Rule IDs are internal, automatically allocated and immutable in GUI editors. Show names and select referenced rules by name; preserve stable IDs in firmware communication and saved data.
- The editable rule time is a delay after conditions first become true, not a requirement that they stay true. Each assigned output has an additional channel delay (default zero); activation waits for their sum. Pyro pulse timing is firmware policy, not a GUI setting.
- Normal Connect searches USB first, then paired outgoing Bluetooth SPP. Display transport and port as read-only text. During searching/closing, keep Connect disabled and show progress; no search-cancel button. Failed/lost sessions clean up automatically so retry needs no manual Disconnect. Manual port selection stays non-editable and hidden by default.
- Normal app startup schedules one automatic USB/Bluetooth connection attempt. Its connection failures stay silent in the UI/console and leave the app disconnected. Manual attempts, errors after a successful handshake and later link loss remain visible. No periodic retry or reconnect on loss; manual Connect remains available after failure. Explicit `--port` replaces that attempt; smoke tests without `--port` stay offline.
- The connection bar has one main row: link/port left, a large centered uppercase DISARMED (green) / ARMED (red), connection controls right. No second row of mode/output/reply-age badges. Unknown/disconnected arming state stays neutral; connection progress uses the existing feedback area.
- Firmware owns physical arming checks. Dashboard `Arm` always requests normal physical flight and is red/disabled while the board's physical preflight has blockers; require ready:true to enable it and use Ricontrolla to refresh. Do not duplicate sensor/actuator checks in Python. Simulation preflight/rejection reasons must not replace the dashboard's physical preflight. Assigned servos may be saved without setup; firmware blocks physical arming until configured.
- Simulation has one unchecked-by-default checkbox, `Uscite fisiche`, fixed while starting/running. Send `sim begin [physical] [record]`; actual mode comes from firmware replies, never from optimistic checkbox state. Physical simulation prepares assigned servos automatically; normal flight always selects physical outputs and requires explicit `servo arm [channel]` preparation. Both share OutputExecutor. GUI preparation/arm decisions use firmware replies; Disarm releases held preparation. Virtual runs reject outstanding physical preparation.
- Dashboard sensors show availability only. Details open in a separate live window: acceleration XYZ, angular-rate XYZ and pressure plots; temperature is numeric at the top right. History grows to 60 seconds of board timestamps, then discards older samples. Poll faster only while open; never substitute fake measurements for unavailable hardware.
- Firmware sleeps after five minutes without operational activity, even with the app connected. Do not add sleep/wake buttons. Background polling stays passive; use status for Dashboard sensor availability and acquire measurements only in the live detail window. Operational commands wake through firmware; clear cached preflight/continuity on power transitions and use firmware replies again after wake.
- The POWER SAVE ACTIVE badge sits at the right of the Dashboard title row and is hidden outside sleep. During sleep, retain the last awake sensor availability labels; after wake use fresh firmware observations. This display cache must not change actual sensor data or preflight checks, and is cleared for a new connection.
- Dashboard preparation shows only applied servo assignments. The manual pyro continuity button uses `pyro continuity`; its result is separate from physical preflight, which must always recheck before arming.
- Servo rows offer Configura/Modifica; position editors display a centered PWM label with a prominent number, adjusted by sliders and ±1/±10 increments. Each endpoint has an Applica and a Live button; only one Live can be active, highlighted green, with other movement buttons disabled. Preview sends `servo preview` while disarmed/idle, at most 10 Hz with one request in flight and only the latest pending value. Closing releases that preview; it never certifies flight preparation or writes flash. Conferma updates the local draft. Selecting an unconfigured servo in Dashboard makes preparation a red Vai alla configurazione action leading to its editor/apply flow.
- Servo configuration contains only the configured flag and initial/final PWM. Keep one configuration codec (v2) throughout firmware/app/exporters; format changes use an explicit fresh start rather than legacy decoding or hidden compatibility settings.

- Simulation replays the firmware-embedded official Hoppino sensor log, without a profile selector. Read duration/progress from firmware replies; the former 25-second synthetic profile is replaced.
- Recorded flight plots include `battery_v` in volts. It is real 5 Hz board telemetry even in simulation; missing battery data stays unavailable, never replaced by a synthetic trace.

## File map and focused changes

| Change | Start here |
|---|---|
| Colours, selection, scrollbars, spacing | `assets/style.qss` |
| USB/Bluetooth search, choice, connect/close lifecycle | `connection.py`; detection/order in `ports.py` |
| App state, operations, firmware replies and polling | `controller.py` |
| Configuration view, rule/channel editing, draft footer | `configuration_page.py`; editors in `dialogs.py` |
| Live sensor detail window and 60-second history/plots | `sensor_details.py`; polling in `controller.py` |
| Recorded flight curves, actuator markers and phase overlays | `flight_plot.py`; CSV loading in `controller.py`, view in `window.py` |
| Pure configuration model and firmware draft transactions | `configuration.py` |
| Window shell, connection bar, other views | `window.py`; shared widgets in `widgets.py` |

- Read/search the relevant component first; expand scope only for actual dependencies. Keep the existing compact Python style unless a specific expression benefits from expansion. Extract further pages only when that work makes their ownership clearer.
- Source launch: `.pio/perpetual_app_venv/Scripts/python.exe perpetual_app/main.py` from the repository root. The stylesheet is loaded at startup and included with the existing assets packaging.
- GUI/connection/configuration changes: `python tools/run_tests.py --suite desktop` (uses the app venv automatically). Add `clients` for transport/protocol changes and the affected C++ suites for firmware semantics. For style-only changes, use a source preview; don't rebuild firmware or portable artifacts.
- Keep tests in the existing desktop suite. Keep documentation proportional: update this map for structural changes and canonical component docs for behavior, without duplicating descriptions or adding per-edit reports.
