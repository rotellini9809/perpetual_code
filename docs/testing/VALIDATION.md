# Validation evidence and remaining work

This is the single dated evidence record, consolidated from the implementation reviews, flash
audit and per-component bring-up notes. Observations below are from **18 October 2026** unless
stated otherwise. They are not fresh measurements performed during documentation cleanup. Local
`.pio` artifacts are ignored and may not exist in a clean checkout; their paths preserve
provenance.

## Software verification

On **21 October 2026**, Qt Connect gained USB-first discovery with outgoing paired-SPP fallback,
read-only transport/COM labels, disabled Connect while searching, hidden manual port selection,
and automatic failure/loss cleanup. The Qt suite checks discovery filtering, delayed close before
fallback, retry without manual disconnect, USB priority after Bluetooth use, late-handshake
isolation and ambiguous-device selection. An actual automatic GUI connection reached **Bluetooth
COM11** and completed configuration synchronization with no error. Evidence is in
`.pio/tests/logs/automatic_connection.json` and `.png`; firmware was not uploaded or changed.

On **20 October 2026**, the first Qt Windows desktop prototype was added with Dashboard,
Configuration, Test/Simulation, Recorded Flights and Console. The shared connection worker
remains the serial owner; a GUI operation executor and queued Qt signals keep waits off the
UI thread. Local configuration and the applied baseline are separate. RAM staging compares
the current/frozen baseline, preserves foreign drafts, orders dependency removal and creates
IDs before conditions. Save success requires firmware persistence/verification and readback.
Unsupported servo preparation/physical tests/physical-output simulation stay unavailable.

All **15 groups** passed with `python tools/run_tests.py --full --desktop`. The opt-in Qt area
tests actual C++ configuration/recording/download formats, invalid/stale/foreign drafts,
uint32 ID editors, responsive pending operations without optimistic Armed state, full local
edit/validate/apply/readback, local changes retained on source loss and old-session result
isolation. The default/full firmware/client areas remain 12/14 groups without a Qt dependency.
Desktop tests never open COM ports.

Real GUI tests synchronized on **COM9 USB** and **COM11 SPP**. Via Bluetooth the GUI started
and observed the onboard virtual profile: **5001 samples, zero rejected navigation frames**,
computed PAD/BOOST/COAST/DESCENT/LANDED and rule-10 virtual pyro at 13175 ms; 18 observed status
points formed the preview. It also mounted/listed **seven retained flights**, downloaded
**sequence 9, 5001 samples** via USB, read the configuration/output-start JSON and plotted it.
The CSV SHA256 matched the previous reference exactly:
`5f2146e0f89ed2de3149309166174b4f3ff59cd4f6cc1aed073df996203a586f`.
No real configuration save, new flash recording, erase, output movement or firmware upload was
performed during these GUI checks; configuration saves were exercised on PC simulated NOR.

The windowed PyInstaller x64 build and portable ZIP were produced with bundled Python/Qt,
app source, version manifest and dependency notices. The packaged EXE opened/rendered all five
views offline and synchronized/rendered through Bluetooth with Python/Qt environment overrides
removed and PATH limited to Windows system directories. A separate Windows machine remains
unverified. Artifacts: `.pio/desktop_build.log`, `.pio/desktop_full_tests.log`,
`.pio/desktop_gui_bluetooth.json`, `.pio/desktop_gui_download.json`,
`.pio/desktop_simulation_bluetooth.png`, `.pio/desktop_flight_usb.png`,
`.pio/desktop_packaged_checks.json`, `.pio/desktop_dist/Perpetual/Perpetual.exe` and
`.pio/desktop_dist/Perpetual-Windows.zip`. The desktop [guide](../technical/DESKTOP.md) is the
canonical workflow/limits reference.

On **20 October 2026**, the PC protocol client moved from the estimate-replay script to
`tools/device_client.py`. The four replay/simulation/download entry points share its serial
opening, handshake and connection worker; the MAIN Bluetooth smoke test borrows its protocol
client for raw retry/disconnect checks. Serial IO, a bounded 32-request FIFO and idle
two-second heartbeats run on one PC worker. Async connect/submit/disconnect Futures and
thread-safe snapshots are available for the next desktop UI; the current monitor is CLI.

All **14 PC groups passed** through the extracted client, including both 9433-sample recording
paths. Added client regressions cover strict/malformed/late/extra replies, ID/command/schema
checks, command-error details without dropping a healthy connection, ambiguous failure without
retry or session reuse, queue ordering/cancellation/capacity/shutdown, heartbeat versus long
commands, callback deadlock rejection, explicit reconnect, failed identity/version/opening and
cancellation during port open. Fault cases use fake byte links; no physical unplug/fault-injection
qualification is implied.

Real **COM9 USB and COM11 SPP** connections passed info/control handshake, status and idle
heartbeat observation. The migrated simulation runner also completed via Bluetooth:
**5001 samples, zero missed ticks**, max cycle **743 us**, mean **263.0 us**, max lateness
**1520 us**, with the same computed phases and virtual rule-10 pyro start. No firmware upload,
module setting change, flash erase or configuration save was performed. Artifacts:
`.pio/client_connection_usb.jsonl`, `.pio/client_connection_bluetooth.jsonl`,
`.pio/client_connection_simulation.log`, `.pio/simulation_b2c890f0591b4243ab367750c6195f3e.json`,
and the current `.pio/tests/logs`. This is a normal-profile run, not an endurance or timing bound.

On **19 October 2026**, MAIN gained simultaneous USB and JDY-34 command endpoints. The shared
`CommandService` replaced the USB-specific name; each endpoint has independent framing,
numbered-request cache and bounded pending reply. Router tests cover owner-only mutation,
observer disconnect, autonomous-run survival/takeover, independent IDs and response
backpressure. JDY-34 tests cover nonblocking read-only startup, single-link mode, notification
requirements, complete AT status snapshot, timer rollover, reconnect/reset, UART overflow and
notifications appended to unfinished/oversized input. All **14 PC groups passed** after the
shared-protocol changes and again after the notification/device-test additions and final
overflow cleanup. MAIN built successfully (about 62 KiB static RAM, 181 KiB program).

MAIN was uploaded and verified on **COM9**, with the existing outgoing SPP port **COM11**.
Actual UART startup, JSON info/status/control requests, radio connected/disconnected flags,
exact-ID retry, RAM-draft ownership, rejection of a foreign USB mutation, observer USB closure,
partial-command radio disconnect, draft discard and reconnect/ID reset passed. No configuration
was applied, recording erased or module setting changed. These checks are available through
`python tools/run_bluetooth_test.py --main --port COM9 --radio-port COM11`.

A complete **25-second onboard simulation controlled through Bluetooth** passed while USB
made 60 reads, attempted a rejected foreign stop and then closed. Result: **5001 samples,
zero missed ticks**, max cycle **767 us**, mean **256.9 us**, max lateness **1366 us**. Computed
phases and virtual rule-10 pyro start matched the existing profile. This single run demonstrates
the shared runtime with both transports; it is not a radio endurance or worst-case timing
qualification. Final shared-endpoint smoke test also passed after upload. Reports:
`.pio/main_dual_transport_final.json` and
`.pio/main_bluetooth_simulation.json`; upload/build logs: `.pio/main_dual_transport_upload.log`,
`.pio/main_dual_transport_build.log`. USB remains enabled normally; name changes remain in the
isolated diagnostic. BLE, radio loss during a recorded run and hardware power-cycle behavior
remain unverified by this integration test.

On **19 October 2026**, the PC test layout was consolidated from **30 C++ files to ten area
suites**, and **nine Python scripts to four groups**. All original C++ scenarios remain;
shared USB feeding and process transport helpers were deduplicated. Three hidden stdin modes
became one `test/fixtures/command_device.cpp`. The old manually wired HIL-to-recorder replay
was removed; its raw IMU/pressure/temperature, navigation and degree/radian checks were moved
to synchronous **and** queued production USB recording/download integration.

The runner now shares incremental g++ objects and dependency files, links selected areas,
compiles with up to four processes, and keeps per-group logs. Navigation replay uses the same
builder instead of recompiling on each replay. Cache regression checks exercise unchanged
builds, changed headers and flags, filenames/root paths with spaces and recovery from compiler
errors. Default runs all ten C++ suites plus synthetic replay/client checks; `--full` adds both
Hoppino groups. Explicit area selection, detailed output and forced rebuild are available.

Measured on this Windows host, sequential invocations with no competing test build:

| Run | Elapsed |
|---|---:|
| Original runner, all 30 C++ + nine Python scripts | 81.58 s |
| Consolidated full run, forced compile (45 objects, 13 executables) | 17.95 s |
| Consolidated full run, warm cache (14 groups, including Hoppino) | 12.42 s |
| Consolidated default run, warm cache (12 groups, no flight CSV) | 2.24 s |
| Consolidated C++ only, warm cache (ten suites) | 1.17 s |
| PlatformIO native prototype, C++ only, cold | 17.70 s |
| PlatformIO native prototype, C++ only, warm | 10.43 s |

These are individual observed runs, not statistical benchmarks. All measured real suites
passed. The isolated `.pio/pio_compact_eval` prototype used the same ten C++ suite sources,
a native environment and custom runner with assertions enabled. Its intentional nonzero-exit
probe failed correctly and was removed afterward; JUnit output was generated. Python
integrations were **not** included in the PlatformIO timings. Keeping the existing entry point
avoids a second orchestration path/custom adapter for those clients. No device upload or
hardware test was performed during this cleanup; firmware build flags were unchanged.

Artifacts: `.pio/tests_before_cleanup.log`, `.pio/tests_cleanup_cold.log`,
`.pio/tests_cleanup_warm.log`, `.pio/tests_cleanup_quick.log`, `.pio/tests_cleanup_cpp_warm.log`,
`.pio/pio_compact_cold.log`, `.pio/pio_compact_warm.log`, `.pio/pio_compact_failure.log`, and
`.pio/pio_compact_cold.xml` / `.pio/pio_compact_warm.xml`. Current area logs are in
`.pio/tests/logs`. Historical counts and artifacts below describe their earlier revisions.

On **19 October 2026**, after the user removed the old pairing, Windows displayed **Perpetual**.
Despite its Settings UI reporting a retry-later connection error, opening the matched outgoing
SPP port **COM11** succeeded; AT status on USB COM9 confirmed an active channel (`+STAT=1,1`).
A second connection verified actual wireless data in both directions using the existing raw
name-test firmware with PWRC high: MCU UART `AT\r\n` arrived byte-for-byte on COM11, and a
unique 38-byte PC radio marker arrived byte-for-byte in the MCU UART buffer. PWRC was restored
low and both ports closed afterward. This verifies Classic SPP on this Windows host, not BLE
GATT or sustained throughput. Logs: `.pio/bluetooth_pairing_status.log`,
`.pio/bluetooth_pairing_open_port.log`, `.pio/bluetooth_spp_bidirectional.log`.

On **19 October 2026**, the focused test added explicit `module reboot` / client `--reboot`
using documented `AT+RESET`, plus read-only MAC/channel/mode queries. COM9 retesting without
any name setters read **Perpetual** for both profiles before and after a successful soft reset
(`+OK`, subsequent `+START=OK`, AT and version replies). MAC **20:21:11:25:13:0A** matched the
Windows device still named **JDY-34** and its outgoing SPP port COM11. This supports a cached
host display-name explanation; a fresh scan is still needed to verify its current advertised
name. All eight queried channels were disconnected during the measurement, so this was not
a connected end-to-end radio test. Mode was 1. A physical module power cycle remains untested.
PWRC is used for AT access, not as a reset/power switch; reboot is an explicit separate action
and never a factory reset or an automatic post-save step. Build/upload and device retest passed:
`.pio/bluetooth_reboot_upload.log`, `.pio/bluetooth_connected_names_before_reset.log`,
`.pio/bluetooth_names_soft_reboot.log`.

On **19 October 2026**, isolated JDY-34 name testing resolved the reversed MCU UART
directions: the user confirmed **module TXD -> GP5**, hence MCU RX=GP5/TX=GP4. Both Bluetooth
diagnostics now use hardware UART1 (`Serial2`); sensors still use hardware SPI1 and flash SPI0.
The actual module answered at **9600 8N1 / CRLF**, PWRC low, with
`+JDY-34-SPP-BLE-V1.52`. Its responses use `OK` for NAME writes, `+NAME:` for SPP readback,
`+OK` for NAMB writes and `+NAMB=` for BLE readback. Initial factory readbacks contained
embedded NUL bytes, preserved in the raw diagnostic transcript rather than normalized.

Both the independent `BLUETOOTH_NAME_TEST` sequence and the reusable `NameController` on
COM9 successfully set and read back **Perpetual** for SPP and BLE. A first command after MCU
reboot was repeatedly unanswered; a second read-only AT succeeded. The controller now permits
two initial AT attempts, then identifies the module and runs each setter/readback once.
A direct rename as the first operation after reboot succeeded through that handshake.
No setter retry or external-flash copy was introduced. Power-cycle persistence of the module
and actual over-radio reception remain untested.

The focused test is selected in `platformio.ini`, uploaded on COM9, and has a dedicated client
`tools/run_bluetooth_name_test.py` with raw hex/ASCII transcripts. Host name tests include
the actual V1.52 trace and bounded handshake/failure paths; diagnostic/client checks and
MAIN/general/focused firmware builds passed. Artifacts: `.pio/bluetooth_name_focus_upload.log`,
`.pio/bluetooth_name_focus_verified.log`, `.pio/bluetooth_uart1_controller_upload.log`,
`.pio/bluetooth_uart1_controller_device.log`, `.pio/bluetooth_uart1_MAIN_build.log`.

The subsequent JDY-34-only cleanup removed alternative-module selection and the single-profile
rename path. Both SPP and BLE names must now be verified on every successful rename. Focused
name/diagnostic C++ suites, the Bluetooth-client Python integration and BLUETOOTH_TEST build
passed. Build log: `.pio/bluetooth_jdy34_only_build.log`.

On **19 October 2026**, the Bluetooth name method passed 30 C++ suites and nine Python
integrations. Focused name/diagnostic suites also passed after matching JDY-34's documented
direct version reply (`+JDY-34-V1.2`) alongside its `+VERSION=` form. Coverage includes
validation, serialized commands, model rejection before setters, JDY-34 SPP/BLE exact readback,
lost ACKs, module rejection, conflicting replies and partial changes
without automatic retry. No external-flash setting/format or configuration change was added.

The updated BLUETOOTH_TEST firmware was uploaded on COM9. Device tests verified invalid names
and raw setters are rejected; a valid request stopped at version discovery with NO_RESPONSE,
`verified:false` and `may_have_changed:false`, with no setter issued. This verifies the failure
path, not an actual hardware rename, module identity or power-cycle persistence. Normal-mode
Bluetooth integration was still pending at that point; the USB entry was `name set`/`name show` in the
diagnostic. Logs: `.pio/bluetooth_name_tests.log`, `.pio/bluetooth_name_upload.log`,
`.pio/bluetooth_name_device.log`. MAIN and the selected diagnostic also passed final builds:
`.pio/bluetooth_name_MAIN.log`, `.pio/bluetooth_name_final_build.log`.

On **19 October 2026**, Bluetooth bring-up added `BLUETOOTH_TEST` in the existing firmware
environment. All 29 C++ suites and the existing eight Python integrations passed; the new
Bluetooth-client integration also passed separately, covering fragmented replies, rate discovery,
failed-scan restoration and distinct wireless echo verification. MAIN and BLUETOOTH_TEST built;
the diagnostic was uploaded on COM9. On-device guards rejected setters, reset, invalid baud and
oversized input without UART writes. GPIO startup uses Pico SDK initialization/preload/direction
ordering because Arduino-Pico `pinMode(OUTPUT)` clears a previously set output level.

Hardware probes received zero UART bytes at 9600, 115200, 19200, 38400, 57600 and 4800 baud,
with both tested AT control levels. PIO reported initialized, RX idle high, and PWRC returned
high after each transaction; these observations do not prove module identity, wiring or RF
operation. The user confirmed that the fitted module is JDY-34; phone advertisements have not
been associated with this board and are not hardware identification evidence. UART/module and
wireless tests were unsuccessful/unverified at that stage. UART/name communication was
subsequently resolved as recorded above; wireless verification remains pending. Artifacts: `.pio/bluetooth_host_tests.log`,
`.pio/bluetooth_MAIN_check.log`, `.pio/bluetooth_final_upload.log`,
`.pio/bluetooth_device_guards.log`, `.pio/bluetooth_hardware_low.json`,
`.pio/bluetooth_hardware_high.json`. Continue with the
[Bluetooth procedure](README.md#bluetooth-bring-up) and identify the board's advertisement.

On **19 October 2026**, orderly hardware-mode log retention passed 28 C++ suites, eight Python
integrations and MAIN. The shared runtime records actual BOOST history independently of flash
samples; normal PAD close submits an asynchronous discard, while simulation/HIL retain their
diagnostic logs. Coverage includes BOOST rejected by a full producer queue, full PAD sample
storage, durable discard acknowledgment, preservation after cancellation, 513 byte cuts during
discard, unchanged healthy history and reuse only after committed invalidation. Physical
activation remains disabled; hardware-mode tests inject sensor data on PC with virtual outputs.
Logs: `.pio/retention_tests.log`, `.pio/retention_MAIN.log`. No board upload or new hardware
measurement was performed. Final targeted storage/runtime/arming/HIL/simulation checks and
MAIN also passed after the follow-up review: `.pio/retention_final_checks.log`,
`.pio/retention_MAIN_final.log`.

On **19 October 2026**, shared arming verification passed 27 C++ suites and eight Python
integrations, plus MAIN. Focused follow-up checks passed after adding acquisition-gap reset and
ordinary-disarm ownership checks. Coverage includes missing-IMU rejection without flash writes
or a retained flight, complete channel blocker masks, immediate simulation with zero calibration
samples, RAM-only physical bias/reference collection, edit/storage/run exclusion during arming,
USB disconnect, cancellation, late readiness failure, flight epoch reset and airborne disarm
rejection. The positive physical transitions use PC fake readiness and exclusively virtual
starts; MAIN still blocks physical activation. Logs: `.pio/arming_tests.log`,
`.pio/arming_targeted.log`, `.pio/arming_final_usb.log`, `.pio/arming_MAIN.log`. Final USB checks
also cover repeated disarm while the worker retains Closing ownership and explicit
simulation-calibration bypass in status. No arming-controller firmware upload was
performed in this step.

Earlier shared runtime/sensor-source verification passed 25 C++ suites, eight Python integrations and
the MAIN build. A PC fake acquisition device feeds raw IMU/pressure through HardwareSensorSource
and the same FlightRuntime used by HIL and autonomous simulation; computed navigation/rules
match HIL, hardware/virtual/raw provenance is passed to recording, and storage faults leave
navigation running. Separate checks cover missing IMU with healthy barometer, independent
freshness, timer rollover, invalid input and USB stale/absent sample rendering. The real driver
now checks data-ready status; acquisition cadence/pairing and IMU operation still need board
verification. The subsequent barometer check is recorded below. No upload was performed during
that software-verification step. Logs:
`.pio/shared_runtime_tests.log`, `.pio/shared_runtime_targeted.log`, `.pio/shared_runtime_MAIN.log`.

The preceding storage verification also built FLASH_TEST, FLASH_DIRECTORY_TEST and
CONFIG_STORE_TEST in the same PlatformIO environment. Host USB corruption and sticky
no-revival/no-resize checks passed after that full suite. [The test guide](README.md) defines
current coverage; case counts there supersede older counts from before the one-sector settings
cut.

After the calibration/mounting update, all 23 C++ suites and eight Python integrations passed
again, along with the MAIN build. Tests compare a dynamic BODY trajectory with raw input under
all six signed-axis mountings, including default +X, sensor-axis bias subtraction, invalid bias
rejection, frozen per-run values, new-session isolation and calibrated-BODY bypass. The Hoppino
reference and recording round-trip still pass. This is host/build verification; no board upload
or calibration-sector writes were performed. Logs: `.pio/calibration_mounting_tests.log` and
`.pio/calibration_mounting_MAIN.log`.

The normal v2 whole-image FNV64 remains `0x6cff49998aaf6eea` after CRC/page-helper, current-only
format and settings-erase simplifications. Header v1 support, optional summary/Legacy states,
periodic checkpoint API and wrapped metadata support were removed. Current-format bytes,
explicit extent ownership and commit ordering remain.

The audit's global recovery obstruction was fixed: confirmed damaged recordings receive
persistent directory status Corrupt, retaining their full extent while healthy flights remain
downloadable. A dedicated host test covers 513 interrupted flag commits, remount/compaction,
explicit release, and separation from incomplete tails/I/O/unsupported headers/directory damage.
Real client/USB-over-pipes coverage rejects the damaged export and preserves all 9433 rows of
another healthy export.

Queued close/abort now return immediately even with a paused worker. Native flash ownership is
retained until acknowledgment; config/new runs/downloads remain BUSY. Tests cover late failures
after accepted close and final simulation status after another HIL recording begins. Terminal
counters are read after acquire acknowledgment.

Asynchronous-close/corruption scenarios have **software/build verification, not electrical
power-cut qualification**. The subsequent sensor-only board log does not qualify those scenarios.
Evidence includes
`.pio/flight_corrupt_tests.log`, `.pio/flight_corrupt_usb_final.log`,
`.pio/flight_corrupt_MAIN.log` and the diagnostic-mode build logs;
`.pio/flash_async_tests_final.log`, `.pio/flash_async_handoff_checks.log`,
`.pio/flash_async_usb_final.log` and `.pio/flash_async_MAIN_final.log` cover the prior step.

## Sensor bring-up - 19 October 2026

The user supplied the MAIN USB `sensors` and `@42 sensors` results after upload: BMP581 ID 0x50,
driver error 0, valid pressure 101631.578 Pa and temperature 24.239 C, with sample ages 1 and
2 ms. This verifies fresh barometer-only acquisition and the diagnostic reply on that board,
not absolute pressure accuracy or flight timing. IMU ID remained 0x00 and initialization failed,
consistent with the known solder problem. SPI error 0 means the read operation returned without
a transport error; the invalid identity still rejects the IMU. A subsequent user arming/preflight
run rejected physical arming and also reported BAROMETER_UNAVAILABLE; `sensors` then returned
barometer ID 0x00 and driver error -5. The cause has not been established. Further sensor
diagnosis is deferred until solder rework at the user's request; no readiness bypass or driver
workaround was added.

## Navigation reference

The independent PC sensor replay and onboard HIL agree over 9433 original Hoppino frames
(integration tolerances 1e-5 for z/vz/bias/acceleration, identical phases/ validity/baro
updates). Changing recorded estimates/phases or converting hPa/Pa does not change computed
results. There were 1603 fresh barometric corrections.

| Sensor-derived phase | Relative time |
|---|---:|
| PAD | 0 ms |
| BOOST | 10138 ms |
| COAST | 12131 ms |
| DESCENT | 14492 ms |
| LANDED | 32162 ms |

Compared to the old quantized recorded estimator, z differs about 0.197 m RMS (max 0.462 m), vz
about 0.130 m/s RMS (max 0.932 m/s). These are regression comparisons, not physical accuracy.
Offline launch reference, input quantization and actual-vs-fixed AHRS dt account for possible
differences; no exact historical trajectory reproduction is claimed.

The independent estimate replay uses recorded DESCENT at 14490 ms and fixture starts
14490/14744/20439 ms. Recomputed sensor navigation with the same fixture starts
14492/14744/20430 ms. The board's saved one-rule USB example produces only Pyro 1 at computed
apogee. Do not use estimate-replay timestamps as expected sensor-navigation results or infer a
fixed plan from the number of starts.

## Flash identification and board timing

Solder rework changed repeated invalid JEDEC reads to EF4018 at all tested modes/ rates. The
read/write test on the final sector passed twice, restoring/verifying its original bytes. This
identifies the W25Q128JV and validates one tested sector, not the entire device or its
endurance. Historical timings below predate asynchronous close: finish measured the full drain,
whereas current finish_us is submission only.

### Observed synchronous baseline - 18 October 2026

MAIN with instrumentation was uploaded to the connected RP2350 on COM9. The flash driver
retained its existing 1 MHz setting. A new sequence 2 recorded all 9433 Hoppino samples with
zero drops, zero rejected navigation frames and no read/program/erase failures. Sequence 1
remained retained. Sequence 2 was then downloaded: its sample CSV is byte-for-byte identical to
the previously verified sequence 1 CSV, and its frozen USB example plan/start metadata remains
correct.

| Scope | Mean | Maximum |
|---|---:|---:|
| Navigation/rules | 389.7 us | 628 us |
| Storage per sample | 4892.8 us | 53431 us |
| Combined step | 5282.5 us | 53929 us |
| Flash read API | 3005.3 us | 7378 us |
| Flash program API | 8695.8 us | 8877 us |
| Flash erase API | 66164.0 us | 72377 us |

Preparation took 27.460 s for the 1 MiB sample reservation plus its header; finish took 75.884
ms. There were 8420 read, 3443 program and 257 erase calls. 858 of 9433 measured steps exceeded
the nominal 5000 us period. The captured report is
`.pio/hoppino_hil_98dac2508387411a8adf4fa33a56e070.timing.json`.

The synchronous path's mean combined step already exceeds the nominal 200 Hz budget, before USB
overhead. A queue must be paired with a separate worker or an asynchronous driver to remove
storage blocking from navigation. At this setting, storage alone uses about 97.9% of the nominal
period on average, leaving little sustained-throughput margin even if computation moves to
another core. Improving that margin and measuring queued-worker behavior are necessary for
periodic-source operation; a queue alone cannot improve average throughput.

### Observed queued worker at 1 MHz - 18 October 2026

Sequence 3 used the 64-sample queue and dedicated core 1, with PC-paced input and no queue
throttling. All 9433 frames passed navigation, but only 9203 were saved: 230 samples were
dropped when the queue filled. There were no flash API failures. Mean producer
compute/submission was 513.2 us, maximum 855 us, with zero measured steps above 5000 us. The
worker mean was 5154.5 us and its maximum 76032 us; queue peak was 64/64. This demonstrates why
bounded producer latency alone does not establish adequate storage throughput.

PC replay elapsed 47.169 s for 47.164 s of source data. 547 sends were late by more than one
nominal period, with 67712 us maximum lateness. The report is
`.pio/hoppino_hil_723c946531ea45deb5231c518a528efe.timing.json`. The recording is retained as
completed with missing samples; that historical format did not contain the RAM dropped-sample
counter. Its stored timestamp gaps remain observable. MAIN was subsequently changed to the
explicit 4 MHz board setting.

### Observed queued worker at 4 MHz - 18 October 2026

Sequence 4 repeated the same PC-paced replay with no queue throttling. All 9433 samples were
saved, with zero drops, zero rejected navigation frames and zero flash API failures. The queue
peak was 4/64. Navigation phases and the single configured virtual Pyro 1 start remained at the
reference timestamps.

| Scope | Mean | Maximum |
|---|---:|---:|
| Navigation/rules | 460.9 us | 758 us |
| Storage submission | 61.6 us | 187 us |
| Producer compute/submission | 522.6 us | 819 us |
| Storage worker iteration | 2037.1 us | 26051 us |
| Flash read API | 839.6 us | 2105 us |
| Flash program API | 3432.8 us | 3777 us |
| Flash erase API | 37020.4 us | 44522 us |

Zero producer steps exceeded 5000 us. Preparation took 12.366 s and finalization 26.091 ms.
Flash API counts were 8420 read, 3443 program and 257 erase calls. Worker time is measured on
the other core and includes verified writes; a worker iteration may exceed one sample period
while the queue absorbs incoming frames.

PC replay elapsed 47.169 s for 47.164 s of source data. Host timing still showed 1459 sends late
by more than one nominal period, maximum lateness 70312 us and maximum USB round trip 7020 us.
This successful run establishes bounded measured producer latency and complete recording for
this USB input; it does not establish autonomous sensor scheduling or worst-case flight
deadlines. Report: `.pio/hoppino_hil_2cac553578164678847fb26d529d375a.timing.json`.

Sequence 4 was downloaded after stop. Its entire 9433-row CSV is byte-for-byte identical to the
verified sequence 1 export, and frozen metadata matches except for sequence number. Stored
timestamps, phases and estimates also match live HIL exactly. The inventory at that check
retained sequences 1 through 4.

## Autonomous profile and storage lifecycle on hardware

The 25-second profile computes phases PAD=0, BOOST=5000, COAST=7000, DESCENT=13175 and
LANDED=22180 ms; the saved one-rule plan starts virtual Pyro 1 at 13175 ms. All 5001 sample
timestamps are spaced by 5 ms in a complete run.

| Observation | Result / provenance |
|---|---|
| Sequence 5, autonomous recording | 5001 saved, zero drops/invalid/missed/over-budget; mean cycle 280.3 us, max 868 us; lateness max 1525 us, queue 4/64, prepare 6.199 s, drain 25.892 ms. `.pio/simulation_ae3460dc158a426b9ddbd49cc4db26c9.json` |
| Disconnect-only profile | Continued across USB reconnect and completion: 5001 samples, no misses/invalid frames, max cycle 674 us/lateness 456 us, zero flash calls. `.pio/standalone_disconnected.json`; this was not a power cut |
| Sequence 6, summary upgrade | 5001 saved, zero drops/invalid/misses, max cycle 846 us, queue 4/64; Valid Completed summary after restart; sample CSV identical to sequence 5 |
| Explicit test deletion/reuse | Only one-sample test sequences 7/8 deleted; 8 reused 7's extent at 0x51B000 with a higher identity; cached old selection rejected; retained sequences 1..6 unchanged at that check |
| Current-only format, sequence 9 | 5001 saved, zero drops/invalid/misses/API failures, max cycle 939 us, mean 236.8 us, lateness 1686 us, queue 4/64; prepare 6.112 s, drain 34.632 ms; CSV identical to sequence 6 with matching configuration/virtual start and Valid Completed summary |
| Old header rejection | Existing v2 sequence 6 readable; v1 sequence 1 rejected as Header/UnsupportedVersion with zero program/erase calls; no migration/format performed |

The original Hoppino sequence 1 export was also verified with the then-v1-compatible reader
before compatibility removal. That is historical evidence, not a claim that current firmware
reads v1. Old on-device v1 records remain allocated/listed until explicit deletion. Inventory
statements above are snapshots, not live inventory.

Key export/verification artifacts:

- `.pio/flight_5_4cd7f2abb2b64747b64aa6630f750bf4.csv` and sibling JSON.
- `.pio/flight_6_20cdaabdfbb04360b72dee3a3b201e07.*`,
  `.pio/storage_upgrade_verification.json`, `.pio/storage_upgrade_reuse.json`,
  `.pio/simulation_b053381ba89b415792c880a59bb2e689.json`.
- `.pio/flight_9_simplified.csv` / `.json`, `.pio/storage_simplify_readback.json`,
  `.pio/storage_simplify_verification.json`,
  `.pio/simulation_319c50a7bb314bfd86551451196e663d.json`.
- Sequence 9/6 identical CSV SHA256:
  `5f2146e0f89ed2de3149309166174b4f3ff59cd4f6cc1aed073df996203a586f`.

## Sensor rate, flash speed and flight log v4 - 18 November 2026

Observed on the Waveshare RP2350 board on COM11, disarmed, no outputs commanded. Artifacts were
kept in a session scratch folder only; the numbers below are the record.

**Sensor SPI.** The SENSOR_TEST `f` sweep (2000 rounds per clock of ID reads, configuration
bursts compared with a 1 MHz reference and live-data plausibility, production pull-up chip
selects) was error-free for both sensors from 1 to 25 MHz; at 37.5 MHz every IMU check failed while
the barometer still passed. Both sensors now run at a 10 MHz request (9.375 MHz actual), about
2.7x below the highest clock seen working; a 16-byte IMU burst takes 33 us instead of 159 us.

**IMU rate.** Accelerometer and gyroscope run at the 480 Hz setting; the factory oscillator trim
(INTERNAL_FREQ_FINE = -6, about -0.8 %) makes the true rate 476.3 Hz. A single-transaction
STATUS_REG + data read lost samples (74 Hz in a tight loop, 464 Hz in MAIN) because reading the
outputs clears data-ready. Through the stream FIFO the idle loop measured **476.2 Hz over 30 s
with 0 sequence gaps, 0 FIFO overruns and 0 tag errors**, although 107 loop pauses exceeded one
IMU period (worst 3.5 ms). After the half-period minimum spacing, frame dt stayed within
1870-2162 us; acquisition used about 6 % of core 0.

**Navigation cost.** The onboard Hoppino replay (6600 steps at 200 Hz) measured filter plus rules
at a mean of 302 us and a maximum of 763 us per step without recording, 394 us / 1007 us with
recording enqueue (about 20 % of core 0 at 476 Hz against a 2100 us budget).

**Flash.** The FLASH_TEST `c` sweep (JEDEC, 64 KiB CRC with 0x03 and 0x0B, one programmed page per
clock in the backed-up test sector, restored and verified) passed up to 37.5 MHz; 75 MHz failed
identification and SPI0 cannot produce 50 MHz. The board now uses 18.75 MHz. With the v3 log,
the recording worker went from 21.8 ms per 1 KiB block (4 MHz, 1 ms busy sleeps) to 8.3 ms
(18.75 MHz, 10 us polling, no pre-write read for sample pages) and 6.3 ms (nibble-table CRC, no
fixed transaction delays); 1 MiB preparation went from 11.9 s to 7.9 s and then 3.3 s with
64 KiB block erases.

**Flight log v4.** Recording the Hoppino replay into 1 MiB (sequence 7) saved **5137 of 6600
samples**: the 8.2 s of PAD were reduced to 175 pre-launch history samples and everything from
BOOST on was kept, with **0 dropped**, no recording error and preparation in 3.28 s. The 93 blocks
held about 55 samples each and cost about 7.8 ms of worker time per block including the
read-back (about 7 % of core 1 at 476 Hz, against about 33 % for v3). Writing the history at
boost was the longest worker job, 64 ms, with a queue peak of 15 of 128. The production
downloader fetched the 93 raw blocks in **3.0 s**; the CSV showed BOOST at 8.202 s, LANDED at the
end, maximum altitude 52.17 m, both virtual pyro starts at 13.056 s, 1337 rows with a fresh
barometer reading and 77 deliberate history gaps. Every test flight was deleted afterwards.

**Layout v2 (device journal, directory v2, flight ring).** After the upgrade the board reported
`Directory`/`Corrupt` on the old layout as expected; `flights format`, `config reset`, one rule,
`config apply` and `config orientation +X` then ran on the new journal, each persisted and
verified, and the configuration was still valid after reflashing. FLASH_DIRECTORY_TEST
(directory banks 0x010000..0x02FFFF, 150 updates with automatic and explicit compaction) and
CONFIG_STORE_TEST (device journal 0x000000..0x00FFFF) passed and restored their regions. Two
onboard Hoppino recordings (sequences 1 and 2, consecutive ring placements) saved **4602 and 4603
of 6600 samples** (4603 is the host model: 175 history samples, full rate from BOOST to LANDED,
LANDED at 10 Hz), **0 dropped**, no storage error, 82 blocks each. Preparation took 1.49 s and
1.50 s (eight 64 KiB erases, about 1.16 s in total, longest 159 ms); growth was not needed. The
worker peaked at 75 ms (history write at boost) with a queue peak of 17 of 128. The 200 Hz
simulation loop had a maximum cycle of 1.37 ms; maximum lateness was 2.7 ms with one missed tick
on the first run and 2.4 ms with none on the second. The download took 3.0 s; the CSV showed PAD
history, 5 ms steps from BOOST to DESCENT and 100 ms steps in LANDED. Both flights were deleted.
Growth during a long flight and ring overwrite are covered by host tests only.

**Host.** All 36 runs of `python tools/run_tests.py --full --desktop` passed with the pinned
WinLibs GCC 14.2 (ISO C++17, the firmware's compiler generation), including synchronous and
queued USB recording/download of the Hoppino HIL flight compared sample by sample within half a
stored LSB, and 2049 power cuts across one v4 block write.

## 480 Hz stress profile, stack and ring wrap-around - 19 November 2026

Observed on the Waveshare RP2350 board over USB (COM11), disarmed, VIRTUAL outputs only, with the
`sim begin record 480` profile ([Simulation](SIMULATION.md#480-hz-stress-profile)) and `diag`.

**Stack.** Core 0 starts on the 4 KiB SCRATCH_Y stack, and that is not enough: right after boot
and a connection it had used 5.6-5.8 KiB, `flights info` took it to 7784 bytes and a recorded
480 Hz run to 7988-8184 bytes of the 8 KiB available down to the heap limit (SCRATCH_X holds no
data with `core1_separate_stack`, so the overflow lands there). `-fstack-usage` shows the cause:
each command layer keeps its own reply buffer on the stack (`CommandService::dispatch` 1528 B,
`CommandRouter::dispatchFrom` 1040 B, `dispatchCommand` 1120 B, handlers 1100-1430 B), and
`ConfigurationStore::decode`/`beginDraft` use about 2.8 KiB each. Core 1 peaked at 2572 of 8192
bytes.

*Fix, same day.* The rarely used replies were moved out of the dispatch frames into separate
functions (`CommandService::dispatch` 1528 -> 80 B, `CommandRouter::dispatchFrom` 1040 -> 264 B,
`dispatchCommand` 1120 -> 264 B), `ConfigurationStore::decode` now decodes in place (2880 -> 56 B)
and `beginDraft` no longer builds a temporary plan (2824 -> 16 B). Measured again: 3112 bytes after
boot and connection (was 5752), 3188 after `preflight` (was 6268), 3736 after `flights recover`,
and a maximum of **5056 bytes** during `sim begin record 480` (was 7988-8184); reading a stored
flight stays below that. Each core's MSPLIM now sits 256 bytes above the bottom of its stack: a
forced overflow on the bench stopped at the limit and the board restarted instead of writing
below it. The cause of that restart was not recorded (`last_fault` stayed empty and the watchdog
reason read FORCE, consistent with a lockup reset), so a stack overflow currently shows as an
unexplained restart; other HardFaults reboot through a handler that records CFSR/HFSR.

*Repeat after the fix.* A third ring run (18 recordings, about 25 minutes, wrap at run 16) passed
with no restart (`reset_reason` stayed `soft`, `last_fault` empty), core 0 stack maximum 5112 of
8192 bytes, core 1 2572, worker job maximum 161 ms, erase 173 ms, queue peak 76 of 128, 0 dropped;
11-30 frames per run were still skipped while the PC polled `sim status`. The first-attempt
restart has not recurred in 36 later recordings; its cause remains unproven.

**Unexplained reboot.** In the first ring attempt the board restarted during the fourth 61 s
recording (uptime 69.7 s afterwards); that flight was retained Completed. The reset cause was
not captured (`diag` now reports `reset_reason` and the raw POWMAN/WATCHDOG bits). The second
attempt, 18 runs, had no restart. Candidates: the core 0 stack depth above, a power glitch or USB.

**Timing at 480 Hz.** With USB silent during the run: 29278 frames, **0 missed**, maximum lateness
1.3 ms, core 0 loop maximum 2.56 ms (3 periods above 2083 us), 17749 samples saved (the host
count), 0 dropped. With the PC asking `sim status` every 0.5-1 s, 18-31 frames per run were
skipped (lateness up to 4.4 ms, loop up to 5.8 ms): USB command handling during flight blocks
core 0 for several IMU periods. Step maximum 1.6 ms (compute 1.5 ms) against the 2.08 ms period,
no step over budget.

**Storage.** Pre-launch history full (341 samples). History write at boost, the longest worker
job: 170-178 ms (64-75 ms with the 200 Hz Hoppino); sample queue peak 81-84 of 128 (about 270 ms
at 480 Hz). One growth step per run, 64 KiB erases up to 179 ms, no dropped sample.

**Ring wrap-around.** `ring_overwrite_usb.py` after `flights format`: 18 recordings of 16 blocks.
Flights 1-15 filled blocks 0-239; flight 16 took 240-252 and 0-2, crossing the end of the ring,
and invalidated only flight 1; flights 17 and 18 invalidated 2 and 3. Every placement followed the
previous flight, extents never overlapped, nothing else changed, and the board's
`overwritten_flights` matched. Flights 4 (oldest retained) and 18 were downloaded and decoded
(17728 and 17727 samples). All test flights were deleted afterwards.

## Physical arming on the bench - 19 November 2026

**Barometer timestamp bug.** The first physical arming never completed: the gyro collector kept
restarting after 3-7 samples. The barometer was stamped when read, the FIFO IMU frame when
sampled (never later than its observation), so a fresh pressure was a few ms "newer" than its
frame; `ArmingCalibration` and `Navigation` require barometer time <= frame time and dropped it.
Besides blocking arming, navigation would have ignored almost every real barometer reading in
flight (altitude/velocity from the accelerometer alone, LANDED never confirmed). Simulation, HIL
and replays share one timestamp, so they never showed it. `HardwareSensorSource` now clamps the
barometer time to the frame it travels with (error a few ms); a host test reproduces the board
case (fails without the clamp, collector ready after 2496 frames with it).

**USB load with the real sensors.** Board still in PAD, servo 1 prepared, no pyro assigned,
`bench_arming_usb.py`, three 60 s phases: quiet; app load (status + control status every 4 s,
ping every 2 s); heavy (plus `sensors` every 0.25 s). 86012 frames in total (about 478 Hz),
**0 sequence gaps and 0 FIFO overruns in every phase**. Core 0 loop maximum 3.56 ms quiet and with
the app load, 4.28 ms with the heavy load; loops longer than one IMU period: 9, +60, +392. Step
maximum 1.03-1.05 ms. Frame dt 1049-2168 us: after a stall the queued FIFO frames are processed
at the half-period minimum spacing. USB traffic therefore delays processing by a few ms but loses
no samples; the frames skipped in the 480 Hz simulation with USB polling come from the simulated
source, which drops late frames by design. Core 0 stack peaked at 3832 bytes.

**Real recording with motion.** Repeated Part A (same result: 0 gaps, 0 overruns, loop maximum
4.44 ms), then `--motion`: shaken by hand for BOOST (COAST 6 ms later), left still on the bench
for about 3 minutes under the app load, lowered about 1 m and set down. DESCENT at 185 s fired the
Drogue rule on servo 1 (recorded output start at the same millisecond), LANDED at 190 s, and the
run closed by itself. Flight 39: 85930 samples, 0 dropped, status Completed, 339 pre-launch history
rows; 85442 full-rate samples from BOOST to LANDED at **476.16 Hz with no gaps**, then the slow
post-LANDED rate. The flight grew 6 times (allocated 3.5 MiB) with 65 block erases, longest 179 ms;
queue peak 84 of 128. Altitude estimate drifted -0.3 to -0.9 m over 3 minutes still, then fell to
-1.9 m when the board was lowered: navigation uses the real barometer.

Three maxima looked alarming and are not flight data:
- Worker maximum 1.63 s: the asynchronous preparation at arming, which erases the first 8 blocks
  (8 x ~180 ms) before the source starts, so no samples exist yet. Recording-time jobs stayed at
  the erase time.
- 1 FIFO overrun in the run (2 by the final `diag`): Part A deltas were 0 and the runtime saw no
  sequence gap, so it happened while the IMU FIFO was not read: during that preparation, before
  `source.start()` re-anchors. The second came after the run ended.
- Core 0 loop maximum 12.3 ms: `diag` loop statistics and the overrun count restart at run start
  but keep counting after it ends, so this includes the post-landing `flights` commands; it cannot
  be placed inside the flight. A stall that short is absorbed by the IMU FIFO either way (step
  maximum 1.52 ms).

`diag` and `flights timing` were then changed so these figures describe only the run: loop and
overrun figures stop at run end, overruns count from the first processed frame, and the initial
erase is reported as `prepare_us`, outside `worker`.

**Full ring with real sensors - 20 November 2026.** Same procedure with that firmware, but the
board stayed on the bench in COAST for about 20 minutes before being lowered, so the recording
filled the whole flight ring: flight 40, 31 growths, 254 block erases (longest 175 ms), the one
older retained flight overwritten, 16.58 MB allocated. 464015 full-rate samples at **476.20 Hz
with no gaps** and longest frame spacing 2165 us, then StorageFull at 985 s: the 108825 later
samples were dropped as designed, and the flight continued without storage. DESCENT, detected when
the board was finally lowered (1193 s), still fired servo 1 and its output start was recorded; the
run closed by itself. Over the whole run (Part A included): 0 sequence gaps, **0 FIFO overruns**,
worker maximum 172 ms (preparation at arming 1516 ms, separate), queue peak 81 of 128, step
maximum 1.52 ms, core 0 stack 3800 bytes. The core 0 loop maximum, now inside the run only, was
16.3 ms: a real stall whose source is not yet located (not a processing step; the IMU FIFO
absorbed it). The altitude estimate drifted between +0.2 and +1.7 m over 16 minutes still
(barometric drift).

**Hand shake detected as a whole flight.** Next run, with `loop_max_at`: Part A loop maxima
3.5-4.3 ms, all in PAD while reading USB input. The shake gave BOOST at 10.535 s, COAST 26 ms
later and DESCENT 189 ms after BOOST (velocity -0.54 m/s at 0.22 m), which fired servo 1; LANDED
at 15.8 s and the run closed after the 15 s landing window (flight 41: 2995 samples, Completed).
The detector confirms BOOST after 3 consecutive frames above 3 g and COAST after 3 frames of
non-positive BODY X acceleration; at 476 Hz that is about 6 ms each, and DESCENT needs only
-0.5 m/s. **A knock of a few ms on an armed rocket could therefore deploy the recovery on the pad.**
Fixed the same day: launch now needs a sample above 3 g and then 1.5 g held for 200 ms, and burnout 30 ms (see [State machines](../technical/STATE_MACHINES.md#normal-flight-machine)). Run maximum loop 5.1 ms at 230.6 s, in LANDED without USB input (the run
closing); the 16.3 ms of the full-ring run did not recur without growths.

## Initial alignment and tilt on the bench - 20 November 2026

Test firmware (identity IMU calibration), physical arming in normal mode, board held by hand and
set against a goniometer; `tools/replay/bench_tilt_usb.py` read the `attitude` field of `status`
for 3 s per position (about 30 readings) while the board stayed in PAD.

| Position (goniometer) | Tilt read | Spread | `up_body` |
|---|---|---|---|
| Upright, at arming | 5.9° | 5.5-6.2° | [0.995, 0.019, 0.100] |
| 20° | 22.7° | 22.2-23.1° | [0.923, -0.077, 0.378] |
| 40° | 39.4° | 39.1-39.9° | [0.772, -0.018, 0.635] |
| 90° | 89.2° | 89.1-89.3° | [0.014, 0.012, 1.000] |
| 90°, turned about 20° around the rocket axis | 89.9° | 89.8-90.0° | [0.003, 0.345, 0.939] |
| Upright again | 1.0° | 0.6-1.5° | [1.000, 0.009, 0.014] |

BODY +X is the rocket axis, the gravity alignment at arming is correct and the tilt agrees with the
goniometer within about 3° over 0-90° (hand-held; the 5.9° at arming is most likely the hand, since
the same position read 1.0° later). A turn around the rocket axis keeps the tilt and moves only the
direction in `up_body`. A 30 s hold at about 40° moved 41.4 → 43.7°: in PAD the accelerometer keeps
correcting the attitude (gain 0.3), so this reflects the hand, not gyroscope drift. **Not covered:**
the gyroscope-only attitude of BOOST and COAST, where the correction gate (|a| within 0.2 m/s² of
1 g) is closed; that is where the bistage 40° check applies.

## Remaining work and limits

Dated observations above retain the firmware/format used at the time. In particular, older
v2/v3 log and default-mounting observations do not describe the current recording header v8,
flight log v4 or the explicit orientation requirement. Current implementation is documented in [technical references](../README.md).

| Area | Current qualification limit |
|---|---|
| Real sensors | Idle IMU cadence/pairing measured (476.2 Hz over 30 s, no gaps; see [Hardware](../HARDWARE.md#sensors)); gyroscope collection during a physical arming works on the board after the barometer timestamp fix, and a hand-driven bench flight reached DESCENT and LANDED with the real barometer (19 November 2026); a real flight profile is not yet measured |
| Calibration and attitude | Per-arming gyro/reference collector, persistent mounting and six-face accelerometer calibration (persistence and UI) exist. The board still uses identity calibration values. Gravity alignment and tilt reading agree with a goniometer within about 3° in PAD (20 November 2026, above); the gyroscope-only attitude of BOOST/COAST is not yet measured |
| Physical outputs | Shared execution, preparation/release and physical simulation are implemented and host-tested; pyro firing with real igniters works on the board and does not reset the MCU. Servo mechanical behavior, interruption and load margins still need bench acceptance |
| Driver timeout | Timeout disables driver readiness; recorder recovery cannot reinitialize it. Restart/reinitialization remains the current route |
| Interrupted/old headers | Unsupported versions fail without migration; current layout v2 (device journal v1, directory v2, flight ring), recording header v8, sample blocks v4, configuration codec v3. A board with the previous layout needs `flights format` and its rules, settings and calibration saved again |
| Scheduler and storage | Worker close is asynchronous, but a stalled worker retains ownership; idle preparation/settings/scans stay synchronous. The pre-launch history write at boost measured 64-75 ms on the Hoppino replay; growth erases with real sensors measured on the bench (up to 31 growths filling the ring, 476 Hz, no gaps; one 16 ms core 0 stall not yet located); worst-case timing in a real flight, watchdog and physical brownout coverage are not established. Ring growth/wrap-around/overwrite, 480 Hz load and stack depth measured on the bench on 19 November 2026 (see above): core 0 stack reduced to 5056 of 8192 bytes with a MSPLIM guard; USB traffic delays processing but loses no real IMU frames; one unexplained reboot |
| Desktop and transports | Six-view USB/SPP app, IMU calibration, orientation, reset/format and power management exist. Separate-Windows qualification, BLE and negotiated binary export remain open |

The [documentation map](../README.md#remaining-work) owns the consolidated
implementation/design backlog. These rows describe evidence limits, not another requirements list.
Host NOR byte cuts model persistence protocols; a restart after a completed recording is not a
physical interruption during program/erase. Normal-profile maxima are not worst-case guarantees.
