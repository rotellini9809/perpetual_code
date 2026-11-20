# Build, test and device diagnostics

Use one level of checking at a time. While editing, run a known area with `--suite`
or use `--quick` to select affected host tests. Finish with one `--changed` run;
reserve `--full --desktop` for releases or changes spanning the whole system.
Do not follow a successful full run with the same unchanged focused suites.
`--plan` previews any selection without compiling, launching Qt or running tests.

`--quick` deliberately excludes Qt and recorded-flight integrations. It is an
intermediate check, not completion of UI or hardware validation. It accepts the
same optional Git reference as `--changed`. No passing result is cached: an
explicitly requested suite always runs, including fault and actuator scenarios.

Host C++ tests use the same compiler generation and language level as the firmware
(Arduino-Pico: GCC 14, `gnu++17`; host: ISO C++17, `-Werror -pedantic`, static runtime).
Once per Windows PC, install the pinned WinLibs GCC 14.2.0 (SHA-256 checked, per user,
no administrator rights or PATH changes) into
`%LOCALAPPDATA%\Programs\WinLibs\14.2.0posix-12.0.0-ucrt-r3`:

```powershell
python tools/setup_host_compiler.py
```

`run_tests.py` and `replay_navigation.py` then use it automatically and print the
compiler banner; without it they fall back to `g++` from PATH with a warning (the
PlatformIO-bundled GCC 5.1 lacks parts of C++17). `--compiler` overrides both.

From the project root:

```powershell
python tools/run_tests.py --quick               # Editing: affected host tests, no Qt/long replay
python tools/run_tests.py --changed --plan       # Preview the final selection, without running it
python tools/run_tests.py --changed             # Finish: affected suites and desktop areas
python tools/run_tests.py --full --desktop       # Release: all host, recorded-flight and Qt checks
python tools/run_tests.py                       # All standard host checks (legacy default)
python tools/run_tests.py --suite bluetooth     # One area
python tools/run_tests.py --suite storage_recording storage_recovery
python tools/run_tests.py --full                # Also replay, record and download all Hoppino samples
python tools/run_tests.py --list
python tools/run_tests.py --suite desktop        # Optional: requires perpetual_app/build.ps1 environment
python tools/run_tests.py --area recorded_flights maintenance   # Only these desktop areas
python tools/run_tests.py --timeout 300         # Per-run limit in seconds (default 120)
python tools/run_tests.py --changed             # Only what uncommitted/new files affect (or --changed REF)
```

`--changed` selects from git: C++ suites through the headers recorded by the last build
(`g++ -MMD`), Python suites on any tool/test-script change, and desktop areas through
`.pio/tests/desktop_map.json`, which every desktop run refreshes with the project files each
area executed. Anything it cannot prove unaffected runs (no map yet, a new area, an app file no
area has run, the test runner itself). Files no host test exercises are listed so they can be
checked on the board.

Every run has a time limit. A Python suite still running after `--timeout` seconds prints the
stack of every thread, with the exact line where it was stuck, and is reported as `TIMEOUT`
(a C++ executable is stopped 15 s later without a stack).

The runner compiles/runs PC executables using g++ and Python. Uploading firmware does not run
these tests. The default **12 groups** need no flight CSV; `--full` runs **14**, adding
`recorded_replay` and `recording` with the sibling Hoppino CSV from [Simulation](SIMULATION.md).
An explicit `--suite recording` also runs that recorded-data group. The compatibility option
`--skip-recorded-data` excludes these two groups even with `--full`.
The Qt area is opt-in with `--suite desktop` or `--desktop` (combine with `--full` for 15 groups).
It uses the isolated desktop Python, keeping default checks free of Qt dependencies. Its native
firmware fixture never opens a real COM port.
The desktop group is split into independent areas, listed in
[desktop_test.py](../../test/integration/desktop_test.py) (`--list`). Each area runs in its own
process with a fresh window and firmware fixture, and is reported as `desktop/<area>`; shared
fixture code lives in `desktop_support.py`. To debug one area without the runner:
`.venv\Scripts\python.exe test/integration/desktop_test.py <area>`.

| C++ file/group | Scenarios kept together |
|---|---|
| `bluetooth` | AT/name transactions, nonblocking startup, link notifications, reconnect/reset/overflow |
| `configuration` | Plan validation, rule evaluation and draft/apply service |
| `navigation` | Kalman/attitude/phases, preflight calibration and hardware sensor source |
| `flight` | Run ownership/protocol values, shared runtime, arming and retention |
| `storage_format` | CRC/endian image, verified writes/timing, summaries and sample blocks |
| `storage_metadata` | Configuration persistence, flight headers/starts and directory A/B |
| `storage_recording` | Recorder lifecycle, allocation and asynchronous worker queue |
| `storage_recovery` | Interrupted writes, deletion/reuse, committed corruption and quarantine |
| `commands` | Shared framing/handlers, independent transport buffers/IDs, ownership and disconnect |
| `simulation` | HIL lifecycle/calibration/timing and autonomous simulation |

Python groups are `clients` (shared connection/protocol, pacing, simulation/Bluetooth clients and build-cache behavior),
`replay` (synthetic estimate CSV), `recorded_replay` (navigation/HIL/estimate client against
Hoppino), and `recording` (synchronous/queued USB recording, real downloader and corruption).
The old hand-wired HIL-to-recorder replay was removed; its raw sensor, pressure and orientation
checks now run through the actual recording/download path. Low-level power-cut models stay
local to their scenarios because their different failure rules are intentional.

C++ sources compile once per changed source/header/compiler/flags, shared across executables.
Up to four compiler processes run in parallel (`--jobs 1` selects sequential builds).
`--rebuild` ignores the cache. Navigation replay uses the same builder, so Python checks do not
keep recompiling its C++ sources. All objects, executables and logs live under `.pio/tests`.
Default output gives one result/time per group; `--verbose` shows individual scenarios, and
failed groups always print their output. Full logs are retained in `.pio/tests/logs`.

The Python clients use `test/fixtures/command_device.cpp`, a PC stdin/stdout endpoint through
the production command router and simulated NOR. C++ test files contain assertions only,
without hidden `--stdio` modes. No automated suite uploads firmware or commands hardware;
manual diagnostics below use the one existing PlatformIO environment.

The `clients` group includes fragmented/invalid/late replies, structured command errors,
unusable sessions after ambiguous failures, handshake identity/version, queued ordering and
cancellation, idle heartbeats, long commands without heartbeat interference, bounded queue
shutdown, explicit reconnect and cancellation while an OS port is opening. These use fake byte
links; recorded groups still exercise the shared protocol through the C++ firmware router.

To observe a real MAIN connection without applying settings:

```powershell
python tools/device_client.py --port COMx --watch 10    # USB
python tools/device_client.py --port COMy --watch 10    # outgoing Bluetooth SPP
```

Close other programs using that port first. These explicit hardware checks show handshake,
transport identity, recent-reply age and background heartbeat state; Ctrl+C closes the session.
The [client lifecycle/API](../technical/USB.md#connecting-from-a-pc) is shared by
the normal replay/download runners and will back the desktop UI.

## PlatformIO test evaluation

The ten current C++ suites were also run with `pio test` in an ignored, PC-only native
prototype. All passed; an intentional failing executable correctly failed the run. Keeping
the current `assert` tests required a small
[custom runner](https://docs.platformio.org/en/stable/advanced/unit-testing/frameworks/custom/runner.html)
and [one test directory per suite](https://docs.platformio.org/en/stable/advanced/unit-testing/structure/hierarchy.html).
The prototype produced JUnit reports, but Python/client integrations still need their own
execution and compiled PC fixtures.

For this project, retain the single Python entry point above: measured cached C++ execution
was 1.17 seconds here versus 10.43 seconds through the native PlatformIO prototype. The
prototype remains under `.pio/pio_compact_eval`; no firmware environment or build flag was
added/changed. The dated [validation record](VALIDATION.md) contains comparable timings and
their limits. Revisit PlatformIO if target-side framework tests or its reporting become a
requirement.

## Coverage

| Area | Principal checks |
|---|---|
| Configuration/rules | Capacity/bounds, IDs/references/cycles, shared validation, immutable rejected drafts, fresh condition triggering, latched rule delays/cancellation and simultaneous output starts |
| USB/lifecycle | Framing, bounded replies, ID conflicts/retries, readback/apply/boot, run identity/locking, disconnect, paused close/abort and late persistence failures |
| Navigation/sources | Quaternion/Kalman behavior, phase logic, actual dt, invalid/repeated/stale inputs, raw/provider equivalence, bias validation/session isolation, six explicitly selected signed mounting axes, calibrated-BODY bypass and mailbox bounds |
| Simulation/clients | Absolute deadlines/rollover, no catch-up burst, skipped ticks, PC pacing, exclusive outputs, virtual-only source path and retained completed reports |
| Shared flight runtime | Direct hardware-source versus HIL navigation/rules, raw sensor recording provenance, virtual starts, exclusive ownership and storage-fault isolation |
| Shared arming | Missing-IMU/no-write rejection, complete blocker masks, immediate simulation, fresh physical calibration, configuration reservation, USB disconnect, cancellation, epoch reset and airborne disarm rejection |
| Flight retention | Physical PAD discard through the shared runtime/worker, retained diagnostic PAD logs, unsaved BOOST with a full queue, full PAD extent discard, uncertain interruption retention and 513 byte cuts during discard |
| Hardware source | PC fake acquisition device: missing IMU with healthy barometer, independent freshness/IDs, invalid measurements and timer rollover; USB stale/absent data rendering |
| Bluetooth diagnostic | Bounded ASCII lines, rejected setters/injected AT commands, exact expected replies, UART rates and independent multi-channel connection snapshots |
| Storage primitives | CRC known vector/incremental splits, little-endian widths, unchanged v2 image, bounds/alignment, erased ownership and typed failures |
| Storage transactions | Header 769, start slot 513, sample block 1025, summary 513, recorder close 2049, directory 2082 and configuration 773 interruption cases |
| Retention/corruption | 513 deletion cuts, 160 identities/reuse/high-water, 513 quarantine cuts, healthy downloads, occupied corrupt extents, remount/compaction and no silent repair |
| Hoppino 2..35 s excerpt | 6600 frames through references/HIL, synchronous and queued recording, fresh decoding, frozen config/temperature and three fixture starts |

Case counts describe the current tests, not independent hardware power failures. The standard
CRC vector is `123456789`; the format fixture hashes to `0xb94a27eca6844a2c`. Quarantine tests
distinguish normal incomplete tails from committed corruption, I/O failures, unsupported headers
and damaged directory
metadata. [Validation](VALIDATION.md) records observed results and their limits.

## Manual firmware modes

Choose exactly one `build_flags` line in `platformio.ini`, build/upload, then open Serial
Monitor at 115200. Restore `MAIN` afterward. Diagnostics hold outputs inactive except for
the explicit physical `F` command in `SMOKE_TEST`; RAM
backup/restore is **not power-loss-safe**. Prefer PC tests while retained flights occupy the
device. On restore failure, do not reboot: repeat mutation tests are blocked to preserve the RAM
backup.

| Flag | Start/repeat | Writes and expected completion |
|---|---|---|
| MAIN | `help`, `status` | Development application; commands determine persistence |
| SMOKE_TEST | Checks/live report on USB connection; `D` repeat init/checks, `F` physical pyro 1 once per boot, `H` help | Current sensors/flash identification, live measurements and continuity CH1..4; no flash/NVM writes or servo motion; F uses the shared executor's 10 ms enable settling and 1000 ms pulse |
| SENSOR_TEST | Automatic scan on USB connection; `r` repeat, `i` init, `s` status, `v` live, `b` barometer checks, `f` SPI speed sweep 1-37.5 MHz, `o` 10 s IMU output-rate check, `h` help | Hardware SPI1 raw ID/status scan then normal sensor initialization; volatile sensor configuration / IMU reset only, flash/NVM untouched |
| BLUETOOTH_TEST | USB `status`, `at AT`; phone `ping` | UART probe/snapshot/echo; explicit `name set` changes module names; external flash untouched |
| BLUETOOTH_NAME_TEST | `probe`, `name set "Perpetual"`, `module reboot` | Raw UART/name test; rename uses module memory; explicit reboot drops Bluetooth links; outputs inactive |
| FLASH_TEST | ID sweep on connection; `r` repeats; `w` starts sector test; `c` sweeps SPI clocks 4-75 MHz (read-only region CRC with 0x03/0x0B, one page programmed per clock in the backed-up test sector, then restored); exact `ERASE ALL` erases the whole device | `w` backs up/restores `0xFFF000..0xFFFFFF`; `ERASE ALL` destroys and verifies all 16 MiB, including settings and flights; `FULL FLASH ERASE VERIFIED` |
| FLASH_DIRECTORY_TEST | `d` | Backs up both directory banks `0x010000..0x02FFFF` (128 KiB); `DIRECTORY TEST PASSED` |
| CONFIG_STORE_TEST | `c` | Backs up the device journal `0x000000..0x00FFFF` (64 KiB: rules, settings, calibration); `CONFIGURATION STORE TEST PASSED` |

`SMOKE_TEST` preserves the board-level check from remote commit `e65ce64`, adapted to current
drivers and selected in the existing PlatformIO environment. Flash identification accepts the
fixed W25Q128JV only; it is not a memory-integrity test. Idle sensor sampling reports once per
second, with freshness checks and no fabricated measurements for a missing sensor. The LED is blue
during initialization, then green only when flash identification and both fresh sensor readings
pass; otherwise red. Continuity results are separate from that LED result.

`F` commands the real pyro 1 once per MCU boot, independently of flight readiness/continuity.
Only a complete standalone command line is accepted (LF/CRLF); letters inside `info`, other
commands, oversized lines or arguments never fire a channel. Framing uses `CommandService`.
It reuses `flight::OutputExecutor` and the current board adapter, not a second pulse timer.
During the pulse the loop only services output timing and discards input: no serial output,
sensor/continuity reads or diagnostic reinitialization. Shutdown continues after USB disconnect;
reconnect or `D` never clears the once-per-boot latch. Servo PWM stays detached, Bluetooth/flight
services are not running, and flash settings/flights remain untouched. `D` retries normal
sensor/flash initialization and prints identities/errors. Detailed raw register/mode/speed
sweeps remain in `SENSOR_TEST` rather than being duplicated.

`SENSOR_TEST` uses GP12 MISO, GP14 SCK, GP15 MOSI, GP26 barometer CS and GP27 IMU CS.
It compares five independent ID reads at 100/500/1000 kHz in modes 0/3, with unselected CS
driven high or released with pull-up as in the current driver. Expected IDs are BMP5 50/51
(this board previously 50) and LSM6DSV32X 70. A separate first read is shown because BMP5
may initially use that transaction to select SPI; there is no extra dummy byte per read.
Before driver initialization, snapshots show barometer ID/revision/chip status/NVM status/
OSR/ODR and IMU identity/control/status registers. Three ID reads per MISO pull setting
(none/up/down) help detect undriven or selection-sensitive responses; pulls are then disabled.
Raw probing and the normal `Sensors` driver share hardware SPI1 sequentially on core 0.

BMP581 warm-start regression: a MCU-only reset can leave the powered barometer measuring
(`ODR_CONFIG=0xBD`, normal mode), with `STATUS=0x01`: core ready, NVM not ready, no NVM error.
The Bosch `bmp5_init()` NVM check then returns -5. `Sensors::beginBaro()` now identifies the
sensor and switches to standby before calling that check, using the Bosch transition delay.
On this board, plain init failed and standby-then-init succeeded in all three paired trials;
after the fix, ten consecutive `i` reinitializations each produced valid pressure samples.
Two subsequent MAIN uploads/reboots without removing power also passed, with 20 valid pressure
readings after each boot; recording storage mounted and its nine directory entries were retained.
To repeat the check, use `i` followed by `v`; barometer readiness must survive reinitialization
without removing power. This does not bypass NVM validation.
See [Bosch BMP581 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf),
sections 4.3.7, 4.3.9 and 7.22.

The scan never writes sensor registers. The following normal driver initialization configures
volatile registers and may reset the IMU; it does not write sensor NVM or flight/configuration
flash. `i` retries only the driver; `r` repeats the scan and driver initialization. `v` toggles
two live reports per second with readiness, validity, new-sample counts, ages, driver errors,
pressure/temperature and acceleration/angular rate. No samples are invented for a failed sensor.
The test waits for USB before probing and repeats its report on reconnect. `MAIN` protocol and
Bluetooth service are unavailable in this diagnostic; switch the build flag back after testing.

BMP5 error **-5 is NVM_NOT_READY**, not a distinct no-device/timeout verdict. The driver checks
STATUS before storing its public chip ID, so ID=00 with -5 must be compared with the raw ID
and NVM_RDY/NVM_ERR bits. Bad bus bytes can also fail that check. Error 0 from these SPI
callbacks only confirms transfers completed, not an acknowledgment from the device. Stable
valid raw IDs with failed initialization differ from absent/unstable IDs; neither a match nor
00/FF alone proves hardware health or a specific solder/power fault. Compare both sensors and
settings before deciding whether to inspect shared SPI/power, chip select or individual joints.
The NVM check follows the [Bosch driver](https://github.com/boschsensortec/BMP5_SensorAPI/blob/master/bmp5.c).

FLASH_TEST reads eight raw JEDEC bytes three times for modes 0/3 at 100 kHz, 500 kHz, 1 MHz and
board 4 MHz (24 hardware reads), plus three test-only software SPI mode-0 reads near 50 kHz.
Sensor selects and outputs stay inactive. Identification uses wake/JEDEC/SR1/legacy-ID only,
with no write-enable, program, erase, status-write or reset. Expected JEDEC is EF4018/EF7018;
legacy EF17 is a different command's ID. Matching IDs prove communication/identity, not cell
integrity. If only hardware SPI fails inspect peripheral/pins/timing; if both fail inspect
supply/soldering/ wiring/control pins before classifying the chip as broken.

`w` first captures and double-reads the sector, checks bounds/alignment, erase, unaligned
multi-page pseudorandom program/full readback and forbidden 0-to-1 changes, then
erases/restores/verifies the original sector. Power loss can destroy its original contents. No
whole-chip erase exists.

The directory diagnostic tests append, compaction, remount and invalidation, then reports
`Original directory regions RESTORED and VERIFIED.` Sample extents in its fixtures are metadata
only and are not accessed. The configuration diagnostic checks armed/invalid-plan no-write
refusal and replacement/reload, then reports `Original configuration region RESTORED and
VERIFIED.` Calibration and flight data remain outside those two diagnostics' write regions.

For live usage rather than destructive diagnostics, follow [Simulation](SIMULATION.md)
and [Recording/download](RECORDING.md). Build/host checks do not qualify electrical
brownouts, physical sensors/actuators or worst-case scheduling.

## Bluetooth diagnostics

In normal use the JDY-34 needs no procedure: MAIN configures a factory module at boot and the app
renames it from **Settings** ([Bluetooth module](../technical/USB.md#bluetooth-module)). The
commands below are for checking the radio link or recovering a module. In them, `COMx` is the
board's USB port and `COMy` the **outgoing** Bluetooth SPP port that Windows created when you
paired the board. Port numbers differ between computers; when in doubt, match the MAC reported
by `at AT+LADDR` to the paired device.

**Radio link with MAIN.** With MAIN uploaded and both ports closed in other programs:

```powershell
python tools/run_bluetooth_test.py --main --port COMx --radio-port COMy
python tools/replay/run_simulation_usb.py --port COMy
```

The first checks the protocol and IDs over both links, a RAM-only draft, rejection of a change
from the other link, and disconnect/reconnect of each link; it never saves settings. The second
runs the onboard simulation over Bluetooth. Both are manual hardware checks, separate from
`run_tests.py`.

**Module recovery (`-D BLUETOOTH_NAME_TEST`).** Use it when a module does not answer MAIN, for
example after being left at another baud rate. MAIN expects **230400 baud, 8N1, CRLF**, MCU TX
GP4 / RX GP5, PWRC GP17 low for AT mode. The diagnostic starts at 9600 baud:

```powershell
python tools/run_bluetooth_name_test.py --port COMx            # read-only probe
python tools/run_bluetooth_name_test.py --port COMx --scan     # search rates, control levels, line endings
python tools/run_bluetooth_name_test.py --port COMx --name "Perpetual"
python tools/run_bluetooth_name_test.py --port COMx --reboot   # AT+RESET, then read the names again
```

Only `--name` changes the module; success requires an ACK and an exact read-back of both the SPP
and BLE names (`NAME_RESULT verified=1`). Setters are never retried. Every byte sent and received
is shown and a transcript is saved under `.pio`. From a serial terminal at 115200: `status`,
`probe`, `baud 230400`, `at AT`, `at AT+VERSION`, `at AT+NAME`, `at AT+NAMB`, `at AT+LADDR`,
`at AT+STAT`.

**Raw UART and wireless echo (`-D BLUETOOTH_TEST`).** Probes the module at the usual rates with
read-only AT commands, then echoes over the radio: from a phone Bluetooth terminal, `ping`
answers `PONG` and `echo hello` answers `ECHO hello`; `send hello` from USB arrives on the phone.

```powershell
python tools/run_bluetooth_test.py --port COMx --radio-port COMy
```

Neither diagnostic initializes sensors, storage or the flight runtime; pyro, enable and servo
pins stay low. Select `MAIN` again and upload it when finished. Historical bring-up results are in
[Validation](VALIDATION.md).

## Live sensor acquisition timing

With MAIN loaded and the board awake and disarmed, run:

```powershell
.venv/Scripts/python.exe tools/measure_live_sensors.py --port COMx --seconds 30 --output .pio/cpu-live/result.json
```

This read-only measurement requires no arming or output commands. `sensors` exposes
`idle_poll` (poll count, cumulative acquisition microseconds, maximum acquisition
microseconds since boot) and `idle_samples` (valid fresh IMU and barometer counts).
Counters cover idle acquisition only; the cumulative microsecond counter wraps at
32 bits. The script reports counter deltas over board uptime, not USB snapshot rate.
The acquisition wall-time percentage excludes navigation, recording, and other loop
work. It is not total CPU utilization or proof of zero sensor overruns. Sample validity
is checked at USB snapshot times only. Flight/simulation load needs a separate test.
