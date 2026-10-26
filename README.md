# Perpetual flight computer

RP2350 / Arduino-Pico firmware inspired by `../recovery_system_avionics`. MAIN currently
provides USB/Bluetooth configuration, a shared flight runtime for sensor navigation/rules/recording,
sensor HIL, autonomous simulation with selectable outputs, idle hardware diagnostics and flash download.
**Real flight and physical simulation share the output executor. Real flight requires explicit
servo preparation and sensor/continuity/storage readiness; the IMU hardware issue is unresolved.**

The **Qt desktop prototype** provides Dashboard, Configuration, Test/Simulation,
Recorded Flights, Settings and Console. During development, launch it from source using
[the desktop guide](docs/technical/DESKTOP.md). Portable ZIP packaging is deferred until the app is ready.

## Build and start

Firmware dependencies are pinned in [platformio.ini](platformio.ini): exact Git commits
for the RP2350 platform and Bosch/ST drivers, and NeoPixel 1.15.5. The pinned platform
also selects a fixed Arduino-Pico framework commit and versioned toolchain archives.
Desktop dependencies are pinned in [perpetual_app/requirements.txt](perpetual_app/requirements.txt).
Update these references explicitly when upgrading dependencies.

```powershell
pio run
python tools/run_tests.py
python tools/run_tests.py --full
```

Select exactly one `build_flags` entry in [platformio.ini](platformio.ini), using
the existing environment: `MAIN` (application), `BLUETOOTH_TEST`, `BLUETOOTH_NAME_TEST`,
`FLASH_TEST`, `FLASH_DIRECTORY_TEST` or `CONFIG_STORE_TEST`. `MAIN` is currently selected;
the isolated radio tests remain available in [Bluetooth diagnostics](docs/testing/README.md#bluetooth-bring-up).
Uploading firmware does not execute the PC tests. The tests need g++ and Python. The default
run covers C++ and client checks; `--full` adds the repository Hoppino CSV replay and
record/download checks. Use `--suite bluetooth` (or another area from `--list`) for focused
work. Shared C++ objects are cached under ignored `.pio/tests`; `--rebuild` forces compilation.

Upload MAIN, open Serial Monitor at 115200 and send `help`, `status`, `sensors` or `config show`. Boot
loads saved settings, stays disarmed and never autoformats or installs an active default plan.
Valid configuration alone does not mean flight readiness. Close Serial Monitor before a Python
client opens the port.

MAIN also accepts the same commands through the JDY-34 (hardware UART1, 9600 baud).
USB stays active every loop. `bluetooth status` reports radio startup/link state;
`control status` identifies the requesting transport and the current edit/run owner.
Windows' paired-device badge is not the application connection check: open the outgoing SPP
port, then exchange `info`/`ping`. See the [shared protocol](docs/technical/USB.md).

The shared PC client now owns handshake, command ordering and idle heartbeat. To inspect the
device and watch actual replies (Ctrl+C closes the connection):

```powershell
python tools/device_client.py --port COM11 --watch
python tools/device_client.py --port COM9 --command "config show"
```

Replay/simulation/download runners and the Qt desktop use the same connection worker.
The command-line monitor remains useful for focused connection checks.
The official Hoppino input is included in [datasets/hoppino_VOLO_UFFICIALE.csv](datasets/hoppino_VOLO_UFFICIALE.csv);
replay tools use this copy by default.

```powershell
python tools/replay/replay_sensors_usb.py --port COM9
python tools/replay/run_simulation_usb.py --port COM9
```

These process simulated sensors through onboard navigation/rules with virtual outputs. For
optional recording, first initialize genuinely blank storage with `flights init`, then add
`--record`. Retained flights are never deleted automatically.
See the [recording guide](docs/testing/RECORDING.md) before storage diagnostics.

## Documentation

- [Documentation map and reading order](docs/README.md)
- [Architecture and source ownership](docs/technical/ARCHITECTURE.md)
- [Configuration and rule semantics](docs/technical/CONFIGURATION.md)
- [Navigation and calibration](docs/technical/NAVIGATION.md)
- [Flight state machines and bistage design](docs/technical/state_macines.md)
- [Storage behavior](docs/technical/STORAGE.md) and [binary formats](docs/technical/STORAGE_FORMAT.md)
- [USB protocol](docs/technical/USB.md)
- [Build/tests and device diagnostics](docs/testing/README.md)
- [Simulation and replay](docs/testing/SIMULATION.md)
- [Recording, download and timing](docs/testing/RECORDING.md)
- [Validation evidence and remaining work](docs/testing/VALIDATION.md)
- [Hardware and datasheets](docs/HARDWARE.md)
