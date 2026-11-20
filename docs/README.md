# Documentation map

Start with the [project README](../README.md) to build, flash and launch. To learn the desktop app,
follow the [app guide](APP_GUIDE.md). The rest of this folder is reference material:

- `technical/` explains the current implementation, including the normal and bistage machines.
- `testing/` contains repeatable procedures and dated evidence.
- [Hardware](HARDWARE.md) covers the pin map, board circuits, sensors, outputs, battery sense and
  datasheets. The
  schematics and PCB are in the [perpetual_pcb](https://github.com/rotellini9809/perpetual_pcb)
  repository.

An implemented feature is not automatically qualified for flight.

## Using the app

| Read | Purpose |
|---|---|
| [App guide](APP_GUIDE.md) | Step-by-step workflow with screenshots, from board setup to reading a flight |

## Technical references and ownership

Each detailed rule has one canonical owner. Other pages summarize and link to it; source
code remains the final reference when checking whether an implementation matches the text.

| Document | Owns | Does not duplicate |
|---|---|---|
| [Architecture](technical/ARCHITECTURE.md) | Source map, processing path, runs and arming, cores and flash, main loop and power | Transition tables, command grammar or binary layouts |
| [State machines](technical/STATE_MACHINES.md) | Normal and bistage flight machines, ABORT, machine selection and the arming machine | Estimator tuning, command replies or GUI instructions |
| [Navigation](technical/NAVIGATION.md) | Frames and units, input freshness, estimator parameters, IMU calibration (`imu` commands), gyro collection at arming and mounting | Flight transition table |
| [Configuration](technical/CONFIGURATION.md) | Rule model and timing, drafts, editing commands, device settings and factory reset | Storage byte offsets and desktop layout |
| [Airbrakes](technical/AIRBRAKES.md) | Control law interface, the continuous gate, closure policy, airbrake commands and logged fields | Storage offsets or a particular control law |
| [Storage behavior](technical/STORAGE.md) | Allocation, queue handoff, close, recovery, corruption and deletion/format contracts | Binary offsets or CLI tutorials |
| [Storage formats](technical/STORAGE_FORMAT.md) | Memory map, versioned fields, CRC/commit and frozen flight calibration metadata | Runtime scheduling or user workflows |
| [USB/Bluetooth](technical/USB.md) | Transports, framing and retries, ownership, PC client, command index, board state, power, arming/output commands and the Bluetooth module | Rule-edit grammar or recording commands |
| [Desktop](technical/DESKTOP.md) | App internals: views, local drafts, acknowledged UI state, source ownership and packaging | User instructions (see the app guide), hardware authorization or firmware state-machine behavior |

## Run and verify

| Read | Purpose |
|---|---|
| [Test guide](testing/README.md) | Host suites, fixtures, build flags and manual diagnostics |
| [Simulation/replay](testing/SIMULATION.md) | Estimate replay, sensor HIL and embedded Hoppino simulation |
| [Recording/download](testing/RECORDING.md) | Memory initialization/format, retained flight export and timing commands |
| [Validation](testing/VALIDATION.md) | Dated measurements, artifacts, fault-model evidence and physical limits |

Validation entries are historical observations, not a claim that every older measured artifact
uses today's format or behavior. Its final limits section describes current qualification gaps.
Do not rewrite a dated result to look as if it tested a later firmware revision.

## Remaining work

The main paths are implemented and documented above: configuration, transport, navigation,
arming, outputs, recording/recovery, desktop and testing. What is still open, from bistage
qualification to the airbrake law, the watchdog and flight acceptance, is listed in one place:
[Validation → Remaining work and limits](testing/VALIDATION.md#remaining-work-and-limits).
