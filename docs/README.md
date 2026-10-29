# Documentation map

Start with the [project README](../README.md) to build and launch. `technical/` explains
current implementation, including the normal and bistage machines; `testing/` contains
repeatable procedures and dated evidence. Device wiring and electrical limits live in
[Hardware](HARDWARE.md). An implemented feature is not automatically qualified for flight.

## Technical references and ownership

Each detailed rule has one canonical owner. Other pages summarize and link to it; source
code remains the final reference when checking whether an implementation matches the text.

| Document | Owns | Does not duplicate |
|---|---|---|
| [Architecture](technical/ARCHITECTURE.md) | Source map, service responsibilities, runtime path, core and device ownership | Full transition tables, command grammar or binary layouts |
| [State machines](technical/state_macines.md) | Flight/arming transitions, normal/bistage selection, eight-sample median, tilt and latched ABORT | Estimator tuning, packet framing or GUI operation instructions |
| [Navigation](technical/NAVIGATION.md) | Units/reference frames, estimator parameters, freshness, gyro collection and mounting transform | Full flight transition table |
| [Configuration](technical/CONFIGURATION.md) | Rule semantics, draft/apply transactions, configuration commands, orientation/reset semantics | Storage byte offsets and desktop layout |
| [Airbrakes](technical/AIRBRAKES.md) | Continuous AND gate, Kalman/control contract, normalized servo travel, closure policy and airbrake CLI | Full storage offsets or a particular flight-control algorithm |
| [Storage behavior](technical/STORAGE.md) | Allocation, queue handoff, close, recovery, corruption and deletion/format contracts | Binary offsets or CLI tutorials |
| [Storage formats](technical/STORAGE_FORMAT.md) | Memory map, versioned fields, CRC/commit and frozen flight calibration metadata | Runtime scheduling or user workflows |
| [USB/Bluetooth](technical/USB.md) | Framing, retries, ownership, PC transport API, readiness and power commands | Rule-edit grammar or recording command tutorial |
| [Desktop](technical/DESKTOP.md) | Six views, local drafts, acknowledged UI state, source ownership and packaging | Hardware authorization or new firmware state-machine behavior |

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

## Remaining work and documentation gaps

The main implemented paths are covered above: configuration, transport, navigation, arming,
outputs, recording/recovery, desktop and testing. The following are unfinished work or missing
acceptance details, rather than undocumented completed features:

| Area | What still needs to be specified, implemented or verified | Owner |
|---|---|---|
| Bistage qualification | Normal/bistage selection, 3 g second-ignition detection, median/tilt ABORT and completion of active outputs are implemented. Qualify thresholds and initial attitude on repaired hardware; invalid-data windows inhibit new starts without inventing a timeout | [State machines](technical/state_macines.md#transizioni-e-casi-limite-implementati) |
| Attitude and calibration | Verify real initial vertical alignment for tilt, qualify repaired IMU cadence and gyro collection, define accelerometer calibration procedure/record/UI | [Navigation](technical/NAVIGATION.md#calibration-and-mounting), [Validation](testing/VALIDATION.md#remaining-work-and-limits) |
| Airbrake controller/application | Firmware contract and logging are implemented with a zero-output placeholder law. Implement/qualify the chosen law and verify endpoints on hardware; graphical controls and log plots are present | [Airbrakes](technical/AIRBRAKES.md) |
| Physical outputs | Bench acceptance for PWM/mechanical endpoints, continuity/firing current, pulse shutdown and brownout during recording; host tests alone do not establish these | [Hardware](HARDWARE.md), [Validation](testing/VALIDATION.md#remaining-work-and-limits) |
| Failure policy | Sensor outage/degraded estimates, watchdog/stalled worker limits, storage-driver timeout reinitialization and failed-arming reservation cleanup | [Navigation](technical/NAVIGATION.md#deliberate-adaptations-and-known-inherited-limits), [Storage](technical/STORAGE.md#failure-diagnostics-and-limits) |
| Future transport features | Direct channel tests, telemetry/monitor subscription and receiver-negotiated binary export; frame/handshake/resume rules are not implemented | [USB/Bluetooth](technical/USB.md#command-reference) |
| Release evidence | Reproducible physical flight acceptance, versioned board/firmware qualification and portable-app checks on a separate Windows machine; GUI firmware updating is not provided | [Test guide](testing/README.md), [Desktop](technical/DESKTOP.md#source-ownership-and-verification) |

The board pin table and datasheets are present; editable schematic/PCB release sources are not
included in this repository. Manufacturing/revision traceability would need those artifacts.
Do not infer them from the simplified board-orientation image.
