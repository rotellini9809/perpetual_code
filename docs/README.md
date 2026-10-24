# Documentation map

The [project README](../README.md) is the quick start. Component documents describe
implemented behavior; guides contain commands; validation records evidence and
limits. [Target requirements](design/REQUIREMENTS.md) describe the planned product.

## Understand the code

| Read | Contents |
|---|---|
| [Architecture](components/ARCHITECTURE.md) | Source map, service ownership, sensor pipeline and core handoff |
| [Configuration](components/CONFIGURATION.md) | Plan model, rules, validation, draft/apply workflow and edit commands |
| [Navigation](components/NAVIGATION.md) | Frames/units, estimator/detector parameters, calibration and inherited limits |
| [Storage](components/STORAGE.md) | Allocation, recording, queues, recovery, corruption, deletion and diagnostics |
| [Storage formats](components/STORAGE_FORMAT.md) | Fixed memory map, binary offsets, CRCs and commit records |
| [USB/Bluetooth protocol](components/USB.md) | Framing, IDs, replies, ownership and transport limits |
| [Qt desktop](components/DESKTOP.md) | Portable Windows prototype, five views, local drafts, build and GUI verification |

## Run and verify

| Read | Contents |
|---|---|
| [Test guide](testing/README.md) | PC suite, coverage and manual device diagnostics |
| [Simulation/replay](testing/SIMULATION.md) | Estimate replay, sensor HIL, autonomous simulation and their differences |
| [Recording/download](testing/RECORDING.md) | Storage setup, recording, export, progress/failures and timing scopes |
| [Validation](testing/VALIDATION.md) | Software results, dated hardware measurements, artifact provenance and next work |

## Hardware and planned behavior

- [Hardware](HARDWARE.md): current pins/chips, driver limits, supplied PDFs and legacy-reference caveats.
- [Requirements](design/REQUIREMENTS.md): PC/phone UI, preparation/arming/tests,
  landing/retention, Bluetooth, binary export, physical simulation and sleep policies.

Each subject has one canonical reference. Add new observations to Validation; update the
relevant behavior/format/guide instead of appending another implementation report.
