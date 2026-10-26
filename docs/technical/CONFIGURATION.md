# Configuration and recovery rules

`RecoveryConfiguration` is a fixed-array RAM model with a pure validator, not a wire/storage
struct. `ConfigurationService` owns applied/draft plans; rule triggers, rule delay timers,
readiness and action starts are separate. Each rule has one common delay, and each output
adds its own channel delay (zero by default).

## Model and validation

There are four pyro and four servo channels. Each physical channel belongs to at most one rule,
including within the same rule. All configured rules are active and once per run; there are no
enabled/repeat fields. Stable nonzero rule IDs define references; duplicate names are allowed.
Renaming/list order cannot change dependencies. Self/missing references, cycles, empty
conditions/outputs, invalid numbers and duplicate ownership are rejected. Rules may be saved
with assigned servos still unset. Physical arming requires configured endpoints, which may
run in either direction; missing setup is reported as `SERVOS_NOT_CONFIGURED` with a channel
mask. Virtual simulation does not require servo setup. Entirely unset settings may remain
zero; explicitly populated settings are validated. The empty model is invalid.

Altitude is launch-relative metres; speed means abs(vertical velocity), not total speed.
Altitude comparisons are strict, speed comparisons inclusive, and ranges include both endpoints
(equal bounds allowed). Phase masks contain BOOST/COAST for both stages and DESCENT; PAD, LANDED and ABORT cannot trigger rules. Runtime limits are
application policy, exposed through capabilities; they are development bounds, not recommended
flight settings. Validation identifies the first error, field, rule ID and zero-based index.
Names reject blank/control/ unterminated values; current USB/Bluetooth accepts ASCII rather than full
UTF-8.

## Rule evaluation

- Call `begin` once with a valid plan, explicit validation limits and a positive
  maximum input age. An active engine rejects replacement. Call `end` before a
  new flight; begin clears all runtime flags and timers.
- Pass monotonic 64-bit milliseconds to `evaluate`. The caller must extend a
  hardware timer across rollover. Backward time is rejected without mutation.
  Snapshot sequence numbers must increase within a flight.
- Phase comes from the navigation detector adapter, using the new explicit enum.
  It must be valid, current and accompanied by an acquisition timestamp. The
  first observed airborne phase latches launch; first DESCENT latches apogee,
  using their supplied phase timestamps. This is not an apogee detector. The
  adapter must deliver transitions promptly for accurate event-relative timing.
- PAD and LANDED never trigger rules. LANDED is latched until a new flight.
  All condition rows combine with AND. Rules evaluate prior trigger state,
  then commit together; zero-delay rule dependencies wait for the next evaluation.
- Each numeric estimate has validity, sequence and measurement time. Conditions
  requiring an estimate reject non-finite, repeated, future or stale values.
  Untriggered rules require fresh inputs; independent conditions do not require unused
  numeric inputs to be valid.
- The first evaluation where all conditions are true latches the rule trigger and
  schedules its outputs. Its delay then runs without rechecking those conditions,
  even if inputs become invalid, a gap occurs or conditions become false. A zero
  rule and channel delay starts an output in that evaluation. `since_rule_ms` counts from the latched
  trigger time, not from the later output start. This is not a persistence/debounce timer.
- Each assigned output starts at the first evaluation on or after the sum of the rule's
  delay and its own channel delay expires. Both delays default to zero and each is bounded by
  `max_timing_ms`; their sum is evaluated in 64 bits. `Evaluation.starts` returns all starts
  for that call (up to eight), labelled by rule/channel and actual logical start
  time. Zero is valid. Consume results immediately; persistent runtime action
  state also retains first start time. There is no physical command or pulse here.
- Previously scheduled starts can occur after LANDED within the lifecycle window. The owner
  must call `closeStarts` at the post-landing deadline or accepted manual disarm.
  It cancels pending starts and prevents further triggers/starts. `end` also
  closes starts. This component does not select the post-landing interval,
  implement physical movement completion, finalize storage or authorize disarm.

Re-reading an unchanged hardware register is not a new measurement and cannot newly trigger a
rule requiring that estimate. Already latched rule delays continue independently of acquisitions.

`RunController` closes pending starts after the development 15-second post-landing window;
EOF/manual stop can cancel them earlier. The engine has no actuator API.

## Save/load policy

`ConfigurationStore` holds one saved copy. Load checks version, integrity and current semantic
policy before assigning output; it never formats or installs defaults. An explicit disarmed
Apply validates/encodes in RAM, invalidates prior applied-plan readiness, erases only
`0x001000..0x001FFF`, writes/verifies pages and commit, then reloads through the boot path.
Supported payload plus metadata fits one sector; compile-time/runtime bounds enforce that. The
remaining sectors are not touched by rule Apply. Board orientation uses a separate sector
within that reservation; see [the memory map](STORAGE_FORMAT.md#board-mounting-record).

An interrupted replacement may lose the old copy. On failure the draft remains, but the old RAM
plan cannot silently reactivate. A storage fault disables further saves until reinitialization
(currently reboot). ConfigError values are None, Armed, InvalidConfiguration, Io, Missing,
UnsupportedVersion and Corrupt. These operations are synchronous and require exclusive idle
flash ownership.
See [the binary specification](STORAGE_FORMAT.md#configuration-package) for offsets.

## First device example

Upload with `-D MAIN` (already selected), open the monitor at 115200 and issue one line at a
time, waiting for each reply:

```text
status
capabilities
config show
config begin
rule add 10 "USB example"
pyro set 1 100
condition add 10 apogee
output assign 10 pyro 1
config validate
config apply
status
```

On a board with missing/invalid settings, begin creates an empty draft. If valid settings
already exist, begin copies them; use `rule list`/`rule show` to inspect them first rather than
re-adding the same ID. No active default plan is created.

Expected Apply fields: `"applied":true,"persisted":true,"verified":true`. Status then reports
valid configuration, but ready remains false. Reboot and run `config show`, `rule show 10`,
`pyro show 1`: saved settings should load again. Draft edits alone never change applied settings
or flash.

## Draft lifecycle

| Command | Behavior |
|---|---|
| config show | Applied-plan availability, store error and rule IDs |
| config begin | Start a draft from applied settings, or empty if no valid settings |
| config draft | Draft availability and rule IDs |
| config validate | Validate active draft, returning error/field/rule/index |
| config apply | Validate, save single copy, reload/verify, then activate |
| config discard | Discard the draft, leaving applied settings unchanged |
| capabilities | Firmware structural and numeric limits |

Mutations require disarmed state and an active draft. Begin rejects an existing draft; discard
explicitly before starting over. Validation failures leave flash and applied settings unchanged.
Once a flash save is attempted, failure invalidates the applied-plan flag because the single old
copy may have been destroyed; draft remains available. A storage fault disables further saves
until reinitialization (currently reboot). Missing/corrupt saved settings block validity but do
not prevent creating a replacement draft when the driver is healthy.

Disconnect discards the draft, not applied settings. Store errors are visible in status/config
show. The lifecycle owner supplies authoritative disarmed state; the router also rejects mutations
while a run or recording worker owns the device.

## Editing and inspection

Names with spaces must be quoted. Inside quotes, backslash escapes only quote and backslash. IDs
are stable nonzero unsigned 32-bit integers. Indices below are **zero-based** and shift after a
condition/output removal. Channel numbers are 1..4. All durations are integer
**milliseconds**, servo pulses **microseconds**, altitudes **metres relative to launch**, speeds
**abs(vz) in m/s**.

```text
rule list
rule add <id> "<name>"
rule show <id>
rule rename <id> "<name>"
rule delay <id> <ms>
rule remove <id>

pyro show <channel>
pyro continuity
pyro set <channel> <pulse_ms>
servo show <channel>
servo set <channel> <initial_us> <final_us>

condition add <rule_id> <type> [parameters]
condition replace <rule_id> <index> <type> [parameters]
condition remove <rule_id> <index>
condition show <rule_id> <index>

output assign <rule_id> pyro|servo <channel> [delay_ms]
output unassign <rule_id> pyro|servo <channel>
output show <rule_id> <index>
```

`pyro continuity` reads only assigned pyro channels and returns `configured_pyro_mask`,
`connected_pyro_mask`, `failed_pyro_mask` and `all_connected`. An open circuit is a successful
measurement with `all_connected:false`, not a command error. Reads are refused during active
runs, storage work or a configuration draft. The command does not move servos, fire outputs,
read sensors or approve arming; preflight and Arm always repeat the hardware measurement.

Each output has an additional delay (zero when omitted in `output assign`). `output show`
returns `delay_ms`; capabilities advertises `output_delays:true`. Configuration codec v2 stores
the rule delay and each channel's additional delay as integer milliseconds. Exported rule and
output snapshots both use `delay_ms`. Only this configuration format is supported.

Physical flight and physical simulation share `OutputExecutor`; virtual simulation reports
actions without hardware actuation. The pyro policy has a fixed 10 ms enable-settle interval followed by one
1000 ms pulse, then switches off and marks the channel fired. There is no retry sequence yet.
Assigning any pyro initializes its stored `pulse_ms` to the firmware policy automatically.
The GUI displays this fixed profile from capabilities and has no editable pyro-duration field;
its `Plan` no longer includes pyro settings. The raw development `pyro set` command remains for
metadata tooling and cannot change `PyroController` timing. Servo settings contain only the
configured flag and initial/final PWM. The executor commands the endpoints directly and holds
PWM until lifecycle shutdown.

`servo set` records explicitly supplied calibration endpoints only; it does not move or prepare
a servo and does not prove mechanical calibration. The [desktop editor](DESKTOP.md#configuration-and-servo-editor) uses `servo preview <channel> <pwm>`
for explicit positioning. `servo release <channel>` releases only
that positioning PWM; the editor sends it on closing. Positioning is disarmed/idle only, validates
PWM against firmware limits, works before endpoints are saved and never writes flash. It invalidates
preparation for that channel and blocks arming/simulation until released; owner disconnect also
releases previews. **Conferma** changes only the local draft.
`servo arm` prepares applied initial PWM; `arm` runs physical preflight. Direct single-channel
tests remain future work. Detailed show commands inspect the
draft if one is active, otherwise applied settings; every response labels its view. `config
show` always describes applied availability; it does not export every field at once. Use
detailed show commands for complete paged readback. Removing a referenced rule reports the
referencing ID. Assigning a channel already owned by a rule is rejected immediately.

Condition types:

| Type | Parameters |
|---|---|
| apogee | None |
| phase | ascent (102), descent (8), airborne (110), or numeric mask: BOOST 1/normal=2, COAST 1/normal=4, BOOST 2=32, COAST 2=64 |
| altitude | `<` or `>` then value |
| altitude_range | Inclusive lower, upper |
| speed | `<=` or `>=` then nonnegative value |
| speed_range | Inclusive nonnegative lower, upper |
| since_launch_ms | Minimum elapsed ms |
| since_apogee_ms | Minimum elapsed ms |
| since_rule_ms | Referenced rule ID, minimum elapsed ms |

Some edits can intentionally leave an invalid intermediate draft (for example a new rule without
outputs or conditions). Validate/Apply enforce all semantic requirements and cycle checks.
Numeric parsing rejects negatives for unsigned fields, overflow, non-finite floats and
out-of-range actuator values. Ranges and references are checked by full draft validation. There
is no arbitrary formula language. condition show reports the format's numeric type/comparison
codes.

Initial development policy: servo 1000..2000 us; fixed pyro pulse 1000 ms;
rule-delay/event timing up to 3600000 ms. These are exposed by capabilities, not
flight recommendations. Names allow 47 ASCII bytes; up to 8 rules, 8 conditions/rule and 8
assignments/rule, with at most 8 globally unique assigned physical channels. Future app limits
must come from capabilities, not independently chosen values.

For numbered requests, use increasing IDs and read `USB.md` for retry/session rules. Commands
outside the phone subset always reply in JSON, including when manually typed without an ID.

## Device orientation and factory reset

These commands operate on device settings separately from a rule draft:

| Command | Behavior |
|---|---|
| `config orientation` | Returns `up_axis` and `orientation_ready`; no saved orientation returns null/false |
| `config orientation +X` | Saves and verifies an explicit axis; accepts +X, -X, +Y, -Y, +Z, -Z |
| `config reset` | Clears applied rules, draft, servo endpoints and saved orientation; preserves flight recordings and the reserved accelerometer-calibration region |

Orientation writes require idle/disarmed state, no active draft and exclusive storage ownership.
Reset also requires idle/disarmed ownership. A rule save never supplies an implicit orientation.
Physical preflight reports `ORIENTATION_NOT_READY` until an axis is explicitly saved, including +X.
Reset acknowledges `reset`, `persisted` and `verified` only after erase/readback succeeds.
The separate state-machine setting is read with `config machine`, saved with
`config machine normal` or `config machine bistage <minimum_velocity_mps>`, and defaults to
normal after factory reset. See [state machines](state_macines.md#3-macchina-di-volo-bistadio--implementata).

`servo release` without a channel explicitly releases all held servo PWM while idle/disarmed;
with a channel it retains the editor-preview meaning described above. Idle `disarm` does not
release preparation. See [command lifecycle](USB.md#arming-and-readiness).

