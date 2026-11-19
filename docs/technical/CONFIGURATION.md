# Configuration and recovery rules

The configuration says when each output fires. It is a set of **rules**: each rule has
conditions, a delay and the outputs it drives. It also holds the servo end points and the
[airbrake](AIRBRAKES.md) gate. Device settings stored next to it (board orientation, state
machine, airbrake enable) are covered [below](#device-settings).

`RecoveryConfiguration` is the in-memory model with a pure validator, `RecoveryRuleEngine`
evaluates it during a run, and `ConfigurationService` owns the applied configuration and the
per-connection draft. None of them depends on hardware, transport or storage.

## Rules

- There are **4 pyro** and **4 servo** channels. A channel belongs to at most one rule.
- Up to **8 rules**, each with up to **8 conditions** and **8 outputs**. Names are up to 47 ASCII
  bytes; duplicates are allowed.
- Each rule has a stable non-zero ID used for references, so renaming or reordering never changes
  dependencies. The app hides IDs and shows names.
- Every rule is active and runs at most once per flight.
- A configuration with no rules and no airbrakes is invalid.

**Conditions** of a rule are combined with AND:

| Type | Parameters | Meaning |
|---|---|---|
| `apogee` | — | Apogee detected (first DESCENT) |
| `phase` | `ascent`, `descent`, `airborne` or a numeric mask | The rocket is in one of the phases |
| `altitude` | `<` or `>`, value | Altitude above the launch point, metres (strict comparison) |
| `altitude_range` | lower, upper | Altitude within the range, ends included |
| `speed` | `<=` or `>=`, value | Absolute vertical speed, m/s |
| `speed_range` | lower, upper | Absolute vertical speed within the range, ends included |
| `since_launch_ms` | ms | Time since launch |
| `since_apogee_ms` | ms | Time since apogee |
| `since_rule_ms` | rule ID, ms | Time since another rule triggered |

Phase masks: BOOST (or BOOST 1) = 2, COAST (or COAST 1) = 4, DESCENT = 8, BOOST 2 = 32,
COAST 2 = 64. `ascent` = 102, `descent` = 8, `airborne` = 110. PAD, LANDED and ABORT can never
trigger a rule.

**Timing.** A rule **triggers** at the first evaluation where all its conditions are true. From
then on its conditions are no longer checked: the rule delay runs even if they become false or the
data becomes invalid. Each output starts when the rule delay plus its own **channel delay** has
elapsed. Both delays default to zero and are at most `max_timing_ms` (3600000 ms). A
`since_rule_ms` condition counts from the referenced rule's trigger, not from its outputs.

**Outputs.** A pyro output fires one fixed pulse: 10 ms to settle the shared enable, then 1000 ms
on. A servo output moves from its initial to its final PWM position (1000–2000 µs) and holds it.
Servo end points can be saved later than the rule, but physical ARM is refused
(`SERVOS_NOT_CONFIGURED`) until every servo used by a rule has them. Virtual simulation does not
need them, except for airbrakes.

**Validation** rejects missing or self references, cycles between rules, empty conditions or
outputs, invalid numbers and channels used twice, and reports the first error with its field,
rule ID and index. An intermediate draft may be invalid while you edit it.

## How a rule is evaluated

During a run the engine receives the flight phase and the estimates from navigation, each with
its validity, sequence number and timestamp. A condition that needs an estimate ignores values
that are not finite, repeated, from the future or stale; re-reading an unchanged sensor is never
a new measurement. All rules are evaluated against the previous trigger state and then committed
together, so a dependency with zero delay starts on the next evaluation.

Launch is latched at the first airborne phase and apogee at the first DESCENT, using the phase
timestamps. Fifteen seconds after landing, or at an accepted DISARM, the run closes: pending
starts are cancelled and nothing new triggers. Output timing on the hardware belongs to
`OutputExecutor`; see [Architecture](ARCHITECTURE.md#runs-and-arming).

## Drafts

Edits never touch the applied configuration directly. They go into a **draft** owned by the
connection that opened it; a disconnect discards the draft, never the applied settings.

| Command | Effect |
|---|---|
| `config show` | Applied configuration availability, store error and rule IDs |
| `config begin` | Start a draft: a copy of the applied configuration, or empty if there is none |
| `config draft` | Draft availability and rule IDs |
| `config validate` | Validate the draft; returns the first error with field, rule and index |
| `config apply` | Validate, save, read back and verify, then make it the applied configuration |
| `config discard` | Drop the draft |
| `capabilities` | Firmware limits used by the validator and the app |

Edits require a disarmed board and an active draft; `config begin` refuses if a draft already
exists. Apply replies `"applied":true,"persisted":true,"verified":true` on success.

Rules, orientation, state machine and airbrake enable are separate records in the device
journal; each save writes a new copy beside the previous one. Boot loads and checks the newest
copy; missing or invalid settings leave the board without a configuration, and the board never
formats anything or installs a default. If a save fails after it started, the draft stays
available, the old plan is not silently reactivated, and further saves are disabled until reboot.
After the reboot an interrupted save falls back to the previous copy, while a complete copy
damaged later reports `Corrupt`. Load and save errors are `Missing`, `Corrupt`,
`UnsupportedVersion`, `InvalidConfiguration` and `Io`. Binary layout:
[Storage format](STORAGE_FORMAT.md#device-journal-v1-block-0).

## Editing commands

Quote names with spaces; inside quotes, backslash escapes only `"` and `\`. Indices are
zero-based and shift after a removal. Channels are 1–4. Durations are integer milliseconds,
servo positions microseconds, altitudes metres above launch, speeds absolute vertical m/s.

```text
rule list | add <id> "<name>" | show <id> | rename <id> "<name>" | delay <id> <ms> | remove <id>
condition add <rule_id> <type> [parameters]
condition replace <rule_id> <index> <type> [parameters]
condition remove|show <rule_id> <index>
output assign <rule_id> pyro|servo <channel> [delay_ms]
output unassign <rule_id> pyro|servo <channel>
output show <rule_id> <index>
servo set <channel> <initial_us> <final_us>
servo show <channel>
pyro show <channel>
pyro continuity
```

`show` commands read the draft if one is active, otherwise the applied configuration, and say
which. Replies are paged per rule, condition and output so each fits in one reply.

`servo set` only stores end points; it never moves a servo. To find end points, the app uses
`servo preview` (see [USB/Bluetooth](USB.md#arming-and-outputs)).

`pyro continuity` reads the pyros used by the applied rules and returns `configured_pyro_mask`,
`connected_pyro_mask`, `failed_pyro_mask` and `all_connected`. An open circuit is a valid result,
not an error. It is refused during runs, storage work or a draft, and never approves arming:
`preflight` and `arm` always measure again.

**Example**, typed in a serial monitor one line at a time:

```text
config begin
rule add 10 "DROGUE"
condition add 10 apogee
output assign 10 pyro 1
config validate
config apply
```

## Device settings

These settings are separate from the rule draft. Changing them requires a disarmed, idle board
and no active draft.

| Command | Effect |
|---|---|
| `config orientation` | Read `up_axis` and `orientation_ready` |
| `config orientation <axis>` | Save the sensor axis pointing to the rocket nose: `+X`, `-X`, `+Y`, `-Y`, `+Z` or `-Z` |
| `config machine …` | Normal or bistage flight machine; see [State machines](STATE_MACHINES.md#machine-selection) |
| `config airbrakes [on\|off]` | Airbrake enable; see [Airbrakes](AIRBRAKES.md) |
| `config reset` | Factory reset: clears rules, draft, servo end points, orientation, the machine selection (back to normal) and the airbrake enable (back to off). Keeps recorded flights and the IMU calibration |

The orientation has no default: physical preflight reports `ORIENTATION_NOT_READY` until an axis
is saved explicitly, even `+X`. Its meaning is defined in
[Navigation](NAVIGATION.md#calibration-and-mounting). Reset replies `reset`, `persisted` and
`verified` only after the erase has been read back.

## Airbrakes

The configuration also holds one continuous AND gate and the set of servo channels reserved for
airbrakes. Reserved channels cannot be used by ordinary rules. A configuration with only
airbrakes is valid. Airbrake channels need distinct calibrated closed and open positions in every
mode, including virtual simulation. Everything else is in [Airbrakes](AIRBRAKES.md).
