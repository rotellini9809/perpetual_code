# Continuous servo airbrake control

The firmware provides an objective-independent control contract. The application-specific
law belongs in [`src/control/AirbrakeLaw.cpp`](../../src/control/AirbrakeLaw.cpp).
The supplied law returns zero on every channel: there is no built-in apogee target,
gain tuning or automatic opening algorithm. Increment `AirbrakeLawVersion` in
[`AirbrakeLaw.h`](../../include/control/AirbrakeLaw.h) whenever the compiled law changes.
This identifier is stored at recording startup; it is not a firmware hash.

## Inputs and outputs

`updateAirbrakes(const ControlInput &, LawState &)` receives:

| Input | Meaning |
|---|---|
| `state[0..2]` | Kalman launch-relative altitude (m), vertical velocity (m/s), vertical acceleration bias (m/s²) |
| `covariance[3][3]` | Complete covariance of those same three states after the current prediction/corrections, including cross terms |
| `acceleration` | Net vertical acceleration used by that prediction (m/s²), after gravity/bias compensation |
| `timeUs`, `dtSeconds` | Source timestamp from the run origin and accepted integration interval |
| `phase` | The selected normal/bistage detector's current phase |
| `valid` | True when the wrapper permits a call; invalid estimates close outputs without calling the law |

Acceleration is derived, not a fourth Kalman state. The 3×3 covariance does not pretend
to contain a separately estimated acceleration variance. There is no covariance threshold
invented by the wrapper: the law may use uncertainty to implement its own policy.

The return value has four normalized `opening` values and a `valid` flag. Index zero maps
to SERVO 1. Only assigned channels are consumed. Finite values are clamped to [0,1]
and quantized to 0.0001; NaN/infinity or `valid=false` on a consumed output closes **all**
airbrakes for that frame. Zero means calibrated closed; one means calibrated open.
There is no PWM output in the law contract. `OutputExecutor` alone converts normalized
travel to a microsecond pulse between the existing initial/final endpoints, including
reversed endpoints. Logs describe commanded travel, not measured position: servos have
no feedback, and pulse rounding can reduce physical resolution.

`LawState` supplies bounded controller memory. It resets on every run and whenever the
gate disables control, so the next enabled frame starts with clear memory. Extend this
type for a particular controller. The law runs synchronously in the navigation loop;
keep it bounded and avoid blocking I/O, delays, heap allocation and hardware calls.
The dedicated API is a code boundary, not a memory-protected sandbox for arbitrary C++.

## Continuous rule and ownership

The applied configuration contains one `AirbrakeRule`: a four-bit servo channel mask
and up to eight conditions combined exclusively with AND. Mask zero leaves no dedicated channels. A separate persistent device setting enables
control; it defaults to off, including after factory reset. Disabling control retains
channel assignments, conditions and endpoint calibration. An enabled rule needs at least one condition and
distinct calibrated closed/open endpoints for every selected channel. Airbrake-only
plans are valid. Selected servo channels cannot also belong to ordinary once-only rules;
pyro assignments remain independent.

The rule uses the existing condition grammar, launch/apogee timers and ordinary rule IDs.
It is evaluated after ordinary rules on each accepted sensor frame; `since_rule_ms`
can therefore observe a newly committed ordinary trigger in that frame. There is no
latch, rule delay, output delay or pulse duration for the airbrake gate. It can enable,
disable and enable again as the conditions change. PAD and LANDED never enable control.
As with normal rules, altitude is launch-relative and speed conditions use `abs(vz)`.

Control requires the saved enable setting, an airborne phase, all conditions true, navigation permission,
a valid finite Kalman state/acceleration/covariance and `0 < dt <= 0.1 s`.
Covariance diagonal entries must be nonnegative. Invalid estimates, false conditions,
navigation inhibition and invalid law output command zero immediately on that frame.
Valid estimation and a true rule may re-enable control. ABORT latches inhibition for
the remainder of the run even if a later caller reports another phase.

While a run is polled, more than 100 ms without a processed frame also closes airbrakes
using the cooperative wall clock, including when no new sensor frame arrives. This
is not an independent hardware watchdog if the main loop itself blocks. Normal physical
flight requires explicit servo preparation; physical simulation prepares at startup.
Virtual simulation runs the same controller and records commands without touching PWM.
Configuration changes are blocked for the entire run.

At stop/disarm the executor commands calibrated closed and retains PWM as idle prepared
servos. `servo release` explicitly releases it when idle. Zero is an active closed
command, not detach. A failed update write triggers best-effort closure on every
assigned channel and terminates the run with `OUTPUT_ERROR`; a failed close cannot
guarantee mechanical closure or certify preparation. Existing active pyro pulses in
ABORT retain their established completion behavior.

## Configuration commands

All edits share the usual `config begin` / `config validate` / `config apply` transaction,
transport ownership and in-flight locks. Detailed reads select the current draft if present.

| Command | Behavior |
|---|---|
| `config airbrakes [on/off]` | Read/save the general enable setting; requires idle/disarmed/no draft |
| `airbrake status` | Passive bounded live snapshot: enable/run/output mode, flags, failed conditions and commanded percentages |
| `airbrake show` | View, `channel_mask`, condition count and `logic:"and"` |
| `airbrake show <index>` | One condition, using ordinary condition JSON fields |
| `airbrake channels <mask>` | Reserve bits 0..3; zero disables control and retains conditions |
| `airbrake add <condition>` | Append one condition |
| `airbrake replace <index> <condition>` | Replace one condition |
| `airbrake remove <index>` | Remove one condition, with following indices compacted |

Indices are zero-based. Conditions referencing an ordinary rule prevent that rule's
removal until the reference is removed. `capabilities` advertises `airbrakes:true`.
The following example reserves SERVO 3, uses its calibrated travel, and permits control
only in COAST 1/normal COAST or COAST 2 above 20 m:

```text
config begin
servo set 3 1100 1900
airbrake channels 4
airbrake add phase 68
airbrake add altitude > 20
config validate
config apply
config airbrakes on
```

Remove any ordinary assignment to that servo before applying. Existing airbrake
conditions must be removed or replaced rather than blindly appending this example.
The supplied zero law still keeps the servos closed even when this gate enables.
Until graphical airbrake controls are added, prepare a reserved channel from the Console
with `servo arm 3` (or all assigned channels with `servo arm`); use idle `servo release`
to detach held PWM.

## Logging

The immutable header freezes the complete configuration, including channel assignment
and gate conditions, plus the enable setting and compiled law version in the calibration snapshot.
Each sample adds four commanded openings, control flags, a failed-condition bitmask
and the assigned channel mask. These join the existing sensor, phase, battery and
abort-reason columns in the same timestamped CSV; ordinary actuator starts remain
in the header's once-only output table. No per-update flash rewrite is introduced.

| Flag value | Meaning |
|---|---|
| 1 | Configured channels |
| 2 | Gate enabled; valid control request accepted |
| 4 | At least one rule condition/gating phase is false |
| 8 | Invalid estimate or integration interval |
| 16 | ABORT latched |
| 32 | Invalid law request |
| 64 | Physical output write failure |
| 128 | No processed sensor frame for more than 100 ms |
| 256 | Navigation currently inhibits control starts |

Flags combine. `airbrake_failed_conditions` bit i identifies condition i; it is zero
when every condition is satisfied. Openings export as `airbrake_1_pct` through
`airbrake_4_pct` in [0,100], with 0.01% steps; unassigned channels export null/CSV nan.
`Enabled` can coexist with a 0% command, particularly with the supplied law.

Transitions are reconstructed by comparing flags/masks in successive saved samples;
the first sample is the initial observed state. Header conditions explain the mask.
This is sampled history, not an event journal: dropped samples can hide short transitions.
A closure between frames, after source loss or at stop has no invented sensor sample;
the next saved frame and recording end reason provide the available evidence.
Opening is the post-clamp command/attempt, not proof of successful physical movement.

The canonical byte offsets and versions are in [Storage format](STORAGE_FORMAT.md):
configuration v3, header v7, samples v3 (76 bytes, nine samples per block), download
`xor32-v2`. The Python downloader and app configuration backend understand/preserve
these fields. Settings has an enable checkbox with the required compiled-law warning.
Configuration/Rules edits the continuous gate; Channels reuses closed/open calibration.
Dashboard shares preparation/release and shows control state and inhibition causes, without opening percentages.
Recorded flights show the airbrake section directly below vertical velocity/acceleration,
only when the flight's persisted enable setting is true. It contains the fixed 0..100%
command graph, control intervals and a table of the flight's saved activation conditions,
combined with AND. Configured channels alone do not make a disabled flight's section visible.
Enabled control does not imply nonzero opening; flags and observed transitions remain in
the recorded data, without a separate transition table in the UI.
Live status uses the existing serialized worker and is polled only for configured enabled
airbrakes on Dashboard (0.8 s USB / 1.5 s Bluetooth); it never wakes sleeping hardware.
There is no legacy format conversion or automatic erase. After updating both firmware
and app source, reapply the configuration and explicitly use `flights format` if the
memory contains older flight headers. Formatting flights preserves device settings.

## Verification

`python tools/run_tests.py --full --desktop` covers the firmware contract, continuous
AND behavior, state reset, invalid values, reversed endpoints, channel conflicts,
preparation, output failure, stale-frame closure, virtual isolation, frozen header and
sample decoding, and existing download/desktop flows. `pio run -e waveshare_rp2350_tiny`
checks the target build. These are host/build checks; mechanical closure and a custom
flight law still require bench/flight validation with the actual hardware.
