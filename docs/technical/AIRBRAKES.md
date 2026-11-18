# Airbrakes

The firmware provides a continuous servo control interface for airbrakes. It decides **when**
control is allowed and turns a normalized opening into servo PWM; the control law that decides
**how much** to open is yours. The supplied law in
[`src/control/AirbrakeLaw.cpp`](../../src/control/AirbrakeLaw.cpp) returns zero on every channel,
so the airbrakes stay closed until you write one.

## Writing a control law

Implement `updateAirbrakes(const ControlInput &, LawState &)` and increment `AirbrakeLawVersion`
in [`AirbrakeLaw.h`](../../include/control/AirbrakeLaw.h) whenever the law changes; the version is
stored with every flight.

| Input | Meaning |
|---|---|
| `state[0..2]` | Kalman altitude above launch (m), vertical velocity (m/s), vertical acceleration bias (m/s²) |
| `covariance[3][3]` | Full covariance of those three states after this cycle's prediction and corrections |
| `acceleration` | Net vertical acceleration used by the prediction (m/s²), gravity and bias removed. Derived, not a fourth state |
| `timeUs`, `dtSeconds` | Time since the run started, and the accepted integration interval |
| `phase` | Current flight phase, normal or bistage |
| `valid` | Always true when the law is called; invalid estimates close the airbrakes without calling it |

The law returns four `opening` values (index 0 is SERVO 1) and a `valid` flag. 0 means the
calibrated closed position and 1 the calibrated open position. Values are clamped to [0, 1] and
quantized to 0.0001. A non-finite value, or `valid=false`, on any used channel closes **all**
airbrakes for that cycle. The law never sees PWM: `OutputExecutor` maps the opening between the
servo's saved closed and open positions, which may be reversed.

`LawState` is the law's memory. It is cleared at every run and whenever the gate disables control,
so each enabled period starts fresh; extend it as your controller needs. The law runs inside the
navigation loop: keep it bounded, with no blocking IO, delays, heap allocation or hardware access.
There is no covariance threshold in the wrapper; the law can use the uncertainty as it sees fit.

## When control is allowed

The configuration holds one **airbrake gate**: a mask of reserved servo channels and up to eight
conditions combined with AND, using the [condition types](CONFIGURATION.md#rules) of ordinary
rules. A separate device setting, `config airbrakes on|off`, enables control; it is off by default
and after a factory reset. Turning it off keeps the channels, conditions and servo positions.

The law is called on a cycle only when all of these hold:

- airbrakes enabled and at least one channel reserved;
- an airborne phase (never PAD or LANDED) and every gate condition true;
- navigation is not inhibiting starts (for example the bistage velocity window);
- finite Kalman state, acceleration and covariance (non-negative diagonal), and `0 < dt ≤ 0.1 s`.

Otherwise the airbrakes are commanded closed in that same cycle. Unlike ordinary rules the gate has
no latch and no delays: control switches on and off as the conditions change. It is evaluated after
the ordinary rules, so a `since_rule_ms` condition sees a rule triggered in the same cycle. ABORT
closes the airbrakes for the rest of the run. If no sensor frame is processed for more than 100 ms,
the airbrakes close as well; this relies on the main loop running and is not a hardware watchdog.

Reserved channels cannot be used by ordinary rules, and each needs distinct calibrated closed and
open positions, even for a virtual simulation. A configuration with only airbrakes is valid.

## Outputs at the end of a run

At stop or DISARM the executor commands the closed position and keeps holding it, like a
prepared servo; `servo release` lets it go. A failed servo write closes every reserved channel as
far as possible and ends the run with `OUTPUT_ERROR`. Physical flight needs the reserved servos
prepared like any other; a physical simulation prepares them itself, and a virtual simulation runs
the same controller and logs its commands without driving PWM. The configuration is locked during
a run.

## Configuration commands

Gate edits use the normal [draft](CONFIGURATION.md#drafts): `config begin`, edits,
`config validate`, `config apply`.

| Command | Effect |
|---|---|
| `config airbrakes [on\|off]` | Read or save the enable setting (disarmed, idle, no draft) |
| `airbrake status` | Passive live snapshot: enabled, active, physical, `channel_mask`, flags, failed conditions and commanded openings in % |
| `airbrake show [index]` | The gate, or one of its conditions |
| `airbrake channels <mask>` | Reserve servo channels (bit 0 = SERVO 1); 0 releases them and keeps the conditions |
| `airbrake add <condition>` | Append a condition |
| `airbrake replace <index> <condition>` | Replace a condition |
| `airbrake remove <index>` | Remove a condition |

Example: reserve SERVO 3 and allow control during COAST (or COAST 1 / COAST 2) above 20 m.

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

In the app: **Settings → Airbrakes** for the enable, **Configuration → Rules → Configure
airbrakes** for the gate, **Configuration → Channels** for the closed/open positions.

## Logging

Every flight stores the full configuration, the enable setting and the law version. Each sample
adds the four commanded openings, the control flags, a bitmask of failed gate conditions and the
reserved channel mask. In the CSV export the openings are `airbrake_1_pct` … `airbrake_4_pct`
(0–100 %, empty for unreserved channels) and bit *i* of `airbrake_failed_conditions` marks
condition *i* as false. Byte layout: [Storage format](STORAGE_FORMAT.md).

| Flag | Meaning |
|---|---|
| 1 | Channels reserved |
| 2 | Control enabled this cycle |
| 4 | A gate condition or the phase is false |
| 8 | Invalid estimate or integration interval |
| 16 | ABORT latched |
| 32 | Invalid law output |
| 64 | Servo write failure |
| 128 | No sensor frame for more than 100 ms |
| 256 | Navigation inhibits control |

Openings are commanded values, not measured positions: the servos have no feedback. Transitions
are reconstructed from successive samples, so a very short change between two saved samples can
be missed. The app's Recorded flights page shows the opening plot and the gate conditions for
flights that had airbrakes enabled.
