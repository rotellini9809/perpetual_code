# Target product requirements

This document consolidates the agreed application, communication and storage design. It is **not
a list of implemented commands**. Current behavior belongs
in [Architecture](../components/ARCHITECTURE.md), [USB](../components/USB.md),
[Configuration](../components/CONFIGURATION.md) and [Storage](../components/STORAGE.md).

| Area | Current boundary |
|---|---|
| Plan/rules/persistence | Implemented with USB draft/apply/readback and virtual evaluation |
| Navigation/simulation | Implemented sensor pipeline/Kalman/detector, USB HIL and autonomous virtual profile |
| Recording | Fixed-budget contiguous storage, A/B directory, independent starts/summary, explicit deletion and Corrupt flag |
| Physical flight lifecycle | Shared real/simulated output executor, explicit preparation, gyro calibration and landing shutdown implemented in software; hardware qualification and separate provisional recording format pending |
| PC app/phone/Bluetooth | Shared MAIN USB/SPP endpoints and initial Qt Windows GUI implemented; phone/BLE validation pending |
| Export/monitor/sleep | Bounded JSON -> PC CSV and five-minute inactivity sleep/command wake implemented; binary transfer and subscriptions planned |

Preserve Recovery System calibration/estimator/detector behavior as the porting baseline
(excluding magnetometer and required hardware adaptations). Separating motion phase from
configurable deployment does not authorize silent detector tuning. All fixture values are
examples, not recommended recovery settings.

Device calibration stores only accelerometer XYZ additive biases; absent calibration uses zero
bias and does not block flight. Gyro bias is recomputed during every arming and stays in session
RAM. The accelerometer calibration procedure and commands will be designed with the user
interface later. Mounting is a separate signed noseward sensor-axis setting (+/-X, +/-Y, +/-Z),
default +X; current navigation uses a fixed right-handed convention for the other two body axes.
See [navigation](../components/NAVIGATION.md#calibration-and-mounting) for units and mappings.

## Application interface

The local PC application has Actuators, Deployment rules, and Review & Apply pages. Keep device
identity/armed status and applied-vs-draft distinction visible. Read/connect/edit operations
alone must not move hardware. Unassigned channels take no automatic action. Editing and physical
tests are disarmed-only.

Actuators show four pyros (read-only fixed firmware pulse profile) and four servos (Initial/Final PWM), plus
the rule using each channel. App calibration explicitly positions one servo with
slider/numeric/step controls and captures the last commanded value. No shaft feedback or
stall/current inference is assumed. Mechanical calibration is distinct from changing stored
endpoints. Holding/release is fixed lifecycle policy rather than a user-editable deployment
option. Low-level positioning/capture command names are still to be finalized.

Deployment cards offer Main, Drogue, Emergency and Custom presets as editable labels/templates,
not exclusive modes. Every rule is active once per flight; exclude it by removal. Stable IDs
survive rename/reorder; duplicate names need channel/ID context and do not imply redundancy.
Each card contains name, AND conditions, a rule delay in ms and one or more assigned
outputs. All outputs share that delay; there is no per-output delay. There are no arbitrary formulas/nested AND/OR. Multiple selected phases form one
membership set; different rows remain AND. Catalog/units/boundaries are canonical in
[Configuration](../components/CONFIGURATION.md). Display event-relative time in
seconds only with an explicit conversion to protocol milliseconds.

Assignments link to the channel's configured action. Each physical channel belongs to at most
one rule. Multiple redundant channels execute as scheduled, without waiting for evidence of
another's failure; they share the same computer. A rule trigger, scheduled action and actual
command start must be displayed separately from physical deployment, which ordinary hardware
cannot confirm. Flight phase is PAD -> BOOST -> COAST -> DESCENT -> LANDED, never
Main/Drogue/Emergency. Landing eligibility must not depend on successful chute deployment.

Display one summary row per deployment: name, readable conditions, rule delay and
assigned channels/actions. Also show unused channels. Provide **Read from board**,
**Validate** and **Apply** operations. Reloading board values should explicitly discard
unapplied edits, not silently overwrite them.

Validation covers missing conditions/outputs, invalid values or ranges, conflicting channel
assignments, missing rule references and circular timer dependencies. Removing a referenced rule
must identify the affected rules. Final validation runs onboard for both the PC app and
Bluetooth console.

When all conditions first become true, latch the rule and start its delay. Assigned outputs
wait for the rule delay plus their individual channel delay, without requiring conditions to remain true. Report success only after the board accepts the
configuration and returns matching settings. The applied flight configuration is fixed while
armed.

Onboard validation must also reject self-references, unknown identifiers, non-finite numbers,
negative durations/speed magnitudes, unsupported phase selections and empty rule
conditions/outputs. Display limits supplied by the firmware instead of inventing different app
limits. Distinguish configuration validity from physical readiness: accepted settings alone do
not mean calibrated sensors, prepared mechanisms or good continuity. An empty recovery plan must
not be represented as flight-ready; final arming readiness belongs to the runtime specification.

Historical displays resolve names/actions from that recording's frozen config, and show actual
output starts separately from samples. Empty/invalid slots do not prove no actuation; zero is a
valid committed timestamp. Sample stopped-full does not close the reserved output-start path. No
rule-trigger history/bitmask is added to the numeric sample CSV. Example emergency card: descent
after apogee, altitude in [Hlow,Hhigh], abs(vz)>=V for T ms, Pyro 2 and Servo 2 starting together;
Hlow/Hhigh/V/T are user-selected placeholders.

## Future transport and results

One command service supports a PC app over Bluetooth/USB and a raw Bluetooth phone terminal.
JDY-34 is the target; verify its profile/baud/AT/session behavior rather than copying JDY23
assumptions. Requests are UTF-8 LF/CRLF lines with lowercase keywords, explicit units and
quoted/escaped names; current USB/Bluetooth remains ASCII. Buffer fragmented lines, split combined lines,
ignore blanks, reject malformed numbers/quotes and discard an oversized line through LF without
executing its suffix. Actual LF is required, not literal backslash-n text. No wall-clock
synchronization, magnetometer commands, generic operation-status/cancel subsystem or remote
in-flight deployment commands are specified.

| Request | Rendering |
|---|---|
| Optional `@id` present | JSON with correlation ID |
| No ID, recognized phone-subset command | Human-readable result/error |
| No ID, other supported command | JSON without ID |
| Unknown unnumbered command | Short readable error pointing to help |

Both renderers consume the same typed result/permissions. Reply JSON is compact one-line
LF-terminated, escapes strings and uses null/validity for nonfinite data. Keep
progress/completion in the initiating format; monitoring uses its subscription format (JSON). No
debug echo or byte-interleaved messages in an app/transfer stream. Malformed IDs need not be
echoed. ID presence is not authentication.

### Phone subset

| Commands | Purpose |
|---|---|
| help, ping, info | Usage, connectivity and identity |
| status, preflight, outputs | Mode/readiness checks and channel status |
| servo arm [<channel>] | Prepare all configured servos, or one assigned/calibrated servo |
| arm, disarm; ds alias | Flight lifecycle |
| test servo <channel>, test pyro <channel> | Immediately start one eligible channel test; no confirmation |
| sleep, wake | Explicit idle power management |

No phone calibration workflow or generic calibration command is needed. The PC app handles servo
calibration. Gyroscope calibration runs automatically during arming, with progress/result
reporting. Detailed rules, simulation controls, configuration editing and other supported
requests without IDs return JSON.

App-oriented is a workflow distinction, not authentication: typing an ID does not prove the
sender is the PC app. Physical motion remains subject to onboard restrictions.

### Accepted actions and completion

| JSON type | Meaning |
|---|---|
| reply | Immediate success or result: accepted for a longer action |
| error | Rejection before execution |
| progress | Intermediate update |
| complete | Terminal outcome of an accepted action |
| data | Opt-in monitoring |

Use command, optional id, stable error codes and human messages. After acceptance, failures use
complete with ok:false. Human replies express the same distinction: “Arming: calibrating gyro”
is not “Armed.” status reports busy/calibration state, so no operation commands are necessary.

Allow bounded read-only inspection while busy, but immediately reject conflicting mutations. No
user-action queue. The storage worker's buffering and normal actuator timing are internal
implementation details, not queued user tests.

## Servo calibration, preparation and flight lifecycle

### PC calibration

While disarmed, the user explicitly moves a servo through app controls (slider, numeric pulse
width or small steps), visually checks the mechanism, and captures the last commanded position
as initial or final. There is no shaft-position feedback assumed, no manual hand-position
capture, and no stall/current-based endpoint detection.

Initial means stowed/prepared; final means deployed. Store the two endpoints in the
draft and use normal Validate/Apply to persist them. Merely connecting, reading or editing a
saved value does not move an actuator. Physical adjustment requires a deliberate positioning
operation within firmware pulse limits. Precise low-level app command spellings remain to be
frozen; they are not part of the phone subset. Keep PWM active during explicit calibration;
release it on procedure end or disconnect, with no extra hold. Clear affected preparation when
calibration starts. Calibration conflicts with tests and flight arming; no motion queue.
Hold/release behavior follows the lifecycle below, not a user-selectable deployment option.

### servo arm [<channel>]

Disarmed only, using valid applied settings, with no competing test/calibration/preparation
movement. With no argument, prepare all and only servos assigned to rules in the applied
configuration. With a channel, prepare that servo only; reject an unassigned channel or one
lacking valid calibration. Leave other channels unchanged. No assigned servos means a successful
no-op with “No configured servos to prepare”; pyro-only plans need no servo preparation.

Command the saved Initial position and report each selected channel's result. Mark preparation
complete when the initial PWM command is accepted; this is software readiness, not
measured position. Report “Initial position commanded; servo prepared.” There is no feedback to
detect a jam, failed movement or true physical position. Preparation neither arms flight
detection nor enables pyros.

Maintain PWM after preparation while waiting for arm, throughout the armed lifecycle and after
deployment. Failed flight arming does not release already-prepared servos. Disconnect alone does
not release preparation; inactivity power saving may subsequently do so. Inactivity timeout is
the explicit exception to holding until disarm.

`arm` requires every assigned servo to be prepared. `servo arm 1`, `servo arm 3`, then `arm` is
valid when channels 1 and 3 are the only configured servos. Both command forms are
phone-friendly.

Invalidate affected preparation on reboot, test/calibration start, relevant applied changes,
release of PWM or known loss of drive/power. On a relevant applied change, release affected
prepared channels so obsolete settings cannot remain marked ready. Runtime readiness is never
restored from flash.

`disarm` and `ds` release all servo preparation and PWM even if flight is already disarmed.
During disarmed servo testing/calibration they stop that servo work, release PWM and report an
interrupted outcome; do not report normal completion. They retain flight-phase restrictions
while armed. Accepted manual disarm at PAD/LANDED blocks future starts
and follows the same bounded active-action/finalization shutdown ordering; it does not permit
residual flight actions after disarm. A running pyro test remains a bounded pulse; reject
conflicting disarm with BUSY until it ends rather than inventing an unagreed pyro-abort
behavior.

### Arming, boot and recording

Boot always disarmed, with outputs inactive and no prepared flags, triggered rules or pending
actions restored. Load and validate the saved recovery configuration from flash; invalid/missing
settings block arming.

Initialize/check storage at startup. Successful initialization, current storage health and
ability to initialize a recording are arming conditions. Prepare erased space and validate
storage readiness before success. A temporary reservation is not a visible flight recording.
Failed arming creates no retained flight log; discard incomplete reservations. Start sample
recording only after successful arming, with a fixed configuration snapshot.

During arming, check continuity sequentially on configured pyro channels only. A detected fault
on any configured output blocks arming and identifies the channels. Ordinary servos provide no
physical-position verification; check valid calibration and preparation only. Run gyro
calibration automatically. If servos are configured, display: “Check that all configured servos
are in their intended initial positions. Physical position cannot be verified automatically.”
This is a reminder, not an additional confirmation request.

Pad disarm closes the recording. At normal recording closure, delete a recording that never
reached BOOST; preserve settings/calibration. After BOOST, retain it even if sample logging
stopped-full. Storage failures during flight must not stop navigation or recovery. Bounded
finalization failures are reported and must not indefinitely retain armed state or PWM.

### Landing and output shutdown

No new rule triggers after LANDED. Previously latched rules may start outputs after their delay
within the existing post-landing window. At expiry, cancel pending starts and
finish already-active bounded movements/pulses while still armed. Continuous servo holding is
not an unfinished movement. Then drain output-start records, finalize logging, release PWM and
disarm. If storage finalization fails, report the failure and still complete output
shutdown/disarming after bounded handling.

After disarm, no flight action starts or continues. Later explicit disarmed preparation,
calibration and tests remain allowed. Never force untriggered rules to execute.

## Physical channel tests

Only `test servo <channel>` and `test pyro <channel>` are supported; no test open or whole-rule
tests. A valid request starts immediately with no confirmation. Remove confirm/cancel from the
test interface. Each test executes the channel's applied action without triggering rules.

There are only two global test states: IDLE and RUNNING. The one slot is shared by every servo
and pyro channel. While RUNNING, reject any additional test immediately with TEST_BUSY /
“Another test is running.” No pending confirmation, queue, deferred start or replacement. This
restriction is for manual tests; automatic flight outputs may operate concurrently.

All tests require flight DISARMED and valid channel action settings. A servo test additionally
requires that channel to be prepared with `servo arm`. On acceptance, atomically claim the test
slot and clear that servo's prepared flag. Execute its calibrated movement from the held initial
position to the final position, hold the final commanded position for exactly 2 seconds, then
disable that channel's PWM and release the test slot. The hold is part of the running test.
Other prepared servos are unchanged. Another test of that servo requires preparation again.

A pyro test starts its configured bounded pulse immediately after eligibility checks and
releases the test slot when the pulse ends. There is no user confirmation stage for either test.
Report pulse start, not confirmed ignition.

```text
> servo arm 1
Initial position commanded; Servo 1 prepared.
> test servo 1
Test started: Servo 1.
Test complete: movement and 2-second hold finished; PWM released; preparation cleared.
```

Test/calibration/preparation movements are mutually exclusive; reject conflicts, never queue
them. Reject flight arming while a test or calibration is active. An accepted test continues to
its bounded completion on disconnect, including the servo hold/release; do not replay it after
reconnection. Explicit `ds` can interrupt disarmed servo work as defined above.

Tests have separate runtime bookkeeping and never consume flight rules or fill flight
output-start slots. Test starts/completions report commanded behavior, not physical success.
Report failures/interruption accurately and clear ownership at terminal cleanup. In
virtual-output simulation, reject physical test requests; switching back to a physical operating
mode is explicit and disarmed.

## Client ownership and disconnect

One controlling client is supported: either PC app or phone terminal, not both. If another
connection attempts control while one owns the interface, immediately return INTERFACE_BUSY; no
multi-client edit merging or action queue. USB can be a secondary download destination for that
same client's explicitly authorized transfer, not a second controller.

Current MAIN coordinates the two physical transports with `CONTROL_BUSY` for foreign mutations;
both may inspect read-only state. An owner is retained during a draft/run and cleared on idle
or its disconnect. The future app/phone/binary-transfer policies below still require UI work.

Request IDs are optional and only provide correlation/retry bookkeeping. The app uses increasing
IDs within a session. Exact repeated ID/payload requests return retained results/status without
repeating physical actions; reused IDs with different payloads return ID_CONFLICT. If a prior
result was evicted, reject stale IDs instead of executing again. Current USB/Bluetooth fixes uint32 IDs
and one result per connection; broader client/security guarantees remain future work.

No cross-reboot exactly-once promise. After disconnect/reboot, query state before any retry.
Never automatically replay physical requests. Unnumbered terminal commands generally have no
deduplication guarantee beyond their eligibility and busy checks; never automatically resend
them.

Disconnection does not disarm flight. Monitoring subscriptions end; active tests follow their
bounded completion policy and calibration PWM is released. Unapplied drafts may be discarded
under the proposed session policy. An accepted configuration flash write may finish despite
disconnection; verify by readback afterward rather than assuming rollback.

JDY-34 UART1 at 9600 baud, Windows SPP traffic and connection notifications are verified in
[Validation](../testing/VALIDATION.md). BLE, endurance and pairing/security still require work.
Do not equate an ID with authentication. If
transport cannot reliably identify connection changes, settle session reset behavior before
claiming session-bound guarantees. USB unplug/BT disconnect recovery must discard partial input.

## Planned command groups

| Group | Proposed commands | Permissions/behavior |
|---|---|---|
| Detailed inspection | capabilities, sensors, storage status | Bounded read-only responses |
| Configuration | config show/begin/draft/validate/apply/discard | Disarmed; Apply validates, persists, verifies and activates |
| Rules | rule list/add/show/rename/remove, rule delay | Draft edits only |
| Conditions | condition add/replace/remove | Supported catalog and AND semantics |
| Assignments | output assign/unassign | Draft channel ownership; immediate start with the rule |
| Actuator values | pyro set, servo set; app positioning/capture operations | Values edit draft; explicit calibration motion disarmed only |
| Simulation | simulation show, simulation set on/off; explicit output-mode selection | Disarmed only; physical or virtual outputs as below |
| Monitoring | monitor start/stop | Opt-in, default off; JSON replies/data |
| Flight summaries | flight list, flight info <handle> | JSON summaries, including stored configuration and output starts |
| Flight clearing | flight clear | Explicit disarmed/non-recording operation; preserve settings/calibration |
| Binary export | dump_bt <flight>, dump_ser <flight> | Destination-specific app/utility download |
| Bluetooth policy | bt status, bt flight set on/off | Explicit settings, not toggles; changes disarmed only |

Reply selection applies consistently: commands outside the phone subset return JSON even without
IDs. Bulk-export refusal may provide a readable phone explanation as described below. No old
erase_mem whole-chip operation in the normal flight-clear workflow. Unknown legacy commands
should explain replacements rather than silently reinterpret them.

## Future binary flight export

Binary is the chosen encoding. No board-generated CSV download, Base64 expansion or JSON array
of samples. Export the complete logical recording: format/header, that flight's configuration
snapshot, reserved output-start table and sample data. The PC app/utility verifies and decodes
it, then produces CSV and metadata summaries.

| Command | Bulk destination |
|---|---|
| dump_bt <flight> | Bluetooth PC app |
| dump_ser <flight> | USB serial app or download utility |

The request can select a destination different from its control connection. Command status
normally returns to the requesting connection; bulk bytes go only to the explicit destination.
Once a destination enters binary mode, its transfer outcomes use framed transfer messages, not
interleaved ordinary text.

No bulk export into a raw Bluetooth phone terminal. list/info remain inspectable, but an export
requires receiver negotiation and readiness on the destination. IDs do not identify an app.
Without a ready receiver, send no binary; explain that the PC app or USB download utility is
required. If readable text is used for an unnumbered phone refusal, treat it as an explicit
export-error exception to default JSON.

Proposed transfer sequence:
1. Resolve requested list selection into a stable internal flight handle/sequence; verify disarmed, non-recording and not being modified.
2. Negotiate transfer identity, log format, total byte length, destination, offset and frame-size limits.
3. Await explicit receiver readiness; timeout leaves control mode intact.
4. Stream uncompressed binary frames with explicit type/transfer identity, logical offset, payload length and checksum. Delimit using frame lengths, never newline bytes in binary.
5. Verify total received length/integrity, report completion and return to command mode using an explicit end-of-transfer exchange.

During binary mode, pause monitoring and suppress debug/progress text on that destination. Its
parser accepts transfer control only, not ordinary command interpretation of binary bytes. Frame
errors, timeout, abort and reconnect require explicit resynchronization; never guess completion
from silence. Define these details before implementing transfer.

Allow resume from a verified offset only if the same stable flight is still retained; otherwise
return STALE_FLIGHT_HANDLE. Do not select a different flight because indices changed. Bounded
buffers and backpressure prevent UART overflow. Prefer streaming useful-size application frames
over acknowledging every tiny radio packet; exact window/ack policy and size require throughput
measurement.

Bulk downloads are disarmed/non-recording operations and must not race reclamation or header
updates. Bound blocking work even then. JDY-34 baud and real link throughput determine speed;
old JDY23 9600-baud assumptions are not requirements. USB and Bluetooth share the log format,
not necessarily chunk sizes or rates.

Exact binary frame layout, checksum, handshake commands, timeout/resume rules and receiver
negotiation remain implementation details to freeze. Configuration transfers are separate: keep
bounded, validated JSON/chunk uploads into the draft; binary flight selection does not force the
same encoding for configuration uploads.

## Scheduling and input validity

Monitoring is off by default, opt-in/session-scoped/rate-limited. Its sequence, timestamps and
validity expose missing/stale data without changing flash logging rate. Coalesce/drop telemetry
under load, retain command outcomes, and reject new work before tracking buffers exhaust.
Communication/storage faults must not block flight processing or automatically disarm it. Do not
queue conflicting user mutations/tests. Reuse the current parser/ID/error definitions where
settled; future errors include TEST_BUSY, INTERFACE_BUSY, SERVO_NOT_PREPARED, RECEIVER_NOT_READY
and STALE_FLIGHT_HANDLE.

Evaluate every rule from one snapshot of current estimates/phase and previously triggered rules,
then commit all newly triggered rules. A dependency on a rule triggered this cycle becomes
eligible next cycle, including a zero-delay dependency. List order cannot change behavior;
cycles/self-references remain invalid. Independent rules and different pyro channels may start
together. Continuity multiplexing during arming does not serialize flight firing.

Reject stale, non-finite and explicitly failed samples; do not process a repeated sample as a
new measurement. Invalid/stale inputs cannot newly trigger a rule; an already latched rule delay
continues independently of later measurements until it expires or starts are explicitly closed.
Already-triggered actions continue according to lifecycle policy. No advanced degraded-sensor
strategy is specified in this revision.

Simulation uses simulated sensor inputs with an explicit choice of PHYSICAL or VIRTUAL outputs.
Default simulated flights to virtual outputs. Physical mode drives actual servo movements/pyro
pulses and retains physical readiness checks. Virtual mode does not drive hardware, skips
physical continuity/preparation requirements and tracks preparation/actions virtually while
retaining configuration validation. Disable physical PWM when entering virtual mode; never mix a
previously prepared physical servo into a virtual run. Mode changes require disarmed and idle;
clear readiness on mode changes.

Both simulation modes emit nonblocking output-start notifications, clearly labelled PHYSICAL or
VIRTUAL, with channel and actual logical command-start time. A notification says “pulse started”
or “servo movement commanded,” never “ignition confirmed.” Use the same typed result data for
rendering: human text for terminal flight control, JSON for identified app control; retain the
selected format through the run. Monitoring subscription is not required for these simulation
notifications. If disconnected, do not delay flight or promise later delivery; persisted starts
remain available in the log. Store sensor/output mode in immutable flight metadata and use the
same reserved slots for virtual starts, visibly labelled as virtual by the decoder. No extra CSV
sample columns are introduced.

## Inactivity power saving

Automatic power saving occurs only while disarmed after five minutes of inactivity, including
when USB/Bluetooth clients remain connected. Canonical command classification and behavior are
in [USB/Bluetooth commands](../components/USB.md#power-saving).
Active tests (including their hold), preparation motion and calibration count as activity;
evaluate idle timeout after such work ends. A prepared servo merely holding while waiting for
arm does not prevent eventual inactivity sleep. Entering power saving releases all PWM and
clears preparation. Wake does not restore preparation; an operational command wakes first and
then executes through normal validation/readiness. Servo preparation must be requested again.
An explicit sleep request during
active work returns BUSY, with no scheduled sleep. Never enter inactivity sleep while armed,
recording, finalizing required writes or servicing an active transfer. Count active transfers as
activity as well.

## Storage and retention requirements

The implemented fixed map, formats/commit checks and corruption handling are
canonical in [Storage](../components/STORAGE.md) and
[Storage formats](../components/STORAGE_FORMAT.md). Keep settings single-copy;
an interrupted save may require reconfiguration, never an active default. Runtime
readiness/rule/fired/timer state is not restored from flash. Historical snapshots are analysis
metadata, not settings fallback. Preserve launch-relative units, finite height range above the
old packed logger's 327.67 m limit, timestamp-zero validity and missing-value distinction; no
magnetometer fields.

Before successful live arm, prepare/check erased sample and metadata capacity. A temporary
reservation is not a visible flight. Failed arming must remove its temporary reservation and
leave no retained recording; successful arm freezes configuration and starts samples. On orderly
close, invalidate the log if actual runtime phase never reached BOOST, even if sample buffering
would hide that fact. Retain real BOOST runs despite dropped samples. After reset, retain
uncertain interrupted logs; do not infer an unflown PAD from missing persisted data. Development
HIL retains PAD recordings for analysis; the shared runtime implements the orderly hardware-mode
discard decision. Current allocation happens after gyro collection and before physical execution;
failed startup preserves prepared servos and uses existing flash-fault recovery. A distinct
provisional allocation format remains future work; electrical qualification is still pending.

Actual physical output starts are the firing-output assertion (not enable settling) or first
commanded calibrated servo movement (not trigger/enqueue). Capture in bounded independent RAM
capacity before asynchronous persistence; prioritize reserved metadata when flash becomes
available, without claiming to preempt an ongoing erase. Preserve first timestamps/idempotent
duplicates. Never erase/retry partially programmed header slots to fill them. Finalization must
establish no future starts, drain/verify pending entries, then acknowledge success or error.
Sample-full must never overwrite the current flight's beginning, reopen samples or close start
recording. Report overflow, full, I/O failure and finalization failure distinctly; retain
durable prefixes and count drops where possible.

The older design proposed circular growth/oldest-first overwrite; it remains deferred. Current
scope uses contiguous fixed budgets and explicit deletion instead; cross-end metadata is rejected.
Any later circular allocation, automatic reclamation or erase-ahead implementation must define
ownership and verify power-cut recovery beyond the current bounded writer. Never erase another retained recording before
durable invalidation, nor settings/calibration while clearing flights. Stable handles cannot
silently select a different flight after list indices change. Do not reconstruct ambiguous
ownership by guessing from checksums or autoformat unknown metadata.

## Remaining implementation decisions and acceptance

Still specify production numeric policies, movement/settling bounds, low-level app
calibration/mode commands and binary
frame/checksum/handshake/window/timeout/resume details. The existing USB implementation has
already fixed its uint32 ID width and one-result cache; those
are documented in [USB](../components/USB.md), not left as open current behavior.
Do not reopen agreed single-client, dual-rendering, immediate single-channel-test or
receiver-negotiated binary-export policies when filling these details.

Acceptance must cover equivalent phone/app validation; no read/connect actuation; servo
readiness invalidation/hold/release and failed-arm behavior; simultaneous flight outputs despite
sequential continuity checks; no untriggered post-landing actions; bounded finalization with no
stuck PWM/armed state; no replay on reconnect; recording-full start persistence;
physical/virtual isolation; pre-BOOST retention based on actual phase history;
corruption/interruptions without losing healthy flights; exact binary/config/export contents and
preservation of settings/calibration.
Electrical/hardware tests are additional to the host [test matrix](../testing/README.md).
