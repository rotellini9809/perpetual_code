# Navigation and calibration

The source baseline is `../recovery_system_avionics/src/Navigation.cpp`, `include/Navigation.h`,
`src/Sensors.cpp` and the old main logging path. The port has four small modules under
src/navigation, with matching public headers and no Arduino, hardware drivers, dynamic
allocation or matrix-library dependency.

## Input and output boundary

`Navigation::begin(referenceAltitude, nominalImuHz, quaternion)` resets navigation. Reference
altitude is absolute pressure altitude in metres from arming; the first IMU step uses nominal
dt, subsequent steps use actual timestamp deltas. The identity quaternion default preserves the
earlier initialization. A different mounting/attitude initialization must be explicitly supplied
and verified.

The mounting selector below defines sensor-to-BODY coordinates, not the initial BODY-to-NAV
attitude. It does not change the inherited quaternion initialization or estimate absolute heading.

`SensorFrame` requires calibrated BODY specific force in m/s^2, gyro in rad/s, pressure in Pa,
microsecond timestamps, independent sequence IDs and validity. BODY +X points tail-to-nose.
Quaternion maps BODY -> NAV with +Z up. Euler output is radians. Pressure conversion is
44330*(1-(P/101325)^0.1903); launch-relative barometer altitude subtracts the reference and
tracked PAD baseline.

Repeated/backward IMU timestamps or sequences are rejected; invalid IMU or dt outside (0,0.1]
seconds freezes numerical navigation for that frame. Fresh invalid frames still advance the
timing anchor so recovery does not integrate an unknown gap. Barometer updates require a new
sequence, finite positive pressure and an acquisition timestamp no more than 100 ms old or in
the future. Re-reading the same barometer register does not create a fresh correction. Numeric
estimates predicted from a fresh IMU frame are fresh estimates even between baro updates.

`NavigationOutput::ruleSnapshot()` maps the fresh result to the rule evaluator, including
invalid flags on rejected input. It does not execute rules or outputs. No live hardware sampling
rates have been changed or tuned by this port.

## Preserved estimator/detector behavior

| Parameter/operation | Baseline retained |
|---|---|
| Kalman state | z, vz, vertical accel bias |
| Initial covariance | diag(10,10,1) |
| Acceleration process variance | 9.0 |
| Bias random walk | 1e-4 |
| PAD baro variance | 0.0225 |
| Airborne baro variance | 0.49..6.25, quadratic with abs(vz)/120 |
| PAD velocity pseudo-measurement variance | 0.0025 |
| Gravity compensation | 9.80665 m/s^2 |
| AHRS correction gate | abs(accel magnitude - 9.81) < 0.2 |
| AHRS beta | PAD .3, BOOST .1, COAST .5, DESCENT .3, LANDED .05 |
| Launch | accel magnitude > 3*9.81 |
| Burnout | BODY X specific force <= 0 |
| Apogee/descent | previous estimated signed vz <= -0.5 |
| Landing | near 1g within 2 m/s^2, gyro norm < .5 rad/s, baro range <1 m, 2 s stability |
| PAD baseline | 3 s stationary gate, 10 s low-pass time constant |

Execution order remains AHRS, vertical projection, baseline, phase detection, Kalman predict,
PAD velocity constraint and fresh barometer correction. Baro variance follows velocity after
prediction. Detector reads the previous cycle's velocity as in the baseline. PAD/LANDED pin
vertical acceleration/velocity; LANDED does not accept new altitude corrections. Covariance
still uses legacy (I-KH)P; no Joseph-form/noise redesign is introduced silently.

## Deliberate adaptations and known inherited limits

- Magnetometer correction and its API are removed, per the target specification.
- APOGEE/CHUTE_DEPLOYED collapse to DESCENT; landing no longer needs deployment
  confirmation. The detector has no actuator/deployment status inputs.
- AHRS now integrates actual dt, matching Kalman timing. Legacy AHRS used a fixed
  nominal period even when replay supplied variable dt to its Kalman filter.
- Initial quaternions are validated/normalized; Euler asin is clamped to [-1,1].
- Explicit input freshness prevents repeated pressure corrections. There is no
  degraded-sensor strategy or indefinite fallback estimate guarantee.
- PAD baseline scheduling remains legacy: the stationary gate and low-pass update
  occur only on fresh barometer cycles, using that step's dt. Thus its wall-time
  behavior depends on barometer cadence; fixing this requires a separate explicit
  review rather than changing an inherited detector parameter during porting.
- Legacy landing behavior is retained: after a valid stability window starts,
  intervening missing barometer samples accumulate time, but landing is confirmed
  only on a fresh barometer sample. A dedicated barometer-outage/reset policy
  remains to be specified before live integration.

## Calibration and mounting

Gyro bias must be measured before launch during each arming session, following Recovery System's
stationary-sample approach. Keep its three values in RAM for that session only; recompute them on
the next arming rather than loading them from flash. `ArmingCalibration` now implements this
collector, entered only by physical mode in the shared controller. MAIN's physical readiness
gate still blocks it on the current board; simulation never enters the collector.

The collector averages 500 fresh finite IMU samples at least 10 ms apart, following the old
500-sample/10 ms baseline without repeatedly counting the same register data. This is about
five seconds at 100 Hz; a 120 Hz source may take longer because samples inside the spacing
window are skipped. Launch reference is the mean of fresh pressure-altitude acquisitions,
counted independently rather than weighting repeated barometer values as fresh samples.

Development stationary gates require corrected acceleration magnitude within 1 m/s^2 of
9.80665 and raw gyro norm at most 0.5 rad/s. Invalid data, stale/future pressure or IMU gaps over
100 ms restart the collection window; repeated/backward IMU identities/times do not count.
Pressure must be finite/positive and no older than 100 ms. These collection gates still need
qualification with the repaired physical IMU; no estimator/detector parameters changed. There
is no automatic calibration timeout: progress remains visible and `disarm` cancels collection.
No calibration samples or gyro biases are written to flash.

A separate pre-arm tilt-consistency/convergence gate is not implemented yet. Its future policy
must tolerate normal wind-driven motion of a rocket on rail buttons: compare estimated gravity
with measurements rather than require a fixed attitude. Yaw is not a convergence prerequisite.
The earlier proposed strict one-second immobility thresholds are not adopted; thresholds and
averaging windows need qualification with the repaired IMU. Existing gyro-bias collection remains
unchanged while power-saving integration is completed.

Persistent accelerometer calibration will contain only three additive biases in sensor-axis
coordinates, in m/s^2. Missing calibration uses (0,0,0) and must not by itself prevent flight.
There are no user-calibrated scale factors or cross-axis matrices. The 4 KiB sector at
0x000000..0x000FFF remains reserved and untouched: its record format, loading/writing and the
interface-driven calibration procedure are deferred until the user interface is defined.
Gravity is part of specific force, not an accelerometer bias to remove while stationary.

`SensorCalibration` is the RAM snapshot passed to `SensorPipeline::begin()`. Its `configured`
flag means arming gyro bias is ready, independently of whether accelerometer calibration exists.
Accelerometer biases default to zero. For raw input, both bias vectors must be finite and are
subtracted before mounting transformation. The snapshot is copied at begin and cleared at end.
Missing accelerometer calibration therefore needs no special pipeline mode.

Mounting is one `upAxis` field identifying the signed sensor axis pointing toward the nose
(up on an upright launch pad). It defaults to `PositiveX`; the six options fix the following
right-handed BODY convention for both corrected vectors:

| Noseward sensor axis | BODY X | BODY Y | BODY Z |
|---|---|---|---|
| +X (default) | x | y | z |
| -X | -x | y | -z |
| +Y | y | -x | z |
| -Y | -y | x | z |
| +Z | z | y | -x |
| -Z | -z | y | x |

No arbitrary rotation matrix is exposed. This supports axis-aligned installations; a tilted
installation needs a later extension. Rotation around the rocket's length is a chosen coordinate
convention, sufficient for current vertical navigation and phase detection, not a measured
physical roll reference for future directional control. Mounting is a device setting separate
from bias calibration; its persistence/interface is also deferred.

### Recovery System reference and current integration

The old `applySensorToBodyFrame` implements x=-z, y=y, z=x despite a contradictory nearby
comment. Do not blindly reuse it on the new board. The input boundary is already calibrated BODY
data; a new physical board adapter must explicitly apply its verified mounting transform and
arming gyro bias/launch reference.

The old main logs data **after** `applyArmBiases` and `applySensorToBodyFrame`. Hoppino's
accel/gyro columns are therefore treated as logged calibrated BODY data; no second rotation or
gyro-bias subtraction is applied in this replay. The pressure column is hPa and explicitly
converted to Pa. Pre-arming calibration samples and the exact historical launch-altitude
reference aren't in this recording. Replay uses the initial one-second pressure-altitude mean as
an offline approximation, not a newly implemented arming-calibration procedure.

MAIN runs this code through the shared SensorPipeline with USB HIL or autonomous
synthetic sensors. See [architecture](ARCHITECTURE.md) and [simulation](../testing/SIMULATION.md).
`HardwareSensorSource` now adapts fresh driver readings to the same runtime. MAIN uses it for
idle diagnostics while physical arming is blocked by IMU soldering and missing activation
integrations. The collector/controller are host-tested; acquisition qualification and persistent
accelerometer storage remain future work. Current executable runs are virtual-only.

## Verification

`python tools/replay/replay_navigation.py` is the independent PC sensor replay. Recorded
estimates/phases are comparison-only; changing them or converting hPa to Pa cannot change
computed navigation. Unit/integration checks cover estimator kinematics/bias/covariance,
attitude normalization, phases, freshness and invalid
input freeze. [Validation](../testing/VALIDATION.md#navigation-reference) records
Hoppino timestamps and differences, which are regression evidence rather than
physical accuracy measurements. [The test guide](../testing/README.md) is the
single entry point for complete regression checks.
