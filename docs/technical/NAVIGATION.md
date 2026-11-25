# Navigation and calibration

Navigation turns calibrated sensor frames into attitude, altitude, vertical velocity and the
flight phase. It lives in `src/navigation/` (`SensorPipeline`, `AttitudeEstimator`,
`VerticalKalman`, `FlightDetector`, `Navigation`) and has no Arduino, driver, heap or matrix-library
dependency. The phase transitions are defined in [State machines](STATE_MACHINES.md).

## Frames and units

- **BODY**: +X points from tail to nose. Sensor axes are rotated into BODY by the saved
  [mounting](#mounting).
- **NAV**: +Z up. The attitude quaternion maps BODY to NAV; Euler angles are in radians.
- `SensorFrame` carries BODY specific force (m/s²), angular rate (rad/s), pressure (Pa),
  microsecond timestamps, independent IMU and barometer sequence numbers, and validity flags.
- Pressure altitude is `44330 × (1 − (P / 101325)^0.1903)`. Altitude above launch subtracts the
  arming reference and the PAD baseline.

The desktop app displays orientation in an X-up reference (the rocket nose-up reads zero); that
conversion is for display only and is described in [Desktop](DESKTOP.md#behavior-by-page).

## Input rules

- An IMU frame with a repeated or older timestamp or sequence is rejected. An invalid frame, or
  `dt` outside `(0, 0.1 s]`, leaves the estimates unchanged for that frame but still moves the
  timing anchor, so an unknown gap is never integrated.
- A barometer correction needs a new sequence number, finite positive pressure and a timestamp no
  more than 100 ms old and not in the future. Re-reading the same register is not a new sample.
- Estimates predicted from a fresh IMU frame count as fresh between barometer updates.
- `NavigationOutput::ruleSnapshot()` hands the result, with validity flags, to the
  [rule engine](CONFIGURATION.md#how-a-rule-is-evaluated).

The first step after `Navigation::begin(referenceAltitude, nominalImuHz, quaternion)` uses the
nominal period; later steps use the measured timestamp differences.

## Estimator

Each cycle runs, in order: attitude update, vertical projection, PAD baseline, phase detection,
Kalman prediction, the PAD velocity constraint and, when a fresh barometer sample is available, the
barometer correction. The detector uses the velocity from the previous cycle. In PAD and LANDED,
vertical velocity and acceleration are pinned; LANDED accepts no new altitude corrections.

| Parameter | Value |
|---|---|
| Kalman state | Altitude, vertical velocity, vertical acceleration bias |
| Initial covariance | diag(10, 10, 1) |
| Acceleration process variance | 9.0 |
| Bias random walk | 1e-4 |
| Barometer variance | 0.0225 on PAD; 0.49 to 6.25 in flight, quadratic in \|vz\|/120 |
| PAD velocity pseudo-measurement variance | 0.0025 |
| Gravity | 9.80665 m/s² |
| Attitude correction gate | \|a\| within 0.2 of 9.81 m/s² |
| Attitude gain (beta) | PAD 0.3, BOOST 0.1, COAST 0.5, DESCENT 0.3, LANDED 0.05 |
| PAD baseline | 3 s stationary gate, 10 s low-pass time constant |

The covariance update uses the standard `(I − KH)P` form. In bistage mode the velocity-median
window takes the velocity after this cycle's prediction and correction; tilt and ABORT are decided
before rules and outputs run.

**Known limits**

- There is no magnetometer and no heading reference.
- There is no strategy for a degraded or lost sensor beyond rejecting invalid data.
- The PAD baseline and the landing window only advance on fresh barometer samples, so their
  timing depends on the barometer rate.

## Calibration and mounting

Three corrections are applied to raw sensor data, in this order, before navigation sees it:

1. **Accelerometer**: `(raw − bias) × scale` per sensor axis, from the saved
   [IMU calibration](#imu-calibration).
2. **Gyroscope**: minus the bias measured at [arming](#gyroscope-and-reference-at-arming).
3. **Mounting**: rotation from sensor axes to BODY.

`SensorCalibration` carries all of them into `SensorPipeline::begin()`; it is copied at the start
of a run and cleared at the end. Each flight header stores the values the flight used.

### IMU calibration

A six-face fit of per-axis bias and scale (no cross-axis terms), stored with its approval state
in the device journal ([format](STORAGE_FORMAT.md#device-journal-v1-block-0)).

| Command | Effect |
|---|---|
| `imu status` | Passive: procedure progress, completed faces, errors, bias, scale, revision and approval; `verification_residual_mps2` (corrected \|a\| minus g on +X, +Y, +Z, `null` for a face not reached) after the calibration's last verification, passed or failed |
| `imu begin` | Start a six-face calibration |
| `imu capture <axis>` | Capture one face, `+X` … `-Z`: that sensor axis must point up |
| `imu verify` | Start a verification: new `+X`, `+Y` and `+Z` captures with the saved coefficients |
| `imu cancel` | Abandon the procedure |
| `imu identity` | **Bench test builds only** (`-D IMU_IDENTITY_TEST` in `platformio.ini`, off by default; `imu status` then reports `identity_test_build`): disarmed, forgets the calibration and a revoked identity, as delivered. Never fly a test build |

Six accepted faces save a new revision that **requires verification**; a successful verification
approves it. A failed attempt saves nothing. Each capture needs at least 240 fresh samples over two
continuous seconds with the board still. During the first 0.5 s after the capture request (the board
settling) a wrong sample only restarts the window; after that, a wrong position, motion or invalid data
stops the capture (`POSITION_OR_MOTION`), as does a window noisier than the limit (`NOISY`), and the
user captures that face again. A gap over 100 ms is not the user's doing and restarts the window.
Acceptance limits:

| Check | Limit |
|---|---|
| Raw magnitude during calibration | Within 2 m/s² of gravity |
| Gyroscope norm | Below 0.1 rad/s |
| Calibration capture gate | Expected axis above 0.94 g, each transverse axis below 0.15 g |
| Calibration result | Bias within ±2 m/s², scale 0.8 to 1.2, and every corrected component of every face within 0.4 m/s² of the ideal (±g, 0, 0): each face aligned within about 2.3° |
| Verification capture gate | Face tilted at most 10° in any direction (transverse magnitude ≤ sin 10° of \|a\|) |
| Verification result | Each corrected face (`+X`, `+Y`, `+Z`) measures \|a\| within 0.35 m/s² of gravity |
| Per-axis standard deviation | At most 0.12 m/s² |

Calibration is deliberately strict (a stand or square block is expected): a face tilt θ costs a
scale error of 1 − cos θ in the fit (0.08 % at 2.3°), and the residual check rejects faces more
than about 2.3° off. Verification is meant to be quick and can be done by leaning the board on
something: it uses only the magnitude of the corrected acceleration, which does not depend on how
well the face is aligned (a per-component check would measure the misalignment, not the sensor).
On a still face the upward axis carries almost all of gravity, so the magnitude tests that axis'
bias and scale to first order (above about 0.35 m/s² of bias or 3.5 % of scale fails); hence
verification needs one face per axis. Coefficients from a face disturbed between captures fail
there.

A procedure takes the configuration lock and link ownership, blocks sleep and runs, and never
enables outputs. It survives a disconnect and times out 180 s after the last capture request.

**Approval lifecycle.** Physical ARM is blocked while the calibration is missing or needs
verification (`IMU_CALIBRATION_MISSING`, `IMU_VERIFICATION_REQUIRED`). A physical flight that
reaches BOOST, or a reboot while armed, uses up the approval and requires a new verification; a
disarm before launch and simulations keep it. If no calibration was ever saved, the first physical
ARM stores an identity calibration (zero bias, unit scale) so the same lifecycle applies. A failed
verification after use revokes that identity permanently.

### Gyroscope and reference at arming

Every physical ARM measures the gyroscope bias and the gravity direction again, with the rocket at
rest; they are kept in RAM for that run only and never reused. Simulation skips this step.

- 500 fresh IMU samples at least 10 ms apart (about 5 s), plus the mean barometric altitude as the
  launch reference.
- A sample counts only if the corrected acceleration is within 1 m/s² of gravity, the gyroscope
  norm is at most 0.5 rad/s and a barometer sample no older than 100 ms is available. A gap over
  100 ms, or an averaged acceleration below 8.81 m/s², restarts the window.
- There is no timeout; progress appears in `status` and DISARM cancels.

The averaged acceleration, after accelerometer correction and mounting, gives the initial attitude
(`AttitudeEstimator::fromGravity`): the shortest rotation that maps it to NAV +Z. Gravity cannot
determine heading, so the rotation around the vertical is a fixed convention. In bistage mode the
attitude is aligned again from the first valid PAD sample close to 1 g.

There is no pre-launch check that the attitude has converged; such a check would have to tolerate
a rocket moving in the wind on its rail.

**Gyroscope-only bench check.** In BOOST and COAST the specific force is far from 1 g, so the
correction gate is closed and the attitude (and the bistage tilt) comes from the gyroscope alone.
To measure that on the bench, a test build (`-D IMU_IDENTITY_TEST`) accepts `nav gyro start` while
a physical run is armed in PAD: it copies the current attitude into a second estimator stepped
with no accelerometer correction. `status` then adds `attitude.gyro_only` (`tilt_deg`,
`up_body`). The copy never feeds phases, rules or the recording, and ends with the run;
`tools/replay/bench_tilt_usb.py --gyro` drives the check (drift while still, known rotations,
fast motion). Fix the board on the [bench stand](https://github.com/rotellini9809/perpetual_pcb/tree/main/mechanical/bench-stand) so it stays still during the 5 s
arming collection: moving it there biases the gyroscope.

### Mounting

The mounting is one saved setting, the signed sensor axis that points to the nose
(`config orientation <axis>`, see [Configuration](CONFIGURATION.md#device-settings)). It must be
saved explicitly before physical ARM. Each choice fixes a right-handed BODY frame:

| Sensor axis to nose | BODY X | BODY Y | BODY Z |
|---|---|---|---|
| +X | x | y | z |
| -X | -x | y | -z |
| +Y | y | -x | z |
| -Y | -y | x | z |
| +Z | z | y | -x |
| -Z | -z | y | x |

Only axis-aligned installations are supported. The roll reference around the rocket's length is a
convention, enough for vertical navigation and phase detection but not a measured roll reference.
Simulation and replayed BODY data keep their own frame and ignore the saved mounting.

## Replay and verification

The embedded Hoppino flight contains BODY data that was already calibrated when it was logged, so
replays apply no second rotation or bias. Its pressure is converted from hPa to Pa, and its launch
reference is the mean altitude of the first second.

`python tools/replay/replay_navigation.py` replays recorded sensor data through an independent PC
implementation; the recorded estimates are used only for comparison. Host tests cover kinematics,
bias, covariance, attitude normalization, phases, freshness and invalid-input handling. See the
[test guide](../testing/README.md) and the dated comparisons in
[Validation](../testing/VALIDATION.md#navigation-reference).
