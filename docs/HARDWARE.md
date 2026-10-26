# Hardware and reference material

The target is RP2350 / Waveshare RP2350 Tiny using Arduino-Pico. GPIO numbers below are RP GPIO
numbers, not package/header pin numbers. The authoritative map
is [board_pins.h](../include/hardware/board_pins.h); it supersedes the old 8 October
bench report and its temporary pin/capacity assumptions.

| Interface | Current assignment |
|---|---|
| W25Q128JV, SPI0 | MISO GP0, CS GP1, CLK GP2, MOSI GP3; board clock 4 MHz |
| LSM6DSV32X / BMP581, SPI1 | MISO GP12, CLK GP14, MOSI GP15; IMU CS GP27, barometer CS GP26 |
| Bluetooth JDY-34 | Hardware UART1 (`Serial2`): MCU RX GP5 receives module TXD; MCU TX GP4 drives module RXD; PWRC GP17 (AT/wake control); no assigned connection-status pin |
| Pyro 1..4 / shared enable | GP9, GP13, GP6, GP7 / GP8 |
| Continuity mux / ADC | Select GP11, GP10 / sense GP28 (ADC2) |
| Battery sense | GP29 (ADC3); 100 kΩ battery-to-ADC, 24 kΩ ADC-to-GND |
| Telemetry/external I2C | GP18/GP19 (TX/SDA, RX/SCL aliases) |
| RGB LED | GP16 |
| Servo PWM 1..4 | GP21, GP20, GP23, GP22 |

Flash is **128 Mbit / 16 MiB**, not the former 8 MiB assumption. Repeated JEDEC EF4018 readings
and the final-sector read/write/restore test passed after solder rework; EF7018 is also accepted
by the driver. Legacy ID EF17 is a different command's identity byte pair, not a JEDEC capacity.
Pin/communication failures
alone do not prove a defective chip. See [diagnostics](testing/README.md).
The [fixed region map](technical/STORAGE_FORMAT.md#fixed-memory-map) is the only
supported product layout. No magnetometer CS is assigned or required.

## Driver boundaries

MAIN measures battery at 5 Hz with 12-bit ADC reads: discard one conversion after input
switching, then average eight. Conversion uses nominal 3.3 V ADC reference and divider ratio
31/6. ADC saturation is unavailable, not a clipped battery value. Reference/divider tolerances
have not been calibrated against a multimeter; battery type/maximum and the GP29 connection
still need hardware confirmation. Voltage is telemetry only and adds no readiness threshold.

MAIN holds pyro outputs/enable inactive and does not initialize servo PWM. It now probes live
sensors and polls them for idle diagnostics through `HardwareSensorSource`. The IMU soldering
problem remains unresolved; fresh barometer pressure/temperature were verified over USB on
19 October 2026. The temporary MAIN bring-up path is commented and does not fake missing IMU
samples. Physical preflight/arming requests now exist, but actual activation remains blocked by
the IMU and any reported configuration/continuity/preparation faults. See
[USB diagnostics](technical/USB.md#sensor-diagnostics).

`Sensors` currently configures accelerometer +/-32 g and gyro +/-2000 deg/s, both at 120 Hz,
with a 50 Hz pressure-enabled BMP581 and 4x pressure/temperature oversampling. Returned
measurements are m/s^2, rad/s, C and Pa. It probes SPI modes, with a lower-speed barometer
attempt. These settings are not the autonomous simulator's 200 Hz IMU rate. Reads now check
accel/gyro data-ready flags and BMP581 data-ready status before declaring a new sample. Barometer
updates remain available after failed IMU initialization. The source preserves independent
acquisition identities and extends micros rollover; timestamps describe polled register
acquisition, not interrupt edges. Cadence, pairing/overruns and mounting/bias calibration must be
verified on hardware before live flight qualification.

The low-level `PyroController` currently permits one active sequence, 10 ms shared enable
settling and a fixed 1000 ms pulse, with fired flags in RAM until begin. It does not require
continuity to accept a firing request. This is not yet the configured concurrent multi-channel
flight executor required by the target design. `update()` must keep running; disabling
clears/cancels outputs.

Continuity selects one channel, settles, discards one ADC conversion, then averages 16 readings.
Raw 12-bit thresholds are <1000 BatteryNotConnected, 1000..2499 Open,
>=2500 Connected; an invalid channel is Uncertain. These are software classifications,
not independent power or ignition verification; separate battery sense is not read by this
method. Continuity multiplexing must not serialize future flight firing.

`ServoController::begin()` centers all four channels, so it can move hardware. Its generic
0..180 deg / 1000..2000 us defaults and 1500 us center are clamped commands, not calibrated
flight preparation. The old GP18/GP19 servo test mapping and direct pyro-enable LED override are
historical and are not current MAIN behavior.

## Supplied PDFs

| File | Relevance |
|---|---|
| [LSM6DSV32X](datasheets/DS_lsm6dsv32x.pdf) | Current IMU reference |
| [BMP581](datasheets/bst-bmp581-ds004.pdf) | Current barometer reference |
| [FS90](datasheets/FS90-specs.pdf) | Servo reference; mechanism endpoints still require calibration |
| [Archived W25Q64JV](archive/datasheets/W25Q64JV_reference.pdf) | Mismatched legacy part; do not derive current capacity from it |
| [Archived JDY23](archive/datasheets/JDY23_reference.pdf) | Mismatched legacy module; do not reuse its AT/baud/profile assumptions for JDY-34 |

Matching W25Q128JV and JDY-34 PDFs have not been supplied locally. Flash power-down/release
commands were checked against the manufacturer-authored
[W25Q128JV datasheet, Rev M](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/7161/256_W25Q128JV.pdf).
Local PDFs are retained unchanged; current hardware facts come from source/observed identification, not an inference
that the archived parts match. Recovery System remains the source baseline for
navigation/calibration/logging comparisons, with explicit board adaptations.

## Status LED and power saving

MAIN uses the same phase colors for real flight and simulation. The clock-driven animator
has no delays, checks at most every 20 ms and writes only when the color changes.

| State | LED |
|---|---|
| Disarmed | Blue; does not imply readiness |
| Physical gyro calibration | Pulsing blue |
| PAD | Amber |
| BOOST | Red |
| COAST | Magenta |
| DESCENT | Cyan |
| LANDED | Green |
| Power saving | Slow continuous rainbow |

The rainbow uses the normal application brightness, 40/255, with no additional dimming.
`sleep` sets detected LSM6DSV32X accelerometer/gyro ODRs OFF, BMP581 deep standby, and
W25Q128JV deep power-down (B9); SPI interfaces are stopped after successful suspension.
Unavailable devices are skipped, including the board's faulty IMU; no synthetic readiness is
introduced. `wake` releases flash power-down (AB) through normal initialization and probes
sensors again. Flash data and applied configuration are preserved.

Pyro enable is off and servo PWM is detached; there is no assigned switch for cutting servo
supply power. USB, JDY-34 and MCU remain operational, including the rainbow. Battery ADC polling
is paused, but the physical resistor divider remains connected. This is peripheral power
saving, not MCU deep sleep; current reduction has not been measured. Entry/transfer guards and
automatic wake behavior are specified in [USB/Bluetooth commands](technical/USB.md#power-saving).
On 22 October 2026, two USB sleep/wake cycles restored fresh BMP581 samples; configuration,
retained flight identities and first/last sample-block readbacks were unchanged. Bluetooth
SPP connected/responded while sleeping and successfully issued wake with USB still usable.
The automatic five-minute test also entered sleep with both links connected and heartbeats
running; an ordinary Bluetooth configuration-inspection request woke the board, restored fresh
barometer data and preserved the configuration and both retained flights.
The faulty IMU remained unavailable. Physical LED appearance/current and repaired-IMU sleep
behavior still require bench verification.

## Legacy porting reference

The earlier recovery arming workflow requested 500 stationary samples at 10 ms spacing (about
five seconds plus processing). That is provenance for the future calibration port, not
calibration currently reachable on this board: the shared collector is implemented but physical
readiness blocks entry. Earlier inactivity power saving used five minutes,
powered down sensors/flash and left Bluetooth available; it did not establish full MCU sleep.
The current implementation uses the same five-minute inactivity interval.

The current phase colors retain the Recovery System convention, combining its cyan
APOGEE/CHUTE_DEPLOYED indications into DESCENT. Old signed-16-bit altitude/velocity
scaling limited both positive ranges to about 327.67 m / 327.67 m/s; current explicit float32
samples avoid that packed-log limit. Obsolete single-parachute/magnetometer/whole-chip-erase
commands are not ported as current commands. Current ownership and supported commands are in
[architecture](technical/ARCHITECTURE.md) and [USB/Bluetooth](technical/USB.md).
