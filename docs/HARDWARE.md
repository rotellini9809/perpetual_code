# Hardware reference

The firmware targets the Perpetual board (Rev 4): a Waveshare RP2350 Tiny running Arduino-Pico.
Schematics, PCB layout, connectors and power design are in the
[perpetual_pcb](https://github.com/rotellini9809/perpetual_pcb) repository. This page covers what the
firmware needs to know about that hardware.

## Pin map

GPIO numbers are RP GPIO numbers, not package or header pin numbers. The authoritative map is
[board_pins.h](../include/hardware/board_pins.h).

| Interface | Assignment |
|---|---|
| W25Q128JV flash, SPI0 | MISO GP0, CS GP1, CLK GP2, MOSI GP3; 18.75 MHz (works up to 37.5 MHz, see FLASH_TEST `c`) |
| LSM6DSV32X IMU / BMP581 barometer, SPI1 | MISO GP12, CLK GP14, MOSI GP15; IMU CS GP27, barometer CS GP26. Interrupt pins are not routed |
| Bluetooth JDY-34 | Hardware UART1 (`Serial2`): MCU RX GP5 receives module TXD; MCU TX GP4 drives module RXD; PWRC GP17 (AT/wake control); no connection-status pin |
| Pyro 1..4 / shared enable | GP9, GP13, GP6, GP7 / GP8 |
| Continuity mux / ADC | Select GP11, GP10 / sense GP28 (ADC2) |
| Battery sense | GP29 (ADC3); 100 kΩ battery-to-ADC, 24 kΩ ADC-to-GND |
| Telemetry / external I2C | GP18 / GP19 (TX/SDA, RX/SCL) |
| RGB LED | GP16 |
| Servo PWM 1..4 | GP21, GP20, GP23, GP22 |

The flash is **128 Mbit / 16 MiB**; the driver accepts JEDEC IDs EF4018 and EF7018. Its
[fixed region map](technical/STORAGE_FORMAT.md#fixed-memory-map) is the only supported layout.
No magnetometer is fitted. A pin or communication failure alone does not prove a defective chip;
see [diagnostics](testing/README.md).

## Sensors

MAIN probes both sensors at boot and polls them through `HardwareSensorSource`, for idle
diagnostics and for physical flight. Missing samples are never faked, and physical activation is
blocked by any reported sensor, configuration, continuity or preparation fault. See
[USB diagnostics](technical/USB.md#board-state).

`Sensors` configures the accelerometer at ±32 g and the gyroscope at ±2000 °/s, both at 480 Hz
nominal (476.3 Hz on the current board: the ODR scales with the factory trim INTERNAL_FREQ_FINE,
0.13 %/LSB), and the BMP581 at 100.2 Hz with 4x pressure/temperature oversampling. Returned
measurements are m/s², rad/s, °C and Pa. Both devices run SPI at a 10 MHz request (9.375 MHz
actual): the SENSOR_TEST `f` sweep read IDs, configuration bursts and live data without error from
1 to 25 MHz, and the IMU failed at 37.5 MHz. The autonomous simulator uses its own 200 Hz IMU rate.

The IMU runs its FIFO in stream mode (accel + gyro at 480 Hz, temperature at 15 Hz). When its RAM
queue is empty, `read()` performs two transactions, FIFO_STATUS1/2 then one burst of tagged words,
pairs accel/gyro words into frames and returns one frame per call; overruns and unexpected tags
are counted and mark the next frame as a gap. A single STATUS_REG + data burst is not used: reading
the outputs clears data-ready, so a sample arriving mid-burst would be lost. The barometer status is
only read once a new 100 Hz sample can be due. The barometer keeps working if the IMU fails to
initialize.

`HardwareSensorSource` timestamps FIFO frames on the IMU period: each frame advances by exactly
one period and is pulled 1/32 toward the observed arrival estimate (status-read time minus newer
queued frames), never later than it, so dt stays regular and drift cannot accumulate. A gap
re-anchors on the observation and breaks sequence continuity. Measured idle over 30 s: 476.2 Hz,
0 sequence gaps/overruns/tag errors despite loop stalls up to 3.5 ms, dt 1809-2165 µs around
2100 µs. Timestamps are estimates within about one poll period, not interrupt edges. Mounting and
bias calibration must be verified on hardware before live flight qualification.

## Outputs

**Pyro.** `flight::OutputExecutor` is the only owner of the outputs and drives any combination of
the four channels at once through `PyroController::drive(enabled, mask)`. Each firing raises the
shared enable, waits 10 ms for it to settle, then holds the channel for a fixed 1000 ms pulse.
The outputs and enable stay inactive at boot and whenever the board is disarmed.

**Continuity.** The mux selects one channel, settles, discards one ADC conversion, then averages
16 readings. Raw 12-bit thresholds: below 1000 BatteryNotConnected, 1000..2499 Open, 2500 and above
Connected; an invalid channel is Uncertain. These are software classifications, not an
independent check of power or ignition, and the battery sense is not read by this measurement.
Continuity is only measured on request, so it never delays a firing.

**Servo.** `ServoController::begin()` only drives the four pins low: servo PWM stays detached at
boot. Servos move only through an explicit preparation, a preview while disarmed, or a physical
run, always to the configured PWM positions.

## Battery

MAIN measures the battery at 5 Hz with 12-bit ADC reads: it discards one conversion after switching
the input, then averages eight. Conversion uses the nominal 3.3 V ADC reference and the divider
ratio 31/6, so the ADC saturates at about 17 V, above a fully charged 3S LiPo (12.6 V). The board
is designed for 2S or 3S LiPo. A saturated ADC reports no value rather than a clipped one. The
reference and divider tolerances have not been calibrated against a multimeter. The voltage is
telemetry only and is not a readiness condition.

## Status LED and power saving

MAIN uses the same phase colors for real flight and simulation. The clock-driven animator has no
delays, checks at most every 20 ms and writes only when the color changes. Brightness is 40/255.

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

After five minutes without operational activity, `sleep` sets the LSM6DSV32X accelerometer and
gyroscope ODRs off, puts the BMP581 in deep standby and the W25Q128JV in deep power-down (B9);
SPI interfaces are stopped after successful suspension. Unavailable devices are skipped; no
synthetic readiness is introduced. `wake` releases flash power-down (AB) through normal
initialization and probes the sensors again. Flash data and the applied configuration are
preserved.

During sleep the pyro enable is off and servo PWM is detached; there is no switch for cutting servo
supply power. USB, the JDY-34 and the MCU stay on, including the rainbow. Battery ADC polling is
paused, but the resistor divider stays connected. This is peripheral power saving, not MCU deep
sleep, and the current reduction has not been measured. Entry/transfer guards and automatic wake
are specified in [USB/Bluetooth commands](technical/USB.md#power-saving). Physical LED appearance,
current and IMU sleep/wake behavior still require bench verification.

## Datasheets

| File | Part |
|---|---|
| [LSM6DSV32X](datasheets/DS_lsm6dsv32x.pdf) | IMU |
| [BMP581](datasheets/bst-bmp581-ds004.pdf) | Barometer |
| [W25Q128JV](datasheets/W25Q128JV.pdf) | SPI NOR flash |
| [JDY-34 manual V1.3](datasheets/JDY-34%20High%20Speed%20Dual-mode%20Bluetooth%20User%20Manual-V1.3.pdf) | Bluetooth module: AT commands, factory defaults (9600, BTMODE1, ENLOG1) and pins |
| [FS90](datasheets/FS90-specs.pdf) | Reference servo; mechanism end points still need calibration |
