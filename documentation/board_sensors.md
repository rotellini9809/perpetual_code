# Board sensors

Fitted components confirmed by the board owner:

| Function | Fitted component | Software driver | CS GPIO | Identity register | Expected ID |
| --- | --- | --- | --- | --- | --- |
| Barometer | Bosch BMP581 | Bosch BMP5_SensorAPI | GP26 | CHIP_ID, 0x01 | 0x50 |
| IMU | ST LSM6DSV32XTR | ST lsm6dsv32x-pid v4.1.1 | GP27 | WHO_AM_I, 0x0F | 0x70 |

LSM6DSV32XTR is an ordering code for the LSM6DSV32X supported by the ST driver.
The Bosch driver supports BMP581. Its shared BMP5 identity check also accepts
0x51; accepting that value alone does not establish that the fitted part is BMP581.

## SPI wiring and initialization

Both sensors share SPI1: SCK GP14, MOSI GP15, and MISO GP12. The firmware
uses four-wire SPI, not I2C device addresses.

Both CS inputs are active-low: HIGH deselects a sensor; LOW selects it.
They require an inactive high level, not a pull-down.

The smoke test configures GP27 and GP26 as INPUT_PULLUP at the start of
setup(), before waiting for USB serial. Sensors::begin() repeats this
configuration before sensorSpi.begin(false), waits 5 ms, then initializes
the barometer and IMU. The false argument keeps chip-select control in software.

The existing transaction helpers deselect both sensors using INPUT_PULLUP,
then drive only the selected sensor's CS LOW as an OUTPUT. At the end of
each transaction, both return to INPUT_PULLUP. Inactive CS therefore relies
on the GPIO pull-up rather than an actively driven HIGH output. Firmware
does not control the pins before setup() runs; external pull-ups determine
the board's inactive levels during reset if required.

The barometer probe tries SPI modes 0 and 3 at 1 MHz and 100 kHz. The IMU
probe tries mode 3, then mode 0, at 1 MHz. Both drivers check device identity
and configure measurements before reporting initialization success.

## Diagnostic limits

Matching drivers and correct CS polarity do not prove the soldering, power,
or bus signals are correct. Hardware operation still needs verification.
IMU error 0 can accompany a mismatched identity because the SPI callback
returns success after transferring bytes without confirming a device response.
BMP5 error -5 is BMP5_E_NVM_NOT_READY; invalid SPI status data can also cause it.

## Troubleshooting output

The smoke test enables initialization tracing with `sensors.begin(&Serial)`.
Normal callers can keep using `sensors.begin()` without extra logging.
Diagnostics live in `src/SensorsDiagnostics.cpp`, as methods of `Sensors`.

At startup, trace lines show each attempted identity probe and each barometer
initialization step. IMU reset and aggregate measurement configuration results
are logged, and identity mismatch is explicitly reported even if transport
error is zero.

`sensors.troubleshoot(Serial)` runs automatically after initialization; send
`D` or `d` to repeat it. It is ignored while a pyro pulse is active. It reports:

- SPI pin assignments and inactive CS input levels (both should read 1).
- Stored initialization results, separate from current diagnostic reads.
- BMP581 ID, revision, interface status, NVM status and measurement configuration.
- Three ID reads per sensor for each combination of mode 0/3 and 1 MHz/100 kHz.
- MATCH, MISMATCH or UNSTABLE for each combination, with diagnostic hints.

The sweep uses copies of SPI settings and does not reset sensors or change
measurement configuration, readiness flags or stored errors. It avoids FIFO
and interrupt-status reads. Its extra SPI transactions and serial output can
affect timing, so it is a bench diagnostic, not a continuous measurement mode.
An observed GPIO HIGH only confirms the MCU pin level, not continuity to the
sensor package. IDs and status values are evidence, not a soldering verdict.

Capture the output from `[sensor probe]` through `SENSOR DIAGNOSTICS END`,
including the initialization trace. Compare 1 MHz against 100 kHz and repeated
IDs before interpreting the NVM bits. Reboot to repeat initialization itself.

## Sources

- [Bosch BMP581 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf)
- [ST LSM6DSV32X datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv32x.pdf)
- [ST LSM6DSV32X product and ordering information](https://www.st.com/en/mems-and-sensors/lsm6dsv32x.html)
- [Bosch BMP5 Sensor API](https://github.com/boschsensortec/BMP5_SensorAPI)
- [ST LSM6DSV32X driver](https://github.com/STMicroelectronics/lsm6dsv32x-pid)
