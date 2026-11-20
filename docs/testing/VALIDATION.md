# Validation evidence and remaining work

This is the single dated evidence record: what was measured or verified, when, and its limits.
Each section carries its date. Local `.pio` artifacts are ignored by git and may not exist in a
clean checkout; their paths record where a result came from. The open work is listed at the end,
in [Remaining work and limits](#remaining-work-and-limits).

## Software verification

The current host suites, what they cover and how to run them are in the [test guide](README.md);
the git history records how they evolved. This section keeps only results that still qualify
behavior. Counts below were true on their date; artifact paths under `.pio` are local and may be
missing in a clean checkout.

### Host-only verification - 19 October 2026

These areas were verified on the PC (C++ suites with fake devices, Python integrations, MAIN
build), with no board upload and no electrical fault injection:

| Area | Covered on the host | Not covered |
|---|---|---|
| Shared runtime | A fake acquisition device feeds raw IMU/pressure through `HardwareSensorSource` and the same `FlightRuntime` as HIL and simulation; navigation and rules match HIL; storage faults leave navigation running | Real acquisition cadence (measured later, see [Hardware](../HARDWARE.md#sensors)) |
| Arming | Blocker masks, missing-IMU rejection without flash writes, RAM-only bias/reference collection, exclusion of edits and runs during arming, USB disconnect, cancellation, airborne disarm rejection | Physical arming (bench result below) |
| Log retention | BOOST history kept independently of flash samples, PAD discard, preservation after cancellation, 513 byte cuts during discard | Real power loss |
| Recording corruption and close | Damaged recordings marked Corrupt while healthy flights stay downloadable (513 interrupted commits, remount, compaction); queued close/abort return immediately; flash ownership held until acknowledgment | **Electrical power-cut qualification** |
| Mounting and bias | All six signed-axis mountings against a dynamic BODY trajectory, bias subtraction and rejection, frozen per-run values | On-board calibration |

### Dual USB and Bluetooth transport - 19 October 2026

MAIN runs the same command protocol on USB and the JDY-34 at once. On the board: handshake,
status, exact-ID retry, RAM-draft ownership, rejection of a change from the other link, closing
the observing link, radio disconnect in the middle of a command, and reconnect all passed
(`python tools/run_bluetooth_test.py --main`). A complete 25 s onboard simulation controlled over
Bluetooth, while USB polled and attempted a rejected stop: **5001 samples, zero missed ticks**,
max cycle **767 µs**, mean **256.9 µs**, max lateness **1366 µs**, phases and virtual pyro as
expected. One run, not a radio endurance or worst-case timing test; BLE, radio loss during a
recorded run and power cycling are not covered.

### JDY-34 bring-up - 19 October 2026

* The MCU UART directions were reversed on the first attempt; the correct routing is module TXD
  → GP5 (MCU RX) and module RXD ← GP4 (MCU TX), on hardware UART1.
* The factory module (`+JDY-34-SPP-BLE-V1.52`) answered at **9600 8N1, CRLF** with PWRC low, and
  ignored the first AT command after an MCU reboot: the startup now tries the initial AT twice.
  MAIN has since moved the module to 230400 baud ([USB](../technical/USB.md#bluetooth-module)).
* Renaming to **Perpetual** was verified by exact read-back of both the SPP and BLE names, and the
  names survived a soft reset (`AT+RESET`).
* Classic SPP carried data byte for byte in both directions on a Windows PC, through the
  **outgoing** SPP port. Windows may keep showing an old cached name until the pairing is
  removed. BLE GATT, sustained throughput and persistence over a module power cycle were not
  tested.

### Desktop app - 20 and 21 October 2026

The Qt app connected automatically (USB first, then paired Bluetooth), ran and displayed the
onboard simulation over Bluetooth (5001 samples, no rejected frames), and downloaded and plotted
a 5001-sample flight over USB whose CSV matched the reference byte for byte. No configuration was
saved to the board during these checks: saves were tested against simulated flash on the PC. The
packaged portable build ran offline on the development PC with Python and Qt removed from the
environment; a separate Windows machine is not yet verified. Workflow and limits:
[Desktop](../technical/DESKTOP.md).

## Sensor bring-up - 19 October 2026

The user supplied the MAIN USB `sensors` and `@42 sensors` results after upload: BMP581 ID 0x50,
driver error 0, valid pressure 101631.578 Pa and temperature 24.239 C, with sample ages 1 and
2 ms. This verifies fresh barometer-only acquisition and the diagnostic reply on that board,
not absolute pressure accuracy or flight timing. IMU ID remained 0x00 and initialization failed,
consistent with the known solder problem. SPI error 0 means the read operation returned without
a transport error; the invalid identity still rejects the IMU. A subsequent user arming/preflight
run rejected physical arming and also reported BAROMETER_UNAVAILABLE; `sensors` then returned
barometer ID 0x00 and driver error -5. The cause has not been established. Further sensor
diagnosis is deferred until solder rework at the user's request; no readiness bypass or driver
workaround was added.

## Navigation reference

*Observed on 18 October 2026.* The independent PC sensor replay and onboard HIL agree over 9433 original Hoppino frames
(integration tolerances 1e-5 for z/vz/bias/acceleration, identical phases/ validity/baro
updates). Changing recorded estimates/phases or converting hPa/Pa does not change computed
results. There were 1603 fresh barometric corrections.

| Sensor-derived phase | Relative time |
|---|---:|
| PAD | 0 ms |
| BOOST | 10138 ms |
| COAST | 12131 ms |
| DESCENT | 14492 ms |
| LANDED | 32162 ms |

Compared to the old quantized recorded estimator, z differs about 0.197 m RMS (max 0.462 m), vz
about 0.130 m/s RMS (max 0.932 m/s). These are regression comparisons, not physical accuracy.
Offline launch reference, input quantization and actual-vs-fixed AHRS dt account for possible
differences; no exact historical trajectory reproduction is claimed.

The independent estimate replay uses recorded DESCENT at 14490 ms and fixture starts
14490/14744/20439 ms. Recomputed sensor navigation with the same fixture starts
14492/14744/20430 ms. The board's saved one-rule USB example produces only Pyro 1 at computed
apogee. Do not use estimate-replay timestamps as expected sensor-navigation results or infer a
fixed plan from the number of starts.

## Flash identification and board timing

Solder rework changed repeated invalid JEDEC reads to EF4018 at all tested modes/ rates. The
read/write test on the final sector passed twice, restoring/verifying its original bytes. This
identifies the W25Q128JV and validates one tested sector, not the entire device or its
endurance. Historical timings below predate asynchronous close: finish measured the full drain,
whereas current finish_us is submission only.

### Observed synchronous baseline - 18 October 2026

MAIN with instrumentation was uploaded to the connected RP2350 on COM9. The flash driver
retained its existing 1 MHz setting. A new sequence 2 recorded all 9433 Hoppino samples with
zero drops, zero rejected navigation frames and no read/program/erase failures. Sequence 1
remained retained. Sequence 2 was then downloaded: its sample CSV is byte-for-byte identical to
the previously verified sequence 1 CSV, and its frozen USB example plan/start metadata remains
correct.

| Scope | Mean | Maximum |
|---|---:|---:|
| Navigation/rules | 389.7 us | 628 us |
| Storage per sample | 4892.8 us | 53431 us |
| Combined step | 5282.5 us | 53929 us |
| Flash read API | 3005.3 us | 7378 us |
| Flash program API | 8695.8 us | 8877 us |
| Flash erase API | 66164.0 us | 72377 us |

Preparation took 27.460 s for the 1 MiB sample reservation plus its header; finish took 75.884
ms. There were 8420 read, 3443 program and 257 erase calls. 858 of 9433 measured steps exceeded
the nominal 5000 us period. The captured report is
`.pio/hoppino_hil_98dac2508387411a8adf4fa33a56e070.timing.json`.

The synchronous path's mean combined step already exceeds the nominal 200 Hz budget, before USB
overhead. A queue must be paired with a separate worker or an asynchronous driver to remove
storage blocking from navigation. At this setting, storage alone uses about 97.9% of the nominal
period on average, leaving little sustained-throughput margin even if computation moves to
another core. Improving that margin and measuring queued-worker behavior are necessary for
periodic-source operation; a queue alone cannot improve average throughput.

### Observed queued worker at 1 MHz - 18 October 2026

Sequence 3 used the 64-sample queue and dedicated core 1, with PC-paced input and no queue
throttling. All 9433 frames passed navigation, but only 9203 were saved: 230 samples were
dropped when the queue filled. There were no flash API failures. Mean producer
compute/submission was 513.2 us, maximum 855 us, with zero measured steps above 5000 us. The
worker mean was 5154.5 us and its maximum 76032 us; queue peak was 64/64. This demonstrates why
bounded producer latency alone does not establish adequate storage throughput.

PC replay elapsed 47.169 s for 47.164 s of source data. 547 sends were late by more than one
nominal period, with 67712 us maximum lateness. The report is
`.pio/hoppino_hil_723c946531ea45deb5231c518a528efe.timing.json`. The recording is retained as
completed with missing samples; that historical format did not contain the RAM dropped-sample
counter. Its stored timestamp gaps remain observable. MAIN was subsequently changed to the
explicit 4 MHz board setting.

### Observed queued worker at 4 MHz - 18 October 2026

Sequence 4 repeated the same PC-paced replay with no queue throttling. All 9433 samples were
saved, with zero drops, zero rejected navigation frames and zero flash API failures. The queue
peak was 4/64. Navigation phases and the single configured virtual Pyro 1 start remained at the
reference timestamps.

| Scope | Mean | Maximum |
|---|---:|---:|
| Navigation/rules | 460.9 us | 758 us |
| Storage submission | 61.6 us | 187 us |
| Producer compute/submission | 522.6 us | 819 us |
| Storage worker iteration | 2037.1 us | 26051 us |
| Flash read API | 839.6 us | 2105 us |
| Flash program API | 3432.8 us | 3777 us |
| Flash erase API | 37020.4 us | 44522 us |

Zero producer steps exceeded 5000 us. Preparation took 12.366 s and finalization 26.091 ms.
Flash API counts were 8420 read, 3443 program and 257 erase calls. Worker time is measured on
the other core and includes verified writes; a worker iteration may exceed one sample period
while the queue absorbs incoming frames.

PC replay elapsed 47.169 s for 47.164 s of source data. Host timing still showed 1459 sends late
by more than one nominal period, maximum lateness 70312 us and maximum USB round trip 7020 us.
This successful run establishes bounded measured producer latency and complete recording for
this USB input; it does not establish autonomous sensor scheduling or worst-case flight
deadlines. Report: `.pio/hoppino_hil_2cac553578164678847fb26d529d375a.timing.json`.

Sequence 4 was downloaded after stop. Its entire 9433-row CSV is byte-for-byte identical to the
verified sequence 1 export, and frozen metadata matches except for sequence number. Stored
timestamps, phases and estimates also match live HIL exactly. The inventory at that check
retained sequences 1 through 4.

## Autonomous profile and storage lifecycle on hardware

*Observed on 18 October 2026.* The 25-second profile computes phases PAD=0, BOOST=5000, COAST=7000, DESCENT=13175 and
LANDED=22180 ms; the saved one-rule plan starts virtual Pyro 1 at 13175 ms. All 5001 sample
timestamps are spaced by 5 ms in a complete run.

| Observation | Result / provenance |
|---|---|
| Sequence 5, autonomous recording | 5001 saved, zero drops/invalid/missed/over-budget; mean cycle 280.3 us, max 868 us; lateness max 1525 us, queue 4/64, prepare 6.199 s, drain 25.892 ms. `.pio/simulation_ae3460dc158a426b9ddbd49cc4db26c9.json` |
| Disconnect-only profile | Continued across USB reconnect and completion: 5001 samples, no misses/invalid frames, max cycle 674 us/lateness 456 us, zero flash calls. `.pio/standalone_disconnected.json`; this was not a power cut |
| Sequence 6, summary upgrade | 5001 saved, zero drops/invalid/misses, max cycle 846 us, queue 4/64; Valid Completed summary after restart; sample CSV identical to sequence 5 |
| Explicit test deletion/reuse | Only one-sample test sequences 7/8 deleted; 8 reused 7's extent at 0x51B000 with a higher identity; cached old selection rejected; retained sequences 1..6 unchanged at that check |
| Current-only format, sequence 9 | 5001 saved, zero drops/invalid/misses/API failures, max cycle 939 us, mean 236.8 us, lateness 1686 us, queue 4/64; prepare 6.112 s, drain 34.632 ms; CSV identical to sequence 6 with matching configuration/virtual start and Valid Completed summary |
| Old header rejection | Existing v2 sequence 6 readable; v1 sequence 1 rejected as Header/UnsupportedVersion with zero program/erase calls; no migration/format performed |

The original Hoppino sequence 1 export was also verified with the then-v1-compatible reader
before compatibility removal. That is historical evidence, not a claim that current firmware
reads v1. Old on-device v1 records remain allocated/listed until explicit deletion. Inventory
statements above are snapshots, not live inventory.

Key export/verification artifacts:

- `.pio/flight_5_4cd7f2abb2b64747b64aa6630f750bf4.csv` and sibling JSON.
- `.pio/flight_6_20cdaabdfbb04360b72dee3a3b201e07.*`,
  `.pio/storage_upgrade_verification.json`, `.pio/storage_upgrade_reuse.json`,
  `.pio/simulation_b053381ba89b415792c880a59bb2e689.json`.
- `.pio/flight_9_simplified.csv` / `.json`, `.pio/storage_simplify_readback.json`,
  `.pio/storage_simplify_verification.json`,
  `.pio/simulation_319c50a7bb314bfd86551451196e663d.json`.
- Sequence 9/6 identical CSV SHA256:
  `5f2146e0f89ed2de3149309166174b4f3ff59cd4f6cc1aed073df996203a586f`.

## Sensor rate, flash speed and flight log v4 - 18 November 2026

Observed on the Waveshare RP2350 board on COM11, disarmed, no outputs commanded. Artifacts were
kept in a session scratch folder only; the numbers below are the record.

**Sensor SPI.** The SENSOR_TEST `f` sweep (2000 rounds per clock of ID reads, configuration
bursts compared with a 1 MHz reference and live-data plausibility, production pull-up chip
selects) was error-free for both sensors from 1 to 25 MHz; at 37.5 MHz every IMU check failed while
the barometer still passed. Both sensors now run at a 10 MHz request (9.375 MHz actual), about
2.7x below the highest clock seen working; a 16-byte IMU burst takes 33 us instead of 159 us.

**IMU rate.** Accelerometer and gyroscope run at the 480 Hz setting; the factory oscillator trim
(INTERNAL_FREQ_FINE = -6, about -0.8 %) makes the true rate 476.3 Hz. A single-transaction
STATUS_REG + data read lost samples (74 Hz in a tight loop, 464 Hz in MAIN) because reading the
outputs clears data-ready. Through the stream FIFO the idle loop measured **476.2 Hz over 30 s
with 0 sequence gaps, 0 FIFO overruns and 0 tag errors**, although 107 loop pauses exceeded one
IMU period (worst 3.5 ms). After the half-period minimum spacing, frame dt stayed within
1870-2162 us; acquisition used about 6 % of core 0.

**Navigation cost.** The onboard Hoppino replay (6600 steps at 200 Hz) measured filter plus rules
at a mean of 302 us and a maximum of 763 us per step without recording, 394 us / 1007 us with
recording enqueue (about 20 % of core 0 at 476 Hz against a 2100 us budget).

**Flash.** The FLASH_TEST `c` sweep (JEDEC, 64 KiB CRC with 0x03 and 0x0B, one programmed page per
clock in the backed-up test sector, restored and verified) passed up to 37.5 MHz; 75 MHz failed
identification and SPI0 cannot produce 50 MHz. The board now uses 18.75 MHz. With the v3 log,
the recording worker went from 21.8 ms per 1 KiB block (4 MHz, 1 ms busy sleeps) to 8.3 ms
(18.75 MHz, 10 us polling, no pre-write read for sample pages) and 6.3 ms (nibble-table CRC, no
fixed transaction delays); 1 MiB preparation went from 11.9 s to 7.9 s and then 3.3 s with
64 KiB block erases.

**Flight log v4.** Recording the Hoppino replay into 1 MiB (sequence 7) saved **5137 of 6600
samples**: the 8.2 s of PAD were reduced to 175 pre-launch history samples and everything from
BOOST on was kept, with **0 dropped**, no recording error and preparation in 3.28 s. The 93 blocks
held about 55 samples each and cost about 7.8 ms of worker time per block including the
read-back (about 7 % of core 1 at 476 Hz, against about 33 % for v3). Writing the history at
boost was the longest worker job, 64 ms, with a queue peak of 15 of 128. The production
downloader fetched the 93 raw blocks in **3.0 s**; the CSV showed BOOST at 8.202 s, LANDED at the
end, maximum altitude 52.17 m, both virtual pyro starts at 13.056 s, 1337 rows with a fresh
barometer reading and 77 deliberate history gaps. Every test flight was deleted afterwards.

**Layout v2 (device journal, directory v2, flight ring).** After the upgrade the board reported
`Directory`/`Corrupt` on the old layout as expected; `flights format`, `config reset`, one rule,
`config apply` and `config orientation +X` then ran on the new journal, each persisted and
verified, and the configuration was still valid after reflashing. FLASH_DIRECTORY_TEST
(directory banks 0x010000..0x02FFFF, 150 updates with automatic and explicit compaction) and
CONFIG_STORE_TEST (device journal 0x000000..0x00FFFF) passed and restored their regions. Two
onboard Hoppino recordings (sequences 1 and 2, consecutive ring placements) saved **4602 and 4603
of 6600 samples** (4603 is the host model: 175 history samples, full rate from BOOST to LANDED,
LANDED at 10 Hz), **0 dropped**, no storage error, 82 blocks each. Preparation took 1.49 s and
1.50 s (eight 64 KiB erases, about 1.16 s in total, longest 159 ms); growth was not needed. The
worker peaked at 75 ms (history write at boost) with a queue peak of 17 of 128. The 200 Hz
simulation loop had a maximum cycle of 1.37 ms; maximum lateness was 2.7 ms with one missed tick
on the first run and 2.4 ms with none on the second. The download took 3.0 s; the CSV showed PAD
history, 5 ms steps from BOOST to DESCENT and 100 ms steps in LANDED. Both flights were deleted.
Growth during a long flight and ring overwrite are covered by host tests only.

**Host.** All 36 runs of `python tools/run_tests.py --full --desktop` passed with the pinned
WinLibs GCC 14.2 (ISO C++17, the firmware's compiler generation), including synchronous and
queued USB recording/download of the Hoppino HIL flight compared sample by sample within half a
stored LSB, and 2049 power cuts across one v4 block write.

## 480 Hz stress profile, stack and ring wrap-around - 19 November 2026

Observed on the Waveshare RP2350 board over USB (COM11), disarmed, VIRTUAL outputs only, with the
`sim begin record 480` profile ([Simulation](SIMULATION.md#480-hz-stress-profile)) and `diag`.

**Stack.** Core 0 starts on the 4 KiB SCRATCH_Y stack, and that is not enough: right after boot
and a connection it had used 5.6-5.8 KiB, `flights info` took it to 7784 bytes and a recorded
480 Hz run to 7988-8184 bytes of the 8 KiB available down to the heap limit (SCRATCH_X holds no
data with `core1_separate_stack`, so the overflow lands there). `-fstack-usage` shows the cause:
each command layer keeps its own reply buffer on the stack (`CommandService::dispatch` 1528 B,
`CommandRouter::dispatchFrom` 1040 B, `dispatchCommand` 1120 B, handlers 1100-1430 B), and
`ConfigurationStore::decode`/`beginDraft` use about 2.8 KiB each. Core 1 peaked at 2572 of 8192
bytes.

*Fix, same day.* The rarely used replies were moved out of the dispatch frames into separate
functions (`CommandService::dispatch` 1528 -> 80 B, `CommandRouter::dispatchFrom` 1040 -> 264 B,
`dispatchCommand` 1120 -> 264 B), `ConfigurationStore::decode` now decodes in place (2880 -> 56 B)
and `beginDraft` no longer builds a temporary plan (2824 -> 16 B). Measured again: 3112 bytes after
boot and connection (was 5752), 3188 after `preflight` (was 6268), 3736 after `flights recover`,
and a maximum of **5056 bytes** during `sim begin record 480` (was 7988-8184); reading a stored
flight stays below that. Each core's MSPLIM now sits 256 bytes above the bottom of its stack: a
forced overflow on the bench stopped at the limit and the board restarted instead of writing
below it. The cause of that restart was not recorded (`last_fault` stayed empty and the watchdog
reason read FORCE, consistent with a lockup reset), so a stack overflow currently shows as an
unexplained restart; other HardFaults reboot through a handler that records CFSR/HFSR.

*Repeat after the fix.* A third ring run (18 recordings, about 25 minutes, wrap at run 16) passed
with no restart (`reset_reason` stayed `soft`, `last_fault` empty), core 0 stack maximum 5112 of
8192 bytes, core 1 2572, worker job maximum 161 ms, erase 173 ms, queue peak 76 of 128, 0 dropped;
11-30 frames per run were still skipped while the PC polled `sim status`. The first-attempt
restart has not recurred in 36 later recordings; its cause remains unproven.

**Unexplained reboot.** In the first ring attempt the board restarted during the fourth 61 s
recording (uptime 69.7 s afterwards); that flight was retained Completed. The reset cause was
not captured (`diag` now reports `reset_reason` and the raw POWMAN/WATCHDOG bits). The second
attempt, 18 runs, had no restart. Candidates: the core 0 stack depth above, a power glitch or USB.

**Timing at 480 Hz.** With USB silent during the run: 29278 frames, **0 missed**, maximum lateness
1.3 ms, core 0 loop maximum 2.56 ms (3 periods above 2083 us), 17749 samples saved (the host
count), 0 dropped. With the PC asking `sim status` every 0.5-1 s, 18-31 frames per run were
skipped (lateness up to 4.4 ms, loop up to 5.8 ms): USB command handling during flight blocks
core 0 for several IMU periods. Step maximum 1.6 ms (compute 1.5 ms) against the 2.08 ms period,
no step over budget.

**Storage.** Pre-launch history full (341 samples). History write at boost, the longest worker
job: 170-178 ms (64-75 ms with the 200 Hz Hoppino); sample queue peak 81-84 of 128 (about 270 ms
at 480 Hz). One growth step per run, 64 KiB erases up to 179 ms, no dropped sample.

**Ring wrap-around.** `ring_overwrite_usb.py` after `flights format`: 18 recordings of 16 blocks.
Flights 1-15 filled blocks 0-239; flight 16 took 240-252 and 0-2, crossing the end of the ring,
and invalidated only flight 1; flights 17 and 18 invalidated 2 and 3. Every placement followed the
previous flight, extents never overlapped, nothing else changed, and the board's
`overwritten_flights` matched. Flights 4 (oldest retained) and 18 were downloaded and decoded
(17728 and 17727 samples). All test flights were deleted afterwards.

## Physical arming on the bench - 19 November 2026

**Barometer timestamp bug.** The first physical arming never completed: the gyro collector kept
restarting after 3-7 samples. The barometer was stamped when read, the FIFO IMU frame when
sampled (never later than its observation), so a fresh pressure was a few ms "newer" than its
frame; `ArmingCalibration` and `Navigation` require barometer time <= frame time and dropped it.
Besides blocking arming, navigation would have ignored almost every real barometer reading in
flight (altitude/velocity from the accelerometer alone, LANDED never confirmed). Simulation, HIL
and replays share one timestamp, so they never showed it. `HardwareSensorSource` now clamps the
barometer time to the frame it travels with (error a few ms); a host test reproduces the board
case (fails without the clamp, collector ready after 2496 frames with it).

**USB load with the real sensors.** Board still in PAD, servo 1 prepared, no pyro assigned,
`bench_arming_usb.py`, three 60 s phases: quiet; app load (status + control status every 4 s,
ping every 2 s); heavy (plus `sensors` every 0.25 s). 86012 frames in total (about 478 Hz),
**0 sequence gaps and 0 FIFO overruns in every phase**. Core 0 loop maximum 3.56 ms quiet and with
the app load, 4.28 ms with the heavy load; loops longer than one IMU period: 9, +60, +392. Step
maximum 1.03-1.05 ms. Frame dt 1049-2168 us: after a stall the queued FIFO frames are processed
at the half-period minimum spacing. USB traffic therefore delays processing by a few ms but loses
no samples; the frames skipped in the 480 Hz simulation with USB polling come from the simulated
source, which drops late frames by design. Core 0 stack peaked at 3832 bytes.

**Real recording with motion.** Repeated Part A (same result: 0 gaps, 0 overruns, loop maximum
4.44 ms), then `--motion`: shaken by hand for BOOST (COAST 6 ms later), left still on the bench
for about 3 minutes under the app load, lowered about 1 m and set down. DESCENT at 185 s fired the
Drogue rule on servo 1 (recorded output start at the same millisecond), LANDED at 190 s, and the
run closed by itself. Flight 39: 85930 samples, 0 dropped, status Completed, 339 pre-launch history
rows; 85442 full-rate samples from BOOST to LANDED at **476.16 Hz with no gaps**, then the slow
post-LANDED rate. The flight grew 6 times (allocated 3.5 MiB) with 65 block erases, longest 179 ms;
queue peak 84 of 128. Altitude estimate drifted -0.3 to -0.9 m over 3 minutes still, then fell to
-1.9 m when the board was lowered: navigation uses the real barometer.

Three maxima looked alarming and are not flight data:
- Worker maximum 1.63 s: the asynchronous preparation at arming, which erases the first 8 blocks
  (8 x ~180 ms) before the source starts, so no samples exist yet. Recording-time jobs stayed at
  the erase time.
- 1 FIFO overrun in the run (2 by the final `diag`): Part A deltas were 0 and the runtime saw no
  sequence gap, so it happened while the IMU FIFO was not read: during that preparation, before
  `source.start()` re-anchors. The second came after the run ended.
- Core 0 loop maximum 12.3 ms: `diag` loop statistics and the overrun count restart at run start
  but keep counting after it ends, so this includes the post-landing `flights` commands; it cannot
  be placed inside the flight. A stall that short is absorbed by the IMU FIFO either way (step
  maximum 1.52 ms).

`diag` and `flights timing` were then changed so these figures describe only the run: loop and
overrun figures stop at run end, overruns count from the first processed frame, and the initial
erase is reported as `prepare_us`, outside `worker`.

**Full ring with real sensors - 20 November 2026.** Same procedure with that firmware, but the
board stayed on the bench in COAST for about 20 minutes before being lowered, so the recording
filled the whole flight ring: flight 40, 31 growths, 254 block erases (longest 175 ms), the one
older retained flight overwritten, 16.58 MB allocated. 464015 full-rate samples at **476.20 Hz
with no gaps** and longest frame spacing 2165 us, then StorageFull at 985 s: the 108825 later
samples were dropped as designed, and the flight continued without storage. DESCENT, detected when
the board was finally lowered (1193 s), still fired servo 1 and its output start was recorded; the
run closed by itself. Over the whole run (Part A included): 0 sequence gaps, **0 FIFO overruns**,
worker maximum 172 ms (preparation at arming 1516 ms, separate), queue peak 81 of 128, step
maximum 1.52 ms, core 0 stack 3800 bytes. The core 0 loop maximum, now inside the run only, was
16.3 ms: a real stall whose source is not yet located (not a processing step; the IMU FIFO
absorbed it). The altitude estimate drifted between +0.2 and +1.7 m over 16 minutes still
(barometric drift).

**Hand shake detected as a whole flight.** Next run, with `loop_max_at`: Part A loop maxima
3.5-4.3 ms, all in PAD while reading USB input. The shake gave BOOST at 10.535 s, COAST 26 ms
later and DESCENT 189 ms after BOOST (velocity -0.54 m/s at 0.22 m), which fired servo 1; LANDED
at 15.8 s and the run closed after the 15 s landing window (flight 41: 2995 samples, Completed).
The detector confirms BOOST after 3 consecutive frames above 3 g and COAST after 3 frames of
non-positive BODY X acceleration; at 476 Hz that is about 6 ms each, and DESCENT needs only
-0.5 m/s. **A knock of a few ms on an armed rocket could therefore deploy the recovery on the pad.**
Fixed the same day: launch now needs a sample above 3 g and then 1.5 g held for 200 ms, and burnout 30 ms (see [State machines](../technical/STATE_MACHINES.md#normal-flight-machine)). Run maximum loop 5.1 ms at 230.6 s, in LANDED without USB input (the run
closing); the 16.3 ms of the full-ring run did not recur without growths.

## Initial alignment and tilt on the bench - 20 November 2026

Test firmware (identity IMU calibration), physical arming in normal mode, board held by hand and
set against a goniometer; `tools/replay/bench_tilt_usb.py` read the `attitude` field of `status`
for 3 s per position (about 30 readings) while the board stayed in PAD.

| Position (goniometer) | Tilt read | Spread | `up_body` |
|---|---|---|---|
| Upright, at arming | 5.9° | 5.5-6.2° | [0.995, 0.019, 0.100] |
| 20° | 22.7° | 22.2-23.1° | [0.923, -0.077, 0.378] |
| 40° | 39.4° | 39.1-39.9° | [0.772, -0.018, 0.635] |
| 90° | 89.2° | 89.1-89.3° | [0.014, 0.012, 1.000] |
| 90°, turned about 20° around the rocket axis | 89.9° | 89.8-90.0° | [0.003, 0.345, 0.939] |
| Upright again | 1.0° | 0.6-1.5° | [1.000, 0.009, 0.014] |

BODY +X is the rocket axis, the gravity alignment at arming is correct and the tilt agrees with the
goniometer within about 3° over 0-90° (hand-held; the 5.9° at arming is most likely the hand, since
the same position read 1.0° later). A turn around the rocket axis keeps the tilt and moves only the
direction in `up_body`. A 30 s hold at about 40° moved 41.4 → 43.7°: in PAD the accelerometer keeps
correcting the attitude (gain 0.3), so this reflects the hand, not gyroscope drift. **Not covered:**
the gyroscope-only attitude of BOOST and COAST, where the correction gate (|a| within 0.2 m/s² of
1 g) is closed; that is where the bistage 40° check applies.

**Gyroscope-only attitude, hand-held.** Same day, test build with `nav gyro start`
(`bench_tilt_usb.py --gyro`): after arming, the attitude was copied into a second estimator with
no accelerometer correction, and its vertical compared with the normal (gravity-corrected) one.
The board was held by hand, **also during the arming gyro collection**.

| Elapsed | Motion | Angle between the two verticals |
|---|---|---|
| 0 s | upright, start | 0.3° |
| 57 s | held still | 3.5° |
| 113 s | to 40° (after 53 s more) | 8.3° |
| 123 s | to 90° | 9.5° |
| 132 s | back upright | 9.5° |
| 173 s | upright | 12.7° |
| 190 s | after 5 s of fast twists and swings | 13.6° |

The divergence grows steadily at 3.4-3.7°/min whatever the motion, always as a rotation about
BODY Y (the gyro-only vertical moves towards BODY +Z): a residual gyroscope bias of about
0.06°/s (1 mrad/s). Rotations themselves add at most about 1-2° (90° and back added nothing
beyond drift; fast motion added nothing), so gyroscope scale and axes are consistent. The bias
is most likely the hand moving during the 5 s arming collection, whose stillness gate (0.5 rad/s)
accepts a hand-held board: about 0.3° of hand rotation there gives this bias. Over 10-20 s of
BOOST and COAST 1 it would amount to about 1°, against the 40° limit. **To repeat** on a stand ([bench stand](https://github.com/rotellini9809/perpetual_pcb/tree/main/mechanical/bench-stand)),
still during arming and for a longer still phase, before deciding whether to tighten the gate.
Vibration, high spin rates and acceleration effects on the gyroscope are not covered on the bench.

## Remaining work and limits

This is the single list of open work: what is implemented but not yet qualified, and what is not
implemented. Dated observations above retain the firmware/format used at the time; in
particular, older v2/v3 log and default-mounting observations do not describe the current
recording header v8, flight log v4 or the explicit orientation requirement. The current
implementation is described in the [technical references](../README.md).

| Area | Status and what is still open |
|---|---|
| Real sensors | Idle IMU cadence/pairing measured (476.2 Hz over 30 s, no gaps; see [Hardware](../HARDWARE.md#sensors)); gyroscope collection during a physical arming works on the board after the barometer timestamp fix, and a hand-driven bench flight reached DESCENT and LANDED with the real barometer (19 November 2026). A real flight profile is not yet measured |
| Calibration and attitude | Per-arming gyro/reference collector, persistent mounting and six-face accelerometer calibration (persistence and UI) exist. The board still uses identity calibration values: run the calibration on the board. Gravity alignment and tilt reading agree with a goniometer within about 3° in PAD (20 November 2026, above); gyroscope-only attitude (BOOST/COAST) tracks rotations within about 1-2°, with 3.5°/min drift after a hand-held arming; to repeat on the bench stand. See [Navigation](../technical/NAVIGATION.md#calibration-and-mounting) |
| Bistage flights | Normal/bistage selection, second-ignition detection (3 g, then 1.5 g held for 200 ms, as at launch), median/tilt ABORT and completion of active outputs are implemented. Thresholds and initial attitude still need qualification on hardware; invalid-data windows inhibit new starts without inventing a timeout. See [State machines](../technical/STATE_MACHINES.md#bistage-flight-machine) |
| Physical outputs | Shared execution, preparation/release and physical simulation are implemented and host-tested; pyro firing with real igniters works on the board and does not reset the MCU. Servo PWM and mechanical end points, interruption, load and firing-current margins still need bench acceptance |
| Airbrakes | Firmware contract, logging, graphical controls and log plots are implemented with a zero-output placeholder law. The chosen law must be implemented and qualified, and end points verified on hardware. See [Airbrakes](../technical/AIRBRAKES.md) |
| Failure handling | No MCU watchdog yet. A driver timeout disables driver readiness and the recorder cannot reinitialize it: a restart is needed. Sensor outage and degraded estimates, stalled-worker limits and cleanup of a failed arming's reservation are not specified. See [Storage](../technical/STORAGE.md#failure-diagnostics-and-limits) |
| Interrupted/old headers | Unsupported versions fail without migration; current layout v2 (device journal v1, directory v2, flight ring), recording header v8, sample blocks v4, configuration codec v3. A board with the previous layout needs `flights format` and its rules, settings and calibration saved again |
| Scheduler and storage | Worker close is asynchronous, but a stalled worker retains ownership; idle preparation/settings/scans stay synchronous. The pre-launch history write at boost measured 64-75 ms on the Hoppino replay; growth erases with real sensors measured on the bench (up to 31 growths filling the ring, 476 Hz, no gaps; one 16 ms core 0 stall not yet located); worst-case timing in a real flight and physical brownout coverage are not established. Ring growth/wrap-around/overwrite, 480 Hz load and stack depth measured on the bench on 19 November 2026 (see above): core 0 stack reduced to 5056 of 8192 bytes with a MSPLIM guard; USB traffic delays processing but loses no real IMU frames; one unexplained reboot |
| Desktop and transports | Six-view USB/SPP app, IMU calibration, orientation, reset/format and power management exist. Not implemented: BLE, direct channel tests, telemetry/monitor subscription and receiver-negotiated binary export (frame, handshake and resume rules). See [USB/Bluetooth](../technical/USB.md#command-reference) |
| Release | Reproducible physical flight acceptance, versioned board/firmware qualification and checks of the portable app on a separate Windows machine. Firmware updates from the app are not provided |

Host NOR byte cuts model persistence protocols; a restart after a completed recording is not a
physical interruption during program/erase. Normal-profile maxima are not worst-case guarantees.
