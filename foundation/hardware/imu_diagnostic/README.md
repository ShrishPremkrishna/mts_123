---
date: 2026-11-01T20:51
tags: [cs123, sensor-diagnostic]
---
# Standalone BNO diagnostic

The [2026-11-02 sensor-only session](../../results/workstream1-imu-sensor-session-20261102.json) exercised this standalone diagnostic on the Pi: inspection, a 30-second baseline, a 60-second comparison, and verified feature restoration. It did not validate robot motion or resolve sensor drift. The exact source passed 836 fake-I2C assertions on the host and Pi; earlier Linux ARM build evidence is [here](../../results/workstream1-offline-ros-20261102.json). Stop the robot/API/native driver and every I2C consumer first. The operator must confirm motors are inactive, the body is supported, and no other process will start using the sensor. `--device-inactive` records that acknowledgment. The executable requires root and runs a bounded `/usr/bin/fuser /dev/i2c-1` check before opening I2C; it refuses busy/unknown ownership. This is a point-in-time check, not a lock protecting against later consumers.

Build separately on the Pi after review; never replace the installed driver:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -pedantic diagnostic.cpp -o bno_diagnostic
```

Inspect product records, calibration flags and effective 02/05/07/08/13 feature fields; queries only, no reset or feature changes:

```sh
sudo timeout -s INT -k 5 20 ./bno_diagnostic inspect NEW_OUTPUT_DIRECTORY --device-inactive
```

Explicitly compare additional reports for 30 seconds; unchanged 02/05 must already be enabled:

```sh
sudo timeout -s INT -k 5 65 ./bno_diagnostic compare NEW_OUTPUT_DIRECTORY --device-inactive --enable=07,08,13 --seconds=30
```

Allowed diagnostic reports are 07 (uncalibrated gyro plus estimated bias), 08 (Game rotation vector), 13 (stability). Only named reports are set, at 100 Hz/100 Hz/1 Hz respectively. Every original feature field must be read first. Direct changes are restored in reverse order; all five features are read back, with one bounded correction for coupled changes. Unknown originals are never assumed disabled. Failures and signals attempt restoration; missing/mismatched restoration is explicit and nonzero. Feature restoration does not reverse background fusion/calibration evolution.

A Set Feature write is not an effective-configuration acknowledgment. After each single write, the diagnostic allows up to one second for an exact full-field match, querying no more often than every 20 ms and retaining mismatched responses. It never resends that write to obtain a match. Restoration uses the same check, then a later all-five validation after at least 20 ms of continued polling; late mismatches fail restoration without another corrective write. All waits remain inside the existing operation/restoration deadlines. These finite readback windows do not eliminate the protocol's missing transaction IDs or prove that configuration can never change later.

For a fresh boot whose inspect replies prove **all five monitored reports disabled**, explicitly opt into temporary baseline reports. This does not call the normal driver's resetting constructor:

```sh
sudo timeout -s INT -k 5 140 ./bno_diagnostic compare NEW_OUTPUT_DIRECTORY --device-inactive --enable-baseline --enable=07,08,13 --seconds=60
```

`--enable-baseline` is accepted only with `compare`. After complete original readback it temporarily changes only the 02/05 report intervals to 10000 microseconds, retaining their other fields. All five features are verified before a fixed 30-second 02/05-only baseline; only then are the named extra reports enabled for the requested comparison duration. An already-enabled monitored report, unknown original or mismatched readback refuses this fresh-boot path. The original snapshots stay immutable and restoration includes 02/05 back to disabled. No calibration setting, reset, tare or DCD write is added. This enables reporting; it is not a calibration treatment or a stationary-health pass.

This opt-in path has a `30 + seconds + 40` second operation bound (130 seconds for the command above), including the final 12-second restoration reserve; ownership checking adds at most three seconds. `capture_start`/`capture_finished` events label `baseline_02_05` and `comparison`. Use those windows when analyzing the two phases. The unchanged summarizer reads all packets, including setup/restoration edges, and is not itself a phase-filtered analysis.

Inspect has a 12-second operation bound; compare without `--enable-baseline` has `seconds+30`, including 12 seconds reserved for restoration. Owner checking adds at most 3 seconds. Kernel I2C blocking, forced kill, power loss or process crashes can prevent restoration; the external timeout is a final bound, not a restoration guarantee. Inspect saved `state.jsonl` before any later sensor use.

Reads, writes, packets, quality flags, raw signed timing deltas and control replies remain in the new exclusive output directory. No reset, tare, calibration-enable or DCD command exists here. Host receipt times are not physical measurement times. Product/feature replies have no transaction ID; serial queries and draining reduce but cannot eliminate stale/unsolicited-reply ambiguity. Product-record completeness remains unknown after the bounded collection.

`python3 summarize.py CAPTURE NEW_SUMMARY.json` compares 02 against 07 minus estimated bias using 5/20 ms host-receipt matching. Exit 0 confirms only procedure completion; missing or unmatched reports remain unavailable. It makes no health or calibration decision. `test_diagnostic.cpp` uses fake I2C only; fixtures distinguish captured 02/05/control bytes from constructed vendor-layout 07 data. The linked session preserves actual ownership checks, I2C responses, restoration, and failed attempts alongside the successful capture. Each later session must establish its own ownership and restoration evidence.
