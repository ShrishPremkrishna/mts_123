---
date: 2026-11-01T20:51
tags: [cs123, sensor-diagnostic]
---
# Isolated BNO diagnostic candidate

**Not deployed or hardware-validated.** Strict Linux ARM compilation and the
fake-I2C tests now pass through `foundation/scripts/check_ros_offline.py`; see
[the build evidence](../../results/workstream1-offline-ros-20261102.json). Stop the robot/API/native driver and every I2C consumer first. The operator must confirm motors are inactive, the body is supported, and no other process will start using the sensor. `--device-inactive` records that acknowledgment. The executable requires root and runs a bounded `/usr/bin/fuser /dev/i2c-1` check before opening I2C; it refuses busy/unknown ownership. This is a point-in-time check, not a lock protecting against later consumers.

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

Inspect has a 12-second operation bound; compare has `seconds+30`, including 12 seconds reserved for restoration. Owner checking adds at most 3 seconds. Kernel I2C blocking, forced kill, power loss or process crashes can prevent restoration; the external timeout is a final bound, not a restoration guarantee. Inspect saved `state.jsonl` before any later sensor use.

Reads, writes, packets, quality flags, raw signed timing deltas and control replies remain in the new exclusive output directory. No reset, tare, calibration-enable or DCD command exists here. Host receipt times are not physical measurement times. Product/feature replies have no transaction ID; serial queries and draining reduce but cannot eliminate stale/unsolicited-reply ambiguity. Product-record completeness remains unknown after the bounded collection.

`python3 summarize.py CAPTURE NEW_SUMMARY.json` compares 02 against 07 minus estimated bias using 5/20 ms host-receipt matching. Exit 0 confirms only procedure completion; missing or unmatched reports remain unavailable. It makes no health or calibration decision. `test_diagnostic.cpp` uses fake I2C only; fixtures distinguish captured 02/05/control bytes from constructed vendor-layout 07 data. Actual Pi ownership checks, I2C responses and restoration still require a separate reviewed sensor-only session.
