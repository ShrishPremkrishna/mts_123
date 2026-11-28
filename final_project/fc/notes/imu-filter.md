# IMU crutch: our own tilt filter in the stock driver (2026-11-24)

**Nov 27:** v4 = v3 + accel bias calibration (`FC_IMU_CAL`), built, not installed: [RCA](rca-20261127-tilt.md). **Status (Nov 25):** v3 (stall guard) built, not installed: [v3](#v3-bno-stall-guard-2026-11-25-built-not-installed). **Nov 24:** v2 installed on mts and on by default in the stock profile (`FC_IMU_FILTER=0` = stock behaviour); see [Deployment status](#deployment-status-2026-11-24).

**Patch:** `fc/robot/imu/stock-imu-filter.patch`, sha `b0d57361…` (v3 = v2 + stall guard; v2 was `983a6e49…`, in git history).
- Applies to the Pi stock driver (`~/pupperv3-monorepo` `0e50afb`, `control_board_hardware_interface` at `e333272`).
- It edits the same files as `stock-bno-dyncal.patch`, so apply one or the other, not both.

**Built .so:** v2 at mts `~/fc-v1/build/imu-filter-v2/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so`, sha256 `19d60fad…a3a2`.
- v1 (`~/fc-v1/build/imu-filter`, `2486fa0a…`) is what the stack was running on 2026-11-24 ~07:45 UTC, through the installed symlink. v2 went to a new directory so the live file was not overwritten.
- **v1 live finding (homed on the stand):** gyro reports arrived at about 100/s and accel at about 127/s. Yet v1 counted the gyro as "frozen" for 20–27 s at a time, and so held the published tilt.
- **This is the chip, not the parser:**
  - Every gyro report is decoded (the count rises 100/s), at the stock byte offsets.
  - The values do change occasionally (0.2 s episodes, bias updates).
  - The independent direct-I2C reader saw the same thing with motors off: the calibrated gyro repeated for 6.5 s.
  - A still BNO repeats ~0. v1 treated that as a freeze.
- **v2 change:** a freeze now also needs the held value to be off zero (more than 0.02 rad/s) or the accel to show motion (more than 0.3 m/s²). The motors-off recording now shows 0 frozen samples, and a new unit test covers it.
- Built RelWithDebInfo, the same build type as the installed stock .so (`01813d58…`).
- The only compiler warning is the stock `unused sequence_number`.
- `colcon test -R test_fc_imu_filter`: 1 test, 0 failures. The output is in `test_output.txt` next to the build.

## Why
The BNO's fused rotation vector (0x05) trusts the gyro too much.
- **Fitted model:** replaying ks4/ks6 through a weak-accel filter with kp ≈ 0.2/s reproduces the chip's false 19–35° tilt.
- **Effect of the fault:** the gyro-X bias and pulses turn into a tilt that is not there, and the policy then "corrects" it.
- **What the walker needs:** only roll and pitch (the direction of gravity) plus the body rates. Yaw does not matter with heading hold off.

## What `FC_IMU_FILTER=1` does
The filter lives in `include/rt/fc_imu_filter.hpp`: header-only, no ROS or Eigen. The driver, the unit test and the Python harnesses all run this same header.

1. **Driver changes:**
   - also enables the calibrated accelerometer (0x01) at 10 ms;
   - parses every report in a packet, not just the first;
   - feeds the filter at each gyro report, with dt taken from the report sequence number.
2. **Filter:** a Mahony complementary filter in the sensor frame, kp 2.0/s and ki 0.3.
   - The accel is low-pass filtered **as a vector** (τ 0.1 s) before it corrects the estimate. Without this, sim showed a steady 4–10° error at the fast gait, and the robot fell 3/3 at 0.75 m/s with no fault at all.
   - The correction gain is halved when |a| is more than 30% from 1 g, and the accel is ignored when |a| is more than 100% off.
3. **Gyro-X cleanup:**
   - **Body still** (Y and Z both under 0.03 rad/s): samples more than 0.015 below the clean X level are replaced by that level. The clean level is tracked as the 90th percentile, because the pulses only go negative.
   - **Body moving:** X minus that level minus the average pulse size learned while still.
   - **Published rate:** X cleaned as above; Y and Z exactly as measured.
4. **Freeze** (gyro bit-identical for 40 ms or more, **and** the held value is off zero or the accel shows motion):
   - the published rate is 0, and the published quaternion is **held bit-identical**;
   - inside the filter, the estimate keeps following the accel.
   - Why: in sim, walking through 0.3–1.5 s freezes still fell 4 times in 6 even with a good tilt estimate, because the policy has no body rate. Holding the output keeps the bridge's existing "frozen" stop working.
   - Gyro and accel both frozen: everything is held.
5. **Fallbacks:**
   - The stock rotation vector is used until the first accel sample arrives, and whenever the accel stream goes missing for more than 0.25 s.
   - Optional tuning by env: `FC_IMU_FILTER_KP` (0–20) and `FC_IMU_FILTER_KI` (0–5).
6. **Logs** (go to robot.log):
   - `FC_IMU_FILTER on: …` at start;
   - every 10 s: report counts, despiked samples, frozen samples, X baseline and pulse mean, bias, and `rv_tilt_diff` (how far the chip's own tilt is from ours);
   - `gyro frozen X s` for each freeze of 0.2 s or more.

## Evidence
**Real chip at rest (motors off):** 0x07 gyro and 0x01 accel from `~/fc-v1/imu/runs/20261124T064621Z…/bno.csv`. The filter agrees with the BNO rotation vector to within 0.016° median (0.05° max). This confirms the axes, the accel sign and the mount.

**Replay of the recorded faulty gyro (ks4, ks6):**
- Truth is the floor-normal direction from joint FK of the production model (the plane through the four feet). It is calibrated against the BNO in a clean still window.
- The stock driver never enabled the accel, so the accel is synthetic: truth gravity plus 0.03 m/s² noise, and while the joints move, ±2 m/s² at 2.5 Hz plus 1 m/s² noise.
- Results at 100 Hz (`fc/robot/sim/imu_filter/results/`, plot `replay_tilt_error.png`):

| session | BNO max tilt error, still | filter max (p99), still and ≥3 s from motion | quiet raw-X std, raw → published |
|---|---|---|---|
| ks4 | 30.9° (35.8° incl. transitions) | **1.0° (0.4°)** | 0.061 → 0.005 rad/s |
| ks6 | 2.7° (19.0°) | **1.5° (0.2°)** | 0.014 → 0.005 |

- While the joints move, both the BNO and the filter read about 1–3° median against FK. FK is unreliable during kicks and swing, so treat those numbers as indicative only.
- In the faulty walking windows of ks4 (800–950 s), the BNO median error was 30° and the filter's 1°.

**Unit test** (`test/test_fc_imu_filter.cpp`, simulated −135° mount):
- clean: 0.08°;
- still, X bias −0.115 plus 2000 pulses/min: 0.07°;
- moving ±6° with 2 m/s² of gait accel plus X faults: p99 2.6°, max 3.5°;
- 1.5 s gyro freezes while moving ±3°: internal estimate max 5.4°. This is the lag of running on accel alone, and is why the published output is held during a freeze.

**MuJoCo** (production policy `ecf3a4ab` on the `fast-gait.md` harness, heading hold off, 3 replicas × 18 s):
- Faults are injected on sensor X: one-sided pulses at 0.03–0.3 rad/s lasting 15–40 ms, bias, and 0.3–1.5 s freezes.
- "stock" means the fitted kp 0.2 BNO model.
- Files: `sim_summary.txt`, `sim_attr.txt`, `sim_final.txt`.

| faults | stock falls | filter falls |
|---|---|---|
| none (stand / 0.3 / 0.75 / sequence) | 0/12 | 0/12 (final header: 0/6 at 0.75 and sequence) |
| mild: bias −0.03, 10 pulses/s | 2/12 | **0/12** |
| X bias −0.115 + 33 pulses/s, no freezes | 6/6 | **0/6** (IMU error p95 3.1–3.6°) |
| freezes every 5 s while walking | 6/6 | 4/6 (before the output hold; with the hold, the bridge's frozen guard stops the walker as today) |
| standing at zero command, any fault | 0 falls; false tilt 11–37° | 0 falls; error 0.3° |

## Deploy (operator, robot on the stand)
Do this only with the session owner's approval, and only with the stack stopped.
```
I=~/pupperv3-monorepo/ros2_ws/install/control_board_hardware_interface/lib
B=~/fc-v1/build/imu-filter-v2
~/fc-v1/robot/fcctl.sh stop
sha256sum $I/libcontrol_board_hardware_interface.so      # expect 01813d58...c7ced5a1 (stock)
cp -p $I/libcontrol_board_hardware_interface.so $B/stock-01813d58.so
cp $B/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so $I/
sha256sum $I/libcontrol_board_hardware_interface.so      # expect 19d60fad...a3a2 (v2)
FC_IMU_FILTER=1 ~/fc-v1/robot/fcctl.sh start stock_hh0 --confirm   # homes the legs
grep FC_IMU_FILTER ~/fc-v1/runs/<new run>/robot.log    # "on: kp=2.000", then 10 s stats, accel=... > 0
```
- **Stand check:** at zero command, `fc.ctl status` tilt should stay under 3° for 5 min. The `rv_tilt_diff` log field shows how far the chip's own tilt has drifted.
- **Then the floor,** with an R3 operator.
- **The variable is needed:** without `FC_IMU_FILTER=1`, the same .so runs stock. The variable reaches the driver only when fcctl is started from a shell that has it set. The app's `ready` op does not pass it.

**Rollback:**
```
~/fc-v1/robot/fcctl.sh stop
cp $B/stock-01813d58.so $I/libcontrol_board_hardware_interface.so && sha256sum $I/libcontrol_board_hardware_interface.so   # 01813d58...
```

## Gaps and cautions
- **Not checked on the real chip:** the accel enable and the multi-report parse. The stack held the IMU the whole time, so the device was not touched. The first stand start is that check: the stats line must show `accel>0`.
- **Accel during the fault is unmeasured.** The note calls it "likely clean". If the accel is also corrupted when the motors are powered, the replay numbers do not hold. Recommended check: log `/imu_sensor_broadcaster/imu` linear_acceleration on the stand with the filter on.
- **The bridge's ImuHealth goes partly blind** with the filter on. Pulse density and bias_x read the cleaned X, so both stay near 0. The tilt check now uses our tilt, and a gyro freeze still trips "frozen". robot.log keeps the fault visible through `despiked`, `pulse_mean` and `rv_tilt_diff`.
- **Yaw** comes from the gyro only and starts at an arbitrary heading. Use the `*_hh0` profiles.
- **Harness:** `imu_sim.py` needs the fast-gait `host` binary (`FG_SIM_DIR`). The replay needs the ks4/ks6 npz extracts.

## Battery monitor lead (checked 2026-11-24)
**Same bus: yes.**
- `battery_monitor.service` (root, 10 s poll) uses busio on `board.SCL/SDA`, which is `/dev/i2c-1`, and talks to the ADC at 0x48.
- The driver uses `/dev/i2c-1`, with the BNO at 0x4B.

**As the cause of these faults: unlikely.**
- **Bus mechanics:** each i2c-dev `read()`/`write()` is one complete kernel-serialized transaction. The ADC traffic goes to another address, and the SHTP header and packet reads are each whole transactions. So a 1 ms ADC read every 10 s can delay one BNO read, but it cannot splice bytes into one.
- **The signatures do not match a bus collision:**
  - A collision would show up as read errors or a growing IMU age.
  - Instead, the freezes showed fresh reports with age ≤ 20 ms but identical content for 0.5 s.
  - The pulses come thousands per minute, against a 0.1 Hz poll.
- **Cadence test:** only one freeze appears in the recorded runs (05:57:50.97). The three app freezes were 67 s and 31 s apart, which is not locked to a 10 s cycle. That is too little data to settle it.
- **Clock caveat:** journal and `systemctl` timestamps this boot predate NTP sync (the service shows a start time of "23:29:23 PST"). They cannot date the service.

**Cheap decisive test if wanted:**
1. Stand, stack up, motors holding.
2. Run `imu_test.py --mode ros` for 10 min with `battery_monitor` running, then 10 min with it stopped (`sudo systemctl stop battery_monitor`, an operator decision; restart it afterwards).
3. Compare pulses per minute and freezes.

With the filter deployed, the timestamps of the `gyro frozen` log lines can also be checked against the 10 s poll phase.

## Deployment status (2026-11-24)

v2 (sha 19d60fad…) is installed on mts via the driver symlink (undo in fc/robot/README.md) and enabled by default in the stock profile. Stand check passed (5 min, tilt 2.0–2.8°); 3 fast floor runs had no IMU incidents.

## v3: BNO stall guard (2026-11-25; superseded by v4, installed Nov 27)

**What happened (mts, run `20261125T070424Z-stock`):** while walking, the I2C bus hung. The kernel logged `i2c_designware 1f00074000.i2c: i2c_dw_handle_tx_abort: SDA stuck at low` (uptime 917.6 s). After that, every transfer failed with `timeout waiting for bus ready`, about 20 ms each.
- The driver's last good line was `getData: Remote I/O error`. After it, every read and write returned `ETIMEDOUT`.
- `pinctrl get 2,3` read SDA1 and SCL1 **both low** for minutes.
- The v2 driver went on publishing the last values. The bridge saw them as `frozen` → lie-down + latch.
- **A stack restart did not recover.** The new run's soft-reset writes also timed out, the report counts stayed at 0, and the orientation was garbage, so the fallen cutoff fired.
- Only a power cycle freed the bus.
- The GT911 touch driver (`waveshare-4dpic` overlays) was bound on the same bus. Those overlays are now disabled (overseer, `#fc-off(2026-11-25 i2c touch)`).
- Conclusion: **re-initialising the chip in place cannot free a hung bus.** v3 makes the stall visible and safe, and recovers whenever the chip, not the bus, is the problem. A hung bus needs `fcctl.sh i2c-recover` (below) or a power cycle.

**Driver v3** (`stock-imu-filter.patch` now contains v1+v2+v3; new header `include/rt/fc_imu_stall.hpp`, test `test/test_fc_imu_stall.cpp`):
- **Stall detection:** a stall is no SHTP input report for `FC_IMU_STALL_MS` (default 200, range 50–2000). With no report at all after start, it is declared after 3 s.
- **Where it runs:** detection runs in the IMU manager's read thread. The I2C traffic and re-inits happen outside `imu_mutex_`, so the real-time `read()` never waits on the bus; it keeps copying the last published output.
- **Re-init:** the start-up sequence again (soft reset, product ID, rotation vector + gyro + accel enables), plus a fresh filter that re-seeds its tilt from the next accel sample.
  - The first re-init comes at `FC_IMU_REINIT_MS` (default 500) after the last report, so a hiccup passes without one.
  - Then backoff 0.5, 1, 2, 4, 5 s… for 10 re-inits, then one every 30 s.
  - Each re-init blocks the IMU thread for about 0.5 s.
- **Clearing:** a stall clears only after 10 fresh reports **and** a seeded filter with live accel.
- **Stall marker** (published while stalled): orientation held from the last valid output, rate 0, **linear_acceleration exactly (0,0,0)**, and timestamps held, so `time_since_measurement_seconds` keeps growing.
  - The walker reads only orientation + rate (source: `neural_controller.cpp`), so it simply stands on the held attitude.
- **Why not the alternatives:**
  - "Stop publishing" is impossible: `imu_sensor_broadcaster` publishes the state interfaces every cycle.
  - `orientation_covariance[0] = -1` is impossible: the covariance comes from static broadcaster params, not from the hardware interface.
  - NaN is dangerous: `read()` deactivates the motors on any NaN state.
  - A live accelerometer is never bit-exact (0,0,0), so the marker is unambiguous once accel has been seen.
- **Logs:**
  - `FC_IMU_STALL on: …` at start;
  - `FC_IMU_STALL stall #N: no input report for X ms`;
  - `re-init #k sent|FAILED (I2C errors) in X ms (stalled S s); next in W s`;
  - `recovered: stall #N lasted S s, k re-inits`.
  - I2C errors are now rate-limited to 1 line per second, with counts. The hang wrote about 50 lines/s.
- **Other changes:** `Output.acc` and `Output.gyro` are now zero-initialised (they were uninitialised in stock).
- `FC_IMU_STALL=0` turns the guard off. `FC_IMU_FILTER=0` still means the stock filter path. Stock never fills the accel, so the bridge cannot see the marker there, and a stall still reads as `frozen`.
- **Build:** mts `~/fc-v1/build/imu-filter-v3/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so`, **sha256 `a64606d880a90cc7fb4056fbcbf51478527f4fcac171c3c7d738da1db1fd1c22`**.
  - RelWithDebInfo; the only warning is the stock one.
  - `colcon test`: 2 tests, 0 failures (`test_fc_imu_filter`, `test_fc_imu_stall`). Output in `test_output.txt` next to the build.
- **Not yet exercised on the chip:** the stall path itself. The happy path is the v2 code plus a report counter.

**Bridge** (`imu_health.py`, `drive_bridge.py`; device-free tests):
- **Arming:** once a live accel has been seen (|a| > 1 m/s²), an exact-zero accel sample counts as the marker → summary `stall_ms`.
- **Short stall (≤ `stall_long_ms`, 1000 ms):** level `warn` (the laptop does not end the run) and bridge hold `imu_stall` (zero command). No lie-down; the walker keeps standing. The hold releases 1 s after valid data returns.
- **Long stall:** `bad`, `bad_cause: "stall_long"` → the existing lie-down + latch.
  - Timing: a hang lies down about 0.2 s (driver) + 1.0 s + 0.75 s (`IMU_LIE_SUSTAIN_S`) ≈ 2 s after the last report.
- **Unchanged:** an unmarked freeze, stale and tilt still lie down as before.
- **Laptop:** `play.py` has words for `imu_stall` / `imu_stall_long`.

**Hung-bus recovery without a power cycle:** `fcctl.sh i2c-recover [--check] [--force]` (`robot/imu/i2c_recover.sh`). It refuses while a stack runs unless `--force`, and `--check` is read-only. Steps:
1. unbind the RP1 DesignWare controller (sudo);
2. if SCL stays low as an input, a device holds the clock and only a power cycle helps (reported);
3. otherwise, up to 16 SCL pulses via `pinctrl` until SDA is released, then a STOP;
4. pins back to `a3 pu`, rebind the controller, probe 0x4B.

Tested against a fake pinctrl and sysfs (`tests/test_i2c_recover.py`). **Not yet run on a hung bus.**
- **Kernel auto-recovery (`scl-gpios` in the i2c1 node) was not done:** it can only be verified by booting it. A pinmux conflict there could fail the i2c-1 probe, which means no IMU at all.

### Deploy v3 (operator, stack stopped; robot on the stand for the restart)
```
L=~/pupperv3-monorepo/ros2_ws/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so
V3=~/fc-v1/build/imu-filter-v3/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so
~/fc-v1/robot/fcctl.sh stop
readlink $L                                    # expect .../imu-filter-v2/... (rollback target)
sha256sum $V3                                  # expect a64606d8...1c22
ln -sfn $V3 $L && readlink -f $L && sha256sum $L
# laptop: stage the bridge, imu_health, fcctl i2c-recover
fc/robot/stage.sh mts
~/fc-v1/robot/fcctl.sh start stock_hh0 --confirm --speed-cap 0.80   # homes the legs
grep "FC_IMU_STALL on\|FC_IMU_FILTER on" ~/fc-v1/runs/<new run>/robot.log
```
- **Stand check (as for v2):** 5 min at zero command, tilt < 3°, and no `FC_IMU_STALL stall` lines.
- **Optional live check of the short-stall path:** none without touching the bus. Do not induce a stall by hand on the robot.

### Rollback
```
~/fc-v1/robot/fcctl.sh stop
ln -sfn ~/fc-v1/build/imu-filter-v2/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so $L
sha256sum $L     # 19d60fad...a3a2 (v2)
```
Robot files: `git checkout <previous commit> -- final_project/fc/robot && fc/robot/stage.sh mts`. The v2 driver with the new bridge is safe: the marker never appears, so the behaviour is as before.
