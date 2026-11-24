# RCA: blue (`mts`) seizing/shaking while standing, 2026-11-24 ~06:00 UTC

Written 2026-11-24 ~06:11 UTC from laptop evidence. Updated ~06:45 UTC with read-only Pi evidence after the battery swap (stack off; the only device access was one battery ADC read).

**Bottom line:** the Pi logs confirm that the stack ran normally and only zero was commanded. They refute a driver/walker stall, a controller switch and a Pi-rail brownout up to the last flushed log line. They cannot see the shaking itself: the last ~15–30 s before power-off were never written to disk. The IMU (BNO output content) remains the leading cause. Battery on the motor rail in the final seconds is not excluded until the old pack is measured.

## Pi evidence (read 06:34–06:45 UTC, read-only)

**Clock anchors:**
- `robot.log` covers 05:51:23 to **06:00:18.5**; its last 3 KB are NUL bytes, a page never flushed at power-off.
- `bridge.log` ends at bridge `up_s` 171 (~06:00:15).
- `~/fc-v1/run/bridge_state.json` was last written at 06:00:26.9. The bridge was alive and receiving then, and the app's last ack was ~06:00:35.
- The journal for boot −1 ends at **05:58:47**. journald had not synced the last ~100 s, so power was cut uncleanly.
- The shaking (tilt > 10° from ~06:00:17) therefore falls almost entirely in this unwritten window.

**Walker and driver (`robot.log`):**
- `neural_controller_default` activated at 05:57:19. No other leg controller was activated after that.
- After activation there are no WARN/ERROR/emergency-stop lines.
- 8942 policy steps from 05:57:21 to 06:00:18.5, at a steady 50 Hz. No step gap exceeds 40 ms. Inference ≤ 3.6 ms.
- `IMU measurement age` stayed ≤ 19.8 ms the whole time, including the 05:57:51, 05:58:58 and 05:59:29 "unchanged ~500 ms" windows.
- So the driver kept receiving fresh BNO reports at ~100 Hz. Those reports carried bit-identical quaternion+gyro values. That is not a reader stall or a starved callback. It is the BNO's output content: either a fusion/output freeze, or quantization while very still. Today's history argues against plain quantization (see cause 1).

**Bridge (`~/fc-v1/logs/bridge.log`):**
- Four `hold None -> imu_stale` / `-> None` pairs, matching the app events.
- `estop` 0, never parked. Commands came only from the laptop session `1feb1a4f…`.
- The `imu_tilt` transition (06:00:19) is in the lost tail.
- The bridge log does not record IMU values, so no tilt or bias series exists on the Pi.

**Power:**
- `journalctl -b -1 -k` and `-b -2 -k`: **zero** under-voltage lines, and no I²C/SPI errors (the only I²C errors are the absent touchscreen at boot).
- `battery_monitor` (10 s poll; it warns below 16.5 V and schedules shutdown below 15.5 V) logged nothing and scheduled no shutdown in boot −1, up to 05:58:47.
- The status op at 05:58:00 read `throttled 0x0`, 63 °C.
- Now, on the new pack: **19.31 V (85 %)**, `throttled=0x0`, EXT5V 4.95 V, 41 °C.

**Not available anywhere:** joint positions/currents, motor temperatures or IMU values during the event. The stock stack records no bag and logs only inference time and IMU age.

## Timeline (UTC = local PST + 8 h)

| UTC | Source | Event |
|---|---|---|
| 05:51:16 | app ops | `ready` (lean stock stack, Pi run `~/fc-v1/runs/20261124T055116Z-stock`) |
| 05:57:24 | app | bridge restarted (pid 3760), IMU ok |
| 05:57:37 | app | push_ball starts (`fc/runs/20261124T055737Z-push_ball`) |
| 05:57:43–48 | run lk + video | tag lost, so cmd zero (`stale`). From t=11.7 s the robot is completely still on video (0 changed pixels). IMU values still change every 4–8 ms |
| 05:57:51 | app | IMU BAD `IMU unchanged for 506 ms`. The sample count kept rising (~250/s), so messages arrived but quaternion+gyro were bit-identical. Run ends |
| 05:58:00 | app ops `status` | phase ready; only `neural_controller_default` active (liedown inactive); mux ok; 63.1 °C; throttled 0x0; battery not read (opt-in) |
| 05:58:58, 05:59:29 | app | IMU BAD `unchanged 546 ms`, `510 ms`; each clears in < 1 s |
| 06:00:19 | app | IMU BAD `tilt 10.5 deg at zero command`; bridge holds zero (`imu_tilt`) |
| ~06:00:35 | app link | last ack. Last IMU summary: tilt 9.6°, **bias_x −0.029 rad/s**, pulse_density 0.008. Bridge: `estopped:false`, e-stop count 0, not parked |
| after | operator | power off. The camera shows the robot lying ~15 cm from where it stood and rotated ~15°, so it moved while commanded zero |

**What the laptop sent:** zero after 05:57:51. No app op after the 05:58:00 `status`: no ready, down or e-stop. On an IMU hold the bridge only publishes zero. It switches controllers only on a link E-STOP or tilt > 45° (its e-stop counter stayed 0).

## Ranked causes

### 1. IMU fault (the known gyro fault, now with freezes) feeding the walker policy. Confidence: medium-high
This combines hypotheses (b) and (c).
- **For:**
  - **Bias:** −0.029 rad/s at the end matches the earlier fault (−0.031 at 05:19, when the robot physically leaned ~14°).
  - **Tilt at zero command:** 10.5° held 2 s. The policy reads orientation (projected gravity) and angular velocity directly from this IMU. A false tilt makes it "correct" a lean that is not there, which is the known lean/jitter mechanism.
  - **Freezes:** the three ~0.5 s identical-value windows came first. Every earlier run today (16 runs, all standing segments) stayed ≤ 33 ms unchanged. The ks4/ks6 replays topped out at ~90 ms. In this same run the values changed every ≤ 8 ms for 2 s while the robot stood perfectly still, then froze. A new BNO behaviour fits that better than ordinary quantization.
  - **Detector blind spot:** pulse density stays low when the body is shaking, because a pulse needs raw Y/Z quiet. The fault can hide from that metric during the very event.
  - **No guard:** nothing in the stock stack acts on a 10° false tilt. The walker's own e-stop is `max_body_angle` 1.5 rad (86°). Our bridge only holds zero, and zero speed does not stop a policy reacting to a bad orientation.
- **Pi update:**
  - **Confirmed:** fresh reports kept arriving with IMU age ≤ 20 ms while their values repeated. The anomaly is in what the BNO reported, not in delivery.
  - **Refuted:** a software stall in the driver, the walker or the bridge.
- **Against / unknown:**
  - A steady −0.03 rad/s median could also be a slow real lean. The camera cannot measure tilt.
  - No IMU values were recorded on the Pi.

- **BNO calibration follow-up (2026-11-24 06:45–06:53 UTC, motors off, no stack, `imu_test.py --mode bno`):**
  - **Gyro dynamic calibration is OFF at boot** (`--report-calib`: accel 1, gyro 0, mag 1). The stock driver only soft-resets and enables 0x05 + 0x02, so every driver session also runs with gyro dyncal off. The chip therefore cannot learn a wrong gyro bias while walking. **The "wrong learned gyro bias saved to DCD" hypothesis is refuted.** With a constant chip bias, the driver's calibrated gyro is the raw MEMS rate minus about 0.002 rad/s, so the raw-X pulses and the −0.03 rad/s offsets seen through ROS are in the sensor signal itself (electrical, vibration or a real lean), not in calibration.
  - **Gyro dyncal ON and OFF look the same at standstill** (3 min each). The bias median stays 0.002 rad/s (1 LSB) with no segment drift. In both, the sample-level bias field dithers by ±3 LSB (about 2000 changes in 3 min), so the dither is not caused by dynamic calibration. Setting the gyro to ON was RAM-only: the chip reported gyro 0 again after a soft reset.
  - **The 06:35 "calibration" verdict was a false positive.** The old rule flagged a raw min/max span above 0.01 rad/s, which a 6-LSB dither trips. Re-analysed with segment medians, that run is clean.
  - **Rotation vector repeats are normal at rest.** The fused 0x05 quaternion repeated bit-identically for 1.3–2.9 s, motors off. The 0x07 gyro repeated for at most 130 ms. A freeze test on the quaternion alone would fire on a healthy chip. 0.5 s with the gyro also identical is still abnormal.
  - **Remaining calibration path:** accel and mag dynamic calibration are ON and saved to DCD. Mag affects only heading. Accel could shift the gravity reference, but now the RV agrees with the calibrated accel within 0.02°, and gravity matches the 45° mount. So there is no current evidence of a bad saved calibration. The DCD has not been cleared.
  - Driver option (not deployed): `fc/robot/imu/stock-bno-dyncal.patch`, see lane-D-gyro.md.

### 2. Battery sag or brownout. Confidence: low (Pi update)
**Pi update:**
- No kernel under-voltage and no battery_monitor warning or shutdown through 05:58:47.
- `throttled 0x0` at 05:58:00.
- The walker loop stayed perfectly regular until 06:00:18.5.
- The IMU anomalies began at 05:57:51, while the Pi rail was demonstrably clean.
- Still open: the last ~15–30 s (the journal is lost), and the motor rail itself, which the Pi does not monitor. Measure the removed pack.

Original analysis:
- **For:**
  - The battery had served a long session: ~1 h of stack earlier plus a re-home and stand.
  - Undervoltage on the motor side can make joints weak or erratic.
  - The operator chose to swap it.
- **Against:**
  - `throttled 0x0` at 05:58 (Pi rail fine then).
  - A brownout would more likely reset the motor board or Pi than produce 2 min of IMU-specific symptoms.
  - The IMU symptoms (bias signature, freezes) are not voltage signatures. However, the BNO sits on the same board and supply, so a sagging rail could also cause BNO resets or freezes. Treat (1) and (2) as possibly linked.
- **Decides it:**
  - `battery_monitor` journal from the previous boot (warning < 16.5 V, shutdown < 15.5 V).
  - `journalctl -b -1 -k` undervoltage lines.
  - Battery voltage measured now on the removed pack.

### 3. Controller/overlay issue from today's changes. Confidence: low
- **For:** today's changes are new: lean node list, inactive `neural_controller_liedown` loaded, speed cap overlay, the mux.
- **Against:**
  - At 05:58 only `neural_controller_default` was active.
  - The liedown controller is inactive and gain-capped (kp 5.0 ≤ walker 5.5).
  - Lean removes only foxglove/camera nodes.
  - The bridge sent only zeros and never switched controllers.
  - No laptop op ran after 05:58:00.
- **Decides it:** a second controller in `robot.log` or a controller-manager switch after 05:58. **Pi update: neither occurred up to 06:00:18.5. Refuted for that window.**

### 4. Motor/joint fault or hot motors. Confidence: low
- **Against:** mostly standing, light load, short floor time. Symptoms started with IMU flags, not current faults.
- **Decides it:** driver fault/over-current lines in `robot.log`, and a physical check of motor temperatures and cables. **Pi update: no driver fault lines, but the stock driver may not log motor faults. Physical check still owed.**

### 5. Something the laptop sent. Ruled out
Last command zero, no ops after 05:58:00, no e-stop.

## Pi checks
Done 06:34–06:45 UTC; results are in "Pi evidence" above. Still owed:
- the voltage of the **removed** pack;
- motor temperatures and cable/IMU-mount inspection (physical).

## Before powering motion again
1. **Physical:**
   - Measure the old and new pack voltage. Use only a full pack (5-cell: well above the 16.5 V warning).
   - Feel all 12 motors for heat.
   - Check the IMU/control-board mount and its screws, and the connectors and cables to the board (the bias and freezes may be mechanical or electrical).
2. **Stand test on the stand, legs free, no floor:**
   - Start the stack.
   - Watch `fc.ctl status` imu fields for 5 min at zero command: tilt < 3°, |bias_x| < 0.01, no unchanged > 100 ms.
   - Stop at the first warn.
3. **If it shows bias or freezes on the stand:** run the lane-D bench test (`imu_test.py --mode bno`, motors unpowered, then battery on) to separate in-chip from electrical causes before any floor time.
4. **On the floor:** only with an R3 operator holding the gamepad, and a fresh battery.

## Software guards to add
1. **IMU bad (tilt or freeze) should go past zero speed to lie down, then deactivate.**
   - Zero speed does not stop a policy reacting to bad orientation.
   - The bridge already has the R3 deactivate path. Use the liedown controller first so the robot does not drop.
2. **Do not relax the freeze check yet.** Commit 598aa0b (another agent) makes identical values at zero command a warn only (bad after 2 s while commanded). Today's evidence says a 0.5 s freeze while standing was new and preceded the incident.
   - Keep standing freezes at least a warn that the app surfaces loudly.
   - Add `linear_acceleration` to the identical-sample test. Accelerometer noise should never repeat bit-for-bit for 100+ samples, so an identical accel means a real freeze.
3. **Oscillation detector in the bridge:**
   - Trigger on joint velocity RMS or gyro RMS above a threshold at zero command for > 1 s.
   - Action: lie down, then e-stop.
4. **Battery in status, polled continuously:**
   - Read it in the bridge, or from `battery_monitor`'s journal, not on the shared I²C bus from fcctl.
   - Hold or lie down below ~16.5 V.
5. **Make Pi logs survive a power cut.**
   - Every log lost its last 15–100 s here: journald was ~100 s behind, and `robot.log`/`bridge.log` lost their unflushed tail.
   - The bridge should `flush()` + `os.fsync()` its log on every hold or level transition.
   - It should also write a small rolling IMU record (tilt, bias_x, frozen_ms, gyro RMS at 5 Hz) to `~/fc-v1/runs/<run>/imu.jsonl`, fsynced every second.
6. **Log IMU health transitions on the Pi.** The laptop only records during runs, and the incident fell between runs. The app should also log `lk` continuously while a robot is standing.
