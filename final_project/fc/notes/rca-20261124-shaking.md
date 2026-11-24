# RCA: blue (`mts`) seizing/shaking while standing, 2026-11-24 ~06:00 UTC

Written 2026-11-24 ~06:11 UTC from laptop evidence. `mts` was powered off for a battery swap and had not answered ssh by 06:11, so **no Pi log has been read yet**. The section "Pi checks still owed" lists what would confirm or rule out each cause.

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
- **Against / unknown:**
  - A steady −0.03 rad/s median could also be a slow real lean. The camera cannot measure tilt.
  - The stock driver may report fresh `time_since_measurement` while the values repeat. The parent noted "IMU age 6–8 ms" in the walker log. That would mean the BNO kept sending identical reports, which is still a BNO-side anomaly but not a starved reader.

### 2. Battery sag or brownout. Confidence: low-medium (untested; no voltage data at all)
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
- **Decides it:** a second controller in `robot.log` or a controller-manager switch after 05:58.

### 4. Motor/joint fault or hot motors. Confidence: low
- **Against:** mostly standing, light load, short floor time. Symptoms started with IMU flags, not current faults.
- **Decides it:** driver fault/over-current lines in `robot.log`, and a physical check of motor temperatures and cables.

### 5. Something the laptop sent. Ruled out
Last command zero, no ops after 05:58:00, no e-stop.

## Pi checks still owed (read-only)
Read-only script: `scratchpad/pi_ro.sh` from this session. The same commands are listed here.
- `journalctl --list-boots`
- `journalctl -b -1 -k | grep -iE "volt|throttl|i2c|spi"`
- `journalctl -b -1 -u battery_monitor`
- the end of `journalctl -b -1`, to see whether it shut down cleanly or lost power
- `~/fc-v1/runs/20261124T055116Z-stock/robot.log` around 05:57–06:01: IMU errors, NaN, motor/CAN faults, controller switches, `imu_latency_seconds`
- the bridge log in the same run dir

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
5. **Log IMU health transitions on the Pi.** The laptop only records during runs, and the incident fell between runs. The app should also log `lk` continuously while a robot is standing.
