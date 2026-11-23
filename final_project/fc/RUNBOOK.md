# Floor day runbook (Nov 24, one robot: blue = `mts`)

Every step runs from the UI. Each run saves `fc/runs/<UTC>-<test>/` (summary in the Runs list). Stop at the
first failed step and fix it. Keep sessions under 10 minutes and let the Pi cool between them.

**Safety.** R3 on the gamepad is the e-stop: it lies the robot down and stays down, and it is the only stop
if the mux dies. Space = stop all, but only while the UI tab has focus (click the field first). The gamepad
stick overrides the laptop while held, then the laptop takes over again. Zero speed does not fix a false
IMU tilt: use R3. `start` homes the legs, so the robot must be on the stand.

## Setup checklist
- [ ] Stand ready, battery charged, R3 within reach, gamepad paired.
- [ ] Rigid robot tag fitted. Corner tags 0-3 down. iPhone Center Stage **off**.
- [ ] Key and code on the Pi: `python -m fc.robot.keygen` once, then **stage** on the robot card.
- [ ] LAN address: run `ssh mts hostname -I`, then start the app with that address:
  `.venv-fc/bin/python -m fc.app --source camera --robots blue --udp-host blue=<LAN IP>`
  (or `fc/run.sh camera --robots blue --udp-host blue=<LAN IP>`). Tests are on `/ops`; `/` is the Play screen.
  Every run now saves `overhead.mp4` in its run folder (`record: 0` turns it off).
  With the 100.x address the card warns "Tailscale".
- [ ] Load preset **slow** (Params, Load preset). The header must show `preset: slow`.

## Steps
| # | UI action | Watch | Pass | If it fails |
|---|---|---|---|---|
| 0 | Motors off. Press **status**. Check the camera frame. Hold the robot at each corner, then facing the east goal | Robot card: auth, RTT p95, loss. Field header: calib err, eye fps | auth ok; p95 < 20 ms; loss 0; calib < 1.5 px; eye ≥ 15 fps; tag seen at all 4 corners; **yaw ≈ 0° facing east** | Yaw near ±180°: set `eye_yaw_offset_deg` to −180 (applies live). RTT high: check `udp_host` |
| 1 | IMU test: **imu test** with `600`, `bno`, `unpowered`. Then the same with `battery_on` | Op output: `verdict`, `gyro_uncal_raw.x.neg_per_min` | Verdict recorded (healthy pulses: 0-4 per min) | `hardware_or_electrical` with motors unpowered: tell staff, plan to replace the board |
| 2 | On the stand: **start** (profile `stock`), then **stop**, then **start** again | Op output says READY; fcctl status shows `mux_ok` true | READY in < 100 s; legs go limp < 10 s after stop | Copy the op output. After a mux crash, wait about 20 s before **start** (see below) |
| 3 | Stand the robot up on the walker with the gamepad (**X**; on `tej_u6` press **Options** first). In the air: **bridge start**, **Drive** (W) at 0.1. Stop it 5 ways: space; Ctrl-C the app; `ssh mts pkill -9 -f drive_bridge`; gamepad stick; R3. Then **Deadman** | Legs | Legs still within 0.3 s (space), 0.5 s (app killed), 0.8 s (bridge killed). Stick wins. R3 stays down. Deadman `stop_time_s` ≤ 0.5 | Any failure: stop the session and record which stop failed |
| 4 | Floor, preset `slow`: **Drive** for 2 min; kill the app once | Robot card IMU; Pi temperature in **status** | IMU ok, no IMU BAD; Pi < 75 °C; no drift into a turn | IMU BAD: the app already stopped, so R3 and note the time |
| 5 | **Step** quick buttons: fwd 0.1 / 0.2 / 0.3 facing east, then west; turn ±0.5; left 0.1 (preset `fast`) | Runs list: gain, lag, bias | `latency_s` < 0.35; `gain_lin` repeats within ±10%; `\|heading_bias_deg\|` < 5 | Bias > 5° in both facings: add it to `eye_yaw_offset_deg` or load `learn`. Copy gains/lag into `sim_*` and rerun Sim eval |
| 6 | **Goto** ×5 (click points, include far corners) | Path on the field | `pos_err_m` < 0.08; ≤ 1 fast reversal; no stale stop (metrics) | Tag lost often: check light and the tag. Overshoot: lower `k_turn` |
| 7 | **Push ball** ×10 from centre, ×5 near a wall | Spinning next to the ball | ≥ 7/10 goals in ≤ 30 s; no tip | Note the run names that failed |
| 8 | **Goalie** ×10 hand-rolled shots | Saves | ≥ 6 saves; no tip | — |
| 9 | A/B, one change at a time: preset `u6-hh-on`, then `stock-hh-off`. **stop** + **start** with the preset's profile, then repeat steps 5 and 7 | Card: profile running vs the preset | Better step numbers decide | A `*-hh-off` profile not staged yet: the load warns and start refuses |

**Record each step:** run folder names, pass/fail, and anything the summary does not show (sounds,
stumbles, how R3 behaved). Every run's `params.json` already records the preset and the running profile.

**Automatic stops.** The app sends zero and ends the test when the bridge reports IMU `bad` (result
`imu_bad`) or `mux_ok: false` (result `mux_lost`). The header turns red. The bridge itself also holds zero
(card: `hold`). A killed mux stays in ROS for about 20 s, and the bridge holds zero while it sees 2
publishers. Wait about 20 s before restarting the profile. `lost_tag` or `edge` on a step: move the robot
back toward the centre and run it again.
