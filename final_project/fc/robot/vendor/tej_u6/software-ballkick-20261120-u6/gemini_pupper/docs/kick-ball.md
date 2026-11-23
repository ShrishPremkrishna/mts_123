# kick_ball: find the ball and kick it

One spoken or typed request ("go find the ball, walk over to it, and kick it", optionally "with your left leg") makes Pupper find the beach ball, walk to it, line up one front paw, kick with the leg kick trick, and stand on the walker again. Offline checks pass on the fake robot and in simulation; robot trials are not done (plan below).

## How it works

| Step | What happens | Code |
| --- | --- | --- |
| Request | Gemini calls `kick_ball(side)`, side `right`, `left` or `auto` (default). Refused at once without a calibration file (`calibrated: true`), with a camera frame older than 1 s, or with approach speeds above the selected walker limits. One queued job. | `lab/part9_ball_kick.py` `kick_ball` |
| Stand | `gemini_walk` takes the legs (also leaves a seated trick through its exit clip). | `robot.switch_to` |
| Find | Stand still and look 1.2 s. If no ball, turn left in 60° steps of commanded yaw at 34°/s and look 1.2 s again, up to 360° of commanded yaw. Only a fix captured during a turn ends that turn early. All searches in one approach share 540° of commanded yaw. A ball needs 3 agreeing fixes. | `robot/ball_kick.py` `find` |
| Side | `auto`: the side of the first sighting; a ball within 5° of centre gets the right paw (the leg with robot kicks). | `choose_side` |
| Approach | Walk at up to 0.30 m/s and turn so that the ball sits at the paw's y, until the ball is approximately 0.90 m ahead. | `drive_to_stage` |
| Stage | Stop-and-look: settle 0.5 s, then the mean of the fixes over 1.5 s with outliers removed (silhouette fixes only). Pulse time = dead time + distance / (gain × speed), at least dead time + 0.03 s. | `measure`, `correct` |
| Calibrate | Pulse 1: 0.5 s at 0.2 m/s. Pulse 2: 0.9–2.0 s, sized so that the ball stays at x ≥ 0.47 m for any walker gain ≤ 1.35 and dead time ≤ 0.35 s. A fit gives travel = gain × 0.2 × (T − dead). Back up to approximately 0.74 m if necessary, align y, then a rehearsal pulse of the creep length must land within 2 cm (and 3σ) of the prediction. Abort if the gain is outside 0.45–1.35, the dead time is outside −0.15 to 0.35 s, a pulse is off the line of the others, or the ball is not seen after a calibration pulse. | `calibrate` |
| Align | Pulses until the ball is at x = 0.50 m ± 3 cm and the paw's y (right −0.046 m, left +0.040 m) ± 1 cm (± 2 cm accepted after 8 pulses). The expected sideways drift of the creep is subtracted only when it is larger than 2σ of its estimate, at most 1.2 cm. | `correct`, `near_y` |
| Creep | Forward x − 0.258 m, less 5 mm + 1σ (aimed short), dead-reckoned from the fitted model, at most 3.0 s, then settle. Abort if margin + 3σ > 3 cm, or the creep is ≤ 0 or > 0.272 m. | `run` |
| Kick | STRICT switch to `trick_leg_kick`; its init and fade-in (0.5 s + 1.0 s), then 1.0 s in `stand_idle` (the hardware kicks started approximately 2 s after the fade); the `stand_idle` loop is confirmed; the native controller starts `kick_right` or `kick_left` (3.0 s) at once from the constant loop. | `play_kick` |
| Return | 0.5 s after the clip, `gemini_walk` takes the legs (the trick policy jitters when the robot is lifted). If the walker refuses for another reason, this server relaxes all leg controllers. While a Stop, a fall, a shared-stop block or a controller fault owns the legs, this server sends no switch request at all. | `play_kick` |

**End states.** "Ball not found", "lost", "not seen" or "moved" end the job as a completed request with no kick (`kick_ball ended without a kick: …`, `details.kick_ball.kicked = false`), so the browser does not pause its tools. Stop, fall, motion block, camera or detector failure, timeout, travel budget and calibration faults end it as a failure. Every end requests zero speed.

**Vision** (`robot/ball_vision.py`, started from the onboard colour detector in `final_project/behaviors`). Red and blue saturated pixels in the floor rows (below +9.8° above the body horizon) form blobs; blobs inside one fitted circle join (a white gore separates the colours). The ball comes from a silhouette fit: a small circle on the unit sphere through the blob's outer edge points, so the white gores do not move the centre (simulated 6-gore ball, 108 orientations: sideways error within ±1 mm at 0.50 m, range error ≤ 1.0 cm at 0.50 m and ≤ 1.2 cm at 0.76 m; it reads long at 1.5 m, up to 9.4 cm). A height gate (ball centre at z = −0.05 m ± (0.04 + 0.03 × range)) refuses objects above the floor, such as a person in a red top and blue jeans. When the fit is not reliable, a colour-box fallback gives a coarse fix that can guide the walk but never a line-up measure. Detection runs in the tool server only while a kick or `locate_ball` runs, at most 12 frames per second, in a worker thread. It does not use the IMU. A camera with no new frame for 1 s, or 5 detector errors in a row, ends the job.

`locate_ball` looks for 1.2 s without motion and reports distance (2 decimals), bearing (integer degrees, left positive), x, y, z, the fit method and the angular radius; with `snapshot_path` set it also writes the annotated floor image.

`tricks/leg_kick/trick.yaml` has `tool_only: kick_ball`, so `play_trick`, `interactive_action` and `/animations` do not offer the trick. `kick_ball` and `locate_ball` are not advertised when the trick is not installed. A `trick.yaml` with the removed `secret` key now refuses to load.

## Calibration file

YAML at `PUPPER_BALL_KICK_CONFIG` (default `~/.config/gemini-pupper/ball_kick.yaml`). `kick_ball` needs it with `calibrated: true`; `locate_ball` works without it. Unknown keys, wrong types and values outside the safe ranges (speeds, rates, budgets, geometry; see `check_params` in `robot/ball_kick.py`) refuse the tool before it queues anything. A larger `stage_near` can need a larger `stage_far`. Defaults are in `robot/ball_kick.py` and `robot/ball_vision.py`.

```yaml
calibrated: true                              # written after stage R3
snapshot_path: /tmp/gemini-pupper/ball.jpg    # optional: locate_ball writes the annotated floor image
vision: {ball_radius: 0.097, y_bias: 0.0, ball_z: -0.05}   # y_bias is subtracted from the measured y
mount: {range_scale: 1.0, x: 0.15, tilt_up_deg: 10.0}      # silhouette fit; do NOT copy the old 0.616
geometry: {contact_x: 0.258, right_y: -0.046, left_y: 0.040}
dry_run: false                                # true: line up, then stop without the kick
```

## Offline checks

From the repository root:

```bash
.venv-api/bin/python -m pytest -q gemini_pupper/tests/test_ball_kick_approach.py gemini_pupper/tests/test_kick_ball_tool.py gemini_pupper/tests/test_ball_vision.py gemini_pupper/tests/test_controller_fault_status.py
```

- `test_ball_kick_approach.py` (57 tests) drives the approach against a simulated walker (dead band, dead time 0–0.2 s, lag, coast, unknown gain 0.55–1.25, sideways drift, yaw drift) and camera (12 Hz, 130 ms latency, noise, correlated bias, dropouts, blind zone, 220° view), including a ball moved during calibration and a flickering false positive. A 300-case Monte Carlo on 2026-11-16: 300/300 lined up; forward error p95 1.3 cm, maximum 2.0 cm, mean 8 mm short, never under the body; sideways ≤ 1 cm in 96 %, ≤ 2 cm in 100 %; approach time median 31 s, p95 43 s.
- `test_kick_ball_tool.py` (28 tests) runs the tool through the real queue and the fake ROS robot: full sequences, no ball, Stop during the approach and during the clip, a fall during the clip, a latched and a stale shared stop during the clip, walker refusal, camera and detector failure, calibration and speed refusals, `locate_ball`.
- `test_ball_vision.py`: 37 NumPy tests (ray-traced 6-gore ball, false positives) run anywhere; 9 OpenCV tests (JPEG decode, remap, HSV, BGR/RGB order) run only where OpenCV is installed, for example on the robot.

None of these checks simulate the leg kick's contact with the ball.

## Robot trial plan

Each stage needs the operator's go. Stop a stage at the first failed check.

**Before any stage:** robot on the floor or the stand as stated, the area clear 2 m around and 3 m ahead, R3 and the app's Stop in reach. CPU below 80 °C (`vcgencmd measure_temp`) and not throttled (`vcgencmd get_throttled` = `0x0`). At rest on the walker, the IMU tilt is below 5° (the 2026-11-16 false tilt reached 20–25°). Course approval for the leg kick policy on hardware is recorded.

| Stage | Setup | Action | Pass | Record |
| --- | --- | --- | --- | --- |
| R0 read-only | Robot on | Check processes (no course stack, no other motor owner), the r6 release folder, `python3 -c "import cv2, numpy"` with the API Python | All present, no other owner | Process list, temperature |
| R1 install | Motors off | Copy the r6 release to a new folder, overlay this branch's Gemini files, write the app manifest and a prepare script with `PUPPER_SESSION_SKILLS` that includes `leg_kick`. In the new folder run `python3 -m pytest -q gemini_pupper/tests/test_ball_vision.py` with the API Python | `persistent_session.py prepare` and `status` pass; all 46 vision tests pass (the 9 OpenCV tests run only here) | New folder, manifest SHA-256, test output |
| R2 start | Robot on the stand | `start-native`, then `start-api` | `trick_leg_kick` loaded and inactive; `GET /tools` lists `kick_ball` and `locate_ball` | Controller list |
| R3 vision | Robot standing still on the walker, floor; calibration file without `calibrated` and with `snapshot_path` | Ball at measured points (x, y) = (0.50, −0.046), (0.50, 0.040), (0.76, 0), (1.0, 0), (1.0, 0.3), (1.5, −0.4); `locate_ball` 3 times each, turning the ball between repeats (gore axis vertical, horizontal, tilted). If the height gate refuses the ball ("not on the floor" in the reply), set `vision: {z_tol: 1.0}` for R3 only. Check each snapshot: the circle lies on the ball outline, including white gores | All fixes `method: fit`. Fit `range_scale` (near 1.0) and `mount.x` from the 0.50 and 0.76 m fixes only (far fixes read long), `y_bias` from all fit fixes, and `ball_z` = the mean z (tighten `z_tol` to about 3 × the z spread). After the fit: across the three ball orientations, sideways spread ≤ 1 cm and range error ≤ 1 cm at 0.50 m. Record the nearest range that still gives a fix. Then write `calibrated: true` | All fixes, the snapshots, the fitted values, the YAML file |
| R4 dry run | Floor, `dry_run: true` | Ball 1.0 m ahead: `kick_ball(right)`. Then ball 1.2 m at 45° left: `kick_ball(left)`. Then ball behind: `kick_ball(auto)` | Robot stops with the ball centre 0.258 ± 0.03 m ahead of base_link (the controller aims approximately 1 cm short) and within ±1.5 cm of the paw line (ruler) | Ruler readings, the `kick_ball` lines in `robot.log` (fixes, fitted gain and dead time, rehearsal error, drift), time |
| R5 right kick | Floor, `dry_run: false` | Ball 1.0 m ahead: `kick_ball(right)` | Contact; walker active afterwards; no jitter; tilt e-stop not triggered | Video, ball travel and direction, peak temperature |
| R6 left kick | Same | Ball 1.0 m ahead, left of centre: `kick_ball(left)` (first left kick on hardware) | Same as R5 | Same |
| R7 voice | Gemini app, hands-free | "Go find the ball, walk over to it, and kick it." Ball 1.5 m away, off to one side | One `kick_ball(auto)` call; the sequence completes | Video, app receipts, outcome |

R4 measures the ball at the walker stance. The trick's 0.5 s re-pose to `stand_idle` moves the joints by at most 0.008 rad, mirror-symmetric (approximately 1 mm forward, 0 sideways, computed by the 2026-11-16 review), so R4 also stands for the kick stance.

Stop during R4–R7: a Stop in the app ends the job with zero speed, and during the kick clip it relaxes the trick (offline-tested); R3 latches the native e-stop, and the server then sends no switch request. If the robot jitters while the leg kick holds the legs, push R3.

**After the trials:** `stop-api`, then the native shutdown from the hardware checklist; leave the r6 release and the course-stack files unchanged.
