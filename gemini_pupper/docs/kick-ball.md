# Kick the ball: kick_at, kick_in_place and kick_ball

Pupper kicks a ball with one front leg from ball coordinates (`kick_at`, the path in use: the field camera gives the coordinates) or from its own camera (`kick_ball`, not reliable on the robot). Since 2026-11-19 the walk, the kick and the return to the walker are one job with no fixed waits between them: the paw touches the ball approximately 2.1 s after the last walk command (5 s before). On the robot on 2026-11-19 the walk-kick-walk job took 5.7 s for each leg (12.5 s before): see "Robot result".

## Timing of the kick (all kick tools)

`play_kick` in `lab/part9_ball_kick.py` activates the leg kick controller at the kick clip itself (`start_clip`), so the clip starts when the controller's blend-in ends. The walker takes the legs when the kick motion ends, 2.6 s into the 3.0 s clip.

| Time after the last walk command | Before 2026-11-19 | Now |
| --- | --- | --- |
| Client poll and a second HTTP tool call | 0.2–0.4 s | 0 (one job) |
| Zero speed on the walker | 0.5 s | 0.15 s (`KICK_SETTLE_S`) |
| Controller blend-in (init + fade) | 0.5 + 1.0 s | 0.2 + 0.3 s (`tricks/leg_kick/trick.yaml`) |
| `stand_idle` settle, endpoint confirmation, clip request | 1.0–1.3 s | 0 (`start_clip`) |
| Kick clip to contact (still 0.4 s, weight shift, wind-up, swing) | 1.45 s | 1.45 s |
| **Paw contact** | **approximately 5 s** | **approximately 2.1 s** |
| Contact to the walker switch | 2.05 s (clip end + 0.5 s) | 1.15 s (motion end) |
| Walker start (its own init 2 s + fade 2 s, `WALK_READY_S`) | 4.2 s | 4.2 s |

Controller service calls add approximately 0.1–0.3 s on the robot (not measured). The walker start is now the longest wait. It is a launch parameter of the walking controller and also sets the speed of the stand-up from lying, so it is not changed here.

Simulation (`foundation/scripts/check_kick_handoff.py`, the production clip player and fade arithmetic with the leg kick policy in MuJoCo, 2026-11-19, 24 cases for both legs): the robot stands in every case, body tilt at most 5.0° in the kick and 3.7° in the return. Ball speed 0.1 s after contact: 1.26–1.34 m/s with the earlier sequence, 1.23–1.40 m/s with the direct clip start and the 0.2 + 0.3 s blend-in from a stand, 0.94–1.50 m/s with a body speed of 0.26 m/s at the end of the walk (also with the earlier 0.5 s wait: 1.06–1.24 m/s). A walker init phase of 0.3 s after the kick is as stable as 2.0 s. The simulation has no walking policy: a position hold stands in for the walker.

**Robot result (2026-11-19 4:52 pm PT, release u5, both legs).** Test: walk 0.305 m (1 ft) and kick, left then right (`kick_at.py 0.553 0.040 --side left`, then `0.553 -0.046 --side right`). Both kick clips were confirmed by the controller, with no fault. The operator watched both runs and said that the result was good.

| From `server.log` | Release u3 (4:20 pm) | Release u5 (4:52 pm) |
| --- | --- | --- |
| Whole job: walk, kick, walker ready | 12.5 s | 5.7 s |
| Walk end to paw contact (estimate: trick activation + blend-in + 1.45 s) | 4.9 s | 2.3 s |
| Contact to a ready walker | 6.4 s | 2.2 s |

Release u5 on the Pupper is `~/pupper-foundation-session-20261029/software-ballkick-20261119-u5`: this branch plus `tricks/walker.yaml` (a copy of `tricks/walker.example.yaml`). Its install and run scripts are in `~/pupper-foundation-session-20261029/kickfast-kit-20261119` (`kickfast2-install.sh`, `kickfast3-prepare.sh`, `kickfast3-session.sh`, `KICKFAST-RUNBOOK.txt`). After a restart of the Pupper, make a new run: copy the two `kickfast3` scripts with a new run name, then `prepare`, `start-native` on the stand, `start-api` (no `--app-manifest`), Options, `stand_up`. Not yet tried on the robot: `--leave 2.1`, `--settle 0`, the pose mode, a kick at a ball with this release.

**Walker start after a kick (2026-11-19).** Measured on the robot with release u3 (4:20 pm PT): 4.9 s from the end of the walk to paw contact, then 6.4 s until the walker obeyed a speed; 4.2 s of that was `gemini_walk`'s own 2 s + 2 s start. The optional file `tricks/walker.yaml` (`walker_config` in `robot/tricks.py`) sets a short start, for example `init_duration: 0.2`, `fade_in_duration: 0.3`, `stand_first: neural_controller_default`, `stand_first_s: 2.3`. The walker then obeys a speed 0.7 s after it takes the legs. The slow course values exist to lift the robot from the floor, so with a short start `Robot.switch_to` first activates the `stand_first` controller for `stand_first_s` whenever the robot does not stand (motors idle, a fall, a trick that ends lying). `tricks/walker.yaml` is not in the repository (`tricks/walker.example.yaml` is): with no file, the course values and the earlier behavior stay. `foundation/hardware/lab6_session.launch.py` applies the file to `gemini_walk` in every motion mode. The kick tools also take `leave` (`--leave`): the time into the kick clip at which the walker takes the legs back (default 2.6 s; 2.1 s is when the paw is down, and the simulation stands with it).

To go back to the 0.5 + 1.0 s blend-in, remove `init_duration` and `fade_in_duration` from `tricks/leg_kick/trick.yaml`. A change of these values needs a new `start-native`, because they are launch parameters; the API refuses to start against a launch with other values.

## kick_ball: find the ball with the onboard camera

One spoken or typed request ("go find the ball, walk over to it, and kick it", optionally "with your left leg") makes Pupper find the beach ball, walk to it, line up one front paw, kick with the leg kick trick, and stand on the walker again.

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
| Kick | STRICT switch to `trick_leg_kick` with `start_clip` set to `kick_right` or `kick_left`; after its init and fade-in (0.2 s + 0.3 s) the clip plays. See "Timing of the kick". | `play_kick` |
| Return | 2.6 s into the clip (controller feedback), `gemini_walk` takes the legs (the trick policy jitters when the robot is lifted). If the walker refuses for another reason, this server relaxes all leg controllers. While a Stop, a fall, a shared-stop block or a controller fault owns the legs, this server sends no switch request at all. | `play_kick`, `Robot.wait_start_clip` |

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
- `test_kick_ball_tool.py` (45 tests) runs the tools through the real queue and the fake ROS robot: full sequences, no ball, Stop during the approach and during the clip, a fall during the clip, a latched and a stale shared stop during the clip, walker refusal, camera and detector failure, calibration and speed refusals, `locate_ball`; `kick_in_place` (the trick takes the legs within `KICK_SETTLE_S`, and the walker takes them back at the end of the motion); `kick_at` (one job, the ball at the chest on the paw line at the switch, dry run, refusals, Stop during the walk; the pose mode: position within 1 cm and heading within 1° on the simulated walker, `settle` 0, `speed`).
- `test_ball_vision.py`: 37 NumPy tests (ray-traced 6-gore ball, false positives) run anywhere; 9 OpenCV tests (JPEG decode, remap, HSV, BGR/RGB order) run only where OpenCV is installed, for example on the robot.

None of these checks simulate the leg kick's contact with the ball. `foundation/scripts/check_kick_handoff.py` does (see "Timing of the kick"); it needs the mjlab environment and a C++ compiler:

```bash
PYTHONPATH=pupper-mjlab/src python foundation/scripts/check_kick_handoff.py --output NEW_DIRECTORY
```

## Operator program: `robot/kick_at.py` (use this one)

`kick_at.py` kicks a ball at known coordinates. The ball position comes from outside the robot (the team's field camera, a tape measure, or `ball_follow.sh --measure` after a snapshot check). X is the ball centre ahead of base_link and Y to the left, in metres.

```bash
python3 gemini_pupper/robot/kick_at.py X Y [--side right|left|auto] [--dry-run] [--fwd-gain 1.31] [--dead 0.14]
                                           [--angle DEG] [--point ball|robot] [--ft] [--speed 0.2] [--settle 0.15] [--leave 2.6]
```

The command makes one call to the API's `kick_at` tool, and that tool does the turn, the walk and the kick as one job. Other programs (the field camera's planner) can make the same call: `POST /tools/kick_at` with `{"x": 0.60, "y": 0.10, "side": "auto"}` and the `X-Pupper-Command-Epoch` header from `GET /health`; optional `dry_run`, `gain`, `delay`. If the planner drives the robot to the ball itself, it calls `POST /tools/kick_in_place` with `{"side": "right"}` when the ball centre is 0.25–0.26 m ahead of base_link on the paw line; the kick then starts 0.15 s later. Neither tool is offered to Gemini (`OPERATOR_ONLY_TOOLS`).

**Kick at a pose (2026-11-19, offline only).** With `--angle` or `--point robot` the same job goes to a pose first: turn to the point, walk straight to it, turn to the kick heading, kick. The angle is in degrees from the heading at the start, left positive.

- `--point robot`: X Y is where the robot centre stands; no ball is necessary. Example, 2 ft ahead and 1 ft to the right, kick straight ahead with the right leg: `kick_at.py 2 -1 --ft --point robot --angle 0 --side right` (turn −26.6°, walk 0.68 m, turn +26.6°, kick).
- `--point ball`: X Y is the ball centre; the robot stops behind the ball on the kick heading, with the ball at the chest on the paw line. This is the shot toward a goal: the angle is the direction from the ball to the goal.
- `--speed` sets the walk speed command (the gain 1.31 was measured only at 0.2 m/s). `--settle` sets the zero-speed time before the kick (0 is permitted). Each turn and walk step starts 0.2 s after the step before it.
- The same options are tool arguments: `angle`, `point`, `speed`, `settle`.
- All steps are timed with no feedback. The turn rate gain is not measured (assumed 1.0), so a pose with two turns has a larger error than the single-turn ball plan.

- Turn in place by θ = atan2(Y, X) − asin(paw_y / r), so that the ball lies on the paw line (right −0.046 m, left +0.040 m). Turns below 3° are skipped.
- 0.2 s at zero speed after the turn, then walk straight until the ball centre is at the chest (0.248 m; kick contact is 0.258 m), timed with the walker model (gain 1.31, start delay 0.14 s, at 0.2 m/s). Then the kick, 0.15 s after the last walk command.
- Offline (`tests/test_kick_at.py`, 41 tests): with turn and walk gains 0.9–1.1, 96 % of kicks land in the window at 0–15° bearing, 91 % at 15–30°, 69 % at 30–45° and 51 % at 45–60°. No camera is in the loop: a wide turn needs fresh coordinates after it.
- Robot, 2026-11-19, with the earlier three-call sequence (0.5 s wait before the trick): left leg, ball at (0.37, +0.017), walk 0.12 m: hit. Right leg, ball at (0.535, −0.039), straight walk 0.29 m: the paw passed beside the ball (side not recorded).
- The walker model was measured with a full stop after each walk. With the 0.15 s wait the trick can take the legs before the walker stops fully, which can end the walk short by an amount that is not measured. Do a `--dry-run` with a ruler first; if the ball is short of 0.25 m, increase `KICK_SETTLE_S` in `lab/part9_ball_kick.py` or decrease `--fwd-gain`.

## Operator program: `robot/ball_follow.py` (not reliable)

On the robot on 2026-11-19, the colour detector confused the ball with a blue chair, a cabinet edge and a bright wall under changing daylight. The fixes below improved it, but the box still leaked onto furniture (a 118 px box on a 95 px ball read 7 cm short). Use its `--measure` mode only with the snapshot check.

`ball_follow.py` is the Lab 5 follow-me (`follow_me_pupper`) retargeted from a person to the ball. It is a ROS node that runs beside the Gemini Pupper session and publishes on `cmd_vel_mux`'s `/person_following_cmd_vel` (the follower's own input; the API's `/llm_cmd_vel` has priority and is quiet between jobs).

- From Lab 5: the camera pipeline of `follow_service.py` (newest frame, 220° equirectangular 700 × 572, 12 Hz) and the state machine of `follow_me.py` (10 Hz, SEARCH ⇄ TRACK, yaw against the offset, `approach_speed`'s law for distance, back off when too close).
- Detector: colour blobs (ball red H ≤ 13 S ≥ 165; blue H 98–130 S ≥ 140; measured against the carpet, H 10–13 S 113–138, and jeans, S ≤ 131), grouped into one ball and widened through the white panels. COCO "sports ball" never fires on this ball. One colour is enough after two sightings in the same place. Position from the box and the camera model of the header-kick work (`ball_geometry`); height gate −0.26 to +0.03 m; lock gate 0.20 m.
- Target: the ball 0.50 m ahead on the kicking paw's line (1.2 cm; 2 cm after 8 s at the right distance). After a 0.6 s quiet wait, the fixes of 0.8–2 s must agree (no drift). Then a timed blind walk to the chest (`margin` −0.01: a long walk pushes the ball, a short one can miss it) and `kick_in_place`.
- Real frames (7 robot frames of 2026-11-18): fixes at 0.26–0.85 m, two colours from 0.32 m out.
- Closed loop (`tests/test_ball_follow.py`; walker gain ±10 %, turn gain 0.7–1.3, start delay 0.08–0.22 s, noise 0.8 cm × (1 + r), 30 % missed frames, 130 ms latency, blind below x 0.28 m), 300 placements per leg: 300/300 kicks in the window, forward error +0.1 ± 1.2 cm, sideways sd 0.6 cm (max 1.6 cm), median 8 s (p95 15 s). Weak points: forward gain ±20 % (26 % of kicks outside the window: set `--fwd-gain`) and 2× noise with 50 % missed frames (21 % time out).

```bash
bash ball_follow.sh right|left [--dry-run] [--measure] [--range-scale S] [--fwd-gain G]
```

## Operator program: `robot/ball_seek.py` (superseded)

On the robot on 2026-11-18 (4:49 pm PT, right dry run), `ball_seek.py` gave up after 60 steps: `locate_ball` fixes jumped 10–25 cm between looks and often missed after 1–5 cm steps, so the robot walked back and forth with the ball in front of it. `ball_follow.py` replaces it.

`ball_seek.py` is a stand-alone client (standard library only) for a test of the last part of the task: walk to a near ball and kick it with one front leg. It runs on the robot while the API runs, and it calls only the HTTP tools. It does not use the walker model or the calibration fit of `kick_ball`.

```bash
python3 gemini_pupper/robot/ball_seek.py right|left|auto [--dry-run] [--fwd-gain 1.31] [--dead 0.14] [--stage 0.55]
```

- Each step is one `locate_ball` look (no motion), then one `turn` or `move`. The program does not move while it looks.
- No ball: turn left in 40° steps. After a full turn with no ball, step away and search again: back 0.45 m, then left 0.6 m, then right 1.2 m. The turn is timed, so the heading after a full turn is not known. The three directions make sure that one step goes away from a ball in the blind zone near the body.
- Ball seen: turn to it, then step until the ball centre is 0.55 m ahead on the paw line (right y −0.046 m, left +0.040 m), tolerance 4 cm forward and 1 cm sideways. A line-up must agree in three looks. A look that jumps more than 0.25 m from the prediction gets a second look.
- The program learns the walker gain and start delay for each axis from each step. It decreases a step after a reversal. After four reversals on one axis, the tolerance on that axis doubles.
- Before the creep, a practice step backs up and walks forward for the creep's own duration, and measures the forward gain. Then the creep walks x − 0.258 − 0.008 m without the camera and calls `kick_in_place(side)`.
- `kick_in_place` (in `lab/part9_ball_kick.py`) plays the leg kick at the current position: walker, zero speed for 0.15 s, the kick clip, then the walker again. `OPERATOR_ONLY_TOOLS` in `robot/tool_registry.py` keeps it out of the Gemini declarations. `POST /tools/kick_in_place` still runs it.
- Ctrl-C, a refused or failed tool call, or an unexpected error sends Stop.

Offline checks: `tests/test_ball_seek.py` (17 tests) drives the program against a simulated robot. The simulated robot has unknown forward, side and turn gains (turn 0.7–1.4), start delay 0–0.22 s, slip, heading drift, camera noise that increases with range, 25 % missed looks, 2 % wrong-object looks, a 0.45 m blind range and a 105° half view. The body pushes the ball. On 2026-11-18, 300 placements for each case (in view, behind, or in the blind zone in front) gave:

| Case | Good kick | Abort (no kick) | Kick outside the window | Forward error | Sideways sd | Simulated time, median / p95 |
| --- | --- | --- | --- | --- | --- | --- |
| right / left / auto | 97 % | 0–1 % | 2–3 % | +0.7 to +0.8 cm ± 1.2–1.3 cm (short) | 0.8–0.9 cm | 53–56 s / 89–104 s |
| harsh: 40 % missed, 5 % wrong-object looks | 91–96 % | 1–3 % | 3–6 % | +0.8 to +0.9 cm ± 1.2–1.4 cm | 0.9–1.0 cm | 73–74 s / 131–152 s |

The window is 3 cm toward the body to 4 cm short of contact, and 2 cm from the paw line. A second check ran the unchanged program over HTTP against `server.py` in fake mode (real job queue, real tool code, simulated camera, walker gain 1.31). Six placements gave six kicks with the chosen leg: (0.85, −0.3) right, (−0.7, 0.2) left, (0.35, −0.03) right (blind zone), (−0.30, 0.05) left (blind zone behind), (0.6, 0.5) auto, and (0.9, −0.1) left. At each kick the ball centre was 0.261–0.266 m ahead and within 1 cm of the paw line. The walker was active again after each clip.

Robot steps for this program (each needs the operator's go; on 2026-11-18 the install kit is `ballseek-kit-20261118` with `BALLSEEK-RUNBOOK.txt`):

1. Install release u3 (u2 plus `kick_in_place` and `ball_seek.py`) and prepare a new run directory. No motion.
2. Robot on the stand: `start-native` (homes the legs), then `start-api` with the u3 manifest.
3. Floor: Options on the DualSense, then `stand_up`. Dry run for each leg (`--dry-run`): measure the ball centre with a ruler (0.55 m ahead, on the paw line).
4. Kicks: `right`, then `left`, ball 0.5–1.0 m away in view. Then `auto` with the ball behind the robot, and `right` with the ball 0.3–0.4 m in front of the nose.

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

R4 measures the ball at the walker stance. The trick's re-pose to its stand (0.2 s) moves the joints by at most 0.008 rad, mirror-symmetric (approximately 1 mm forward, 0 sideways, computed by the 2026-11-16 review), so R4 also stands for the kick stance.

Stop during R4–R7: a Stop in the app ends the job with zero speed, and during the kick clip it relaxes the trick (offline-tested); R3 latches the native e-stop, and the server then sends no switch request. If the robot jitters while the leg kick holds the legs, push R3.

**After the trials:** `stop-api`, then the native shutdown from the hardware checklist; leave the r6 release and the course-stack files unchanged.
