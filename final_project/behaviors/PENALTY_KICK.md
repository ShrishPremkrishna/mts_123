# Penalty kick: find the ball, line up, leg kick

Summary: one activation makes Pupper find the ball, walk to a pose on the line from the ball to a chosen point in the goal, and kick with the left or right front leg (the standing leg-kick trick policy in the kick slot). The walker does all locomotion through `/cmd_vel`. The camera guides the approach, and odometry guides the last 0.19 m, where the camera does not see the ball. The pipeline passed 90 offline tests and kicked in 600 of 600 random kinematic-sim setups (2026-11-16 6:30 pm PT). On mts-pi (2026-11-16 9:35 pm PT), the 32 tests passed, the node started under ROS 2 Jazzy and refused activation with no stack running, and the launch file parsed. It has not run with the course stack, and it has not moved the robot.

## Files (`final_project/behaviors/`)

| File | Purpose |
| --- | --- |
| `penalty_kick.py` | Pure-Python logic: `PenaltyParams`, the `PenaltyKick` state machine, `StrikeSequencer` (slot switch, clip request, watchdog, switch back) |
| `penalty_kick_node.py` | ROS 2 node: services, `/kick_cmd_vel`, controller switches, clip request, stops |
| `launch/penalty_kick.launch.py` | Private `cmd_vel_mux`, the node, and (optional) `ball_detector_node` |
| `penalty_sim.py` | Kinematic world (walker lag, efficiency and drift; camera latency, noise and blind zone; IMU drift; kick slot) and a Monte Carlo sweep |
| `tools/walk_calibration.py` | Floor measurement of the walker efficiency and drift, with the ball as a landmark |
| `test_penalty_kick.py` | 32 tests: whole attempts in the sim, strike faults, node replay with stand-in rclpy |

Reused: `ball_detector_node.py` (colour mode for the beach ball), `run_through_kick.py` (types), `_fake_ros.py`.

## Sequence

| State | Robot | Next state |
| --- | --- | --- |
| SEARCH | Stands still and looks (the first look is 2 s and also measures the IMU yaw drift). With no consistent ball (4 detections within 0.15 m), it turns 90° in place and looks again, up to 450°. | APPROACH. Abort: no ball, or IMU drift above 3°/s. |
| APPROACH | Walks at up to 0.30 m/s to the staging pose: the ball at (0.60, strike_y) in the kick-heading frame. It faces the ball, so the camera keeps it. A ball in the way is passed on a 0.60 m orbit. A ball older than 0.5 s gives zero travel. After 2 s, the robot searches again (it first steps back if the ball is too close to see). | MEASURE within 5 cm and 5°. |
| MEASURE | Stands 0.6 s, then averages detections for 0.8 s. | CREEP within 1 cm lateral and 1.5°; otherwise CORRECT. After 4 corrections: CREEP within 2 cm and 3°, otherwise abort. |
| CORRECT | Turns on the IMU yaw, then a timed step at 0.08 m/s. | MEASURE |
| CREEP | Walks straight at 0.15 m/s, heading held on the IMU. The camera corrects the lateral position down to 0.45 m and learns the sideways pull; the last part is odometry. It stops when the predicted ball x, minus the coast after a zero command, reaches 0.258 m. | SETTLE |
| SETTLE | Stands 1 s and checks the estimated placement (x ±5 cm, y ±2 cm, ±3°). `strike:=none` ends here (dry run). | STRIKE, or abort |
| STRIKE | STRICT switch walker → `neural_controller_student`; after `kick_delay_s` (slot init + fade-in), publish `kick_right` or `kick_left` on `/neural_controller_student/clip`; 0.5 s after the clip returns to `stand_idle`, STRICT switch back. Watchdog as in `leg_kick_buttons.py`. A refused switch back is tried 3 times, then `/emergency_stop`. | OBSERVE, or abort after the switch back |
| OBSERVE | Stands 3 s and records the farthest ball detection: `shot_travel_m`, `shot_err_deg` (shot direction minus the wanted direction). | DONE |

Kick geometry (from `handoffs/2026-11-16-leg-swing-kick.md`): ball centre at x = 0.258 m, y = −0.046 m (right kick) or +0.040 m (left kick) from `base_link`. The ball leaves at −3.4° (right) or +3.3° (left) from the body heading, so the robot faces the aim minus that bias. `side:=auto` chooses the side with the shorter walk to the staging pose (ties go right).

## The goal point

`goal_frame` sets how the robot knows where to send the ball:

- `ball` (default): `target_x`, `target_y` are the goal point from the ball, along the robot's **start heading**. Put the robot down facing the goal direction (the robot's position does not matter). For a goal 2 m past the ball, aimed 0.2 m left of the centre: `target_x:=2.0 target_y:=0.2`.
- `start`: the goal point in the start frame (origin at the robot's start position). The robot must turn only in place until it first sees the ball.
- `topic`: a live goal point on `/soccer/goal` (`PointStamped`, robot yaw-only frame). Use it when goal perception or overhead tracking exists.

The heading comes from the IMU yaw. Thus the aim error is the IMU heading error. The node measures the drift rate at rest during the first look and removes it.

## Robot procedure

Before the session: get Shrish's go for the Pi hold (checkpoint `foundation/results/interruption-recovery.json`) and Tej's go for each robot write. Clear a 3 m lane past the ball. One operator keeps R3 in reach. Check the IMU and the temperature as in `handoffs/2026-11-16-ball-kicks-next-session.md` ("Robot health first"): with the walker still, the reported roll and pitch must agree with the stance, and the Pi must not throttle.

1. Copy only the penalty files (from `final_project/behaviors/`). Do not use `--delete`, which erases `~/kick/logs/`. Do not copy the whole folder, because other sessions own the robot copies of the other kick files. The copy on 2026-11-16 9:34 pm PT used the robot's `run_through_kick.py` (older than the Mac copy, with all the names the penalty code imports).
   ```sh
   rsync -aR penalty_kick.py penalty_kick_node.py penalty_sim.py test_penalty_kick.py PENALTY_KICK.md launch/penalty_kick.launch.py tools/walk_calibration.py mts-pi:kick/
   ```
2. Start the course stack with teleop off (`ros2 launch neural_controller launch.py teleop:=False`). Make sure that the leg kick is the square-slot policy (handoff section 5). Push X so the walker stands.
3. Optional, saves 3.5 s per kick: load the kick slot with `init_duration: 0.5` and `fade_in_duration: 0.5` (the `/tmp/kick_student.yaml` method in the ball-kicks handoff), then use `kick_delay_s:=2.0`. Check the first kick of a session with the default 5.5 s.
4. Launch (`gateway:=false` on this robot's older stack):
   ```sh
   ros2 launch ~/kick/launch/penalty_kick.launch.py gateway:=false detector:=true target_x:=2.0 target_y:=0.0
   ```
5. Calibrate (next section) and set the values as launch arguments or with `ros2 param set /penalty_kick <name> <value>` (always decimals: `0.75`, not `1`).
6. Dry run: launch with `strike:=none`. Start an attempt and measure the real ball position to the kicking paw at the end. Compare with `placement` in the state message.
7. Kick: `strike:=leg_kick` (default).
   ```sh
   ros2 service call /activate_penalty_kick std_srvs/srv/Trigger
   ros2 topic echo /penalty_kick/state
   ```
   R1/R2 select the right kick, L1/L2 the left kick for the next attempt. R3 or a face button stops at once; `/deactivate_penalty_kick` stops and switches the walker back.
8. Record: `ros2 bag record -o ~/kick/logs/<name> /penalty_kick/state /soccer/ball /kick_cmd_vel /joint_states /imu_sensor_broadcaster/imu /neural_controller_student/clip_status`. The node log has one JSON line per state change and a final summary line. Add each attempt to `trial_log.csv`.

## Calibration

The shot direction changes approximately 7° per cm of lateral ball placement error (sim run 2: ±1.4 cm gave −12.5° to +6.5°). In the sim, the default walker values against a random walker gave a p90 lateral error of 1.88 cm and a p90 shot error of 13.4°. Calibrated values gave 0.77 cm and 5.4°. Thus do these steps before the kicks.

1. **Detector lateral offset.** Put the ball on the body centre line at 0.6 m and at 1.0 m from `base_link` (centre line = midway between the front paws). Read `python3 ~/kick/tools/ball_stats.py 3`. On 2026-11-16 the detector reported y = −0.040 m at 0.6 m and −0.043 m at 1.0 m, but the placement was not on a measured centre line. A constant offset sets `cam_y` (cam_y = −mean y). An offset that grows with range sets `cam_yaw_deg` (= −atan2(y, x) in degrees). Check again at 0.6 m and 1.0 m.
2. **Walker.** Ball 1.2 to 1.5 m straight ahead, walker standing, penalty kick idle:
   ```sh
   python3 ~/kick/tools/walk_calibration.py --vx 0.15 --seconds 3   # creep_efficiency, walk_drift
   python3 ~/kick/tools/walk_calibration.py --vx 0.30 --seconds 2   # speed_efficiency
   python3 ~/kick/tools/walk_calibration.py --vy 0.08 --seconds 2   # correction steps
   ```
   Do each 3 times. `efficiency` → `creep_efficiency` / `speed_efficiency`, `drift_per_m` → `walk_drift`. The camera measures both, so the range calibration (`range_scale`) also scales the efficiency. A turn above 2° during a straight walk means that the creep heading hold carries more load than the sim assumes.
3. **Strike point.** In the dry run (and during the kick delay of a real attempt), measure the ball centre relative to the kicking paw. If the real ball is reliably off, correct `strike_x` / `strike_y_<side>` or the creep efficiency, not the kick.

## Safety

- Activation refuses unless only the mux publishes `/cmd_vel`, the IMU is fresh, the gateway is released (`gateway:=true` stacks), the kick slot is inactive (not checked with `strike:=none`), the walker is active, and no other neural controller is active.
- Zero travel on a ball older than 0.5 s; travel budgets (6 m approach, the creep length + 6 cm); timeouts on every state.
- Stops: R3, a face button, `/emergency_stop`, a latched gateway (no switch: the operator or the e-stop owns the controllers); a second `/cmd_vel` publisher or a stale IMU (switch back if in the strike).
- Strike: the same watchdog as `leg_kick_buttons.py` (window mean of max joint speed above 1.5 rad/s outside a kick), the switch back 0.5 s after the clip, `/emergency_stop` after 3 refused switch-back tries. Ctrl-C during the strike publishes `/emergency_stop`, because the switch back needs the executor.

## Offline evidence (2026-11-16 6:30 pm PT)

`python3 -m pytest -q final_project/behaviors`: 90 passed (32 in `test_penalty_kick.py`). `python3 penalty_sim.py --runs 300` (random start within 0.8 to 2.5 m and ±150° of the ball; goal point 1.5 to 3 m past it, within ±100° of the robot-to-ball line; walker efficiency 0.65 to 0.95, lag 0.10 to 0.25 s, drift ±5 cm/m; IMU drift ±1°/s):

| p50 / p90 | Calibrated (seed 11) | Default walker values (seed 12) |
| --- | --- | --- |
| Attempts that kicked | 300 / 300 | 300 / 300 |
| Lateral placement error | 0.30 / 0.77 cm | 0.79 / 1.88 cm |
| x placement error | 0.57 / 1.18 cm | 2.0 / 3.66 cm |
| Shot direction error | 2.1 / 5.4° | 5.5 / 13.4° |
| Miss at the goal point | 0.08 / 0.21 m | 0.20 / 0.57 m |
| Time to the strike | 17.7 / 23.7 s | 17.6 / 23.3 s |

Then the strike takes approximately 9.7 s with `kick_delay_s` 5.5 (approximately 6 s with 2.0). The heading part of the shot error is 0.24° p50 and 0.55° p90; the lateral placement causes most of it. The closest approach to the ball before the creep was 0.443 m.

The sim does not model gait dynamics, the walker at small commands (below approximately 0.05 m/s it may not move), the ball contact, a moving ball, the real detector (false blobs, other red or blue objects, lighting), or a detector lateral bias above 1 cm. A 4 cm detector bias (possible from the 2026-11-16 readings) would give approximately 28° of shot error, which is why calibration step 1 comes first.

## Open items

- Not run with the course stack or on the floor. First robot steps: the dry run, then a kick with the ball.
- The left kick has not run on the robot (handoff section 4).
- The detector sees the ball down to 0.59 m (measured). `stage_x` 0.60 assumes this; `blind_range` 0.45 assumes it also sees 0.45 to 0.59 m. If it does not, raise `blind_range` to 0.55 and expect more odometry error. Detections closer than 0.45 m are not used; SETTLE records them as `near_detections` for later evaluation.
- The IMU fused-yaw fault (`handoffs/2026-11-16-fr-leg-jiggle.md`) can drift up to 5°/s; above 3°/s the attempt aborts. The detector runs without IMU levelling by default (`detector_imu:=false`).
- Integration with the Gemini stack: add `/kick_cmd_vel` to that stack's mux, and call the two services from a harness tool (dispatch and cancellation in the harness, control here).

## Next: a run-up kick

`strike` selects the strike. A future `run_up` strike can reuse SEARCH, APPROACH and the fine alignment with a larger `stage_x` (the run-up length), then replace CREEP, SETTLE and STRIKE with a timed run-up whose kick policy reads the ball offset. The kick-heading geometry, the side choice and `StrikeSequencer` (switch, clip, watchdog, exit) stay the same.
