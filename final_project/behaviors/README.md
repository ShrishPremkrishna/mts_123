# Run-through kick (walker-based)

A local controller that kicks with the deployed walking policy: it walks Pupper to a point behind the ball on the aim line, turns to the aim, then drives the walker through the ball and brakes. No new policy is trained; the controller only sends `/cmd_vel`-style velocity commands. This is the TA's suggested approach and the proposal's version-1 striker. The learned joint-level kick (`pupper-mjlab/src/mjlab/tasks/pupper_kick/`) is the version-2 candidate.

## Files

| File | Purpose |
| --- | --- |
| `run_through_kick.py` | Pure-Python controller (no ROS, no numpy): states, limits, latency compensation, manual mode, field-to-robot conversion |
| `run_through_kick_node.py` | ROS 2 service around it (`input_mode` manual, robot or field), same shape as `follow_me_pupper/robot/follow_service.py` |
| `learned_kick.py`, `learned_kick_node.py` | Command bridge for the learned header-kick policy: entry window, command slot, controller switches, zero exit |
| `penalty_kick*.py`, `penalty_sim.py`, `launch/penalty_kick.launch.py`, `tools/walk_calibration.py` | Penalty kick with the leg-kick trick: find the ball, line up on the ball-to-goal line, kick with the left or right front leg. See `PENALTY_KICK.md` |
| `policies/header_kick_cpu_run2.json` | Learned policy (CPU run 2) with its manifest (hashes, validator, simulation results) |
| `launch/kick_trial.launch.py`, `tools/` | Trial launch, walker speed-cap config, policy installer, scripted ball for stand tests |
| `ROBOT_RUNBOOK.md`, `trial_log.csv` | Robot procedure and the trial record |
| `test_run_through_kick.py`, `test_kick_nodes_offline.py`, `_fake_ros.py` | Kinematic-model tests and offline node replay with stand-in rclpy |

## How it works

| State | Command |
| --- | --- |
| ALIGN | Walk at up to 0.35 m/s to the point `run_up` (0.8 m) behind the ball on the aim line, facing the ball. If the ball is in the way, orbit it at 0.6 m. Go to CHARGE when within 6 cm and 0.08 rad of the aim and the ball is within 4 cm of the body axis. |
| CHARGE | Ramp to `charge_speed` (1.0 m/s) at 1.5 m/s², hold the aim heading, correct sideways to keep the ball on the axis. |
| COAST | Inside the 0.40 m camera blind zone: hold the command and dead-reckon the ball until 0.25 m past contact. |
| BRAKE | Zero for 0.6 s, then DONE. |
| ABORTED | Zero. Causes: no fresh ball for 1.5 s while aligning (zero travel while stale), ball lost before the blind zone, a timeout or travel budget, or a stop (service, R3, face button, latched gateway). |

Each ball observation is moved from its capture time to now with the commands sent since then, scaled by `speed_efficiency`. Without a goal, the kick goes straight through the ball.

## Interface

- Ball: `/soccer/ball`, `geometry_msgs/PointStamped`, ball centre relative to `base_link` in the yaw-only frame (x forward, y left, m), `header.stamp` = image capture time on the robot clock.
- Goal (optional): `/soccer/goal`, same frame. Aim = direction from the ball to the goal.
- Output: `/kick_cmd_vel` `Twist` at 20 Hz while active, one zero `Twist` at the end; state on `/run_through_kick/state` (JSON).
- Services: `/activate_run_through_kick`, `/deactivate_run_through_kick` (`std_srvs/Trigger`). Every `KickParams` field is a ROS parameter.

## Robot setup

Follow `ROBOT_RUNBOOK.md`. For trials on the plain course stack (`teleop:=False`), `launch/kick_trial.launch.py` starts a private `cmd_vel_mux` with `/kick_cmd_vel` as its only input. Inside the Gemini stack, the harness owner adds `/kick_cmd_vel` to that stack's mux inputs instead. Speeds above 0.49 m/s need the walker cap raised (`tools/make_course_config.py --speed`).

Safety limits in the controller: zero travel on a stale ball while aligning, a 2.5 m align travel budget and a run-up + overrun + 0.3 m charge budget (they end a kick whose ball estimate is wrong but looks fresh), 10 s align and 4 s charge timeouts. The node refuses to start unless the e-stop gateway is released and exactly the mux publishes `/cmd_vel`, and stops on R3, a face button, a latched gateway or a second `/cmd_vel` publisher.

## Ball and body geometry

Without a bumper, the chassis front spans approximately 0.11 to 0.19 m above the floor at the 0.145 m standing height, and the front legs are approximately 0.16 m apart. A 15 cm ball is hit about 30° above its centre and still goes forward. A ball smaller than about 11 cm can pass under the chassis into the legs. Use a ball of 15 cm or more, or the foam bumper described in `pupper-mjlab/src/mjlab/tasks/pupper_kick/README.md`.

## Checks

```sh
python3 -m pytest -q final_project/behaviors
```

The tests use a kinematic stand-in (first-order velocity lag, 80% efficiency, 10 Hz camera with 0.1 s latency, noise and a 0.40 m blind zone). In a 200-setup sweep (ball 0.6 to 2.0 m away, bearing ±0.8 rad, goal direction ±1.2 rad) every run reached the ball, with 90th-percentile lateral error 1.5 cm and heading error 0.4° at contact. This checks the logic only: the shipped walker's physics and the ball impact are not simulated. `test_kick_nodes_offline.py` replays both ROS nodes against stand-in rclpy (`_fake_ros.py`); they have not run under real ROS (no ROS on the Mac).
