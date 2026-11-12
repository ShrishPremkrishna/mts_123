# Run-through kick (walker-based)

A local controller that kicks with the deployed walking policy: it walks Pupper to a point behind the ball on the aim line, turns to the aim, then drives the walker through the ball and brakes. No new policy is trained; the controller only sends `/cmd_vel`-style velocity commands. This is the TA's suggested approach and the proposal's version-1 striker. The learned joint-level kick (`pupper-mjlab/src/mjlab/tasks/pupper_kick/`) is the version-2 candidate.

## Files

| File | Purpose |
| --- | --- |
| `run_through_kick.py` | Pure-Python controller (no ROS, no numpy): states, limits, latency compensation |
| `run_through_kick_node.py` | ROS 2 service around it, same shape as `follow_me_pupper/robot/follow_service.py` |
| `test_run_through_kick.py` | 12 tests against a kinematic walker and camera model |

## How it works

| State | Command |
| --- | --- |
| ALIGN | Walk at up to 0.35 m/s to the point `run_up` (0.8 m) behind the ball on the aim line, facing the ball. If the ball is in the way, orbit it at 0.6 m. Go to CHARGE when within 6 cm and 0.08 rad of the aim and the ball is within 4 cm of the body axis. |
| CHARGE | Ramp to `charge_speed` (1.0 m/s) at 1.5 m/s², hold the aim heading, correct sideways to keep the ball on the axis. |
| COAST | Inside the 0.40 m camera blind zone: hold the command and dead-reckon the ball until 0.25 m past contact. |
| BRAKE | Zero for 0.6 s, then DONE. |
| ABORTED | Zero. Causes: no fresh ball for 1.5 s while aligning (zero travel while stale), ball lost before the blind zone, timeout, or deactivate. |

Each ball observation is moved from its capture time to now with the commands sent since then, scaled by `speed_efficiency`. Without a goal, the kick goes straight through the ball.

## Interface

- Ball: `/soccer/ball`, `geometry_msgs/PointStamped`, ball centre relative to `base_link` in the yaw-only frame (x forward, y left, m), `header.stamp` = image capture time on the robot clock.
- Goal (optional): `/soccer/goal`, same frame. Aim = direction from the ball to the goal.
- Output: `/kick_cmd_vel` `Twist` at 20 Hz while active, one zero `Twist` at the end; state on `/run_through_kick/state` (JSON).
- Services: `/activate_run_through_kick`, `/deactivate_run_through_kick` (`std_srvs/Trigger`). Every `KickParams` field is a ROS parameter.

## Robot setup (not done; robot and harness owners decide)

1. Add `/kick_cmd_vel` to the `cmd_vel_mux` inputs (`gemini_pupper/robot/launch/gemini_pupper.yaml`), below the joystick. The mux has a 0.05 deadband and drops a source after 500 ms.
2. For `charge_speed` above 0.49 m/s, load `follow_me_pupper/robot/speed_cap.yaml` (raises `speed_cap` and `max_vx_forward` to 1.5 m/s).
3. Start slow: `charge_speed:=0.5`, then raise it. Measure and set `speed_efficiency` (achieved / commanded speed), `blind_range` (closest reliable detection) and `contact_x` (body front + ball radius).

## Ball and body geometry

Without a bumper, the chassis front spans approximately 0.11 to 0.19 m above the floor at the 0.145 m standing height, and the front legs are approximately 0.16 m apart. A 15 cm ball is hit about 30° above its centre and still goes forward. A ball smaller than about 11 cm can pass under the chassis into the legs. Use a ball of 15 cm or more, or the foam bumper described in `pupper-mjlab/src/mjlab/tasks/pupper_kick/README.md`.

## Checks

```sh
python3 -m pytest -q final_project/behaviors/test_run_through_kick.py
```

The tests use a kinematic stand-in (first-order velocity lag, 80% efficiency, 10 Hz camera with 0.1 s latency, noise and a 0.40 m blind zone). In a 200-setup sweep (ball 0.6 to 2.0 m away, bearing ±0.8 rad, goal direction ±1.2 rad) every run reached the ball, with 90th-percentile lateral error 1.5 cm and heading error 0.4° at contact. This checks the logic only: the shipped walker's physics and the ball impact are not simulated, and the ROS node is compiled but not run (no ROS on the Mac).
