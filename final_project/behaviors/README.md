# Shooting behaviors

Recovered November 16, 2026 from `/home/pi/kick`, then corrected in this checkout.
[RECOVERY.json](RECOVERY.json) preserves the original hashes; the untouched
experiment and its archive remain available. [Reconciliation findings](../PI_RECONCILIATION.md)
separate source, simulated results and actual recorded robot outcomes.

The first candidate uses the existing walking policy to align behind a ball and
run through it. This follows the TA progression: establish measured contact with
locomotion before training a specialized shot. The learned header policy is
preserved as an **unaccepted experiment**: its recorded tilt-stop and late
activation handling remain unresolved. Nothing here starts with a source sync.

## Components

| File | Purpose |
| --- | --- |
| `run_through_kick.py` | Pure controller: align, charge, blind-zone coast, brake; manual, robot and field inputs |
| `run_through_kick_node.py` | ROS adapter with stock-controller/configuration checks and bounded fault handling |
| `ball_detector_node.py`, `ball_geometry.py`, `ball_color.py` | Onboard ball detection and capture-stamped, IMU-levelled robot-relative coordinates |
| `learned_kick.py`, `learned_kick_node.py`, `policies/` | Recovered target-conditioned learned candidate, held from activation |
| `launch/kick_trial.launch.py` | Shot adapter and private mux on top of an already running plain course stack |
| `tools/` | Recovered experimental helpers; the installer changes native source/config/policies and rebuilds, so it is not a baseline setup step |
| [ROBOT_RUNBOOK.md](ROBOT_RUNBOOK.md) | Current supervised stock-shot baseline and evidence to collect |

## Stock shot

The ROS node defaults to **0.45 m/s charge** and **0.3 m/s alignment**. It polls
controller ownership and default-controller parameters asynchronously, admitting
only `neural_controller_default` with stock model SHA-256
`ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712`.
Requests beyond the configured envelope (at most 0.49 m/s forward) are rejected.
The generic pure controller retains its experimental defaults; use the ROS node's
explicit baseline when evaluating hardware.

This prevents a concrete error in the recovered setup: dead reckoning integrated
requested speed even when the native walker clipped it. The fix rejects that
mismatch; it does not measure achieved speed. `speed_efficiency`, braking,
contact geometry and ball travel still need physical measurements.

Activation also requires a released e-stop gateway and expected `/cmd_vel`
publisher count. Missing/stale evidence, another active controller or a changed
model refuses a start. Loss of authority during a shot requests emergency stop
and aborts. Zero alone is not a stop for the learned target-conditioned slot.

The guard reads configured values, **not the native controller's loaded snapshot**.
The native controller caches model/configuration at initialization. Begin with a
verified fresh session and keep its configuration fixed. Sampled evidence and
publisher counts are not exclusive ownership; receiver-side command expiry is
still needed if the mux itself dies. Normal completion means commands finished,
not that the ball was hit.

## Interfaces and perception limits

- Output: `/kick_cmd_vel` (`Twist`), state: `/run_through_kick/state` (JSON).
- Services: `/activate_run_through_kick`, `/deactivate_run_through_kick` (`Trigger`).
- Ball: `/soccer/ball` (`PointStamped`), metres, x forward/y left in
  `base_link_yaw`; stamp is original image capture time on the robot ROS clock.
- Optional `/soccer/goal` uses the same point contract. In `input_mode=field`,
  both points and `/soccer/robot_pose` (`PoseStamped`) must use `frame_id=field`.
  Every field point needs a robot pose with its **exact capture stamp**, including
  a goal captured at a different time from the ball. A bounded 128-pose cache
  tolerates either callback arrival order; missing matches withhold the point.
  No overhead producer was recovered. It must supply synchronized snapshots;
  this adapter does not estimate poses between independently timed samples.

As of November 17, the stock shot consumer rejects wrong/empty frames, invalid,
future, expired, duplicate and reordered captures. Goals retain capture time;
arrival never extends their lifetime. Ball/goal captures are independently moved
to the current robot frame using the shot's command history before aim is
calculated. A capture preceding activation or retained history is unavailable.
Field orientation must be finite and unit length (up to 1% normalization error),
with a defined horizontal heading. Restart perception after a ROS clock reset;
an active camera shot aborts if its clock moves backwards.

`require_goal=true` waits for a goal even before its first packet. With the
default false, a fresh goal at activation or accepted during the shot latches goal
intent for that shot. Missing, stale or unsynchronized goal feedback pauses
ALIGN and aborts after `lost_timeout`; CHARGE aborts immediately. It does not
silently become a straight-through shot. An expired cache from a previous shot
does not latch the next activation. Goal-free shots and manual measurements
remain supported; the established blind-zone COAST behavior is unchanged.

Compensation assumes a stationary ball and goal and estimates motion from
commands scaled by `speed_efficiency`. It is not odometry and cannot correct
gait slip, a moving ball or timestamps from different clocks. The pure controller
also retains explicit `aim` for callers supplying an already-current direction;
the ROS adapter uses stamped goal points instead.

The onboard detector selects a valid preceding `base_link` IMU orientation for
an image. `imu_max_age=0.2` limits the IMU-to-image capture gap;
`max_image_age=0.5` limits image age at publication. It withholds coordinates for
missing/invalid/stale IMU, unsupported frames, or invalid/reordered capture times.
It never relabels unlevelled coordinates as `base_link_yaw` or refreshes a capture
stamp on receipt. The bounded 256-sample history covers these defaults at the
configured 260 Hz IMU rate. Restart the detector after a ROS clock-epoch reset;
ordering high-water marks intentionally do not automatically recover across it.

Color detection was tuned for a red/blue beach ball. The user's temporary
seven-inch soft mini basketball needs new detection/range calibration. Use manual
measured coordinates for the first stock-gait contact trials so detector errors
do not confound the result. Camera mounting, ball radius and empirical range
scale remain calibration inputs, not established accuracy guarantees.

## Offline checks

From the repository root:

```sh
.venv-api/bin/python -m pytest -q final_project/behaviors final_project/pupper_fc/tests
```

`test_run_through_kick.py` uses a kinematic walker with separately delayed ball
and goal captures, plus analytic turn/translation geometry; node tests use fake
ROS to exercise admission, matching field snapshots and goal-loss handling.
`test_ball_perception.py` checks geometry, source-time/frame handling and image
publication without a camera. `ros_stock_guard_smoke.py` checks actual ROS service
and topic wiring with synthetic services in an isolated, network-disabled
container. It must not run on the robot's ROS graph. The dated receipt in
`foundation/results/shot-baseline-guards-20261116.json` records exact source and
container identities. These checks do not simulate motor dynamics or ball impact.
