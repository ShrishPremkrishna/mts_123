# Movement and stock-shot trials

Updated November 16, 2026. The [shared build handoff](../ROBOT_BUILD_AND_TEST_HANDOFF.md)
explains release selection and ownership. The [checkpoint](../../foundation/results/interruption-recovery.json)
records current access. Commands below are for the selected, supervised session;
a directory existing or tests passing does not authorize motion.

## Select the runtime once

Use a completed, pinned release with a compatible Pi-native controller, the repaired
driver, stop gateway, expanded configuration and exact stock actor. The first
November 16 contact trials used an older driver and failed physical acceptance;
see [their receipt](../../foundation/results/stock-shot-session-20261116.json).
The Ubuntu-built `movement-20261116-v1` is held. Source synchronization is not native deployment.

Follow the handoff's preflight/startup commands. Preflight verifies the boot,
selected files, policy/config link and target ABI while no manager is running.
The selected launch checks ROS package resolution before it can home. After
startup, the loaded check verifies the newly started manager, supplied config,
and actual mapped driver/controller libraries. `movement_trial.py --execute`
requires this receipt and repeats the identity check before movement.

The selected stock launch starts one mux, one stock walker (inactive), the stop
gateway and feedback. It does not start camera, voice, alternate policies or a
shot adapter. Home once on the stand; move to clear floor and activate only with
the supervisor ready. Keep this session between trials. Profiles are selected at
startup; changing a ROS parameter store does not change cached native settings.

## Short axis trials before ball contact

Use the release's copied tool, not a changing checkout. Populate `PUPPER_RELEASE`,
`PUPPER_RUN` and `PUPPER_PROFILE` from the selected release; `PUPPER_RUNTIME_RECEIPT`
is the completed preflight file. Stop any shot adapter before these trials.

```sh
# Receipt-only dry run: no ROS and no movement.
python3 "$PUPPER_RELEASE/final_project/behaviors/tools/movement_trial.py" forward \
  --profile "$PUPPER_PROFILE" --speed 0.2 --duration 1 --out "$PUPPER_RUN/trials"

# With the supervisor watching, this moves the robot.
python3 "$PUPPER_RELEASE/final_project/behaviors/tools/movement_trial.py" turn_left \
  --profile "$PUPPER_PROFILE" --duration 1.5 --out "$PUPPER_RUN/trials" \
  --runtime-receipt "$PUPPER_RUNTIME_RECEIPT" --execute
```

Use `forward`, `backward`, `left`, `right`, `turn_left` or `turn_right`. Defaults
are 0.2 m/s forward/backward, 0.15 m/s lateral, 0.35 rad/s turning and one second.
Each pulse is at most three seconds. It refuses missing/stale stock evidence,
wrong policy, incompatible runtime, missing 960-value observation, competing
publishers and a latched gateway. It never homes, arms or switches controllers.
A refused trial publishes no cleanup zeros into someone else's stream.

| Profile | Tool ceiling, further restricted by loaded native configuration |
| --- | --- |
| `walk` | Forward 0.49 m/s; backward/lateral 0.3 m/s; turning 1 rad/s |
| `sprint` | Forward 1.0 m/s; backward 0.3 m/s; lateral 0.5 m/s; turning 1 rad/s |

The sprint profile is an **explicit physical candidate**, supported by simulation,
not a claim that these limits are reliable on carpet. Its initial pulses still
use the low defaults. Progress to 0.45, 0.7 and 1.0 only after the preceding
forward/braking observation is controlled. Do not change gains or motor limits
while diagnosing command delivery. A configured high ceiling does not command
that speed by itself.

Every pulse prints its result immediately and saves one JSON. It records requested
commands alongside the native policy input: **20 newest-first frames × 48 values**,
with command at indices **6:9**. Forward/lateral matching compares X/Y; heading
hold may intentionally change yaw, which remains in the record. Missing feedback
prevents a start or ends the diagnostic pulse. Add the observed travel, stopping,
direction and stability to the operator observation; a completed command is not
physical acceptance. IMU yaw is sensor evidence, not an external heading reference.

The current source distinguishes a finished pulse timer from a verified command
path: execution returns success only when the policy input matched, zero feedback
arrived and the selected processes remained valid. A finished pulse with missing
evidence returns status 5 and prints `command_verification=UNVERIFIED`; stopping
and refusing a start have separate statuses. This reporting change is offline
only until a new release is selected; frozen v2 retains its earlier reporting.

The new controller expires silent velocity input after 250 ms, replacing the
request with zero. This does not prove braking distance, protect against a hung
controller, or cancel target-conditioned learned policies. R3/power remain the
supervisor's stop controls. Sender and mux timeouts are additional layers, not
proof that the robot stopped.

After the November 16 lateral fall, include the selected controller's
`observation`, `policy_output`, `position_command`, `imu_latency_seconds` and
`policy_inference_latency_seconds` topics, plus `/rosout`, in the existing
joint/IMU/command/Stop recording. Observation precedes inference and the final
position-command publication; recording both helps locate a control stall.
Compare message source stamps with recorder receipt times: a recorder delay
alone does not establish a stalled controller. Validate the logging correction
while supported before another floor trial; the failed session is closed.

### Keeping the trial client ready

The reviewed client also supports `--session --execute`: one ROS process stays
ready between trials, with separate receipts and the actual client source hash.
Use the explicitly selected client directory; frozen older releases do not gain
this mode when the source checkout changes. Native release and preflight receipt
remain unchanged for a client-only update.

```sh
python3 "$PUPPER_CLIENT/final_project/behaviors/tools/movement_trial.py" \
  --session --execute --profile "$PUPPER_PROFILE" \
  --runtime-receipt "$PUPPER_RUNTIME_RECEIPT" --out "$PUPPER_RUN/trials"
```

Send JSON lines on its input: `{"command":"status"}` or
`{"command":"prepare","axis":"forward","speed":0.7,"duration":3}`.
Preparation returns a token; send `{"command":"run","token":"RETURNED_TOKEN"}`
only when the operator is watching and the lane is clear. It runs once, with no
queued follow-up. `stop`, `quit`, input disconnect, or an authority fault closes
the client; none rehomes the robot. Motion still requires a ready active stock
controller. Ordinary requests retain the three-second maximum and command limits.

Current source additionally supports explicit `--lateral-calibration` for
left/right only, at no more than 0.15 m/s and 20 seconds. In persistent mode,
each prepare must include `"lateral_calibration":true`; it is never inherited.
The same feedback, authority, identity and Stop guards remain. This extension
has software-test evidence only until selected and physically qualified; the
logging-correction release still contains the earlier three-second client.
A duration or command integral is not a measured six-foot traverse.

Options releases an ordinary gateway latch **and activates the remembered
controller**. Confirm the resulting active state; do not issue a redundant
STRICT activation. A gateway fault is different from an ordinary Stop and
cannot be cleared by Options. Preserve the same healthy native session across
trials to avoid repeated homing. Software Start remains a separate offline
change until its reviewed release is selected.

Timed turning is not a commanded angle: the November 16 three-second turn at
0.8 rad/s produced about 123 degrees of reported yaw. Do not call that a complete
180-degree turn or start a return sprint without checking the return direction.

## Headbutt after movement is controlled

First measure a no-ball advance and braking distance on the actual surface. Then
use the temporary seven-inch soft ball/no bumper and measure real contact and
ball travel. Start centered before offsets. The onboard multicolor detector is
not calibrated and did not drive the previous manual shots; do not let detector
errors obscure gait or contact geometry.

The selected stock session already owns its mux. After closing the pulse tool,
start only the shot node, not a second `kick_trial.launch.py` mux:

```sh
python3 "$PUPPER_RELEASE/final_project/behaviors/run_through_kick_node.py" \
  --ros-args -p input_mode:=manual -p charge_speed:=0.45
```

Set fresh measured coordinates before each activation (metres, body centre,
x forward/y left). Manual x is 0.9–1.5 m and |y| at most 0.5 m.

```sh
ros2 param set /run_through_kick manual_ball_x 1.0
ros2 param set /run_through_kick manual_ball_y 0.0
ros2 service call /activate_run_through_kick std_srvs/srv/Trigger
```

The shot node retains its 0.49 m/s ceiling even in a sprint-capable native session.
Its dead reckoning integrates requested motion, not measured travel. `done` only
means the command sequence ended. Record contact/miss, ball distance/direction,
robot stability and stopping; preserve failures. Do not activate the learned
header or foot-kick slot through this velocity route: their targets have different
semantics, and zero is not their stop command.

When finished, stop the active behavior, have the supervisor support Pupper,
then close the recorded owned manager/launch/recorder and verify shutdown. Do not
replace binaries, pull source into a live release or reuse old boot/PID receipts.
