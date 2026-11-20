# Pupper build and test handoff

Updated November 19, 2026. The [checkpoint](../foundation/results/interruption-recovery.json)
owns the selected release, profile hashes, current operator and next actions.
This document describes the procedure; historical releases do not assign work.

## Current path

The last floor-tested release is `movement-20261117-logfix`. Its sprint profile
produced the recorded forward and short lateral results. The
[legacy guarded walk](../foundation/results/staged-field-session-20261119.json)
was assembled on the Pi using the existing native binaries. Its loaded runtime,
camera stopping and physical walking await qualification. Direct command-deadline
operation requires the selected Pi-compatible controller and a separate runtime
check. The [staging report](../foundation/results/staged-field-session-20261119.json)
records target build and release evidence.
The [November 17 source kit](../artifacts/pupper-fc/2026-11-17/fc-strict-trial-source-kit-20261117T160403Z/summary.json)
contains separate source archives for the native controller and software-Start
gateway. It covers the guarded movement work through that date. The November 19
overhead producer, clock adapter and offline pose planner live in the active
checkout. The kit has no Pi binaries, camera environment or field calibration.

| Selection | Motion path | Preparation |
| --- | --- | --- |
| Existing stock release | Twist through mux to native | Select its existing whole release/profile. |
| Guarded legacy candidate | Twist through mux and camera guard | Staged walk profile, all-obstacle guard, original native binaries. |
| Direct FC candidate (`fc_deadline_v1`) | Original-deadline command through camera guard to native | Target-built controller and strict 15-role source index; explicitly select the new controller digest. No active mux. |

The [hardware checklist](../foundation/HARDWARE_CHECKLIST.md#guarded-walk-candidate)
owns assembly commands. Assembly copies files and starts nothing. The software-Start
service additionally requires a target-built gateway and its generated service
bindings; controller-only assembly does not add it. The accepted Options activation
path remains available. A source kit, successful build or file copy never selects
the currently running release.

## Target build for camera-guided play

Stock straight trials use the floor-tested release independently of this build. The direct-deadline
controller and software Start gateway need the Pi's Debian 12 ARM64 runtime.
The local Ubuntu Jazzy image is suitable for simulation and has incompatible
C++/Python dependencies for deployment. [Build audit](../foundation/results/build-environment-audit-20261116.json).

The prepared runner lives at
`/home/pi/pupper-fc-source/native-build-runner-X6aa22bm/build-one.sh`.
For a new build, check the staging report for completed packages first. When the
Pi is idle, run the runner with Bash and the single argument
`neural_controller`, then `joy_utils`. Each invocation uses an isolated build
directory. The runner's source and dependency checks passed on the target.
Retain its build log, behavioral test results and installed-file hashes before
assembling a separate direct-deadline release and verifying its loaded libraries.
The [staging report](../foundation/results/staged-field-session-20261119.json)
records the runner digest, fallback release and outstanding checks.

`pupper_gait_deploy/install.py` and `deploy.sh --no-launch` update the shared
platform install. Use the isolated runner for this build.

## One selected release, one persistent physical session

One operator owns motion, camera configuration and restarts. When taking over an
existing session, exchange boot, release/profile, run directory, process identities
and motor state. Reuse a matching homed session and its receipt for successive
trials; do not prepare, home or activate again for every command.

Use a fresh bounded connection after a battery swap or reboot:

```sh
ssh -S none -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 mts
```

The previous shared SSH connection could stay stale for minutes after power-off.
This bypasses it without changing the user's SSH configuration. An unavailable
Pi is not permission to restart services or automatically home on reconnect.

In a Pi Bash shell, select the reviewed release, profile, manifest and hash.
The [stock straight trial](#first-camera-free-straight-trial) gives the four
values for the baseline. Source the selected environment in each Pi terminal.
Before homing, check the boot and attached gamepad with motors inactive:

```sh
source "$PUPPER_RELEASE/environment.sh"
cat /proc/sys/kernel/random/boot_id
awk 'BEGIN { RS="" } /DualSense|Wireless Controller|Handlers=.*js[0-9]/ { print $0 "\n" }' /proc/bus/input/devices
```

Confirm the expected controller has an input handler. If missing, reconnect it
and repeat the check. The [November 19 field session](../foundation/results/staged-field-session-20261119.json)
closed after the operator supported the robot, the driver deactivated and the
recorder exited. Its preflight and loaded receipts belong to that session.
For a new session, confirm ownership and idle hardware, then create a fresh
preflight in a new run directory even if the boot and stock manifest still match:

```sh
PUPPER_RUN=$(mktemp -d /home/pi/pupper-fc-runs/coordinate-stage1-XXXXXXXX)
python3 "$PUPPER_RELEASE/foundation/hardware/runtime_identity.py" preflight \
  --manifest "$PUPPER_MANIFEST" --sha256 "$PUPPER_MANIFEST_SHA256" \
  --output "$PUPPER_RUN/preflight.json" &&
  printf 'export PUPPER_RUN=%q\n' "$PUPPER_RUN"
```

Preflight returns `motion_started=false`. Copy the printed
`export PUPPER_RUN=…` line into **each new Pi terminal** and set the same
release/profile environment there. Before using any terminal for the recorder,
launch or trials, confirm it can read this session's receipt:

```sh
test -r "${PUPPER_RUN:?Set the run path printed by preflight}/preflight.json"
```

After this check passes, start one recorder in the selected environment if the session has none:

```sh
ros2 bag record -o "$PUPPER_RUN/telemetry" /joy /cmd_vel /kick_cmd_vel \
  /joint_states /imu_sensor_broadcaster/imu /neural_controller_default/observation \
  /estop_controller/stop_status /emergency_stop /rosout
```

With the supervisor ready, R3 and power accessible, and Pupper supported,
start the selected launch in its own terminal:

```sh
export PUPPER_RUNTIME_RECEIPT="$PUPPER_RUN/preflight.json"
export PUPPER_IMU_STARTUP_TRACE="$PUPPER_RUN/imu-startup.csv"
set -o pipefail
ros2 launch "$PUPPER_RELEASE/foundation/hardware/stock_session.launch.py" 2>&1 | tee "$PUPPER_RUN/native.log"
```

**This can home the legs.** It starts the manager, Stop gateway, joystick input
and inactive walker. Guarded selections add their pinned guard; direct FC omits
the mux. Camera production and the gameplay bridge are separate selected processes.
Keep the owning terminal and logs. From the same selected environment, check:

```sh
python3 "$PUPPER_RELEASE/foundation/hardware/runtime_identity.py" loaded \
  --receipt "$PUPPER_RUN/preflight.json" > "$PUPPER_RUN/loaded.json"
```

The check verifies the supplied configuration, mapped libraries and required
process identities. Look for homing completion and the opened DualSense
joystick in the launch log. On the supported robot, use Options
once and check `ros2 control list_controllers` for `neural_controller_default`
active before the operator sets it on the mark. Strict FC also requires
matching loaded contract/clock and
fresh native binding to the bridge's source before arm. A replaced bridge source
requires a deliberate native Stop/Start; disarmed zero traffic does not arm it.

For stock isolated pulses, use the [movement runbook](behaviors/ROBOT_RUNBOOK.md)
and stop other command publishers. Do not use Twist pulse helpers as a test of
the strict FC path: it deliberately has no Twist input. Qualify the selected
bridge/guard/controller path on the stand before its bounded floor trials.
Record actual distance, heading, stability and braking separately from command
receipts. The stock observation layout is 20 frames × 48 values, with command
indices 6:9 in the newest frame.

## First camera-free straight trial

Use these Pi settings in the [session procedure](#one-selected-release-one-persistent-physical-session)
above. They select the prior floor-tested native settings. The baseline requests
0.2 m/s and can run independently of either camera. [Floor evidence](../foundation/results/gait-gate-20261116.json).

```sh
export PUPPER_RELEASE=/home/pi/pupper-fc-releases/movement-20261117-logfix
export PUPPER_PROFILE=sprint
export PUPPER_MANIFEST="$PUPPER_RELEASE/sprint-manifest.json"
export PUPPER_MANIFEST_SHA256=a4656f22a8c0e5d96bcfec6d3bbbacd6c45e8bc67ea73473a1b139d3756b44c1
```

On the Mac, generate the nominal 20 cm straight target from the active repository.
The scale of 1 is an initial estimate. Keep the plan and mark the start position,
heading, target and the robot reference point used for tape measurements.
Choose a repeat count before starting, for example three attempts. Record x
forward along the start heading, y to its left and yaw counterclockwise.

```sh
.venv-api/bin/python final_project/behaviors/tools/pose_trial.py \
  --start 0 0 0 --target 0.2 0 0 --speed 0.2 --distance-scale 1
```

On the Pi, use the same release environment, profile and run directory as the
native terminal. With the robot ready and one deliberate Options activation
confirmed, start its resident trial client once:

```sh
set -o pipefail
python3 -u "$PUPPER_RELEASE/final_project/behaviors/tools/movement_trial.py" \
  --session --execute --profile "$PUPPER_PROFILE" \
  --runtime-receipt "$PUPPER_RUN/preflight.json" --out "$PUPPER_RUN/trials" | tee -a "$PUPPER_RUN/trial-session.jsonl"
```

1. Send `{"command":"status"}` and check readiness. In a clear lane, send
   `{"command":"prepare","axis":"forward","speed":0.2,"duration":1}`. When the
   operator is watching and the response has `event=prepared` and `problem=null`, send
   `{"command":"run","token":"RETURNED_TOKEN"}` with that attempt’s token.
2. Read the individual trial receipt. Require `outcome=completed`,
   `command_verification.passed=true` and a recorded
   `summary.zero_confirmed_in_observation_s`. Observe the physical stop, then
   measure the endpoint before repositioning. Preserve interruptions, refusals
   and missing measurements. Check the cause before another attempt.
3. Restore the start and heading for each repeat. Use a fresh prepare/run token.
   On the Mac, record a JSON array with a unique `trial_id`, `outcome`, `x` and
   `y` in metres and `yaw` in **radians** for each attempt. Use `null` for
   unmeasured values. Score with
   `.venv-api/bin/python final_project/behaviors/tools/pose_trial.py --target 0.2 0 0 --measurements endpoints.json`.
   The score reports RMS distance to the target and RMS spread about the mean
   endpoint. A paired x/y measurement counts even when heading was unavailable. Spread needs
   at least two endpoints. Heading errors use the recorded yaw independently.

The frozen stock client supports timed axis pulses. For the first physical
IMU turn, the separately selected client at `dc3e124` verified the loaded
stock runtime and accepted a relative +20° request. The IMU reported +15.6°
at the six-second cutoff. Qualify sustained heading and a confirmed stop in
isolated turns before combining them with translation.
[Turn receipt and source selection](../foundation/results/staged-field-session-20261119.json).
See the [movement runbook](behaviors/ROBOT_RUNBOOK.md#direct-deadline-qualification-candidate)
for the guarded candidate, which also requires a real onboard camera producer.

## Mounted overhead camera, motors inactive

1. Print pages **2–9** of the [tag packet](overhead_perception/tags/pupper_fc_tags.pdf)
   on US Letter at **Actual Size / 100%**. Page 1 is a placement guide with
   readable duplicate IDs. Keep it outside the camera view. Check the printed
   black squares with a ruler. Floor IDs 0–5 are 17 cm per side and robot IDs
   10/11 are 8 cm. Preserve the white border and the robot card's FRONT marking.
2. Centre tags 0, 1, 2, 3 on west-south, east-south, east-north, west-north
   corners respectively. The east goal lies between 1 and 2. For inward
   placement enter the measured x/y inset in `tags.corner_inset` (metres).
   Tags 4 and 5 go at midfield on the south and north sides. Centre each goal
   on its end line. Measure both clear openings between the inside edges of
   the posts. The current model represents two equal, centred goal mouths.
   Replace the 0.82 m placeholder in `field.goal_width` with the measured
   common opening. Tape the field length and width too.
3. Mount the camera with all four corner tags visible, including their white
   borders. Place the loose tag 10 level on a support at the measured
   standing tag height. Set `robots.striker.tag_height` and orient FRONT
   along the known heading. For the mounted robot also measure its
   `tag_offset` from the body centre (forward, left). Use tag 11 and the
   `keeper` settings for the second robot.

On the Mac, start from the active repository root. The interactive camera
uses full OpenCV in its own environment. The November 19 Mac setup passed
all 70 perception tests and a four-frame simulated corner calibration using
Python 3.12.8 and OpenCV 5.1.0 (Cocoa). For a new checkout, create it once:

```sh
cd final_project/overhead_perception
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Create a run folder and copy the [field configuration](overhead_perception/config/field.yaml)
into it from the perception directory:

```sh
mkdir -p out
camera_run=$(mktemp -d out/field-camera-XXXXXXXX)
cp config/field.yaml "$camera_run/field.yaml"
```

Enter the measurements above in the copy. Leave `net.host` at `127.0.0.1`
for Mac-only observation. In the same terminal, start the live dashboard:

```sh
.venv/bin/python -u scripts/run_perception.py --source phone --config "$camera_run/field.yaml" \
  --corners --dashboard --print > "$camera_run/perception.log"
```

At the field centre, near both goals and near a sideline, hold the card on tape
marks and point FRONT toward +x, then +y (0°, +90°). Record tape pose and
dashboard pose for each position and a short moving pass. Count frames with
`calibration_valid=true` and `robots.striker.measured_this_frame=true`;
`visible` alone includes predictions. Retain the camera log, field YAML,
lens/resolution and tag height. Check height, scale and orientation against
the marks before putting the tag on a robot. The pink ball can be added when
available. Measure its radius and tune its colour with the dashboard's `b`
picker under the arena lights.

For later Pi ingress, set `net.host` to the intended Pi address and run
`python3 final_project/behaviors/overhead_ingress.py --duration 15` from the Pi
source checkout for receive-only discovery. Capture the producer session and
calibration revision. Corner calibration estimates length and width anew.
Check them against tape and use the **full-precision values** from a valid
frame's `field` in the FC field JSON. The dashboard rounds to centimetres.
Alternatively, configure the measured field and tag-centre coordinates and
use measured-tag calibration. The adapter checks incoming dimensions and goal
segments to 0.000001 m. Both ingress and bridge need the same geometry file.
It contains `field` with `length`, `width`, `goal_width`, `ball_radius`,
`robot_radius`, and `bumper_offset` (centre to front contact point, metres),
and `attack_signs` with opposite +1/−1 values for `pupper-1` and `pupper-2`.
Measure lens intrinsics and capture latency with the selected camera mode.
Network clock admission and physical stopping are separate tests.
[Camera tools](overhead_perception/README.md#remaining-bring-up-on-the-arena)
· [FC field configuration](behaviors/README.md).

## Operator kick after the straight baseline

The stock sprint release runs only the walker. The
[operator kick](../gemini_pupper/README.md#operator-soccer-kick-candidate)
requires a separately prepared Gemini release, its clip assets and the same
`PUPPER_SESSION_SKILLS=leg_kick` selection in both launcher and API. Obtain
the applicable course clearance for this policy and review its replay before
hardware use. Close the stock session before selecting the kick release.

For each no-ball side trial, confirm the walker owns the legs, submit one
operator kick with the current command epoch and follow its returned job ID
in `GET /queue/status` (`recent_outcomes`). Record the requested side, observed
foot, matching clip/frame, terminal result and restoration of the walker.
The main operator path uses a 0.5-second kick initialization and 1.0-second
fade. Tej’s u5 is the first comparison candidate. The Pi logged both legs
on the same commanded 0.305 m approach. Default settings took 5.76/5.63 s
(left/right). Zero settling time and a 2.1 s clip exit took 4.98/5.03 s.
All four jobs reported locomotion active. Tej’s estimated 2.3 s from walk
end to the default clip’s contact phase needs an actual ball and synchronized
video to validate. Use the default as the reference when checking whether the
faster settings preserve balance, contact and the first walking step.
Record the selected release, start pose, stability, clip progress and the
first observed response to a new walk command for each comparison.
[Tej's u5 trial and settings at 30b84a0](https://github.com/ShrishPremkrishna/mts_123/blob/30b84a096deab79a43a966224ccd571c21f5662a/gemini_pupper/docs/kick-ball.md#timing-of-the-kick-all-kick-tools)
· [Main kick selection and software tests](../foundation/results/staged-field-session-20261119.json).

For contact trials, use the intended ball and record its marked starting
position relative to the selected paw, visible foot contact or miss, ball
travel direction and distance, and robot stability. Time actual contact and
the return to walking from the same external recording.

## Close and recover

Zero velocity, controller Stop and full driver shutdown have different effects.
The owner supports the robot as needed, closes ingress and the owned native
session, then verifies process exits and recording closure using the
[shutdown procedure](../foundation/HARDWARE_CHECKLIST.md). Keep failed runs and
uncertain observations; do not kill by broad process name or overwrite evidence.
On the frozen stock client, idle `quit` or `stop` exits the client while the
native session can remain active. Check native deactivation and driver shutdown
separately.

Rollback selects a previously recorded **whole release/profile** after the old
session closes. Never replace a live library or mix historical manifests. Native
parameters are cached at initialization, so changing the parameter store is not
proof of applied settings. A profile change needs a fresh coordinated session.

From the Mac checkout, start offline checks with:

```sh
python3 foundation/scripts/session_status.py
.venv-api/bin/python -m pytest -q final_project/behaviors final_project/pupper_fc/tests
```

The checkpoint links the tested source kit, current results and pending physical
acceptance. [Earlier v2/v3 work](../foundation/results/movement-offline-followthrough-20261116.json)
and the [fall investigation](../foundation/results/gait-gate-20261116.json) remain
historical evidence; they are not a second queue of next actions.
