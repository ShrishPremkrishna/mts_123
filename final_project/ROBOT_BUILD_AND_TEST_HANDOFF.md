# Pupper build and test handoff

Updated November 19, 2026. The [checkpoint](../foundation/results/interruption-recovery.json)
owns the selected release, profile hashes, current operator and next actions.
This document describes the procedure; historical releases do not assign work.

## Current path

The last floor-tested release is `movement-20261117-logfix`. Its sprint profile
produced the recorded forward and short lateral results. The new guarded walk
and direct command-deadline modes are software candidates, not deployed releases.
The [November 17 source kit](../artifacts/pupper-fc/2026-11-17/fc-strict-trial-source-kit-20261117T160403Z/summary.json)
contains separate source archives for the native controller and software-Start
gateway. It covers the guarded movement work through that date. The November 19
overhead producer, clock adapter and offline pose planner live in the active
checkout. The kit has no Pi binaries, camera environment or field calibration.

| Selection | Motion path | Preparation |
| --- | --- | --- |
| Existing stock release | Twist through mux to native | Select its existing whole release/profile. |
| Guarded legacy candidate | Twist through mux and camera guard | Default guarded assembly; retains native binaries. |
| Direct FC candidate (`fc_deadline_v1`) | Original-deadline command through camera guard to native | Target-built controller and strict 15-role source index; explicitly select the new controller digest. No active mux. |

The [hardware checklist](../foundation/HARDWARE_CHECKLIST.md#guarded-walk-candidate)
owns assembly commands. Assembly copies files and starts nothing. The software-Start
service additionally requires a target-built gateway and its generated service
bindings; controller-only assembly does not add it. The accepted Options activation
path remains available. A source kit, successful build or file copy never selects
the currently running release.

## Build before requesting robot supervision

The observed Pi is **Debian 12 ARM64**, GCC 12.2, glibc 2.36, with
`GLIBCXX_3.4.30` and `CXXABI_1.3.13`. `/opt/ros/jazzy` exists, but its complete
build dependency identity is not frozen. The cached Ubuntu 24.04/Jazzy container
is for offline testing: its binaries and CPython 3.12 bindings are not qualified
for this Pi. Matching architecture and ROS distribution is insufficient.

1. Capture the actual target compiler, Python/SOABI, ROS underlay and dependency
   identities. Use a fresh shell with the intended underlay, not accumulated overlays.
2. Follow the source kit's package-only build recipes in fresh source/build/install
   directories. Keep `/home/pi/pupperv3-monorepo/ros2_ws/install` and every selected
   release unchanged. Missing dependencies are preparation work, not a reason to
   attempt hardware startup.
3. Run the affected behavioral native tests with the new library first in
   `LD_LIBRARY_PATH`; inspect `ldd` on the test executable to catch an older overlay.
   Preserve source hashes, exact commands, compiler/underlay identity, test results
   and installed-file hashes. The recipes distinguish behavioral tests from the
   package's unrelated lint checks.
4. Assemble a new release and keep its completion receipt. Retain the target build
   receipt separately: selecting a library digest does not prove which source built it.
   Complete dependency/import checks before asking the user to place the robot.

`pupper_gait_deploy/install.py` and `deploy.sh` target the mutable platform install;
`deploy.sh --no-launch` still rebuilds and installs. They are not staging checks.
The source kit instead uses their maintained file mappings without executing them.

For a candidate ELF file, the existing local compatibility screen is:

```sh
python3 foundation/scripts/check_pi_abi.py \
  --target foundation/results/build-environment-audit-20261116.json \
  --docker-image cs123-ros-jazzy-slew-check:latest \
  /absolute/candidate/libneural_controller.so /absolute/candidate/estop_controller
```

It reads metadata without executing candidate code. Exit 0 means the recorded
architecture/version requirements passed; 1 is mismatch and 2 is inspection failure.
It does not establish full ROS/dependency compatibility or replace target `ldd`.
Refresh the baseline after target runtime changes. No complete Pi sysroot/offline
builder has yet been verified. [Build audit](../foundation/results/build-environment-audit-20261116.json).

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

For a new owned session, select the exact release/profile and create a new run
directory. Populate the variables from its reviewed receipt; paths/hashes below
are not defaults. Preflight starts no ROS or hardware:

```sh
source "$PUPPER_RELEASE/environment.sh"
export PYTHONDONTWRITEBYTECODE=1
mkdir "$PUPPER_RUN"
python3 "$PUPPER_RELEASE/foundation/hardware/runtime_identity.py" preflight \
  --manifest "$PUPPER_MANIFEST" --sha256 "$PUPPER_MANIFEST_SHA256" \
  --output "$PUPPER_RUN/preflight.json"
```

Only with the supervisor ready and Pupper supported, start the selected launch:

```sh
export PUPPER_RUNTIME_RECEIPT="$PUPPER_RUN/preflight.json"
export PUPPER_IMU_STARTUP_TRACE="$PUPPER_RUN/imu-startup.csv"
ros2 launch "$PUPPER_RELEASE/foundation/hardware/stock_session.launch.py"
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
process identities. Then use one deliberate activation, with the supported robot
and supervisor ready. Strict FC also requires matching loaded contract/clock and
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

Use the stock release `/home/pi/pupper-fc-releases/movement-20261117-logfix`
and its `sprint-manifest.json` for this comparison. Set `PUPPER_RELEASE` to
that directory, `PUPPER_PROFILE=sprint`, and `PUPPER_MANIFEST` to
`$PUPPER_RELEASE/sprint-manifest.json`. The recorded `PUPPER_MANIFEST_SHA256` is
`a4656f22a8c0e5d96bcfec6d3bbbacd6c45e8bc67ea73473a1b139d3756b44c1`.
The profile identifies the native settings used for the prior floor run.
The trial below requests only 0.2 m/s. Its stock path uses no camera or gameplay
bridge. Follow the fresh-run preflight and supported startup above. Preserve the
native and recorder outputs for each session. [Floor evidence](../foundation/results/gait-gate-20261116.json).

On the Mac, generate the nominal 20 cm straight target from the active repository.
The scale of 1 is an initial estimate. Keep the plan and mark the start position,
heading, target and the robot reference point used for tape measurements.

```sh
.venv-api/bin/python final_project/behaviors/tools/pose_trial.py \
  --start 0 0 0 --target 0.2 0 0 --speed 0.2 --distance-scale 1
```

On the Pi, use the same release environment, profile and run directory as the
native terminal. With the robot ready and one deliberate Options activation
confirmed, start its resident trial client once:

```sh
python3 "$PUPPER_RELEASE/final_project/behaviors/tools/movement_trial.py" \
  --session --execute --profile "$PUPPER_PROFILE" \
  --runtime-receipt "$PUPPER_RUN/preflight.json" --out "$PUPPER_RUN/trials"
```

1. Send `{"command":"status"}` and check readiness. In a clear lane, send
   `{"command":"prepare","axis":"forward","speed":0.2,"duration":1}`. When the
   operator is watching and the returned problem is clear, send
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
   endpoint. Only measured endpoints contribute to those distances.

The frozen stock client supports timed axis pulses. It has no IMU-heading
request. Select the newer client and qualify isolated turns before combining
them with translation. See the [movement runbook](behaviors/ROBOT_RUNBOOK.md#direct-deadline-qualification-candidate)
for the guarded candidate, which also requires a real onboard camera producer.

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
