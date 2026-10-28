---
date: 2026-10-27T15:31
tags: [cs123, foundation]
---
# First robot session: establish a trustworthy baseline

This checklist prepares the engineering branch for the final project. Offline
tests establish software behavior and data contracts; they do not establish that
this particular Pupper can execute a motion reliably. Newly trained policies need
the course TA's approval before their first hardware run.

## Before changing the robot

- Have someone physically beside Pupper explicitly confirm it is ready for the
  specific test. Know the gamepad emergency-stop control, keep it reachable, and
  verify it with the already approved baseline under the course setup procedure.
  A voice stop, web stop, SSH session, or healthy network is not a substitute.
  A successful ROS controller-deactivation response is evidence about software
  state, not a physical guarantee of zero motor torque; confirm the observed stop.
- Record which Pupper this is, current battery indication, mechanical condition,
  foot/leg calibration, test surface, existing launch method, and the last known
  working behavior. Establish safe space and the course's support arrangement for
  first motor tests. Stop on unexpected motion, instability, overheating, lost
  telemetry, or uncertain controller state; inspect the cause before continuing.
- Preserve the current checkout, uncommitted changes, installed configuration,
  calibration, and actual policy files before replacing anything. Capture hashes
  and the current controller list. A Git commit alone does not preserve ignored
  models, installed ROS files, calibration, or local changes.
- Keep the working baseline available in its original directory. Deploy this
  engineering branch separately. Identify the exact previous launch/configuration
  and policy paths to restore; do not rely on a filename such as `latest`.

## Collect the read-only inventory

On the robot, from this checkout, first run:

```bash
bash foundation/scripts/robot_inventory.sh > /tmp/pupper-inventory-before.json
```

After sourcing the robot's existing approved ROS environment, observe the already
running graph without launching or changing controllers:

```bash
bash foundation/scripts/robot_inventory.sh --ros > /tmp/pupper-inventory-ros-before.json
```

The script prints JSON to standard output; these redirections create the report
files. It reads versions, repository revisions/changes, installed paths, policy
and configuration hashes, CPU temperature/throttling, device presence, controller
states, hardware interfaces, and short telemetry samples. It records camera frame
stamps and encoded sizes, not photographs. It reads no credentials, environment
dump, shell history, API keys, or remote repository URLs. It never launches ROS,
publishes commands, switches controllers, sets parameters, or downloads anything.

The inventory also hashes the source/installed robot descriptions and installed
hardware-driver library, and extracts the declared motor target bounds and homing
settings. In ROS mode it reads only the `robot_description` parameter from the
controller manager and robot-state publisher, when available. Compare those named
driver bounds against `configs/class-driver-joint-limits.json`; a model's mechanical
joint range is a different quantity. Unexpanded xacro values, missing parameters,
and source/installed differences remain visible. A description or library hash
alone cannot prove which values a running process actually loaded.

Check the report against the running setup. Missing ROS packages or unavailable
services are evidence gaps, not successful checks. Battery state and physical
condition still need direct observation. Compare joint names/order, positions,
velocities and efforts; inspect receipt rates, timestamp progression and gaps.
The classical controllers deliberately stop after a 0.25-second sensor lapse;
Optional Lab 1 additionally requires valid model windows at its recorded sample
timing. Do not claim a model is ready just because `/joint_states` exists.

## Establish the current baseline first

1. Use only the already approved controller/policy to verify standing, a brief
   low-demand movement, and stopping, with the operator ready. Record the actual
   response, surface and battery state. Do not replace the baseline first.
2. Verify joint/IMU feedback and the camera stream while stationary. Check image
   orientation and white balance, timestamps, Hailo availability, detector model,
   and an actual detection. The laptop checkout currently lacks the Hailo model
   and installed ROS workspace; discover the robot's real paths rather than
   copying that absence onto the robot.
3. Compare the baseline's policy/configuration hashes with the intended branch.
   Confirm the controller that actually owns the motor command interfaces.
   Classical PD/IK nodes command `forward_command_controller`; the neural stack
   uses separate locomotion/trick controllers. They are separate operating modes.
   Never run competing motion stacks or assume an existing `ros2_control_node`
   means the Gemini mux and its controllers are installed and active.
4. Run the Gemini file preflight before its first launch, then inspect the real
   controller/service/topic state. Confirm joystick priority through the mux,
   follower stop behavior, the physical e-stop, and a software stop separately.
   A new motion must stop following before it starts. A failed controller switch
   or unknown response must not authorize the next queued motion.

## Enable capabilities one at a time

| Capability | First evidence needed | Gate/default |
|---|---|---|
| Lab 1 bounded PD and sampled delay | Correct joint, measured start pose, torque feedback, target bounds, stop behavior | Single-joint supported course setup; delay begins at zero |
| Lab 2 FK | Measured joint order/signs match predicted foot position | Read-only geometry check before changing motion |
| Lab 3 single-leg IK / heuristic gait | Bounded reachable cache, correct controller, sensor-loss shutdown | Separate classical stack; changed trajectories require fresh validation |
| Lab 4 learned gait | Exact checkpoint/config, simulation evaluation, prior hardware evidence | Preserve current approved gait; compare candidates before replacement |
| Lab 5 person following | Camera/detector, target lock, stopping distance, lost-target/frame behavior | Explicit Lab 5 follower; no silent simpler-follower fallback |
| Clothing-color recovery | Lighting and clothing samples, target crossing/occlusion trials | `FOLLOW_USE_COLOR=1` only after separate validation |
| Timed obstacle detours | Course obstacle setup and observed stop behavior | `FOLLOW_AVOID_OBSTACLES=1` only after separate validation; not general collision avoidance |
| Lab 6 motion tools / queue | Finite limits, actual execution/outcomes, motion transitions, follow interruption, stop | Test short single actions before sequences |
| Existing policy tricks / course jump | Current robot compatibility and the jump table/gait contract | Software presence is not robot validation |
| Sit/Stay/Shake | Real exported policy, clip transitions, completion feedback and simulation results | Training/evaluation plus TA approval before first hardware use |
| Spider stance and slow walking | Both trained policies, entry/exit transitions, evaluation-derived speed limits | Training/evaluation plus TA approval; no following/running in this stance |
| Optional 1 learned contact pause and explicit vision baseline | Exported model/normalizer/schema, held-out-recording evaluation, correct live feature timing; separately assess visual material/incline classification | Opt in with `--contact-model`; verify shadow predictions before contact can pause motion; classifiers do not automatically control locomotion |
| Optional 2 faster reference gait and bootstrapping | Actual baseline/candidate checkpoints, captured reference manifests, multi-seed mixed-gait and bumpy evaluations, speed/torque/slip tradeoffs | Promote only after comparison; no claimed real speed from reference feasibility |
| Optional 2 Mystery jump | A real successful simulated flight/landing trace, matching scheduler/export, landing-load and recovery inspection | Simulation experiment first; TA approval before first new-policy hardware run |
| Optional 3 character/hidden tricks | Real Gemini session, deterministic unlock/lock behavior and real motion artifact | Password outside repo; identity claims never unlock; new motion still needs training/TA gate |

For Gemini, use explicit component paths where needed. The default follower mode
is `lab5`; `disabled` is an intentional mode for tests without following. Color
recovery, obstacle detours and always-on detection all default off. The latter
changes heat/load, so observe throttling and responsiveness if enabled. See
[Gemini operation](../gemini_pupper/docs/foundation.md) for exact environment flags
and policy-installation contracts. Optional capabilities may need data/model
artifacts that code implementation alone cannot supply.

For Optional 1, transfer the exported `contact.pt` and its matching `metadata.json`
as one versioned artifact; verify both checksums after transfer. The metadata owns
channel order, resampling rate, windowing and normalization; verify live units and
effort semantics against the [interface audit](../implicit_compliance/results/2026-10-27/deployment-interface-audit.json). First record a
shadow-mode stream: inference may log predictions but must not change the motion.
Use `python3 -m implicit_compliance.shadow --model <export-directory> --out <new-trial-directory> --seconds 60`; it subscribes only and never enables the trajectory pause.
Check actual timestamps and receipt latency, available effort fields, active PD
gains, homing offsets, CPU/latency, false triggers and missed/delayed contact on
the intended single-leg trajectory. The local driver publishes a PD-based effort
estimate, not measured torque; the historical CSV driver/gains remain unverified.
DenseTact tactile-camera material/incline accuracy is a separate result from this live contact
test. Enable the one-second pause only after the predictions and thresholds are
checked on this robot. Preserve the model and threshold version with the trial.

Include a controlled detection-stall check while ready to stop: loss of frames
must cancel blind detours and cease follower velocity commands. A detector process
that is alive but stalled is still a loss of evidence. Verify the physical result
as well as `/cmd_vel`; a zero command does not prove the motors stopped.

## Capture results and keep rollback concrete

For each change record the revision and policy hashes, active controllers,
enabled flags, battery/surface, requested action, actual result, stop result and
relevant telemetry. A video of a supervised trial and its matching timestamps
help distinguish requested speed from achieved speed. Record failures as well as
successful trials. Avoid recording bystanders or credentials unnecessarily.

If a test fails, stop and return to the preserved approved setup using its exact
configuration and policy files. Reconfirm controller ownership and stop behavior
before another movement. Gemini quarantines uncertain controller activation;
inspect/restart the controller stack before restarting its tool server. Do not
clear a fault by repeatedly issuing motion requests. Preserve the failed run's
logs and inventory for diagnosis; do not overwrite the baseline artifacts.

The first session can complete inventory, backup, the existing approved baseline,
camera/detector checks and supervised integration checks without any new trained
policy. Newly trained motions and learned optional models remain separate gates.
The minimum user inputs are the robot identity/access path, someone beside it,
the working baseline/start procedure, and any existing TA-approved checkpoint or
model artifacts. API credentials stay in the existing secret store, never in the
inventory or this repository.
