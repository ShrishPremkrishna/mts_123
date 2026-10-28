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

First discover the Pi's actual working checkout, launch entry point and installed
ROS workspace. The user's proposed connection is `pi@mts`; reachability and the
installed paths have not been established. Do not infer them from the laptop's
layout. Stage this checkout separately; do not run its installer or setup script
against the approved workspace while discovering it.

On the robot, from the separately staged checkout, first run:

```bash
bash foundation/scripts/robot_inventory.sh > /tmp/pupper-inventory-before.json
```

The default component paths are labeled discovery candidates, not proof of the
active installation. Repeat with the discovered paths using `--repo-root`,
`--monorepo-dir`, `--gemini-dir`, `--follower-dir` and `--deployment-dir` as needed.
These arguments inspect existing directories; they do not install anything.

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

The inventory also hashes source/installed launch Python files, the installed
neural-controller and hardware-driver libraries, and robot descriptions. It
extracts the declared motor target bounds and homing settings. In ROS mode it reads only the `robot_description` parameter from the
controller manager and robot-state publisher, when available. Compare those named
driver bounds against `configs/class-driver-joint-limits.json`; a model's mechanical
joint range is a different quantity. Unexpanded xacro values, missing parameters,
and source/installed differences remain visible. A description or library hash
alone cannot prove which values a running process actually loaded. For each
running neural controller, read its named `model_path` parameter and compare that
exact file with the inventory; the allowlisted directory scan may miss a policy
loaded from a different directory. Record the actual launch invocation separately
without environment dumps or credentials.

Symlinks, ancestor links, special files and common credential patterns are refused
by the hash reader. A symlink-install workspace may therefore show explicit hash
errors. Inspect each link target, then select its real reviewed file path for
backup; retain the original link-to-target mapping as metadata. Do not silently
follow links, ignore refused entries, or substitute a similarly named source
file for the installed library. The screens reduce accidental credential capture;
they cannot prove arbitrary file contents are free of secrets.

Check the report against the running setup. Missing ROS packages or unavailable
services are evidence gaps, not successful checks. Battery state and physical
condition still need direct observation. Compare joint names/order, positions,
velocities and efforts; inspect receipt rates, timestamp progression and gaps.
The classical controllers deliberately stop after a 0.25-second sensor lapse;
Optional Lab 1 additionally requires valid model windows at its recorded sample
timing. Do not claim a model is ready just because `/joint_states` exists.

## Make the baseline and staged transfer verifiable

Use [robot_artifact_backup.py](scripts/robot_artifact_backup.py) after the inventory
identifies the real files. It accepts only a reviewed list of absolute regular-file
paths, never whole directories or globs. For the baseline, include the actual loaded
policies and descriptors, launch/configuration/calibration files, installed neural
and hardware-driver libraries, relevant locally changed source files, and a
nonsecret record of the Git revision, package versions and exact previous launch
procedure. Keep the original checkout and installed workspace intact. A backup of
selected files does not preserve everything in a checkout or establish a runnable
rollback by itself.

Create a plan with this shape, replacing the example path with a file discovered
on this robot and adding every needed file explicitly:

```json
{
  "schema": "pupper_explicit_artifact_v1",
  "purpose": "robot-baseline",
  "files": [
    {"label": "approved-locomotion", "path": "/actual/discovered/policy.json", "role": "policy"}
  ]
}
```

Supported roles are `policy`, `configuration`, `calibration`, `launch`,
`controller_library`, `driver_library`, `source`, `model_metadata`, `inventory`
and `build_identity`. Keep API keys, secret stores, shell history and private keys
out of the plan and any freeform launch record. Do not archive a whole home folder,
`.env`, raw environment dump or generic application configuration directory.

```bash
python3 foundation/scripts/robot_artifact_backup.py create \
  --plan /absolute/path/reviewed-baseline-plan.json \
  --output /absolute/path/new-baseline-artifact
```

The output directory must be new. Copies are private and nonexecutable; the
manifest records original modes, exact source paths, sizes, per-file capture
times and SHA256 digests. Capture only after relevant file writers are quiet.
Each file is checked for changes and atomic path replacement while being copied;
this is not an atomic snapshot of multiple files or a running application.
A failed capture remains marked `INCOMPLETE` and cannot verify. Source changes
during capture, linked paths and unexpected secret material are explicit failures.
The tool never changes a source file, starts ROS, contacts the network, selects a
policy, installs files or restores them.

Record the printed `manifest_sha256` outside the artifact. Copy the artifact to
a durable location off the Pi, then independently verify the copy using that
recorded value:

```bash
python3 foundation/scripts/robot_artifact_backup.py verify \
  /absolute/path/copied-baseline-artifact \
  --expected-manifest-sha256 THE_RECORDED_64_HEX_DIGEST
```

Verification without the expected digest checks internal file integrity but does
not pin the manifest's identity. Extra, missing, linked or altered files refuse
verification, as do executable/public file modes or nonprivate artifact directories.
Preserve the artifact's private permissions during transfer. Output writes are
anchored to the newly created directory, so a racing symlink cannot redirect them. A matched file hash proves copied bytes, not that the file was
loaded or that the robot can execute it safely.

For a proposed transfer, create a separate explicit plan with purpose
`staged-transfer`. Each file must additionally include `expected_sha256` from its
reviewed artifact or source audit; a missing or mismatched pin refuses capture.
Baseline plans may also pin inventory hashes this way. Capture only the reviewed
revision/artifacts, then verify the recorded manifest digest after transfer. Leave it in a separate staging directory;
creation/verification never installs it. Pair a policy with its descriptor,
walking companion or contact-model metadata as applicable. Compare staged assets
against [the local policy file audit](results/installed-policy-file-audit.json)
and the experiment's reviewed report. Discover and back up the Pi baseline first;
never substitute laptop candidate identities for unknown installed robot hashes.

Before any replacement, write down the exact approved launch command and original
workspace, the baseline files it resolves, and the operator's stop/restart
procedure. Preserve symlink mappings and any required package/build environment.
Rollback means stopping the changed stack and returning to that preserved setup,
then confirming ownership and observed stopping; this helper deliberately has no
automatic restore or motor-control command.

## Establish the current baseline first

1. Use only the already approved controller/policy to verify standing, a brief
   low-demand movement, and stopping, with the operator ready. Record the actual
   response, surface and battery state. Do not replace the baseline first.
2. Verify joint/IMU feedback and the camera stream while stationary. Check image
   orientation and white balance, timestamps, Hailo availability, detector model,
   and an actual detection. The course Hailo8L model is now downloaded and
   [content-pinned](results/lab5-detector-model.json); this does not test accelerator
   compatibility or inference. An isolated ROS Jazzy build has exercised the
   production controller, but the Pi's installed workspace and binaries remain
   unverified; discover its real paths and versions before deploying.
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
5. Before reusing a newly installed Gemini stack, run `robot/startup_readiness.py`
   from its Gemini robot directory in the intended ROS environment. It verifies
   matching policy/configuration paths and loaded **read-only** capabilities:
   reference acknowledgement (`pupper_reference_v1`), policy activation receipts
   (`pupper_policy_v1`), interactive request receipts (`pupper_clip_v1`), and the
   complete position-target-slew contract when a policy requires it. Old binaries
   or writable/copied marker values must fail. Do not place these loaded capability
   markers in launch YAML, and do not restore transient `execution_request_id`
   values from a parameter dump. A successful check establishes software
   compatibility; supervised motion and stopping checks remain separate.
6. During short supervised tool trials, confirm request-correlated playback
   receipts and interruption behavior. A finite or interactive trick must report
   its current activation/request endpoint before a handoff; a published command
   or elapsed timer is insufficient. A playback receipt is not measured standing,
   flight or landing. Missing or inconsistent receipts quarantine further motion;
   inspect the stack rather than bypassing the guard.

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

The branch retains the shipped default gait/jump. The reviewed COM-X Sit export
is a **local simulation foundation asset**, not a Pi-approved replacement; its
CPU +2.5 cm COM case wobbles. Spider training variants and Optional 2 gait/jump
experiments remain separately reviewed research artifacts unless an explicit
reviewed report says otherwise. Their presence in `artifacts/` is not permission
to install them. Verify current file hashes against the local file audit instead
of relying on labels such as “best” or “latest.”

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
