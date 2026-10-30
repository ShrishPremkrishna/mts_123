---
date: 2026-10-27T15:31
tags: [cs123, foundation]
---
# Foundation operation

This branch is the engineering implementation, separate from the student's Lab 6
submission. Original creation credit remains with Peng Xu and the CS123 staff.

## What is implemented

The tools validate motion limits, walk/turn/arc, stop following before a new
movement, play policy or reference tricks, and sequence interactive clips. The
queue distinguishes acceptance from completion. `robot_status` and `/queue/status`
include the latest outcome. A failed job cancels dependent queued work; a stop
preempts request acceptance. Failed exit transitions and missing clip completion
feedback are reported rather than treated as successful movement.

Reference playback carries a unique request ID; finite policies carry an ID
frozen at activation; interactive clips additionally carry a per-request ID
through their queue and automatic successor clips. Completion requires matching,
fresh controller feedback for the expected final frame or loop. Receipts are
published after successful inference and command writes, and never override a
stop, fall or unresolved controller fault. A restarted server can leave an
already-active trick only after its own new clip request receives a matching
receipt. These messages confirm controller playback, not measured posture or
physical landing. See the [review and exact protocols](../../foundation/results/controller-completion-v2-reviewed.json).

The combined stack requires the rebuilt controller with read-only reference,
policy and interactive capability descriptors. Copying marker values into YAML
is refused. Current tools accept the earlier correlated finite receipt format,
but interactive requests require the new protocol. The controller remembers the
last 64 interactive request IDs per activation for duplicate refusal; the tool
server uses a new UUID for every request and does not retry motion automatically.

A timed-out or cancelled controller or follower activation may still execute remotely. The
server keeps its future, attempts deactivation when the late response arrives,
and latches a motion quarantine. Stop reports the actual cleanup result without
silently rearming. Use the physical e-stop and inspect/restart the controller
stack before restarting the tool server; cancelling a client request cannot stop
a request already sent to ROS.

The proposed Pupper character is in `lab/prompt.md`. Its greeting/handshake,
handshake joke and conversation-local memory are in-context behavior, not learned
weights or persistent memory. Voice behavior and visual interpretation still need
a live Gemini session; the offline tests do not demonstrate them.

## Offline checks

From `gemini_pupper/`, using the workspace virtual environment:

```bash
../.venv/bin/python -B lab/test.py
../.venv/bin/python -B lab/test.py optional_jump
../.venv/bin/python -B -m pytest -q tests/test_foundation.py -p no:cacheprovider
../.venv/bin/python -B robot/preflight.py
```

The preflight reads files and validates policy data contracts; it never starts
ROS or moves anything. On a laptop it should report the missing installed ROS
workspace and may report the missing Hailo model. `robot/start.sh` runs this check
before launching anything. A successful preflight is not a hardware readiness
certificate: it cannot establish physical condition, camera calibration, services,
emergency-stop operation, or policy performance.

When ROS is already running, `start.sh` first inspects it without changing it.
Reuse requires this branch's combined-launch manifest, unchanged policy and
descriptor files, configured controllers, feedback broadcasters, the mux with
joystick priority, the selected follower services/flags, and matching controller
parameters. A course-only or stale stack is refused before any server is replaced.
With the operator beside the robot, preserve the approved baseline and use the
course stop/setup procedure before explicitly launching the combined stack; never
start a second competing motor stack. When no manager is running, `start.sh`
starts hardware as part of the launch. The driver's `on_activate()` performs
homing even if every neural policy is spawned inactive, so an operator must be
present before this launch. The same inspection runs before the server starts.
Every resolved native JSON policy, including the installed locomotion policy,
is validated before the combined launch executes. The manifest proves file and
configuration consistency, not loaded binary compatibility or physical readiness.

The combined deployment also requires the rebuilt `joy_utils` package. The
installer stages its source, arbitration header, CMake, package manifest and
constructor/core/ROS tests from the bundled monorepo; missing inputs cause
refusal before installation writes.
Readiness requires exactly one `/estop_controller` with the generated ordered
`controller_names`, `switch_button_indices`, `estop_index` and
`estop_release_index`, all with read-only parameter descriptors. Startup overrides
remain supported; changing exposed parameters on an old node cannot repair its
cached mapping. See the [immutable configuration review](../../foundation/results/joystick-immutable-parameters-reviewed.json).

The four face-button mappings remain X → default, circle → three-legged,
triangle → original walk, square → student. `default_alt` and every Gemini
policy/walking controller are appended only for deactivation coverage. The
configured stop is right-stick press (index 12); Start (index 9) activates the
last face-button-selected controller, initially the default. It does not restore
the last Gemini-selected trick or merely clear a passive stop flag.

The rebuilt joystick node is now a shared, latched software stop authority. It
starts latched. Stop wins a whole joystick frame and cancels queued activations;
holding Stop blocks face/Start activation. After acknowledged deactivation, let
go and press Start freshly to activate the last face-selected controller. An
early Start press is consumed, not saved for automatic rearming. A timeout,
ambiguous submission or failed motion switch remains faulted until the stack is
inspected/restarted; Start does not clear a fault.

All ordinary controller switches use the public gateway. The launch remaps only
the controller-manager node to its backend service, so controller clients in the
same process still use the gateway. Fresh controller inventory precedes each
switch. Only the exact joint-state and IMU broadcaster types may activate while
latched. Direct backend access is unsupported; this is not a ROS security boundary.
STRICT/BEST_EFFORT is retained, with a one-second backend budget and a 1.5-second
local queue/response budget. Unknown remote outcomes are retained rather than
retried as if no request had been sent. A known late activation is followed by a
fresh deactivation request; software acknowledgement is not physical stopping.

Readiness requires the immutable `pupper_stop_v1` marker, a fresh matching runtime
instance, the gateway service ownership and the exact broadcaster types. The
Gemini server blocks motion on missing/stale status (over 0.5 seconds), a latch,
a fault or a changed authority instance. It also inhibits immediately on the
emergency topic; repeated stop messages do not enqueue repeated switches. A
healthy startup latch is compatible with structural readiness, but still blocks
motion until the explicit physical release. Gateway exit requests whole-stack
shutdown; it is not an independent physical emergency stop and need not interrupt
hardware homing immediately.

Neither an empty controller inventory nor a latched status means motor power or
holding torque is off. The original driver retains homing targets even without
an active motion controller. Use the operator's physical power stop when needed.
The [isolated ROS integration evidence](../../artifacts/hardware/2026-10-29/stop-gateway-integration/reviewed-v1/review.json)
covers arbitration and transport failure cases, not robot motion or a measured
stopping guarantee. The earlier
[independent joystick review](../../artifacts/hardware/2026-10-29/joystick-independent-review/review.json)
remains the historical record of the replaced non-latching implementation.

## One explicit follower

`PUPPER_FOLLOWER_MODE=lab5` is the default. The service and Hailo model must both
exist or startup fails with an actionable error. There is no silent fallback to
the simpler follower. Paths resolve to sibling directories in the combined repo,
then the standalone directories in the robot's home folder. Explicit overrides:

| Variable | Meaning |
|---|---|
| `FOLLOW_ME_DIR` | Path to the Lab 5 repository, containing `robot/follow_service.py` and `robot/yolov8n_seg.hef` |
| `PUPPER_MONOREPO_DIR` | Monorepo with the built ROS workspace |
| `PUPPER_GAIT_DEPLOY_DIR` | Deployment source, including the policy validator |
| `FOLLOW_USE_COLOR=1` | Enable the optional clothing-color target recovery; default off |
| `FOLLOW_AVOID_OBSTACLES=1` | Enable optional timed obstacle detours; default off |
| `FOLLOW_ALWAYS_DETECT=1` | Detect between follow requests for the eyes; default off |

Use `PUPPER_FOLLOWER_MODE=disabled` intentionally when testing without following.
`legacy` explicitly selects the monorepo follower and logs its missing Lab 5 target
lock/stopping-distance features. It cannot enable the Lab 5 options. Color matching
and timed detours are experiments, not general person identity or collision-free
navigation. Validate their operating conditions before turning them on.

Following cannot start in a walking trick or after a fall. A failure to confirm
that following stopped refuses new motion; an explicit stop also requests
controller deactivation through the gateway and reports whether it succeeded. A physical e-stop remains
necessary if ROS or the hardware stops responding. On a tool-server restart,
the server first turns off any follower left running by its predecessor. It
refuses motion until shutdown is confirmed; use stop to retry after service repair.

The stop reply distinguishes cancelled commands from measured stopping. Sending
zero walking speed leaves the locomotion controller running; selected simulated
mass/inertia cases keep moving despite that command. Deactivating a controller
confirms software state, not motor torque, physical stillness or a lying pose.
The tool and prompt must not turn either acknowledgement into a physical claim.
See the [default-policy diagnostic](../../foundation/results/spider-default-inertial-ablation-reviewed.json).

## Policies and training

The current locomotion policy is the course default. Launch checks its trained
command ranges against the tools and verifies that any reference trick (the jump)
matches its trained jump table and timings. A new gait is not a drop-in replacement
unless these contracts still hold. Compare policies before choosing one; running
caps describe requested speeds, not measured achieved speeds.

`get_trick.py` requires the deployment validator. It stages downloads, validates
the complete trick, and preserves an older installation under `tricks/.backups/`
before replacing it. `artifact_manifest.json` records source runs and file hashes.
No controller is activated by fetching a policy.

Local Colab/TensorBoard exports can be installed without a W&B account:

```bash
python3 robot/get_trick.py --policy-file /path/to/sit/policy.json --name sit_stay_shake
python3 robot/get_trick.py --policy-file /path/to/spider-clips/policy.json --walk-policy-file /path/to/spider-walk/policy.json --walk-limits /path/to/evaluated-limits.json --name spider_walk
```

Local and W&B sources use the same staging, validation, backup and descriptor
checks. Local source paths and hashes are recorded. Keep the paired exported
policies, limits and evaluation together; installation does not establish that
the policy earned a hardware trial. Mixed local/W&B source arguments are refused.

The completed `sit_stay_shake` descriptor is generated when its real policy is
fetched. Its actions are `shake_left`, `shake_right`, `stand_up`; `stand_up` is the
exit. No placeholder policy is shipped. `spider_walk` requires both the clip policy
and its walking policy, plus `--walk-limits <file.json>` describing evaluated limits.
That file contains `evaluation_source`, `walk_policy_sha256`, `speed`, `max_forward`, `max_sideways`
(metres/second), and `turn` (degrees/second). Do not copy training command caps into
it as if they were achieved performance. Every walking-policy import requires
limits bound to that exact export's SHA-256; the reviewed JSON is preserved as
`walk_limits.json` alongside the policies and hashed in the installation manifest.
A matching hash identifies the reviewed model; it does not certify the review's
quality or hardware approval. The loader also enforces the narrower
side of asymmetric training ranges, since the tools use symmetric limits.

Before accepting a walking pair, the loader compares its entry endpoint, every
idle-loop frame and exit-start joint pose with the walking policy's default
stance. The 1e-6 rad tolerance covers six-decimal motion export and float32
encoding; it is not a permitted physical mismatch. Both policies must use the
fixed controller joint order of contract v2 and matching joint-limit arrays.
The walking reference must name all 12 joints in that order. Earlier v2 trick
exports omit joint names and retain the contract's fixed order; any supplied
names must agree. This catches an accidental upright-policy/Spider pairing but
does not validate physical controller handoff or tracking quality.

The installed Sit policy passed the eight existing scenarios on both CPU and
GPU with class driver limits after its torso x-COM training range was expanded
to ±2 cm. The +2 cm case improved; +2.5 cm remains outside that range and shows
a CPU/GPU disagreement. See the [review](../../foundation/results/lab6-sit-comx02-reviewed.json)
for the exact policy and boundary. Spider refinements remain separate experiments. The generated `secret_dance` artifact has completed
generation, repair, training and review; its actual movement is a gentle standing
rise/sway/return. Its seven-scenario screen and native finite-clip activation/hold
are documented in the [review](../../foundation/results/generated-motion-trained-review.json).
These policies still require TA review before a first supervised hardware run.
Existing recorded policy tricks and the course jump are available in software;
that is not evidence of validation on this particular robot.

## Optional Lab 3: hidden tricks

Set `secret: true` in a real trained trick's `trick.yaml`, or pass `--secret` when
fetching it. The robot owner creates a UTF-8 password file outside the repo at
`~/.config/gemini-pupper/secret_password`, or supplies its path with
`PUPPER_SECRET_FILE`. There is no shipped password and no password in the prompt.

The `unlock` tool compares the supplied phrase in code. It does not move the robot.
Locked tricks and actions are absent from the lists the model sees. A successful
unlock supplies their descriptions; calls are checked again when queued work
executes. `lock_tricks`, `lie_down`, `stop`, a fall or a server restart revoke the
unlock. Locking alone does not jerk the robot out of a pose: `stand_up` remains
available to leave safely, and `stop` can halt movement.

Unlock arguments are redacted from tool debug logs in the server, terminal chat,
and browser interface. A spoken password still enters the Gemini conversation;
this is a demonstration of deterministic gating, not identity authentication or
a secret suitable for reuse elsewhere. Prompt instructions, reported identity,
and camera appearance do not grant the unlock.

## Physical validation

Preserve the current robot configuration and policy first. Verify the physical
emergency stop and known baseline, then camera/follower behavior, then one new
component at a time. Use the existing course motion reports and simulation replay
before changed motions. Newly trained policies require the course's TA approval
before first hardware use. No hardware validation or API session is implied by
the test output in this branch.
