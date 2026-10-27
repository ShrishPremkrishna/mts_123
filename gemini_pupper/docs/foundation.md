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

A timed-out or cancelled controller activation may still execute remotely. The
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
that following stopped refuses new motion; the stop path also tries to relax the
controllers and reports whether that request succeeded. A physical e-stop remains
necessary if ROS or the hardware stops responding. On a tool-server restart,
the server first turns off any follower left running by its predecessor. It
refuses motion until shutdown is confirmed; use stop to retry after service repair.

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

The completed `sit_stay_shake` descriptor is generated when its real policy is
fetched. Its actions are `shake_left`, `shake_right`, `stand_up`; `stand_up` is the
exit. No placeholder policy is shipped. `spider_walk` requires both the clip policy
and its walking policy, plus `--walk-limits <file.json>` describing evaluated limits.
That file contains `evaluation_source`, `speed`, `max_forward`, `max_sideways`
(metres/second), and `turn` (degrees/second). Do not copy training command caps into
it as if they were achieved performance. The loader also enforces the narrower
side of asymmetric training ranges, since the tools use symmetric limits.

Sit/Stay/Shake and Spider training, simulation scoring and TA review remain
necessary before the first supervised hardware run. UniMate's generated secret
motion also needs generation, cleanup, tracking training and evaluation. Existing
recorded policy tricks and the course jump are available in software; that is not
evidence of validation on this particular robot.

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
