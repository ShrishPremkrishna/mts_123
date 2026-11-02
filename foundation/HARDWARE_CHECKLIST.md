---
date: 2026-10-27T15:31
tags: [cs123, foundation]
---
# Workstream 1 robot sessions

[Current state](results/interruption-recovery.json) owns deployment, authorization,
active owners and the next unresolved failure. This file owns the reusable
procedure. [Capability evidence](README.md) owns what the robot has actually
shown. Do not restart historical processes or repeat old timed floor choreography.

## Prepare without consuming supervision time

- Freeze the selected source, models, configurations and scenario list. Finish
  affected tests and package installation before the operator arrives. Reuse the
  existing Python environments and persistent `lab6_session.launch.py`.
- Select only the needed skills with `PUPPER_SESSION_SKILLS`; explicit empty
  selects stock locomotion and unset refuses launch. Launch and API use the same
  selection and `PUPPER_GEMINI_DIR`. Keep original gains, joint limits and caps.
- Keep one robot owner. Record exact processes so shutdown stops only this session.
  Browser/API development uses the explicit fake server with voice disconnected
  unless a cloud test is intended. Confirm the visible FAKE/ROBOT destination.
- Define a small batch and its decision before testing: the baseline, proposed
  change, measured outcome and reason to stop or revert. Prepare successful and
  failure/interruption cases offline. Three clean repetitions are an initial
  acceptance screen, not general reliability.

## One capture for each persistent session

Reuse ROS MCAP, API/native logs and the camera recorder. Keep their original files
and frozen input manifest together. A reusable offline report should extract these
inputs; do not add a new recorder for each hypothesis.

Record each request, server acceptance, queue start/finish/cancellation, native
controller/clip events, commanded velocities, measured joints and IMU. Server
job IDs correlate API work; native receipt IDs identify controller playback.
Neither proves physical success. Use monotonic time for durations within one
process, wall time to align sources, and record timezone/clock assumptions.

Use a fixed external camera view to see feet, body, slips and transitions. Pupper's
camera provides a separate view of the environment and can corroborate large
heading changes; it cannot see all its own contacts or directly measure metres
of travel. A ruler/floor marks and visible timing marker make distance and video
alignment measurable. Preserve missing views and uncertain timing as missing.

Separate speech input/release, model/tool arrival, server acceptance, queue wait,
controller execution, audio playback and observed physical response. Browser
callbacks alone do not establish acoustic Stop latency. Display estimates as
estimates; report actual measured intervals separately. Reported IMU angles are
sensor output, not external truth or calibrated translation.

## Resolve a known sensor failure before tuning motions

If the current record identifies unreliable feedback, the next block begins
supported with motors inactive. Use the reviewed standalone
[IMU diagnostic](hardware/imu_diagnostic/README.md) separately from the native
controller. Inspect current identity/configuration first. Its comparison mode
requires explicit additional reports and verified restoration; it is not a
calibration fix or a readiness certificate. No reset/homing is needed for this
sensor-only block. Do not compensate for bad sensing by changing gains/rewards.

## Use supervised time for prepared physical comparisons

1. Confirm current placement, battery, physical stop readiness and exclusive
   ownership. Repeat controller discovery or button diagnostics only if connection
   state changed or evidence requires it. Keep direct Stop/R3 independent of voice.
2. When a fresh motion session is justified, home once on the stand. Place on the
   floor while inactive; Options releases the gateway when the operator is ready.
   Keep the same homed session between ordinary trials. API Stop cancels work and
   requests zero velocity; R3 latches the abort; full driver shutdown disables the
   boards. These are different operations.
3. Establish undisturbed stock standing and Sit/hold/standing behavior with the
   recording. If the baseline is inconsistent, diagnose that layer before adding
   voice or more skills. Avoid running more motions merely to fill a checklist.
4. Run prepared action chains: Sit/both paws/stand/walk, selected finite tricks and
   returns, then an interrupted queued sequence. Compare each transition as well
   as each isolated skill. Capture drift, tip/slip, duration and repeatability.
5. Use the same accepted chains through voice. Include ordinary conversation,
   camera understanding, one acceptance acknowledgment, refusals, cancellation
   and reconnect behavior. Separate interpretation failure from execution failure.
6. Integrate the tested follower and Spider pair from their owners. Generated
   motion and push-up changes need demonstrated improvement before promotion.
   Do not infer arbitrary self-righting from a normal standing command.

The operator watches the whole prepared batch; request one concise assessment at
its end unless something fails. Use recordings to review missed details instead
of immediately consuming another battery on the same trial. For substantial code
work, release the robot and finish offline. New/changed motions require simulation
and the applicable current user/course authorization.

## Finish and resume reliably

Stop owned software, verify exits/driver disable, and preserve complete outputs
with exact input identities. Verify archive members as well as the archive hash.
Keep failures and rejected candidates. Update current state with the next decision,
active processes (if any) and missing evidence. Do not turn passing software tests
or a successful demonstration into an accepted physical capability.
