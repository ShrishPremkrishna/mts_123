---
date: 2026-10-27T15:31
tags: [cs123, foundation]
---
# Workstream 1 robot sessions

[Current state](results/interruption-recovery.json) owns deployment, authorization,
active owners and the next unresolved failure. This file owns the reusable
procedure. [Capability evidence](README.md) owns what the robot has actually
shown. Do not restart historical processes or repeat old timed floor choreography.

## Maintained deployment commands

Use `hardware/package_app.py` to package the final reviewed source/build,
`hardware/stage_app.py` to stage its application overlay, and
`hardware/persistent_session.py` for the selected persistent profile. They replace
copying the dated session scripts into another run. The generic Gemini `start.sh`
and gait `deploy.sh` are different paths: both can initialize hardware, and
`deploy.sh --no-launch` still installs/rebuilds. Do not use them as staging checks.

Package locally after tests and the browser build are finished. Use **one** approved
final freeze, not a merge of historical manifests. Its `files` array contains
`{"path": "repo/relative/path", "sha256": "64-hex-digest"}` entries for every selected
runtime source, browser build input and complete current `web/dist` file; an
optional `source_checkout_head` must match the expected commit. The freeze may
also pin reviewed tests. Unchanged skill assets and native identities are inherited
from the baseline manifest, never replaced from the current checkout.

```sh
python3 /Users/shrish/workspace/cs123-foundation/foundation/hardware/package_app.py \
  --repo /Users/shrish/workspace/cs123-foundation \
  --baseline-manifest "$BASELINE_MANIFEST" --baseline-manifest-sha256 "$BASELINE_SHA256" \
  --freeze-manifest "$FINAL_FREEZE" --freeze-manifest-sha256 "$FREEZE_SHA256" \
  --expected-head "$REVIEWED_COMMIT" --client-bundle "$BUILT_INDEX_JS" \
  --output "$NEW_LOCAL_PACKAGE" --destination "$NEW_REMOTE_RELEASE" \
  --add-file "$NEW_RUNTIME_FILE"
```

These variables are explicit reviewed paths/hashes; `BUILT_INDEX_JS` is the actual
`index-*.js` from the completed build. Repeat `--add-file` only for newly selected
runtime files, or omit it if there are none. Additions must be pinned in the same
freeze. The local output must be a new directory outside the source checkout and
preserved baseline. The command creates the manifest, verified archive and final
`packaging-receipt.json`; it does not build, deploy, start processes or generate
launch recipes. A missing completion receipt means inspect the partial output,
not overwrite/retry. Stage only the resulting verified manifest/archive below.

`stage_app.py --manifest MANIFEST --manifest-sha256 SHA256 --archive ARCHIVE`
verifies the complete payload and selected baseline before copying into the
manifest's **new** destination. It never installs, builds, starts processes or
calls the robot API. The old release remains the rollback. The allowed overlay
contains Gemini, the named persistent helpers/launch, and Lab 5 follower code and
perception assets, plus the byte-identical pinned baseline URDF; native libraries, policy weights and baseline gains/config
cannot be replaced by this command. Keep the resulting `app-stage-manifest.json`,
`APP-SELECTED-SHA256SUMS` and `app-stage-receipt.json` with that release.

Source the already selected ROS environment; use the existing API Python. Set
`ROS_DOMAIN_ID=168`, explicit `PUPPER_SESSION_SKILLS` (empty selects baseline),
`PUPPER_SESSION_MOTION_MODE=walking` and `PUPPER_FOLLOWER_MODE=disabled` or `lab5`.
Lab 5 requires `FOLLOW_ME_DIR` pointing to the selected follower checkout or its
`robot` directory, including the sibling `lab` modules, calibration and HEF.
`class_run` is an explicit alternate motion mode; it is not selected by default.
Do not substitute the current checkout's native build for the selected binaries:
current source can contain driver changes that were never deployed.

Prepare a new run without launching anything:

```sh
"$SESSION_PYTHON" "$APP_RELEASE/foundation/hardware/persistent_session.py" prepare \
  --run-dir "$RUN_DIR" \
  --app-manifest "$APP_RELEASE/app-stage-manifest.json" \
  --app-manifest-sha256 "$STAGE_MANIFEST_SHA256" \
  --native-library "$NATIVE_LIBRARY" --native-sha256 "$NATIVE_SHA256" \
  --imu-library "$IMU_LIBRARY" --imu-sha256 "$IMU_SHA256" \
  --telemetry-topics "$TOPICS_JSON"
"$SESSION_PYTHON" "$APP_RELEASE/foundation/hardware/persistent_session.py" status --run-dir "$RUN_DIR"
```

The variables above are explicit selected paths/hashes, not defaults to guess.
`TOPICS_JSON` is a JSON array of absolute ROS topics including `/joint_states` and
`/llm_cmd_vel`; include selected controllers' `/clip` and status topics. Preparation
uses the existing profile generator, pins its complete assets/configuration and
records app rollback, exact native/IMU identities and the Python executable path/virtualenv prefix. It requires the config/URDF beside the selected launch and a new run
folder and leaves failed preparation evidence for inspection.

With the operator ready on the stand, the separate `start-native --run-dir RUN`
command starts one owned camera, one MCAP recorder and the selected launch. It
waits up to 90 seconds for fresh homing completion, captures manager ownership,
and checks that the selected libraries are mapped. A successful return is not
permission to release motion. Failure preserves the receipt; there is no retry.
The helper uses the exercised camera settings; an optional existing MJPEG/external
camera recorder remains separately owned and must also be closed by its owner.

`start-api --run-dir RUN` starts only the API after native readiness and checks
that its own listener serves healthy ROBOT mode. `stop-api --run-dir RUN` requests
API Stop and waits for the owned API to exit; it leaves the homed native session
intact. Without an override, the next `start-api` uses the original pinned release.

For a reviewed app-only update, first disconnect voice, stage the new release,
and use the reviewed helper from that release against the **existing** run.
Run these separately; continue only after Stop and API exit are confirmed:

```sh
"$SESSION_PYTHON" "$APP_UPDATE/foundation/hardware/persistent_session.py" stop-api --run-dir "$RUN_DIR"
"$SESSION_PYTHON" "$APP_UPDATE/foundation/hardware/persistent_session.py" start-api \
  --run-dir "$RUN_DIR" \
  --app-manifest "$APP_UPDATE/app-stage-manifest.json" \
  --app-manifest-sha256 "$UPDATE_MANIFEST_SHA256"
```

This records a separate API attachment and preserves the original native session,
process ownership, follower, controller configuration, policies and gains. Those
contracts must match; changing them still requires new profile preparation.
No `prepare` or `start-native` belongs in this app-only sequence. Reconnect voice
explicitly after checking the new API identity. To roll back after a confirmed API
exit, run `start-api` without the override. An unconfirmed spawn requires receipt
and process-identity recovery first; do not guess a PID or retry automatically.
The attachment path has passed offline tests and independent review; its first
live upgrade/rollback acceptance is still pending. A closed native session cannot
be recovered by this command.

`shutdown --run-dir RUN` closes owned ingress, uses the existing coordinator to
stop the manager before launch cleanup, then closes owned camera/recorder children.
It never name-kills unrelated processes or uses SIGKILL. Inspect recording closure
and driver logs separately: verified process exit is not verified physical motor
release. App Stop, API process exit and full native shutdown remain distinct.

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
