---
date: 2026-10-27T15:31
tags: [cs123, foundation]
---
# Pupper session commands

This reference describes the existing release and persistent-session helpers. The
[checkpoint](results/interruption-recovery.json) stores dated release observations.
Establish current robot ownership, boot and running processes before a physical
session. The [evidence index](README.md) links prior observed behavior.

Motor startup and motion require a supervising operator with the robot supported
as appropriate and access to R3/power. Source installation, app activation,
controller Stop and full motor-driver shutdown have different effects below.

For the stock-only Pupper FC movement session, use the
[build/test handoff](../final_project/ROBOT_BUILD_AND_TEST_HANDOFF.md) and
[movement runbook](../final_project/behaviors/ROBOT_RUNBOOK.md). That separate,
pinned launch does not start the Lab 6 API, voice or camera. The application
packaging/session commands below remain the Lab 6 path; they do not select or
build the new stock movement release.

## Guarded walk candidate

`hardware/assemble_guarded_release.py` copies a verified stock walk baseline into
a **new** release directory and overlays the indexed guarded-command and gameplay
bridge Python sources. By default it keeps
the native controller, driver, gateway, mux, policy and gains unchanged. It adds
the optional camera command path, preserves the old release for rollback, and
removes copied sprint selection. This is file assembly only; it starts nothing.

The separate direct FC deadline candidate additionally needs a target-built
controller. Select it explicitly with `--deadline-controller` and
`--deadline-controller-sha256`, together with the strict source index (including
the deadline helper and updated stock-evidence module). Assembly pins the new
controller, selects `fc_deadline_v1`, disables alternate command inputs and
omits mux launch/continuity dependencies. Policy, gains, driver and gateway stay
unchanged. Its record reports `native_unchanged=false`; inherited native source
hashes remain baseline provenance, not proof of the new build. Retain the new
target build receipt separately. Cached Ubuntu test binaries are not Pi builds.

Use the frozen source kit linked by the current checkpoint. On the target
filesystem, select the baseline manifest and its independently recorded digest,
the kit's `guard-sources.json` and digest, and the existing ROS-capable Python.
For the current strict 15-role kit, include the explicitly rebuilt controller:

```sh
python3 foundation/hardware/assemble_guarded_release.py \
  --baseline-manifest "$BASELINE/walk-manifest.json" --baseline-sha256 "$BASELINE_SHA256" \
  --source-root "$KIT" --source-index "$KIT/guard-sources.json" --source-sha256 "$SOURCE_SHA256" \
  --destination "$NEW_RELEASE" --guard-python "$ROS_PYTHON" \
  --deadline-controller "$TARGET_CONTROLLER" --deadline-controller-sha256 "$TARGET_CONTROLLER_SHA256"
```

For legacy guarded assembly, select its matching 13-role source index and omit
both deadline-controller options. The indexes are not interchangeable.

Default assembly pins the all-obstacle policy. `--arena-ball-contact` is an
explicit candidate selection for soccer: only class `"32"` from the expected
lab model and label map is exempt. Its generated policy file is hashed in the
manifest and reflected in the verified process arguments. It does not establish
recognition of the intended pink ball; collect ball/person coverage before any
supervised contact trial. Never enable it as a remedy for missing camera data.

No paths or hashes above are defaults. A failed assembly retains any partial
destination; do not reuse it. The resulting `STAGING.json` identifies the new
manifest. Source the new `environment.sh`, then use that release's existing
`runtime_identity.py preflight` before any supervised launch. Preflight probes
imports with the selected Python without initializing ROS or motors. It does not
prove camera throughput or runtime behavior. The newer native software-Start
service needs a separate Pi-compatible gateway build; this assembly does not add
it, and the existing Options path remains available.
The frozen kit includes a source-only `native-start/` companion and package-only
build recipe. The cached Ubuntu gateway requires `GLIBCXX_3.4.32`, while the
recorded Debian Pi provides through `3.4.30`; do not stage that binary or its
CPython 3.12 service bindings. Build and verify against the actual target
underlay during preparation, before requesting physical readiness.

For camera-only coverage, support the body at measured normal walking camera
height/pitch; keep motors inactive. Mark person/ball ranges and lateral offsets,
record fixed model/projection settings and original frame identities, and retain
a separate validation sequence. The November 17 floor recording did not measure
camera posture; the normalized bottom threshold is angular, not a measured
stopping distance.

Before motion, check capture age **and spacing**, detector scene coverage, selected
publisher routing, and clear/blocked/stale stopping with supported legs. Camera
producer source selection is separate from this native release and must use the
reviewed capture-stamped producer. A guard-only exit retains native standing;
restart only as an explicit new selection. Existing required-native-process
shutdown behavior remains unchanged. Physical stopping distance and heading
accuracy still require a supervised floor trial.

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
python3 foundation/hardware/package_app.py \
  --repo . \
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
As of November 4, the attachment path had passed offline tests and independent
review; its first live upgrade/rollback acceptance was still pending. A closed native session cannot
be recovered by this command.

### Onboard voice in the exclusive robot session

After the selected API is healthy and its onboard bootstrap is enabled, explicitly
start the owned voice browser. Use the API's exact loopback origin and the same
private bootstrap directory configured in its environment; no key belongs in the
command. The launcher does not start the API, ROS or motors.

```sh
"$SESSION_PYTHON" "$APP_RELEASE/gemini_pupper/robot/kiosk.py" \
  --url "$ONBOARD_API_ORIGIN" \
  --bootstrap-dir "$PUPPER_ONBOARD_BOOTSTRAP_DIR" \
  --stop-on-browser-exit
```

The option is intentional for our exclusive onboard session. On browser exit or
handled launcher interruption, it requests the existing **global Stop** against
the original API instance. It cancels other clients' work on that same instance,
so do not use this option for a shared robot session. A replaced API rejects the
stale request. Opt-in launch requires the guarded-stop capability before opening
the browser; old APIs are refused, and there is no unrestricted Stop fallback.
Generic launcher behavior without the flag is unchanged.

An unconfirmed Stop produces a warning and nonzero exit; it does not prove the
robot stopped physically. Launcher SIGKILL, renderer hangs, unavailable API/host
and silent network failures remain outside this exit-based protection. Stop does
not fully release the motor driver. Keep the supported shutdown procedure below
for ending a physical session. As of November 4, this exit behavior had offline
evidence and awaited its first watched acceptance.

`shutdown --run-dir RUN` closes owned ingress, uses the existing coordinator to
stop the manager before launch cleanup, then closes owned camera/recorder children.
It never name-kills unrelated processes or uses SIGKILL. Inspect recording closure
and driver logs separately: verified process exit is not verified physical motor
release. App Stop, API process exit and full native shutdown remain distinct.

## Runtime inputs and ownership

The launch and API use the same `PUPPER_GEMINI_DIR` and explicit
`PUPPER_SESSION_SKILLS`. An empty skills value selects stock locomotion; an unset
value refuses launch. The session profile identifies controller gains, joint
limits and velocity caps. A changed profile needs its own simulation and
supervised evaluation.

One operator owns physical commands, service restarts, camera and detector use
for a block. A persistent homed session supports multiple ordinary trials and
app-only restarts. Controller discovery and homing depend on the current
connection and session state. R3 and physical power access remain available
independently of voice. Browser development has an explicit FAKE-server path.

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

For a short voice timing comparison, open Robot controls and use **Clear timing**
before the trial, then **Download timing JSON** afterward. This is an in-memory
window capped at 8 connections, 512 records and 256 KiB; it excludes conversation,
media and raw request IDs. Reloading loses it. The export states dropped records
and clock limitations. SDK sends, scheduled audio and tool handling are not
physical or acoustic measurements; retain the existing server/native evidence
for those layers. No provider or motion test is required merely to open Debug.

## Sensor investigation and physical observations

The standalone [IMU diagnostic](hardware/imu_diagnostic/README.md) runs separately
from the native controller, with Pupper supported and motors inactive. It reads
identity/configuration and can compare selected reports with verified restoration.
It does not home. Its source and earlier results are available when a sensor
question warrants investigation.

An API Stop cancels active and queued work and requests zero velocity. R3 latches
the abort. Full driver shutdown sends the board-disable sequence. Confirm motor
release from physical observation when ending a session. The standing command
has limited posture preconditions and supplies no arbitrary self-righting skill.

The useful physical comparison depends on the change being evaluated. Existing
recordings cover motion chains, policy transitions, interruption, voice and
camera behavior. A fresh trial records its selected release, request, observed
outcome and any missing view. Unobserved or ambiguous motion remains unresolved.

## Finish and resume reliably

Stop owned software, verify exits/driver disable, and preserve complete outputs
with exact input identities. Verify archive members as well as the archive hash.
Keep failures and rejected candidates. Update current state with the next decision,
active processes (if any) and missing evidence. Do not turn passing software tests
or a successful demonstration into an accepted physical capability.
