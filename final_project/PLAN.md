# PupperFC rebuild plan — Phase 4 execution

Updated November 23, 2026. Phase 4 is paused after a usable software checkpoint at the user's request; future implementation requires a resumed task. Offline work, including isolated no-device work on the powered Pi, comes first. Physical tests are separated below; their batching and scheduling are deferred. The packages below define acceptance, not completed implementation or hardware qualification. The [architecture review](../foundation/results/fc-architecture-review-20261122.md) explains the changes; the [brief](README.md) owns the product vision and the [checkpoint](../foundation/results/interruption-recovery.json) owns current activity.

## Intended result

One command launches the complete Mac app. Home/Field/Robot/Settings/Play, calibration, per-robot teleop, experiments, recordings and recovery use the same maintained operations. A compatible Pi candidate runs one continuous execution path per robot with the existing finite foot kick and independent native Stop. Field and onboard observations retain their actual meaning; persistent calibration does not change during play. Both opponents can choose offense or standing defense independently.

At the end of offline work, this system is integrated, tested through the actual UI, built for the actual Pi and staged without activation. Simulated behavior and device-free native checks are labeled as such. Physical accuracy, stable standing, contact and real matches remain unqualified until the separate tests pass.

## Delivery status

**Delivered software checkpoint:** source `c58ff09` is the frozen production-runner rehearsal with dedicated Play, rendered overhead perception, goal/reset/Resume, teleop and terminal Stop/reconnect. It is an ideal-capture synthetic comparison, not acceptance of the selected camera-delay/native-entry contract or physical behavior. The [closing handoff](REBUILD_HANDOFF_20261123.md) and [checkpoint evidence](../foundation/results/fc-phase4-checkpoint2-20261123.json) identify the exact verified scope.

**Preserved shared-primitive work, no longer active:** canonical Pitch/goal geometry, independent tactics and motion-qualified prediction, Ball Eye metric admission, native fault handling and the selected native-chain harness are preserved on `codex/pupper-fc-phase4-unaccepted-20261123` at `05fd01422a9a082dd2babc1c5c065ed3b9460618`. Their [manifest](../foundation/results/fc-phase4-preserved-candidates-20261123.json) separates source from acceptance. Native Pi functional and C++/Python wire checks passed, but lint is non-green and final target-chain/pair acceptance remains pending. Timing measurements showed actual native activation after the claimed deadline; no end-to-end correction is accepted. All provider work is finished or explicitly paused, and queued implementation must not auto-resume.

**Still required offline:** none of the following is dropped by the backend work:

- Finish the coherent Pitch v2 setup/producer/ingress/planner/referee switch, measured goal-front and nominal-mouth display, variable field/marker setup, and Field Finish returning Home. Complete near-ball comparison and the actual optional local consumer while keeping unmeasured live budgets disabled.
- Finish Home's Blue/Field/Red connection flow, upper-right Settings, soccer presentation, dedicated Play with the overhead image and each participant's onboard feed, readable pose/target/age and real kick/Stop receipts. Validate saved/synthetic images through the same read-only feed adapter; absent live cameras must remain visibly absent.
- Add recording and export through shared app operations, including original observation/event timing, source/model/profile identity, raw and processed images, drop/flush/completeness receipts and credential exclusion. Then expose calibration, lifecycle, teleop, stationary/changing-ball kick, loss/reacquisition, rolling interception, one-robot standing-defense and second-robot experiment controls through the UI.
- Complete independent two-opponent attack/defense, clearance, held/lost-peer and goal/reset/resume scenarios using the production executor. Freeze a standalone Mac package and complete inactive Pi source/model/native pair, verify selected executable/message/policy and health-wire boundaries, close evidence, and run the final frozen app's browser and affected regression checks. Robot two's absence does not block its offline software path.

**Physical-only qualification:** P1–P6 below still require the user's setup and observations: actual geometry/latency/error, Home/Stand/jitter diagnosis, movement/braking and interruption bounds, contact/recovery, rolling defense and a second physical robot. Preparing their UI operations and recording flows is required offline; performing them is not authorized in this block. Missing Mac footage is recorded as a gap, while the original Pi recordings are locally complete and hash-verified.

## What “offline, including the Pi” means

- Mac: source integration, unit/contract tests, recorded/synthetic perception, simulation, browser validation, release assembly and evidence analysis.
- Pi: read-only inventory, preservation of existing logs/models, isolated target builds, linkage/import checks, explicitly device-free native/ROS tests, replay/inference on saved images, and file-only staging of an inactive candidate.
- Excluded: native hardware initialization, Home, motor/controller activation, changes to a running sensor/camera service, reboot, active-profile selection or any command that silently replaces a live session. A live camera or sensor capture still requires its specific setup; saved-image inference does not.
- “Connect,” “prepare,” “runner-only” and “no-launch” are not proofs of no physical effect. Some FC prepare paths shut down a previous session or start deferred perception. Use audited file-only preparation and isolated no-device launches. An isolated ROS domain alone does not prevent device access.

The Pi was reachable in this planning session, with Python 3.11.2, CMake/Ninja/g++/rsync and about 47.9 GiB free. This supports the work plan, not a clean build or motor-state claim. Recheck ownership and executable identities before implementation-side Pi work.

## Ordered work packages

The numbers give integration order. Several packages run concurrently once their shared interfaces are defined; they are not nine serial waits for hardware.

### 1. Select the integration baseline and prove Pi build compatibility early

**Work:** use the active FC checkout and source-by-source integration map for main, match, Tej overhead and recovered u6. Preserve unique teammate work and the exact old releases. Select the existing FC executor, native kick/Stop, field solver, Ball Eye geometry and Tej aiming calculations; exclude the bridge's independent motion ownership from the new entry point. Record Mac/Pi source, model, driver, environment and profile identities. Recover the remaining raw recording archive with a verified resumable transfer when feasible; its absence does not block synthetic software checks.

On the Pi, identify the real target underlay/toolchain and run a monitored compile/link smoke test for the selected native packages in an isolated build location. Do not install test artifacts over the active build. Confirm the existing Ball Eye model can load for saved-image inference. Keep training provenance unknown if unrecovered.

Refresh the preserved native build recipe against the integrated source closure: its November 19 version has fixed inputs and supports only the controller and gateway packages. Changed driver instrumentation needs its actual target build/test recipe; do not invent an unsupported option or reuse an older successful build as evidence for new source.

**Output:** a concrete selected-source map, a compatible build recipe, preserved baseline identities and a minimal release description understood by the Mac and Pi.

**Offline acceptance:** selected source imports and target code builds/links on the real Pi without starting a hardware node; missing dependency/ABI incompatibilities are resolved now. A missing Mac run recording is recorded as missing rather than treated as a passed replay.

### 2. Establish shared app operations and an immediately usable UI shell

**Depends on:** package 1's selected boundaries; UI layout can start while the build runs.

Define only the existing interfaces that cross components: accepted observations, target proposals, session/control ownership, and requested/accepted/controller-completed/physically-observed outcomes. Extend current helpers and types; do not add a workflow engine or universal world-state service.

Expose discovery/manual connection, selected-release description, field setup, Home/Stand requests, Play/hold/Stop/rearm, per-robot teleop and recording through the same backend operations. Add a labeled replay/test mode that runs those operations against an explicit test backend. Live mode cannot silently fall back to synthetic observations or readiness.

Wire Home, Field, Robot, Settings and Play from the outset. Put Offline Validation and experiment entry points in the UI. Maintained scripts may implement an operation behind a button; an operator or agent must not need a separate terminal sequence to complete an acceptance test.

**Output:** one-command app launch, navigable screens, explicit live/replay state, operation results and recording identity.

**Offline acceptance:** a browser-driven setup/Stop/recovery rehearsal completes through real app routes with labeled test results; missing participants/cameras produce actionable messages, not invented readiness. The shell grows continuously through the remaining packages.

### 3. Unify observation semantics and physical geometry

**Depends on:** the selected observation/profile interfaces. Runs alongside packages 4 and 7.

Preserve original per-entity capture times, source, frame, calibration revision and measured-versus-predicted status. Add uncertainty/method fields only where a consumer uses them; unmeasured bounds stay unknown. Correct color/tag heading time conventions together; stop invalid timestamp repair; reject outliers before they update tracking; expire body-frame corrections after motion/loss. Keep independent overhead and Pi-local estimators and assemble their admissible outputs without inventing one shared capture time.

Use one physical field/ball/goal definition with per-camera and per-robot calibrations. Separate marker layout, playable bounds and surveyed goal fronts. Support variable field dimensions from known printed scale, the six perimeter marker roles and extra markers. With insufficient markers, ask only the remaining geometric constraints. Show markers, axes, field outline and adjustable yellow 0.6096 m goal mouths in Field Setup, with Finish returning home.

Integrate Ball Eye's useful ray/projection/outline geometry; compare it and corrected pink proposals on identical saved images. Preserve tag and color/nose alternatives for comparison. Freeze persistent heading/motion/contact learning during play; provide explicit saved profiles and comparison results instead.

**Output:** consistent inputs for planning/close alignment, complete field setup and deterministic detector/calibration comparisons.

**Offline acceptance:** stale/future/duplicate/outlier/recalibration cases cannot become fresh evidence; the reproduced capture-yaw defect is eliminated for both adapters without breaking tags. Geometry consumers agree on goals and bounds. Detector comparisons report error only when independent labels exist; synthetic sphere checks remain synthetic. Missing real footage becomes a prepared UI capture task.

### 4. Consolidate robot lifecycle, native diagnostics and teleop

**Depends on:** packages 1–2; proceeds in parallel with package 3.

Use one Pi execution path per robot for autonomy, experiments and teleop. Preserve existing session/intent ownership and independent native Stop. Reconnection, server replacement and Stop end old intent; readiness is not inferred from text or absent health.

Make Home/Stand/movement/kick/restored-walking outcomes explicit and consistent. Instrument distinct joint-exchange, report-validity and admission failures, including the exact failed predicate, report ages, generation/deadline and bounded recent history. Reuse confirmed driver timing/publication fixes. Audit transition-specific conditions; fix a demonstrated software defect without widening joint limits/gains or claiming the physical jitter solved.

Implement per-robot teleop in Settings now as the simplest client of this execution path. Autonomous and teleop ownership cannot overlap; loss of input and Stop follow the same expiry/cancellation mechanism. Provide UI tests for all transition outcomes and named errors.

**Output:** target-built native diagnostics, shared execution/lifecycle operations and usable teleop in replay/device-free mode.

**Offline acceptance:** Stop during activation/kick, delayed receipts, missing health, process replacement, double requests and disconnect/rearm behave correctly in the selected device-free chain. Native kick completion requires the matching request/clip and actual walker-restoration receipt. Software holds, latched Stop and emergency damping remain distinct. Physical Home/Stand and stillness remain pending.

### 5. Complete the one-robot continuous approach and foot-kick slice

**Depends on:** usable outputs from packages 2–4.

Move Tej's stage/route/aim calculations into target generation for the existing continuous FC executor. Keep overhead and local sensing independent of UI previews, slow requests and recording. Refine approach targets while walking, slow into paw placement, commit one finite foot kick and resume from actual restoration evidence. Keep the first contact mode deliberately stationary; add smoother contact only against measured timing.

Use action-specific loss/reacquisition: current local ball evidence does not refresh global pose; blind-job quotas are removed. Tag-loss continuation requires a measured heading/drift/clearance budget. Prepare the implementation and tests while leaving this capability unqualified in live profiles. Keep front-camera person/obstacle hold/Stop behavior, with the selected soccer-ball exception tied to the actual detector/model rather than a blanket bypass.

**Output:** the entire one-robot UI flow through observed approach, foot-kick request, recovery and replayed goal/reset/resume. Normal one-robot gameplay stays offense.

**Offline acceptance:** browser-driven stationary and changing-ball scenarios, gaps/reacquisition, abort during kick and first resumed command all exercise the same selected executor. Record observation-to-decision-to-command latency under replay and slow-consumer load. Simulated foot contact and goal outcomes are clearly labeled; receipts do not turn into physical-success claims.

### 6. Add moving-ball play, standing defense and independent opponents

**Depends on:** the shared interfaces; pure tactic work can start earlier, integrated acceptance follows package 5.

Implement ball prediction and intercept selection using a movement profile that includes speed, turning/braking and kick delay. Synthetic profiles allow deterministic tests; real values require physical measurement. Predictions retain their source age and bounds. Missing measured parameters block only the dependent live capability, not the rest of the offline work or supervised measurement workflow.

For defense, predict an own-goal crossing and choose a reachable interception; otherwise retreat to cover the physical mouth and stand/block. With uncertain ball motion, use a conservative goal-side blocking position. No sit motion. Expose a one-robot defensive experiment without changing normal solo offense.

Remove the referee's global single-attacker assumption. Each opponent independently selects offense/defense with hysteresis, its own goal and its own execution owner. Preserve per-robot Stop/teleop, identity and feeds. Resolve opponent-clearance cases through approach, kick and reset, including a held/missing peer; unconditional deadlock is not a successful match test. The second physical robot is not needed to implement these paths.

**Output:** replay-complete rolling interception, standing defense and two-opponent games, plus diagnostic scenarios for role switching and clearance.

**Offline acceptance:** both robots may attack simultaneously without sharing a motor owner; role choices remain stable across noisy observations. Test defend/intercept/retreat branches, stale ball, held/lost opponent, scoring and deliberate reset/resume. Simulated arrival/braking assumptions remain visible and unqualified.

### 7. Finish the full operator experience and experiment suite

**Depends on:** package 2; develops throughout packages 3–6, rather than starting after them.

Finish the FIFA-style Home with PupperFC branding, two robots in kits, Settings and three connection actions. Finish attractive Field/Robot setup and a dedicated Play screen: overhead field plus one onboard feed for every participant, score, role/action, robot pose/heading, targets/errors, source age/latency, and understandable holds/recovery controls. Keep feeds and recording from blocking control. Show clean one-robot and two-robot layouts and honest missing/disconnected states.

Every remaining physical question gets a ready-to-run UI experiment: stationary geometry/tag/color capture, near-ball comparison, sensor/body diagnostics, directional motion/turn/arc, controlled tracking loss/front-camera stopping, both-foot kick repeats, moving interception, standing defense and second-robot commissioning. Each names the setup, measured values, stopping condition, raw recording, result and unresolved question. Saving an observation must not invent a physical outcome; allow the operator to record contact/miss/intervention explicitly.

Include one Offline Validation operation for the maintained software checks, with progress/results and exportable diagnostics. Complete browser/computer-use validation loops on actual screens, including keyboard/focus behavior, stalled cameras, reconnect, disabled actions, one/two participants, responsive layouts and screenshots. Final art improves here, with functional checks running throughout.

**Output:** all agreed screens, feeds, controls and experiments usable through the app, plus reviewed visual evidence.

**Offline acceptance:** every planned acceptance/rehearsal workflow can be run without a separate terminal sequence after launch. No stale or placeholder UI status claims the robot is healthy, physically standing or successfully kicking. All tests use the same backend operations as normal play.

### 8. Assemble and verify the complete inactive Mac/Pi candidate

**Depends on:** frozen integrated sources for packages 3–7. Incremental Pi build checks already occurred in packages 1 and 4.

Use the maintained FC release assembler and selected native build recipe; the older Lab 6 app packager is not a universal FC deployment path. Include the actual field helper, target-built native components, Ball Eye model/config and selected policies. Verify exact executable/library/config identities and imports from the staged tree, not accidentally from the developer checkout. Keep hardware activation and profile selection separate from staging.

Run the selected Mac-to-Pi protocol, inference on saved images and native wire/transition tests with the audited device-free backend on isolated ports/domain. Measure load/timing without interpreting it as camera exposure or physical response. The current live release stays unchanged.

Use the supported FC `stage`/`verify` and static native-inspection surfaces after extending their selected file closure. Low-level persistent-session preparation can create new run files without launching; its manifest-specific preflight checks selected imports. Do not substitute the higher-level FC prepare/Connect path, which can mutate an existing session. Inspect partial output before retrying and verify test processes have exited.

**Output:** a complete paired release, build/staging receipts, explicit capabilities and an inactive candidate ready for supervised selection.

**Offline acceptance:** clean-target import/link checks, model loading and the entire selected protocol pass with no hardware driver/device initialization; mismatches fail with a specific reason. Release metadata is visible in the UI and included in every run. Device-free checks prove software behavior only.

### 9. Freeze, adversarially validate and clean the handoff

**Depends on:** packages 5–8 integrated.

Run affected tests during implementation; after the sources freeze, run one broader regression and full UI rehearsal. Use exact Fable 5.2 high-effort review on consequential ownership/time/geometry changes and integration, with source adjudication. Native tests must use the chosen executables and message contracts. Recheck only after relevant changes/failures.

Exercise capture loss/future/outlier data, calibration changes, delayed operations, Stop/restart races, stale peer, slow recording and partial artifact collection. Confirm recordings close and identities remain attached to failures. Remove dead code from the new entry point and duplicate launch instructions. Preserve unique branches and frozen old releases; operational retirement/deletion of the old path follows physical replacement acceptance. Update one concise brief, this plan, relevant setup documentation and the current checkpoint.

**Output:** reproducible UI-driven validation, one coherent release candidate, exact known unknowns and ready physical experiments. No orphan agent/reviewer/build processes or incomplete archive labeled complete.

**Offline acceptance:** a fresh session can launch the app, run software validation, inspect the staged candidate and open the next physical experiment from the UI. All milestones have functional software coverage; none is mislabeled as physically done.

## Parallel work and priorities

Let implementation, provider and build tasks finish without wall-clock termination limits; monitor them with progress updates. Robot command expiry, Stop and native safety deadlines remain unchanged.

After a small shared-interface agreement, use three concrete worker lanes plus root integration:

| Lane | Main ownership | Runs without physical qualification |
|---|---|---|
| Perception/geometry | Field producer, timestamp adapters, onboard observation path, physical profiles and comparison fixtures | Packages 3 and relevant package 6 prediction inputs |
| Pi/runtime | Native diagnostics, selected build/release helpers, lifecycle/teleop operations and device-free checks | Packages 1, 4 and 8 |
| UI/experiments | Static UI, agreed server/lobby adapters, views, recordings and experiment operations | Packages 2 and 7 throughout |
| Root integration | Shared interfaces, controller/kick/tactics integration, file ownership and final verification | Packages 5, 6 and 9 |

Assign one owner per file, especially shared lobby/server/integration modules, before edits. Reviewers are read-only; use available slots as workers finish rather than invent a fourth concurrent worker lane. Opus labor and Fable second opinions remain the preferred external models where source-sharing approval permits them; a blocked optional review does not block local work.

Critical sequence: **selected baseline/build smoke → shared operations → honest inputs plus one executor → one-robot UI slice → richer tactics and complete UI → frozen Pi candidate and final verification.** UI design, instrumentation, saved-image comparison and test preparation are concurrent work. Finish the first software slice before spending effort tuning later milestones. Do not wait for robot two to implement its software path.

Do not spend the offline block on speculative new policies, broad detector training, Hailo conversion, arbitrary admission relaxation or an automatic self-righting implementation. Reopen a bounded experiment when data shows it addresses the remaining failure. Preserve required fall stopping and deliberate recovery; physical self-righting remains a separate bounded feasibility question, not a hidden promise.

## Physical tests the user will need to run later

These are dependency groups, not scheduled batches. All scripts, displays and recording flows are prepared by packages 2–9.

| Group | What needs the real setup/user | What it decides | Prior register |
|---|---|---|---|
| P1: geometry and perception | Measure ball/tag/camera/goal geometry; place ball at near/contact positions and robot near/far at known headings; record actual lighting/coverage | Accurate field/goal model, near-ball method, tag versus color, actual exposure/processing age and usable error bounds | T1–T3 |
| P2: body and lifecycle | Support the robot for candidate selection/Home/Stand; external-reference motor-disabled sensor capture, supported transition, warm zero-command standing, Stop/rearm | Exact startup cause and sensor-versus-body deterioration; whether the selected body release is usable | T4–T5 |
| P3: motion and interruption | Short measured forward/lateral/turn/arc and braking trials; controlled localization/network loss and person/obstacle stopping | Real arrival, turn, stopping and tag-loss limits; physically effective interruption | T6 |
| P4: complete foot-kick slice | Place stationary ball, run full UI approach/kick/recovery, record both feet and all misses/interventions | Actual contact window, goal heading, kick delay/scatter, stable restored walking and benefit of onboard refinement | T7 |
| P5: rolling ball and standing defense | Roll balls on known paths and toward the own goal; observe intercept/retreat/block behavior | Prediction quality, reachable defense, contact under motion and real failure cases | T8 |
| P6: second robot and final match | Per-robot calibration and qualification, independent role/teleop checks, opposing play and repeated full app startup/reset | Whether software generalizes to robot two; real two-opponent game, reliability and demo footage | T9 |

Physical parameters remain unknown until measured: exposure delay/error, tag/ball accuracy, warm stability, usable speed/turn/braking, tag-loss drift, kick contact/timing/scatter and opponent clearance. They live in the selected profile with provenance; replay uses explicitly synthetic values rather than filling the live profile with guesses.

The first body result creates a decision: fix a named software defect and repeat the relevant test, or investigate the indicated physical failure. Do not bypass admission or automatically transplant a new loop onto u6. Preserve any genuinely qualified full release as rollback; the reported bridge run alone is not a qualified fallback. After physical data changes a profile or code, repeat affected checks rather than rebuilding every unrelated layer.

## Completion and delivery

The offline handoff contains the full Mac app, inactive Pi candidate, source/model/profile identity, browser-verified screens, scenario fixtures, passing software checks, closed evidence and all physical experiments accessible in the UI. It explicitly separates implemented/replay-tested, target/device-free-tested and physically verified behavior.

The December 2 video and December 3 live-demo deadlines remain the user-confirmed targets. Preparing recordings and export now supports them; footage, performance claims and the final demonstration still require physical runs. Existing accepted ball replacement plus one deliberate resume cue remains supported, as do fall stopping and front-camera obstacle interruption. Earlier numerical match-success targets and historical coursework details remain evidence in Git and prior reports, not invented achievements or reasons to delay the first useful slice.

Body diagnosis is the biggest schedule uncertainty. It does not prevent this offline plan, but software completion alone cannot guarantee a playable robot by the deadline. Once the user schedules hardware time, the prepared body diagnostics can run without waiting for tactics or art to be finished.
