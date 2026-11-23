# PupperFC: adversarial review of the core problems

November 22, 2026. **Phase 2 investigation; all remedies below are proposals.** No product code, deployment, robot service, camera configuration or motion changed in this review. Product sources: main `836d2fb`, match `3e5f81f`, Tej overhead `28f3e8c`, and the hash-identified recovered u6 runtime. The preceding documentation checkpoint is `3ab031a`.

The [reconciliation](fc-phase2-reconciliation-20261122.md) remains the evidence and physical-test register. This review changes how its F1–F12 work should be organized: build one complete execution path around a few consistent rules, rather than repair every competing path. It does not declare the physical cause of late-session jitter solved.

## Conclusion

The central problem is that useful components have grown into several partially overlapping systems. They disagree about what an observation means, when movement can be revised, who owns a running action, and what counts as readiness or success. Calibration then tries to compensate for errors whose causes have not been separated. Finally, the physical body itself has unresolved startup and sustained-standing failures.

Rebuild the integration around the existing continuous FC executor, native Stop and exclusive kick transition. Preserve useful perception, planning and policy work. Retire the bridge's production motion loop after a replacement slice is physically qualified. This is substantial consolidation, not a rewrite of ROS, locomotion, camera calibration or every UI screen.

## Six causes and direct remedies

### A. Physical behavior is not yet a dependable foundation

**Problem:** a controller being active does not establish that the robot is standing correctly or will remain stable. The recorded late deterioration and immediate FC startup faults are different phenomena. In one zero-command interval the recorded tilt/gyro/joint activity deteriorates substantially, but there is no independent body reference establishing whether bad sensing caused motion, motion corrupted sensing, or both participated in a feedback loop. Four startup faults occur within 1.6 ms of activation; two are joint exchange and two first-claim expiry, despite a generic IMU label.

**Direct remedy:** select one frozen body release and make its Home → Stand → Move → Kick → restored walking lifecycle explicit. Each transition must have a real completion condition and a specific failure reason. Preserve the native limits, independent Stop and admission checks while identifying the failing predicate. Qualify warm standing against external body observations before using that release to estimate speed, heading error or kick timing.

This requires targeted instrumentation and physical tests, not a promised sensor-mode or firmware cure. Session6's better windows are counterevidence to a simple inevitable-temperature explanation; their telemetry is not a substitute for external stillness. Existing main driver timing/publication corrections remain useful without proving they cure jitter.

### B. The system sometimes loses the difference between a measurement and a guess

**Problem:** the bridge can label an old robot observation with a later ball timestamp. u6 repairs invalid camera timestamps into apparently recent ones. A rejected ball jump can become the next accepted reference; an old body-relative correction can survive robot movement. These are different ways of losing the origin and meaning of information.

Main already rejects coasting observations and preserves original per-robot capture times. Keep those checks. A newly reproduced counterexample is more specific: color heading is emitted at capture time, while ingress and the gameplay adapter subtract angular velocity × frame age as if it were extrapolated. An exact-function synthetic ingress replay changes 0.5 rad to 0.4 rad. Correct the producer/consumer convention together, including tag and color paths. Main's camera timestamp is also receipt minus configured latency, so correct network-clock conversion does not establish the true exposure time.

**Direct remedy:** extend the existing observation boundary with consistent per-entity records: original capture time and timing uncertainty, coordinate frame, source/session, calibration revision, measurement method, and measured/predicted/invalid status. Preserve rejection and uncertainty through every consumer. Prediction retains its last real observation and remains necessary for moving-ball interception; it is never relabeled as a new measurement.

Add metadata only when an actual consumer needs it; reuse existing fields. An unmeasured timing or position bound stays **unknown**, not a guessed number. Initially, use post-stop overhead heading and current onboard ball evidence for contact. Enable continuation through tag loss only after its drift/error budget is measured; a single sample of each is not automatically sufficient evidence.

Keep the overhead field estimator and each robot's local ball estimator separate. Assemble a read-only view for a decision without pretending all values were captured together. Each action uses the evidence it actually needs. Fresh local ball imagery may support final alignment during brief tag loss if goal heading, clearance and body stability remain bounded. It cannot refresh global pose or authorize blind routing/defense. A new camera/calibration revision invalidates dependent targets and corrections.

### C. The approach loop waits instead of continuously responding

**Problem:** the bridge issues a finite movement, waits for completion, settles, samples several frames and decides again. It does not process overhead frames during ordinary movement. The last five logged jobs span 18.868 seconds, with 6.196 seconds inside active jobs and 12.672 outside. The remainder includes several possible costs; it is not a measured network-only or scheduling-only delay. Lost tracking can then trigger multiple predicted jobs, including a kick.

**Direct remedy:** adapt Tej's useful stage/route/aim calculations into target proposals for FC's existing continuous approach executor. Process current observations while moving; let target updates change the next bounded command. Separate slow images, preview rendering, recording and requests from the control/Stop clock. Keep the actual foot strike as an exclusive finite action, with native completion and restored-walker receipts.

The first physical slice should use a deliberate stationary contact commitment. Smooth moving-ball contact is a later, measured extension. Continuous feedback can oscillate if delay/noise is too high, so this is not a claim that faster ticking automatically improves behavior. Compare capture-to-command age and physical reaction under the same gait. Detector latency still matters; hardware acceleration should follow a measured bottleneck, not be assumed unnecessary or the primary cure.

### D. There are several definitions of who owns the robot and what is finished

**Problem:** the main/match app, selected Pi FC release, and recovered u6/bridge path are different operational products. They use different launch, readiness and action-result rules. The bridge reacquires permission per operation, accepts text as readiness and can treat missing health as healthy. That can resurrect old intent or report success without the required physical evidence. This review does not establish that two owners were simultaneously commanding the reported run.

**Direct remedy:** select one compatible Mac/Pi release pair. One Pi execution path per robot accepts autonomous, teleop and experiment requests. Bind an armed run to its release/session and deliberate operator authorization; Stop, replacement or restart ends that intent. The native controller retains independent fault/Stop authority. Keep distinct permissions for distinct resources; do not replace them with one universal process or lock.

Report requested, accepted, controller-completed and physically observed outcomes separately. A kick receipt proves playback/restoration, not ball contact or a goal. UI is a client of these operations, never the author of readiness. Per-robot movement/kick ownership is separate from game roles: remove the referee's global single-attacker assumption for two opposing robots, which may both attack.

### E. Calibration is compensating for unknown errors

**Problem:** learning tag heading from walking direction mixes mounting geometry, sideways gait drift, real yaw and sensor error. Learning from a predicted starting pose or a skipped kick can compound the error. Meanwhile, image geometry and guessed physical constants disagree: panorama roundness rejects a close spherical ball; camera mounts are estimated; deployed and main ball radii differ; marker bounds can be mistaken for physical goal planes.

**Direct remedy:** freeze persistent heading, motion and kick calibration during play. Keep online position/velocity estimation and ball prediction. Measure geometry independently, then fit optional movement/contact corrections from actual actions with genuine before/after observations. Evaluate them on separate trials before deliberately selecting a profile.

Use one shared physical field/ball/goal definition, with separate camera calibrations and per-robot mounts. Preserve Ball Eye's ray/sphere/outline geometry and compare it with corrected color proposals on the same captures. Do not rewrite detection without that comparison. A larger rigid tag is a testable information-quality improvement, not a verified cure. Separate marker layout, playable bounds and actual goal planes: main's current `goal_geometry` rejects planes farther than one ball radius outside the marker-defined end, so the reported 0.3 m offset cannot simply be entered without changing that model. Verify the front-post positions rather than confuse net depth with goal-plane offset.

### F. A complete UI run has not been the unit of delivery

**Problem:** individually successful scripts leave setup, recovery, experiments and evidence collection outside the app. That permits different prerequisites and results depending on who launches the test. Missing Mac recordings also prevent checking the reported physical outcomes.

**Direct remedy:** make one selected release and one complete recorded run the deliverable. The UI and automated checks invoke the same maintained operations. Each run identifies the software, models and physical profile, records observations/actions/failures and verifies recording closure. Reuse the existing transactional lobby, setup and session helpers. Do not introduce a general workflow engine or another dated launcher.

The functional UI is part of the first slice. Visual work can proceed in parallel; no terminal-only test is considered an integrated product capability. Recording should be bounded and asynchronous so it does not recreate the control delay.

## Proposed responsibilities

| Owner | Responsibility | Must not infer |
|---|---|---|
| Mac | Overhead field observations, calibration, referee/score, independent tactics for each opponent, UI | Native readiness, physical kick success, freshly observed pose from old data |
| Each Pi | Local ball observations, continuous approach, exclusive movement/kick/teleop execution, session outcomes | New authorization from a reconnect, global heading from ball bearing alone |
| Native control | Bounded joint actuation, actual transition receipts, independent Stop/fault response | Ball contact or goal scored |

These are ownership boundaries, not a required process count. Tej's geometry becomes planning input; FC's continuous executor remains the movement mechanism; the native kick remains the foot strike. The proposed architecture is not yet physically qualified.

## Remove, retain, and patch deliberately

**Remove from the final autonomous entry point after replacement qualification:** the bridge's finite-job polling/SSH motion loop, per-job permission reacquisition, blind-job quota, text-substring readiness, persistent online calibration learning and synthetic timer-based kick completion. Keep frozen originals as evidence and diagnostic candidates; none automatically qualifies as a complete rollback release.

**Retain:** native Stop/expiry and valid admission checks; existing continuous targets and same-stream target-update support; exclusive kick restoration; current field solver/tag/color tracking; Ball Eye geometry; Tej's stage/aim calculations; existing useful UI transactions and policy candidates. Preserve unique teammate work while consolidating.

**Small fixes still needed in the selected path:** color-heading time normalization; rejected-fix handling; goal-plane validation; native fault snapshots; existing driver timing/publication corrections. They correct local mechanisms inside the shared design. Do not spend the implementation budget hardening every feature of an owner that will be retired.

Do not delete a check merely because it blocked a run. First distinguish a real physical constraint from duplicated orchestration or an incorrectly implemented predicate. Conversely, do not add another layer that repeats readiness without new evidence. Clock/error limits should be related to measured action tolerance, not loosened or multiplied speculatively.

## Evidence needed to accept or reject this direction

This is dependency analysis for Phase 2, not authorization to begin Phase 4.

1. **Without motion:** replay stale/future/outlier/calibration-change inputs; stop/server-replacement races; color versus tag heading semantics; skipped kicks and actual receipt sequences. The selected path must retain original evidence and never resume an old action after Stop. Compare target-update latency using the same recorded observations. A type/schema change alone is insufficient if adapters still fabricate data.
2. **Small stationary capture batch:** actual ball at contact and farther away, near/far robot tag at opposite headings, actual tag plane/ball diameter/goal posts. Replay the same images through existing candidates. Expand T1–T3 only where the first captures leave an unresolved distinction.
3. **Body discrimination:** externally referenced motor-disabled sensor capture, instrumented supported Home/Stand, then warm zero-command standing on the selected release. These distinguish startup admission, sensor behavior and powered feedback; motor-disabled stability alone cannot qualify standing.
4. **One complete physical slice:** UI launch/setup → Home/Stand → observed approach → real foot contact → restored walking → acknowledged Stop/rearm, with video and run closure. Select a new release only after old ownership has ended; never hot-swap libraries under a live controller.
5. **Expand on measured capability:** short repeated forward/lateral/turn/arc and controlled tracking loss, both-foot contact repeatability, rolling interception and standing defense, then second-robot qualification. Preserve a complete qualified release as rollback, not a mix of libraries from different versions.

There is a concrete decision after the instrumented startup/standing trial: a named software defect gets a targeted repair and repeat; a genuine physical failure moves to the corresponding physical diagnosis. If a different runtime is proposed, it must pass the same release, Stop and standing qualification. Do not let integration continue indefinitely without that result, and do not call the legacy bridge a proven demo fallback. Phase 3 must assign a bounded time budget to this decision using the actual available hardware sessions.

Do not run every test-matrix cell before the first useful slice. Use the smallest test that distinguishes competing explanations. Warm-standing duration must still cover the reported late onset. No calendar cutoff can turn an unqualified u6/FC hybrid into a proven fallback. Because Tej's Mac raw recordings are still missing, use synthetic fault cases for mechanism tests and a new motors-off UI capture for real perception comparison; do not imply the missing run can already be replayed end to end.

## Coverage of the agreed product

| Requirement | Consequence for this design |
|---|---|
| One-command launch; Home/Field/Robot/Settings/Play | Same app operations for discovery/manual connection, calibration, Home, arm, tests and recovery. Initial Play requires actual readiness and admissible ball/participant observations. |
| FIFA-style home and attractive dedicated Play | Preserve two kits, branding, Settings and three connection actions; validate layout through the UI. Play displays overhead plus one onboard feed for every participant, scores, roles, poses, targets, failures and latency. |
| Variable field, six perimeter tags or more | Derive metric layout from known printed scale and marker roles; ask only unresolved measurements. Show markers/axes/outline and adjustable yellow goal lines; physical mouths remain 0.6096 m. |
| Foot strike, smooth approach, recovery | Keep the actual trained kick candidate; verify contact, heading, ball outcome and the first stable resumed walking step separately. |
| One robot always offense | Normal one-robot play stays offense; an explicit experiment can exercise defense with that robot. |
| Moving ball and two independent opponents | Use measured arrival/turn/braking bounds. Referee owns score; each robot chooses its own role rather than sharing one global attacker slot. |
| Standing defense | Predict an own-goal crossing; choose a reachable interception with measured arrival margin. Otherwise retreat to cover the goal mouth and stand/block. With uncertain ball motion, use a conservative ball-to-goal blocking position. No sit motion; qualify actual blocking/response physically. |
| Teleop and all comparisons through UI | Same per-robot execution owner for teleop. UI experiments compare onboard near-ball value, tag versus color, both-foot repeatability and gait speed/stability. |
| Clean handoff and no fake success | Current brief/checkpoint remain the entry points. Software replay, controller receipts and physical outcomes remain distinct. Older evidence stays historical. |

## Relation to the earlier work register

| Core issue | Existing work/test coverage | Changed emphasis |
|---|---|---|
| A: dependable body | F9; T4–T6 | Identify the predicate/cause before changing admission or estimating motion. |
| B: honest observations | F1, F2, F6; T1–T3, T6 | One consistent admission boundary, not unrelated timestamp patches. |
| C: continuous response | F3, F10; T6–T8 | Retire the serial production loop rather than optimize its waits. |
| D: one selected execution path | F3, F4, F8, F12; T5, T7, T9 | Integrate a compatible release/session; preserve independent native protection. |
| E: identified geometry/calibration | F5–F7; T1–T3, T6–T8 | Freeze persistent adaptation; measure separate causes before fitting corrections. |
| F: UI run as deliverable | F8, F11, F12; T1–T9 | Functional UI and closed evidence in the first slice, not a later wrapper. |

The original F/T entries remain useful acceptance cases. Their grouping here supersedes treating all twelve as independent patches to the bridge.

## Independent adversarial passes and adjudication

Root wrote an independent first-principles pass, checked the sources and reran the isolated color-heading counterexample. Two separate **exact Claude Fable 5.2, high-effort** instances completed successfully (85.87 s and 93.45 s, both exit 0). They received the user vision, source/evidence pack and then explicit challenges to the root proposal; pass two also directly read existing target-stream and goal checks. Two **exact Opus 5.5, high-effort** labor reviews completed (99.4 s and 114.66 s, exit 0), with root/source-agent adjudication. Provider suggestions are review input, not evidence by themselves.

**Disagreements that changed the recommendation:**

- Fable objected that F3/F4 would harden an owner we should retire. Accepted: move reusable planning into the existing FC path rather than fully rehabilitate bridge scheduling/authority. Preserve its frozen evidence until the replacement is qualified.
- Fable preferred freezing persistent learning to adding many learning guards. Accepted for the current recovery; online state estimation remains essential. Only introduce empirical corrections if measured residual error requires them; then compare on separate trials before trusting them.
- Fable challenged the root proposal's broad metadata and qualification language as potential new ceremony. Accepted: extend existing interfaces only for a consumer, keep unknown bounds explicit, use small discriminating tests and an evidence-based startup/cutover decision. No world-model service, universal lock or deployment framework.
- Fable initially claimed main shared the bridge's stale-row bug. Rejected after checking ingress, gameplay and per-robot capture handling; pass two withdrew it. The specific color-heading convention defect remains independently reproduced.
- Fable's suggested fallback to the bridge after a calendar deadline is **not accepted as qualified**. Reported contacts lack camera confirmation, and the body/Stop/health issues apply there too. The selected FC path is also not physically proven by this review. Preserve candidates without inventing a successful full-game baseline.
- Reject “FC never starts” from four selected failure logs, all latency attributed to the queue, or an observed concurrent-owner collision. Reject deleting checks unless they map only to physical failures: stale authority, cross-session receipts and deadline failures are also real software failure modes. Reject blanket box-detection prohibition, a universal one-frame contact rule, or merely deleting the goal bound without reconciling geometry. These need actual consumer accuracy requirements and consistent physical models.

Provider receipts, the synthetic counterexample and source pointers are indexed in [the review record](fc-architecture-review-20261122.json). Raw review packs and responses are preserved locally under `outputs/pupper-fc-architecture-20261122/`; they are ignored evidence, not product code or a new current-status log.
