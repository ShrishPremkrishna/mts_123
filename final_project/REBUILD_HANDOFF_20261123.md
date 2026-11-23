# Pupper FC software checkpoint handoff — November 23, 2026

The user requested closure after the next usable software checkpoint. That checkpoint is accepted; the whole offline rebuild and physical milestones are not complete. [PLAN.md](PLAN.md) retains the full acceptance scope; [README.md](README.md) is the product brief. The [current checkpoint](../foundation/results/interruption-recovery.json) remains authoritative for access, owners, processes and machine-local state. This document explains the product, practices and evidence rather than prescribing one future implementation.

## Product objectives

The intended result is a complete, usable Mac operator app paired with a compatible Pi runtime for one or two soccer-playing Pupper robots. One command should launch it. Setup, play, diagnosis, experiments, recordings and recovery should be accessible in the app; an operator should not need a separate terminal workflow to complete an acceptance test.

The user's original priority order was a one-robot UI vertical slice, kick quality, moving-ball interception, two robots, teleop, then polish. Subsequent steering emphasized a fully usable UI throughout development. This is product priority context, not a requirement to reproduce the implementation sequence used here.

- **Home and navigation.** Home, Field, Robot, Settings and Play are first-class screens. Home foregrounds Blue robot, Field camera and Red robot connections, in that order, with Settings at the upper right and recognizable soccer presentation. Discovery and manual connection, release/health description, missing-device states, deliberate Home/Stand and independent Stop/reconnect must be understandable. Reconnection must not automatically initialize or home hardware.
- **Field setup.** Automatically calibrate from all six perimeter markers; with fewer visible markers, ask only for the missing geometric constraints. Support variable field dimensions from known printed scale, the marker roles and extra markers. Display markers, axes, field outline and adjustable yellow nominal 0.6096 m goal mouths. Marker coordinates, playable boundaries and observed goal fronts are different facts. Preserve an observed mouth's discrepancy from the nominal width rather than inventing an accuracy tolerance. Finish returns Home. Tag and color/nose tracking remain available for comparison.
- **Play and robot views.** Dedicated Play combines overhead imagery with one onboard view per participant, pose/target/error/age and actual command/kick/Stop receipts. Missing feeds remain visibly unavailable. Preview is read-only and independent of control; it must not become a second camera owner or manufacture fresh observations. Each robot has its own identity, mode, operator connection and teleop controls. Autonomous and manual motion ownership cannot overlap.
- **Continuous approach and foot kick.** Deliver a smooth continuous approach, deliberate foot strike and verified recoverable walking afterward. Refine targets while walking, slow into foot-specific alignment, commit one finite kick, verify the requested action and restored walking, and then continue. Start with stationary contact. Normal single-robot play stays offense; deliberate ball replacement plus a Resume cue is an accepted flow. A kick/controller receipt is not evidence of ball contact or a physical goal.
- **Perception and close alignment.** Use overhead field imagery and available onboard information for responsive localization and close alignment. Carry original capture times, source/frame/calibration identity, measurement-versus-prediction meaning and usable uncertainty through admission. Local evidence must not fabricate global pose or become live physical evidence merely because a live profile was selected. Compare available onboard ball methods quantitatively on identical images, including the recovered Ball Eye work and pink proposals. Missing calibration, radius, timing or error measurements remain unknown. Persistent calibration/heading/contact learning stays frozen during play.
- **Moving-ball play and defense.** Predict ball motion and choose reachable intercepts using explicit speed, turning, braking and kick-delay assumptions. Synthetic profiles support offline work; measured live bounds are separate. Defend an own-goal threat by interception when reachable, otherwise retreat to cover the physical mouth and stand/block—not sit. Expose a one-robot defensive experiment without changing default solo offense.
- **Independent opponents.** Both robots can select attack or defense stably, avoid noisy switching, and retain their own goal, execution owner, Stop and telemetry. A global single-attacker rule is not the final match design. Cover head-on approaches, possession/clearance, noisy role switches, held or lost peers, scoring, reset and Resume. The absence of a second physical robot must not block implementing its software path.
- **Lifecycle and diagnostics.** Autonomy, teleop and experiments need consistent ownership and recovery behavior. Make prepare/Home/Stand/movement/kick/restored-walking outcomes distinct. Report specific joint-exchange, IMU, admission, generation and deadline failures, plus useful commanded-versus-measured joint evidence. Source fixes do not establish that startup jitter or warm instability is cured. Preserve native Stop, ownership and fault behavior; do not widen joint limits or gains to obtain a passing demo.
- **Interruption and recovery.** Front-camera person/obstacle observations must support effective hold/Stop behavior. Any soccer-ball exception must be tied to the selected detector/model rather than broadly ignoring obstacles. Opponent clearance includes moving, held and lost peers. Fall stopping, visible faults, manual repositioning and deliberate rearm must work; automatic self-righting remains an unresolved feasibility/capability item, not delivered recovery.
- **Experiments and recordings.** All calibration, lifecycle, teleop, approach/kick/recovery, observation-loss, rolling-defense and second-robot tests should be accessible through the app. Include forward, lateral, turn and arc trials; quantitative tag-versus-color comparisons under lighting and latency changes; kick-repeatability/contact/scatter comparisons; and policy speed/stability comparisons. Record original observation/event timing, requests/results/decisions, source/model/profile identities and raw/processed images where available. Export must describe dropped/incomplete data and flush/close status, and exclude credentials and reusable authorization capabilities. Latest-only preview is not complete recording.
- **Delivery.** The final offline handoff should include a standalone Mac package, an exact complete inactive Pi candidate, selected executable/configuration/message/policy checks, browser-verified flows and closed evidence. Physical geometry, stability, contact, stopping, defense and real two-robot reliability require later P1–P6 observations. December 2 video and December 3 live-demo dates remain the user-confirmed targets; they are not evidence that the robot will be physically ready.

## Development practices the user cares about

Prioritize speed to useful outcomes over ceremony. Avoid bloat, keep current README/setup documentation coherent, call out real blockers, and remove gates/checks that cannot justify the concrete failure they prevent. Work in usable, inspectable increments. Keep a frozen working checkpoint available while the next one changes. A loaded backend paired with changing frontend files is not a frozen release. Prefer shared, established primitives and modular changes to accumulating patches or creating a second controller/lifecycle. These are design principles, not a requirement to preserve every current abstraction; future agents should challenge the architecture when the evidence warrants it.

The requested collaboration model was orchestration at the root, with GPT-6 Astra integration/workers, exact Fable 5.2 at high effort for orchestration and architecture review, and exact Opus 5.5 at high effort for broad implementation. Give implementation agents relevant repository context, tools and outcome ownership, and let them solve the problem. Coordinate file ownership and inspect returned model identities, including nested reviews. Do not substitute a model alias and assume it resolved to the requested version. Selected source sharing with those providers was explicitly approved; credentials remain excluded.

Do not impose task/provider/build wall-clock kill limits. Monitor progress and let substantive work finish. This does not remove application cleanup, transport, command-expiry or robot safety contracts. A proven stuck test or mutant can require diagnosis and cleanup; distinguish that from cancelling useful work because time elapsed.

Validate the consequential boundaries, not just a large test count. Run affected checks, use actual app routes/buttons, and verify selected native binaries, messages, configuration and policy assets without devices before physical use. Repeat broad checks when changed sources or unresolved evidence justify them. Keep exact failed and final inputs/results; do not erase a counterexample during cleanup or relabel an earlier run as testing changed inputs. Preserve source identity, software execution receipts and observed physical capability as separate facts.

The active integration checkout is `/Users/shrish/workspace/cs123`; `cs123-foundation` is historical. One owner controls physical operations. This block authorized no motion, Home, camera/sensor activation, robot service changes or active profile selection. Local commits and isolated device-free Pi work were authorized. The latest user instruction also authorizes publishing and saving all work consistently. This session chose the existing GitHub remote and separate accepted/unaccepted codex branches, without force or merging main. All agents/providers finish or pause with handoffs; queued product/recording implementation was not started.

## Learnings and evidence

These are observations and open design questions, not deterministic instructions to implement a particular patch.

1. **Actual boundaries exposed defects missed by isolated tests.** The selected gateway emitted FC operator fields that the Python StopStatus parser rejected. The strict full schema is repaired in `eeda8df`. The actual production replay also exposed Resume checking the executor with a timestamp sampled before its control step advanced time; the same commit repairs that observation order.
2. **Expired motion and failed neutral publication are different outcomes.** An expired manual command could be replaced and published as genuine zero, then still raise a publication failure and terminate the session. `3c3c18b` distinguishes this successful-neutral case while other evidence remains current. The retained red reproduction failed; 36 affected checks then passed, followed by focused repeated-idle/fresh-input and failed-neutral checks. No stale motion deadline was renewed. See the checkpoint's command-expiry evidence.
3. **The timing contract is not end-to-end qualified.** The maintained selected-chain container measured roughly 88–94 ms from request to switch send and 106–148 ms for the gateway round trip. Actual fixture activation occurred 11–46 ms after the claimed `start_before` in recorded runs. A Python send gate is not an actual native activation gate. With the selected 0.12 s capture delay, one 700-frame scenario issued no kick request. The [timing review](../foundation/results/fc-phase4-timing-review-20261123.json) separates history, freshness, request waiting, RPC observation and control-loop authority as useful concepts. Its scratch patch is unapplied and unaccepted: allowing older captures or a provisional 0.5 s request allowance is not a qualified correction. No end-to-end timing/admission fix has been accepted.
4. **Geometry and measurement provenance matter across consumers.** Marker bounds can differ from the playable area and surveyed goal fronts. Capture heading must remain distinct from extrapolated heading, including color and tag producers. New geometry/tactics experiments and local-ball work must be judged against those contracts and their own exact source/evidence, not the currently accepted preview. An earlier local-ball counterexample admitted synthetic-status input under a manually qualified live profile. The final draft tightened calibration-status admission, but calibration status alone is not measurement origin; the source-mode contract and actual executor consumer remain unaccepted. The default live local feature remains unqualified.
5. **Native software improvements do not diagnose the body physically.** Source review found invalid SPI replies could be decoded into apparent joint state, and nonfinite command/state paths needed explicit handling. The frozen driver work and target checks are recorded separately from actual Home/standing. C++ health bytes were decoded through the selected Python reader/helper using explicit recorded-clock, synthetic ROS and expected-PID fixtures; this tests format/predicates, not live ownership or physical Home. The actual Pi driver passed 78 functional checks; the final fixture-only change was separately relinked/tested on Pi with unchanged production objects. Lint is not all green and final release/target-chain acceptance remains pending. The native candidate is preserved as unaccepted-for-release, not selected onto the live Pi.
6. **Ball Eye compatibility is narrower than accuracy.** The recovered 12,647,163-byte ONNX model (`ae3a4592…90a827c`) loaded and produced finite `[1,5,2100]` outputs on the Pi's OpenCV CPU path with no device access. Blank and synthetic-circle inputs produced no accepted detections at the tested threshold. Training provenance, mounted accuracy, true ball radius and contact-range visibility remain unqualified. Saved-image/synthetic geometry results must not be reported as real-world detection success.
7. **Evidence has explicit gaps.** All three original Pi MCAPs (1,267,442,408 bytes total) and the retained metadata were locally hash-verified; Tej's missing Mac footage remains missing. The native harness provider deleted the original exploratory parser-failure directory; log excerpts survive, but not a complete original failure archive. Final selected passing runs remain preserved. Model/time-cap history is recorded in provider receipts: an early job was cut off by a pre-override timer; one native nested review used Sonnet, and the geometry nested review resolved to Fable 5.1. Those are not counted as requested-model review evidence. Later forced pins and separate exact Fable reviews are identified explicitly.

## Progress and closing checkpoint

The accepted software source is `c58ff09a75acebc8cccd7788f2340bd5759f5a80`. The final immutable archive SHA256 is `706e32c4a241ae3a7aa86fbcc87a682095e7eee8fc50dfb765b5a59ccbeec133`. [Checkpoint evidence](../foundation/results/fc-phase4-checkpoint2-20261123.json) preserves the actual browser observations, source identities, screenshots and limits.

| Commit | Preserved outcome |
|---|---|
| `ecfd97f4559b100ca8d3084cf864c374cd413b4c` | First usable synthetic I/O/lobby checkpoint; source retained, old port 8876 preview now closed. |
| `36ad531` | Strict helper/gateway/runner integration and shared observation admission. |
| `0ebd0f1` | Pure motion contract separated from ROS imports. |
| `eeda8dff5e96ac76ab27ca07b59a614480f50979` | Strict FC gateway status schema and post-runtime Resume clock. |
| `3c3c18b` | Expected expired manual motion publishes neutral without poisoning the standing session. |
| `68bd548c0376c9027bf75f7f90ea64c7e5b35fd0` | Production-runner replay, dedicated Play/operator adapter and standalone laptop source closure. |
| `260c77fabcd9051ba12e23fa2746155c239c616e` | Source identity test uses its own Git fixture and works in extracted archives. |
| `c58ff09a75acebc8cccd7788f2340bd5759f5a80` | Final software checkpoint: ended-match Standby is distinguished from terminal Stop. |

The closing app uses actual RunnerSession, OperatorDrive, GuardedPath, CombinedRuntime, Robot queue, Executor and selected kick code inside an isolated synthetic-I/O robot process. It is still an ideal-capture timing comparison. Home/Stand, real camera/board/native hardware, physical contact and the real operator Stop link are not rehearsed by this browser fixture. The separate native chain exercises additional actual native boundaries with its own listed substitutions. Onboard feeds, recording and full experiment surfaces are not delivered by this checkpoint.

The browser exercised a goal, explicit ball replacement, Resume and a second goal; End/outside-ball refusal/recenter; teleop movement, idle zero and fresh-input re-engagement; two participants and terminal Stop; reconnect; narrow/mobile layouts; and visible offline-check results. A separately labeled saved live-shaped kick edge tested the same display adapter. The final app is idle with no participants. Development acceptance passed 77 Python and 4 Node checks. The immutable `68bd548` archive passed 76 Python and 4 Node checks; its one failure was a test assuming a Git checkout. The final archive passed the corrected identity case, 4 Node checks and syntax, followed by actual final End/Stop/mobile/source browser checks. We did not falsely count a repeated full suite on changed sources. The extracted source-package test also started the actual synthetic runner without importing from the checkout; a final complete Mac/Pi release pair is still pending.

**Preserved experimental source:** branch `codex/pupper-fc-phase4-unaccepted-20261123`, commit `05fd01422a9a082dd2babc1c5c065ed3b9460618`, contains 46 changed source paths. The [preservation manifest](../foundation/results/fc-phase4-preserved-candidates-20261123.json) records every changed-file SHA256, the native 37-file freeze and component evidence. This branch includes geometry/tactics, local-ball/Ball Eye and native/harness work; it is not a release candidate accepted for use on the body. Geometry's main Opus process was gracefully interrupted at the user's closing boundary while its nested review remained incomplete. The supervisor returned OS exit 0 with a null final result and a diagnostic: this is a paused review, not a successful completed review. Its actual nested Fable 5.1 provenance is retained. Ball Eye and timing review completed normally; timing's reference patch remains only a local unapplied proposal. All those transcripts and large evidence archives remain in local `outputs/`, not Git.

**Repository and publication:** the accepted branch is `codex/pupper-fc-phase2`; the experimental branch is named above. The remote is [ShrishPremkrishna/mts_123](https://github.com/ShrishPremkrishna/mts_123). This session publishes those two refs; it does not force-update, merge main or delete branches. Fresh remote inspection showed main at `836d2fbefbf28face265b89a8524842224ec1699` and match at `3e5f81f1ae05eb78e79203a95ace319f389f231d`; teammate branches were preserved. The closing publication receipt at `outputs/pupper-fc-phase4-20261122/integration/publication.json` records the final documentation-bearing remote tip as well as the exact accepted software and experimental commits. The accepted checkout has been restored to its accepted source; closing commits add current documentation and qualification metadata only. Ignored raw evidence and local environments are intentionally retained. No unexplained source edits should remain after the closing commit; verify with Git before resuming.

**Processes and launch:** all Claude/provider implementation jobs are closed or explicitly paused, and runtime/UI agents are finished. Only the delivered app remains on [localhost:8877](http://127.0.0.1:8877), launcher PID 83697, server PID 83707, tool session 79666, with no replay robot children. Its immutable root is `/private/var/folders/7c/__k49cw562d4txf0dslj1wcm0000gn/T/pupper-fc-checkpoint2-final-xi_44e3p`. Old 8876/initial 8877/fixture 8878 servers are closed. PIDs are a dated observation: check command identity before any later cleanup. From the accepted checkout, the verified launch is:

```sh
PUPPER_FC_PYTHON=/Users/shrish/.codex/worktrees/fc-teleop/cs123/.venv-fc/bin/python ./pupper-fc --mode replay
```

This reuses the verified environment without modifying it. The standalone setup path is documented in `pupper_fc/LAPTOP.md`. Pi access was last checked on boot `11a2d1d3-bc84-447f-959b-6312369444be`; the existing `field_redundant_imu/events/right` selection was not changed. No current posture, torque or physical readiness follows from process inventory.

## Remaining work and physical validation

Remaining offline work includes a usable selected-timing/native-admission contract; review and coherent integration of canonical Pitch/tactics and local metric evidence; the complete Field setup and Home presentation; actual per-participant read-only onboard feed display; recording/export and experiment controls; independent two-opponent acceptance; and final standalone Mac/inactive Pi pairing with frozen browser/native/evidence checks. Several useful source experiments exist, but their preservation must not be mistaken for acceptance.

A reasonable next session could first assess the preserved outcomes and the timing/authority boundary, then choose its own architecture and sequence for the remaining product slices. Recording before physical experiments and keeping a working preview available are useful priorities, not instructions to reproduce this session's designs. No queued follow-up implementation should resume merely because a prompt file exists.

Physical P1–P6 remain separate: geometry/perception; body/Home/Stand/jitter diagnosis; motion/braking/interruption; full foot-kick/contact/recovery; rolling-ball standing defense; and second-robot/final match reliability. Their operations and recording flows can be completed offline, but unknown physical parameters must not be filled with synthetic values in a live profile.

## Agent-readable snapshot

This is a handoff snapshot, not a second live status record. Read the current checkpoint before acting.

```json
{
  "schema": "pupper_fc_handoff_v1",
  "as_of": "2026-11-23",
  "state": "paused_after_accepted_software_checkpoint",
  "authoritative_checkpoint": "foundation/results/interruption-recovery.json",
  "product_scope": "final_project/PLAN.md",
  "active_checkout": "/Users/shrish/workspace/cs123",
  "accepted_ref": "refs/heads/codex/pupper-fc-phase2",
  "software_commit": "c58ff09a75acebc8cccd7788f2340bd5759f5a80",
  "unaccepted_ref": "refs/heads/codex/pupper-fc-phase4-unaccepted-20261123",
  "unaccepted_commit": "05fd01422a9a082dd2babc1c5c065ed3b9460618",
  "publication_receipt": "outputs/pupper-fc-phase4-20261122/integration/publication.json",
  "progress_evidence": "foundation/results/fc-phase4-checkpoint2-20261123.json",
  "preservation_evidence": "foundation/results/fc-phase4-preserved-candidates-20261123.json",
  "learnings": [
    {
      "finding": "Python send authority did not bound actual native activation",
      "evidence": "outputs/pupper-fc-phase4-20261122/runtime/native-chain-timing/summary.json",
      "status": "unresolved; reference patch unapplied"
    },
    {
      "finding": "Successful neutral on expired manual input must differ from failed publication",
      "evidence": "commit3c3c18b and checkpoint command-expiry red/green receipts",
      "status": "software repair accepted"
    },
    {
      "finding": "Actual gateway schema and post-step Resume time differed from isolated assumptions",
      "evidence": "commiteeda8df and immutable browser/native chain receipts",
      "status": "software repair accepted"
    },
    {
      "finding": "Capture provenance, geometry and local-ball calibration require coherent admission",
      "evidence": "preserved candidates manifest and observation review",
      "status": "experimental integration unaccepted"
    }
  ],
  "physical_qualification": "unqualified; P1-P6 pending",
  "active_jobs": [],
  "retained_preview": "http://127.0.0.1:8877; idle synthetic app only",
  "do_not_auto_resume": [
    "queued implementation prompts",
    "physical/profile/service actions"
  ],
  "recommended_on_resumption": [
    "Read exact handoff/checkpoint/source manifests and choose the next product outcome",
    "Reassess timing/native admission with retained measurements before accepting realistic-delay play",
    "Independently review preserved geometry/local perception/native work; architecture remains open",
    "Complete feed/recording/experiment/setup UI and independent opponents, then exact inactive pair and final acceptance"
  ],
  "pending_offline": [
    "timing/native admission",
    "geometry/tactics integration",
    "local metric admission",
    "complete setup/presentation",
    "onboard feeds",
    "recording/export",
    "experiment controls",
    "two-opponent acceptance",
    "final Mac/Pi pair and QA"
  ]
}
```
