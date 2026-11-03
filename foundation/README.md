---
date: 2026-10-27T15:34
tags: [cs123, foundation]
---
# Pupper engineering foundation

This branch turns the Fall 2026 course software into a reviewed, reproducible
foundation for a final project. Coursework remains in the separate `main`
checkout. **Workstream 1 and the full foundation are not complete.** Code,
training, simulation, controller completion and observed robot behavior are
different evidence; none alone establishes a generally reliable capability.

## Start here

From `/Users/shrish/workspace/cs123-foundation` on `pupper-foundation`:

```sh
python3 foundation/scripts/session_status.py
```

This local command never contacts or starts the robot. It identifies current
owners, activity, tested Python, blockers and next actions. The
[recovery record](results/interruption-recovery.json) alone owns operational
status and exact deployed versions; do not reconstruct them from old reports.
Use the [three workstreams](WORKSTREAMS.md) for ownership and acceptance goals,
and the [hardware checklist](HARDWARE_CHECKLIST.md) for the supervised workflow.
Read only the report relevant to the next decision.
The [offline workflow improvements](results/workstream1-offline-workflow-20261101.json)
add a reusable closed-session report and server action timeline. They improve
measurement and diagnosis; they do not close the physical reliability gap.

## What the evidence supports

- **Observed on this robot:** bounded direct and voice movement, standing,
  Sit/ten-second hold/both paws/standing, automatic Sit-to-walk exit, and
  cancellation of active and queued work. The
  [corrected-driver session](results/workstream1-imu-candidate-physical-20261101.json)
  records the actual sequences, operator observations and finite-trick limits.
  The [later spoken Stop trial](results/workstream1-voice-repair-physical-20261101.json)
  includes audible acknowledgments and recorded cancellation; it does not measure
  acoustic Stop latency or establish a universal stopping bound.
- **Not yet dependable:** vision-selected paw choice, complete context behavior,
  responsive speech across conditions, generated-motion quality, general policy
  robustness, follower integration and the required Spider walking handoff.
  Selected successful trials do not close these gaps.
- **Failures remain part of the baseline:** earlier finite routines turned
  unexpectedly and a paw attempt tipped the robot. A supported stationary IMU
  then drifted about 110° in 20 seconds. The
  [failure record](results/workstream1-hardware-imu-failure-20261101.json) and
  [reviewed parser/freshness repair](results/imu-parser-candidate-20261101.json)
  explain the driver work. Later improved trials do not prove every failure was
  caused by that defect or that sensor bias cannot recur. The later
  [voice-off hold and motor-disabled capture](results/workstream1-idle-sensor-recurrence-20261101.json)
  reproduced large sensor-heading changes against a qualitatively stationary
  camera view, including about 69° in a minute after motor shutdown. Sensor
  validity remains unresolved; current operational decisions belong in the recovery record.
  The [saved-image comparison](results/workstream1-camera-sensor-consistency-20261102.json)
  now quantifies the discrepancy: four background patches keep the same
  integer-pixel matches across 57 saved frames, while heading changes 65.6° over
  the 54 overlapping sensor/camera samples. This is image consistency evidence,
  not calibrated visual odometry or a diagnosis of the cause.
  The [November 2 controlled session](results/workstream1-supported-recurrence-20261102.json)
  localizes recurrence to quiet standing before the first skill, despite a quiet
  supported startup. Three completed Sit/paw chains do not establish reliability:
  later heading drift contradicts the camera, and persists after controller
  deactivation. [Independent device packets](results/workstream1-post-motor-sensor-20261102.json)
  narrow the fault beyond API/policy execution without identifying its cause.
  The [raw-gyro comparison](results/workstream1-raw-gyro15-20261102.json) captures
  ADC counts and verifies configuration restoration, but its quiet motor-off
  interval does not reproduce or explain the large fault. A separate
  [queued-reading guard](results/workstream1-queued-imu-age-20261102.json) rejects
  provably old sensor reports; it is reviewed offline and remains undeployed.
- **Software feedback has a narrower meaning than motion:** “queued” follows
  actual queue insertion; controller receipts prove matching playback progress,
  not physical posture. Current voice work retains conversation and commands
  together, with server-derived action acknowledgments. The
  [speech-boundary fix](results/workstream1-generation-seal-20261101.json) removes
  an avoidable playback wait; the [ER/Puck alternative](results/workstream1-er-receipts-candidate-20261101.json)
  is separately evaluated. Natural-language interpretation, free-form claim
  screening and generative speech remain fallible. Use current state for which
  source and model are actually running.

The [November 2 voice/product session](results/workstream1-voice-product-20261102.json)
added an atomic handshake and batched model tool replies. Spoken handshake,
compound movement and running-walk cancellation worked in bounded supervised
checks. A native change removed avoidable constant-idle waits; the watched repeat
was controlled with shorter pauses. The [recovered recording](results/workstream1-clip-admission-20261102.json)
now shows all eight recorded clip requests reaching first-applied status in
3.627–25.682 ms; these are software receipt intervals, not physical onset. The
[recorded joints and targets](results/workstream1-standing-return-data-20261102.json)
show motion through the remaining two-second interpolation and two-second policy
fade. A [separate initial-Sit padding reduction](results/workstream1-sit-entry-padding-20261102.json)
saved only 0.258 seconds overall and worsened subsequent simulated walking; it
was rejected, with production unchanged. The same session exposed a separate limitation: “find me” caused
model-generated turns and walking into a backpack. **Camera understanding is not
navigation or obstacle avoidance.** Capability metadata and explicit model guidance
now disclose that gap; they do not enforce the purpose of every valid movement
primitive or make a route safe. The [offline follow-up](results/workstream1-navigation-mitigation-20261102.json)
records two declined navigation requests and a preserved direct compound request;
the confirmation-gate design decision is deferred. A [saved-image follow-up](results/workstream1-navigation-camera-20261102.json)
retained the refusal with a visible backpack and preserved an explicit compound
request; it does not establish realtime camera delivery or guaranteed refusal. A separate
[media setup fix](results/workstream1-media-admission-20261102.json) avoids waiting
for microphone permission in a known-hidden tab.

The [request-order repair](results/workstream1-request-ordering-20261102.json)
prevents overlapping Gemini tool packets from changing command order while
retaining immediate Stop admission. The [duplicate-ID guard](results/workstream1-duplicate-dispatch-20261102.json)
also prevents a replayed provider call from queuing the same movement again.
Two current-service fake execution trials passed; model response latency remains
variable. Two further [non-execution probes](results/workstream1-semantic-nonexecution-20261102.json)
retained useful conversation without sending motion commands; these are bounded
observations, not a general intent-understanding guarantee. The
[full simulated Sit/paws/standing/walk comparisons](results/workstream1-completion-driven-sit-20261102.json)
save 2.67 seconds through earlier constant-idle transitions, with mixed slip and
effort results. A separate [standing-return shortcut](results/workstream1-standing-return-20261102.json)
was rejected because its faster return substantially increased simulated effort.
The [ER2 video probe](results/workstream1-er-vision-20261102.json) incorrectly called
a seated endpoint standing; use its annotations for inspection, not acceptance.

The [later startup/voice closeout](results/workstream1-startup-gravity-physical-20261102.json) separates heading drift from gravity stability in the startup screen. The revised screen passed recorded negative controls, 13 selected Pi tests and 54 current-source driver tests; one homing and an observed voice right-paw/standing return succeeded. Sensor calibration and long-duration reliability remain unresolved. A user-reported mixed sequence is retained separately because this run's browser/API logs contain only the two initial actions.

## Course coverage

This is a capability map, not a percentage-complete claim. The
[handout coverage ledger](results/course-capability-coverage.json) preserves the
inspected source hashes, locators and code inventory; its dated operational
comments are historical. Source handouts live in the class library under
`Fall2026Classes/Cs123/References/`.

| Area | Implementation and evidence | Remaining boundary |
|---|---|---|
| Lab 1 | Bounded signed PD, measured-pose entry, consistent position/velocity targets, sampled delay and sensor-loss handling; [runtime tests](tests/test_pd_runtime.py). | Supported joint experiments, timing, gains and thermal response require physical validation. |
| Lab 2 | Four-leg FK, read-only recording, MCAP import and validated export to Lab 3; [recording review](results/lab2-recorded-motion-review.json). | Saved front-right record is essentially static; moving back-right replay exceeds a knee bound and is refused. Physical frame/sign calibration remains. |
| Lab 3 and extensions | Bounded IK, unreachable-target rejection, interpolated gait caches, cycloidal swing, additional patterns and tuner conversion; [position](tests/test_position_controllers.py) and [single-leg](tests/test_single_leg.py) tests. | Reachability is not dynamic stability. Live tuner ownership, fresh feedback and transitions belong to workstream 2. |
| Lab 4 | PPO training/evaluation/export, recovered team policies and [matched clamp comparison](results/lab4-stablegait-clamp-gpu-comparison.json). [Native comparisons](results/shipped-policy-native-screen.json) support retaining the shipped gait. | Better tracking can worsen slip, effort or reverse stability. No general promotion or measured hardware speed rating. |
| Lab 5 and options | Following, target persistence, clothing-color recovery, timed detours and detection watchdog; [integration guide](../follow_me_pupper/README.md). [Camera/Hailo detection](results/live-camera-perception-reviewed.json) ran on the robot. | Full following, target recovery, color calibration, stopping and Lab 6 takeover need workstream 3's physical evidence. Timed detours do not establish geometric clearance. |
| Lab 6 tools and UI | Bounded move/turn/arc/wait, serialized jobs, cancellation, transitions, selected profiles and fake HTTP; [runtime guide](../gemini_pupper/docs/foundation.md). Direct and voice examples are linked above. | Broader repeatability, spoken refusals, context and correct model arguments remain. Explicit [routine quantity validation](results/workstream1-trick-quantity-20261101.json) prevents unsupported counts; a playback is not a counted physical repetition. |
| Lab 6 Sit/stay/shake | [Expanded-COM policy](results/lab6-sit-comx02-reviewed.json), [native full sequence](results/sit-interactive-default-return-reviewed.json), and observed robot sequences above. | +2.5 cm COM remains outside training range and backend-sensitive. Repeated real sequences and interruption coverage remain bounded, not general fall recovery. |
| Lab 6 Spider | Entry/idle/exit, separate slow walker and controller handoff; [matched sensing study](results/spider-encoder-observation-study-reviewed.json) and [native chains](results/spider-encoder-native-reviewed.json). | Nominal returns settle; all eight selected adverse default returns remain moving. Walking pairs remain research assets, not an accepted integrated capability. |
| Supplied finite tricks | [Twelve-policy native screen](results/course-tricks-native-screen.json), [standing-ending returns](results/finite-tricks-default-return-reviewed.json), [lying-ending later returns](results/lying-tricks-later-default-return-reviewed.json). Four standing-ending routines have bounded physical observations above. | Do not generalize those observations to all twelve. Sneeze speed, lying exits, transitions and push-up drift remain limits. [Retrained yaw candidate](results/pushup-yaw-training-20261101.json) improves simulated yaw but worsens tracking, target steps and clipping versus supplied weights; no automatic replacement. |
| Supplied reference jump | [Actual stopped-jump API path](results/gemini-jump-api-reviewed.json) and paired no-jump controls. | Recovery only 2/4 after fresh activation and 2/4 after walking; controls 4/4 each. Earlier forward-motion success does not establish a reliable standing jump or hardware result. |
| Optional Lab 1 | Aligned motor/tactile datasets, recording-level splits, multitask/PCGrad models, contact export and single-leg pause; [data/model evidence](../implicit_compliance/README.md). | Standard Pupper lacks the tactile/single-leg setup. Weak labels and effort semantics need calibration; terrain heads do not control locomotion. This is not a whole-body contact reflex. |
| Optional Lab 2 | Faster/mixed references, capture/bootstrap, reward and terrain comparisons; [experiment protocol](../pupper-mjlab/configs/optional2/README.md). Corrected flat/Bumpy studies and [Mystery continuation](results/optional2-mystery-reviewed.json) are preserved. | No promotion. Mystery yields only 0/0/1/1 strict flight-and-recovery events across four seeds. Earlier double-offset runs remain labeled altered-reference experiments. |
| Optional Lab 3 | Character/context, local secret gate, pinned UniMate generation, motion repair and trained tracking; [context study](results/optional3-context-20261101.json) and [server-session counter follow-up](results/workstream1-context-and-transport-followup-20261101.json). | Generated sway is physically attempted but jittery/unaccepted; [corrected-reader evidence](results/generated-motion-corrected-reader-20261101.json) and [clearer-reference failure](results/generated-motion-clearer-reference-20261101.json) retain that limit. Subsequent [matched bob training](results/generated-bob-training-20261101.json) already reproduced visible motion, but [loaded lower-leg hull contacts](results/generated-bob-contact-extent-20261101.json) keep its strict screen failed. A [bounded friction comparison](results/generated-bob-friction-pair-20261102.json) changed actual hull support friction but barely changed the bob and passed neither strict contact screen; it does not justify another training sweep. Conversation memory is not identity authentication. [Synthetic vision tests](results/workstream1-vision-paw-native-20261101.json) do not validate physical camera-to-paw choice. |

## Contracts that change how results should be read

1. **Actuation and contacts:** simulations must name controller clamps, the
   tighter absolute [driver joint bounds](results/local-driver-position-limit-audit.json),
   [realized inertial randomization](results/pupper-randomization-audit.json)
   and [full body/leg contacts](results/tracking-body-contact-audit.json).
   The driver bounds are not a per-step 0.1 rad slew limit. Disabled hulls and
   torso-supported motion can make an animation misleading.
2. **Sensing:** real controllers observe measured joint angles. New locomotion
   experiments explicitly select `--encoder-observation-mode deployment_measured`;
   historical `legacy_true` checkpoints retain their provenance. See the
   [hardware-coordinate audit](results/encoder-hardware-semantics-review.json).
   Driver host-receipt freshness is not physical sensor age; older logged-age
   perturbations were not verified sensor-delay replay.
3. **References and jumps:** exported class joint tables are absolute. Do not
   add the default pose again. The [seed correction](results/optional2-class-seed-correction.json)
   and [capture audit](results/optional2-capture-independent-review.json) retain
   rejected originals. Flight requires no body contact, upward COM motion and
   sustained landing recovery; five 50 Hz samples span 80 ms, not 100 ms.
4. **Transitions and stopping:** evaluate continuous entry/playback/return with
   the actual default controller. [Selected adverse default-start cases](results/spider-default-restoration-independent-review.json)
   remain moving even after eight seconds of zero command. Playback receipts
   are [activation/request-correlated](results/controller-completion-v2-reviewed.json),
   but are not physical-stillness measurements. Ordinary rest retains controller
   damping; full driver shutdown is a different operation. Cloud speech has no
   proven Stop-latency bound; physical stopping remains independently available.

Classical PD/IK experiments and the learned behavior stack share geometry and
data contracts, but must not compete for motor interfaces. The learned stack
routes tools through checked queue/ownership transitions; following relinquishes
control before another behavior, and interactive skills use their explicit exits.
Full follower and Spider handoffs remain acceptance work, not presumed integration.

## Reproduce, preserve and extend

Use the tested interpreter and validation commands in current state, and
`pupper-mjlab/uv.lock` for training dependencies. The
[environment inventory](results/local-environment.json) records an observed setup,
not a universal platform guarantee. Run checks affected by a change; source hashes
and exact commands in each dated report determine what a passing result covers.
Typical offline checks from the outer checkout are:

```sh
python3 foundation/scripts/check_foundation.py --output /private/tmp/pupper-validation-NEW
python3 foundation/scripts/check_ros_offline.py --output /private/tmp/pupper-ros-NEW
```

The main runner discovers and import-checks the separate API and numerical
interpreters; explicit `--api-python` and `--numerical-python` overrides are supported.
The ROS check uses the existing local Docker image with networking disabled and
no device mounts. Start Docker Desktop first; its optional rebuild recipe is
`foundation/scripts/ros-offline.Dockerfile`. The
[Linux ARM evidence](results/workstream1-offline-ros-20261102.json) records the image
and source identities, actual memory-controller tests and sensor compiler fix.
These checks never deploy or contact Pupper. The
[environment validation record](results/workstream1-environment-validation-20261102.json)
preserves the final 11-group pass, the earlier fixture failure, and the restricted
Claude source review.

Native parity checks live in `pupper_gait_deploy/test/` and simulation checks in
`pupper-mjlab/tests/`. Missing dependencies are environment gaps, not passing tests.
Course support repositories, ROS references and pinned UniMate sources are inputs;
their presence does not authorize replacing the robot's class image.

Keep exact code, model, reference, configuration, seed, exits and raw evidence with
every experiment. Dated reports under [results](results/) point to verified
archives in `artifacts/` and `/Users/shrish/workspace/pupper-foundation-artifacts/`.
Preserve failed runs and originals. Training `/content` and temporary directories
are not durable storage. Reuse [checkpoint collection](scripts/preserve_colab_run.py),
[verified restore](scripts/restore_colab_snapshot.py) and
[bounded final downloads](scripts/download_colab_archive.py); demonstrate one
verified checkpoint before a long run and preserve the full final archive before
release. A periodic per-file snapshot can contain different moments and does not
prove job completion. Inspect actual owners/exits before retrying any interrupted job.

A useful final-project baseline is a frozen, repeatedly evaluated set of skills
with measured limits. The class does not itself provide mapped navigation, reliable
person identity, depth-based obstacle clearance, object manipulation, calibrated
force sensing or arbitrary fall recovery. Pick a concrete missing behavior and a
repeatable metric, then compare against that baseline on held-out scenarios.
More trained policies and passing tests alone do not mean Pupper is “maxed out.”
