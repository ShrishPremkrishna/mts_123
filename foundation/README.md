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
  caused by that defect or that sensor bias cannot recur.
- **Software feedback has a narrower meaning than motion:** “queued” follows
  actual queue insertion; controller receipts prove matching playback progress,
  not physical posture. Current voice work retains conversation and commands
  together, with server-derived action acknowledgments. The
  [speech-boundary fix](results/workstream1-generation-seal-20261101.json) removes
  an avoidable playback wait; the [ER/Puck alternative](results/workstream1-er-receipts-candidate-20261101.json)
  is separately evaluated. Natural-language interpretation, free-form claim
  screening and generative speech remain fallible. Use current state for which
  source and model are actually running.

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
| Optional Lab 3 | Character/context, local secret gate, pinned UniMate generation, motion repair and trained tracking; [context study](results/optional3-context-20261101.json) and [server-session counter follow-up](results/workstream1-context-and-transport-followup-20261101.json). | Generated sway is physically attempted but jittery/unaccepted; [corrected-reader evidence](results/generated-motion-corrected-reader-20261101.json) and [clearer-reference failure](results/generated-motion-clearer-reference-20261101.json) retain that limit. Conversation memory is not identity authentication. [Synthetic vision tests](results/workstream1-vision-paw-native-20261101.json) do not validate physical camera-to-paw choice. |

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
python -m pytest foundation/tests implicit_compliance/tests gemini_pupper/tests
python gemini_pupper/lab/test.py
python foundation/scripts/check_foundation.py
```

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
