# Pupper FC

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. An overhead iPhone camera observes field/robot markers, a bright ground ball and two goals. Gameplay uses a 2D field model. The initial shot uses locomotion and a front bumper; foot kicking and a special goalie posture are stretch goals.

## Current agreement and owner steering — November 17, 2026

- Shrish currently owns movement/sprinting, locomotion-based headbutts, gameplay and integration. Tej iterates on the foot-kick policy. Ryan owns overhead perception, localization and proposed actions/shot targets; our harness translates those into responsive robot execution. Marcus’s existing gait exports remain available for comparison; the active ownership checkpoint governs robot access. Shrish builds the 8 × 12-foot arena with approximately 2-foot goals; no bumper is fitted yet.
- The owner accepted manual ball replacement and one deliberate resume cue after goals for the first working game; robots reposition themselves with software still running. Physical self-righting gets a bounded feasibility investigation before a demo commitment. Fall stopping, ordinary standing and physical get-up remain distinct capabilities.
- Two-robot soccer remains the target, with penalty shootout as fallback. On November 17 the owner prioritized coordinate/heading navigation, explicit offense/defense selection, measured latency, broader gait qualification and onboard person avoidance before further contact trials with the wrong ball. The intended ball is pink, size 5, with flower patterns. Person entry into the field should make both robots avoid the person. The proposed architecture and acceptance scope are under discussion in the plan; no new hardware session is active.
- Power-on should prepare the selected stack automatically with minimal setup. A supported startup pose and one deliberate start/arm action are accepted; their implementation needs verification. Other game-rule decisions can wait.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposals, integration responsibilities and acceptance evidence. Working notes cover [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and evidence — November 17, 2026

Active development is in the `main` checkout, currently `/Users/shrish/workspace/cs123`. The sibling `cs123-foundation` checkout preserves earlier engineering work and its November 4 checkpoint; it is not the current FC handoff.

| Capability | What the evidence establishes | Remaining acceptance |
| --- | --- | --- |
| Movement | After removing per-step controller logging, three-second left/right floor tests at 0.15 m/s were stable and stopped cleanly; a marked 0.45 m/s, two-second forward repeat traveled roughly 30 inches. Earlier low-rate turns worked. [Trials and fall diagnosis](../foundation/results/gait-gate-20261116.json). | The original 1.32-second interruption's exact cause remains provisional. Long lateral endurance, repeatability, braking distance and exact-angle turns remain unaccepted. |
| Shooting | [Recovered behaviors](behaviors/README.md) include stock-gait run-through, onboard detection and an experimental foot-kick candidate. The latest contact attempt did not reach the ball; placement was uncertain, and a subsequent marked no-ball run confirmed 30-inch travel. | Reliable contact and shot direction remain unproved. Further contact trials await the intended ball and calibrated geometry. [Earlier trial evidence](../foundation/results/stock-shot-session-20261116.json). |
| Gameplay | [Sandbox](pupper_fc/README.md) implements approach/alignment/strike, goal restart, disturbances, Stop and replay. The transport-free integration executor accepts fresh shared state and expiring per-robot intents; 191 tests plus 48 subtests pass. | Live robot transport, Ryan's actual overhead producer, opponent/defense and two-robot commissioning. |
| Navigation and local vision | Offline pose/heading control and shot staging exist. The older [follower](../follow_me_pupper/README.md) provides a reusable YOLOv8n-seg/Hailo detector and optional image-based obstacle heuristic. | No physically accepted coordinate control, tactical role selector, reactive goalie or person-aware soccer navigation. A front camera alone does not establish all-around coverage. Simulation geometry and motion limits need hardware-specific calibration. |
| Recovery/startup | Existing fall stopping and session tools are reusable. An explicit software Start candidate passes offline tests; two independent-review defects were corrected. | Follow-up review, resident caller integration and physical acceptance remain. Cold startup still homes on a stand; physical self-righting and FC power-on-to-ready remain unproved. |

Run `.venv-api/bin/python -m final_project.pupper_fc` from the active repository for the sandbox. Its contact/motion model is not physical validation. The [build/test handoff](ROBOT_BUILD_AND_TEST_HANDOFF.md) distinguishes the Ubuntu test container from Pi-compatible deployment binaries; source synchronization does not select the running release. [Policy comparison](../foundation/results/movement-readiness-20261116.json) and [Pi reconciliation](PI_RECONCILIATION.md) retain candidate/provenance details without assigning new work.

Current processes, authorization, pending recording archives and next actions belong only in the [checkpoint](../foundation/results/interruption-recovery.json). The [delivery plan](PLAN.md) holds the course deadlines and accepted progression and acceptance targets.

Reusable robot components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources.

From the repository root, run `python3 foundation/scripts/session_status.py` for dated task, branch and access state in [the session checkpoint](../foundation/results/interruption-recovery.json). Verify current ownership and authorization before hardware work.

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the [November 10 snapshot](https://github.com/ShrishPremkrishna/mts_123/blob/acd705d9207a024cdaf98c4dd368feba41a6ed4e/final_project/README.md).
