# Pupper FC

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. An overhead iPhone camera observes field/robot markers, a bright ground ball and two goals. Gameplay uses a 2D field model. The initial shot uses locomotion and a front bumper; foot kicking and a special goalie posture are stretch goals.

## Current agreement — November 16, 2026

- Shrish currently owns movement/sprinting, locomotion-based headbutts, gameplay and integration. Tej iterates on the foot-kick policy. Ryan owns overhead perception, localization and proposed actions/shot targets; our harness translates those into responsive robot execution. Marcus’s existing gait exports remain available for comparison; the active ownership checkpoint governs robot access. Shrish builds the 8 × 12-foot arena with approximately 2-foot goals; no bumper is fitted yet.
- The owner accepted manual ball replacement and one deliberate resume cue after goals for the first working game; robots reposition themselves with software still running. Physical self-righting gets a bounded feasibility investigation before a demo commitment. Fall stopping, ordinary standing and physical get-up remain distinct capabilities.
- Two-robot soccer remains the target, with penalty shootout as fallback. The next major result is a complete live play after a short measured gait-readiness gate. The owner authorized execution of this progression after the takeover investigation.
- Power-on should prepare the selected stack automatically with minimal setup. A supported startup pose and one deliberate start/arm action are accepted; their implementation needs verification. Other game-rule decisions can wait.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposals, integration responsibilities and acceptance evidence. Working notes cover [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and evidence — November 16, 2026

Active development is in the `main` checkout, currently `/Users/shrish/workspace/cs123`. The sibling `cs123-foundation` checkout preserves earlier engineering work and its November 4 checkpoint; it is not the current FC handoff.

| Capability | What the evidence establishes | Remaining acceptance |
| --- | --- | --- |
| Movement | Two 0.45 m/s, two-second requests each traveled roughly 30 inches, straight and stable. Low-rate turns worked. A three-second leftward test fell after a 1.32-second feedback interruption. [Trials and diagnosis](../foundation/results/gait-gate-20261116.json). | Resolve the control interruption before more floor testing. Lateral stability, braking distance and exact-angle turns remain unaccepted. |
| Shooting | [Recovered behaviors](behaviors/README.md) include stock-gait run-through, onboard detection and an experimental foot-kick candidate. Guards and coordinate/timestamp fixes pass offline checks. | Earlier shots grazed or were unstable; reliable contact and shot direction remain unproved. No contact trial on v3. [Earlier trial evidence](../foundation/results/stock-shot-session-20261116.json). |
| Gameplay | [Sandbox](pupper_fc/README.md) implements approach/alignment/strike, goal restart, disturbances, Stop and replay. The transport-free integration executor accepts fresh shared state and expiring per-robot intents; 191 tests plus 48 subtests pass. | Live robot transport, Ryan's actual overhead producer, opponent/defense and two-robot commissioning. |
| Recovery/startup | Existing fall stopping, session tools and controller checks are reusable. | Physical self-righting and FC power-on-to-ready are unimplemented. |

Run `.venv-api/bin/python -m final_project.pupper_fc` from the active repository for the sandbox. Its contact/motion model is not physical validation. The [build/test handoff](ROBOT_BUILD_AND_TEST_HANDOFF.md) distinguishes the Ubuntu test container from Pi-compatible deployment binaries; source synchronization does not select the running release. [Policy comparison](../foundation/results/movement-readiness-20261116.json) and [Pi reconciliation](PI_RECONCILIATION.md) retain candidate/provenance details without assigning new work.

Current processes, authorization, pending recording archives and next actions belong only in the [checkpoint](../foundation/results/interruption-recovery.json). The [delivery plan](PLAN.md) holds the course deadlines and accepted progression and acceptance targets.

Reusable robot components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources.

From the repository root, run `python3 foundation/scripts/session_status.py` for dated task, branch and access state in [the session checkpoint](../foundation/results/interruption-recovery.json). Verify current ownership and authorization before hardware work.

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the [November 10 snapshot](https://github.com/ShrishPremkrishna/mts_123/blob/acd705d9207a024cdaf98c4dd368feba41a6ed4e/final_project/README.md).
