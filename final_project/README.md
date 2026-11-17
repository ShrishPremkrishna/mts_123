# Pupper FC

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. An overhead iPhone camera observes field/robot markers, a bright ground ball and two goals. Gameplay uses a 2D field model. The initial shot uses locomotion and a front bumper; foot kicking and a special goalie posture are stretch goals.

## Current agreement — November 16, 2026

- Shrish currently owns movement/sprinting, locomotion-based headbutts, gameplay and integration. Tej iterates on the foot-kick policy. Ryan owns overhead perception, localization and action/shot geometry. Marcus’s existing gait exports remain available for comparison; the active ownership checkpoint governs robot access. Shrish builds the arena.
- Autonomous fall detection, physical self-righting and resumption are separate P0 work required for the base project. Physical get-up ownership is open. The ordinary stand/restart path is not get-up. A first gameplay baseline can develop while recovery remains unfinished; recovery acceptance stays open.
- After goals, a person may replace the ball and give one resume signal. Software stays running and robots manage restart positioning. Routine rehoming or manual robot repositioning between plays is outside the intended base demo.
- Power-on should prepare the selected stack automatically with minimal setup. A supported startup pose and one deliberate start/arm action are accepted; their implementation needs verification. Other game-rule decisions can wait.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposals, integration responsibilities and acceptance evidence. Working notes cover [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and access

November 16: [recovered Pi shooting/perception experiments](behaviors/README.md) now include a stock-gait run-through controller, ROS adapters, onboard ball detection, tests and an experimental learned-kick policy. [Reconciliation findings](PI_RECONCILIATION.md) distinguish real trial receipts from unverified contact outcomes and identify integration blockers. Overhead field tracking is still missing from inspected source. These recovered nodes are not automatically enabled or connected to the gameplay prototype.

Offline corrections now check stock policy/configuration and command limits before shots, abort on controller changes, and preserve ball-coordinate frame/capture-time validity. Fake-robot ROS and regression checks support these changes; measured stock-gait contact, detector calibration and live gameplay integration remain open. The first physical baseline uses the existing stock gait and manual ball coordinates, before specialized policy training. The initial November 16 trials produced a graze and an unstable/unclear offset approach; they did not establish a working shot. An older installed driver was discovered and its shutdown crashed. The [session evidence](../foundation/results/stock-shot-session-20261116.json) separates those observations from possible causes.

A bounded [eight-actor simulation comparison](../foundation/results/movement-readiness-20261116.json) supports testing stock forward speed/braking and Markus’s ablated actor for lateral motion before new training. New receiver-side command expiry and a [short instrumented trial loop](behaviors/ROBOT_RUNBOOK.md) pass offline checks. The separate Pi-native movement release is staged; its new combination still needs supervised acceptance. The [build/test handoff](ROBOT_BUILD_AND_TEST_HANDOFF.md) explains shared ownership and prevents source/build/runtime confusion. Onboard temporary-ball tracking is secondary to movement and does not replace the final overhead tracker.

The [offline approach-and-shot prototype](pupper_fc/README.md) now runs a Python gameplay controller in an interactive 2D sandbox. It supports approach/alignment/strike, goal return and restart, tracking disturbances, Stop and recorded replay. Run `.venv-api/bin/python -m final_project.pupper_fc` from the repository root. The observation, velocity-command and strike boundaries are documented for teammate integration. This is not a real soccer runtime or get-up implementation; simulated contact and motion do not establish physical performance.

Reusable robot components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources.

From the repository root, run `python3 foundation/scripts/session_status.py` for dated task, branch and access state in [the session checkpoint](../foundation/results/interruption-recovery.json). Verify current ownership and authorization before hardware work.

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the [November 10 snapshot](https://github.com/ShrishPremkrishna/mts_123/blob/acd705d9207a024cdaf98c4dd368feba41a6ed4e/final_project/README.md).
