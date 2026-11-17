# Pupper FC

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. An overhead iPhone camera observes field/robot markers, a bright ground ball and two goals. Gameplay uses a 2D field model. The initial shot uses locomotion and a front bumper; foot kicking and a special goalie posture are stretch goals.

## Current agreement and owner steering — November 17, 2026

- Shrish currently owns movement/sprinting, locomotion-based headbutts, gameplay and integration. Tej iterates on the foot-kick policy. Ryan owns overhead perception, localization and proposed actions/shot targets; our harness translates those into responsive robot execution. Marcus’s existing gait exports remain available for comparison; the active ownership checkpoint governs robot access. Shrish builds the 8 × 12-foot arena with approximately 2-foot goals; no bumper is fitted yet.
- The owner accepted manual ball replacement and one deliberate resume cue after goals for the first working game; robots reposition themselves with software still running. Physical self-righting gets a bounded feasibility investigation before a demo commitment. Fall stopping, ordinary standing and physical get-up remain distinct capabilities.
- Two-robot soccer remains the target, with penalty shootout as fallback. On November 17 the owner prioritized coordinate/heading navigation, explicit offense/defense selection, measured latency, broader gait qualification and onboard obstacle avoidance before further contact trials with the wrong ball. The intended ball is pink, size 5, with flower patterns. Front-camera stop/hold/resume is sufficient; hybrid/all-around coverage and detours are not required. Ryan's pose feed is unavailable for several sessions, so independent offline work is authorized. Current process/access state is in the checkpoint.
- Power-on should prepare the selected stack automatically with minimal setup. A supported startup pose and one deliberate start/arm action are accepted; their implementation needs verification. Other game-rule decisions can wait.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposals, integration responsibilities and acceptance evidence. Working notes cover [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and evidence — November 17, 2026

Active development is in the `main` checkout, currently `/Users/shrish/workspace/cs123`. The sibling `cs123-foundation` checkout preserves earlier engineering work and its November 4 checkpoint; it is not the current FC handoff.

| Capability | What the evidence establishes | Remaining acceptance |
| --- | --- | --- |
| Movement | After removing per-step controller logging, three-second left/right floor tests at 0.15 m/s were stable and stopped cleanly; a marked 0.45 m/s, two-second forward repeat traveled roughly 30 inches. Earlier low-rate turns worked. [Trials and fall diagnosis](../foundation/results/gait-gate-20261116.json). | The original 1.32-second interruption's exact cause remains provisional. Long lateral endurance, repeatability, braking distance and exact-angle turns remain unaccepted. |
| Shooting | [Recovered behaviors](behaviors/README.md) include stock-gait run-through, onboard detection and an experimental foot-kick candidate. The latest contact attempt did not reach the ball; placement was uncertain, and a subsequent marked no-ball run confirmed 30-inch travel. | Reliable contact and shot direction remain unproved. Further contact trials await the intended ball and calibrated geometry. [Earlier trial evidence](../foundation/results/stock-shot-session-20261116.json). |
| Gameplay | [Executor](pupper_fc/README.md) connects leased automatic offense/defense, ball-facing guard motion, retained goal events and explicit goal resume. A deterministic two-robot replay exercises tracking/lease/camera loss, goals, resume and Stop. A resident per-robot bridge exercises actual ROS routing, expiry and holds against fake native feedback; concurrent domain tests kept one robot moving when the other stopped, expired or lost its guard. Separate mirrored ROS rehearsals naturally scored in the simulator, returned home and resumed with one cue. They use idealized one-robot physics, not a shared-ball match. [Overnight evidence](../foundation/results/overnight-execution-20261117.json). | Pi deployment/acceptance of the bridge, Ryan's actual overhead producer and clock conversion, physical defense and two-robot commissioning. Scripted referee events are not scored goals. |
| Defensive positioning | Ball-clear guard targets and transit correct reproduced goalward ball pushing, bumper contact during retreat, and sideline blocking. Independent replay verified 28 reachable cases without moving the ball; mirrored ROS rehearsals reached guard in about ten seconds using only recorded final velocity commands. Missing-ball and unreachable deep-ball cases hold. | This is an approximate-motion result. Physical defense, moving-ball interception and opponent clearance remain open. Some longer approaches exceed 15 seconds; the heuristic does not guarantee every reachable route. |
| Shared tracking and command age | Required-participant mode preserves original captures and holds on missing/stale tracks. A reproduced 491 ms stale-command tail led to an explicit direct FC command mode that preserves original deadlines through the camera guard to the native controller. Real guard/controller failure tests enforce expiry despite relay failure and replay; ROS bridge checks cover startup, native source binding, tracking loss and Stop. | The legacy Twist release retains receipt-based expiry. The new mode requires a target-compatible native rebuild and physical acceptance; it is not deployed. Track freshness and software expiry do not establish opponent clearance or physical braking. |
| Navigation and local vision | Feedback-controlled IMU orientation turns and capture-stamped camera envelopes are software candidates. The optional pinned launch routes the mux through the camera guard; source/process checks and cancellation behavior distinguish camera holds from native faults. Isolated ROS tests reach the actual controller timeout with memory hardware. Raw camera review exposed a missed tower obstacle. A larger offline model recognized it better but exceeded the live deadline on all 245 frames and introduced carpet false holds at lower confidence. | A frozen source kit and target-side assembly are prepared; these changes are not deployed. Physical pose control, turn accuracy and camera-based stopping remain unaccepted. Detector coverage, frame timing, thresholds and braking geometry need independent validation. The larger model and a tested camera crop are not selected for live use. An explicit, model-bound ball-contact guard mode is an offline candidate; pink-ball/person coverage remains open. |
| Recovery/startup | The persistent client has explicit software Start with no homing or automatic retry. Fable 5.2 and independent review exposed a native-only Stop that left gateway Start uncancelled. Both clients now request the gateway's latch as well as native damping. Actual ROS tests verify old-ticket rejection and corrective deactivation after a delayed activation; backend lifecycle is synthetic. | Pi-compatible release selection and physical acceptance remain. A gateway latch acknowledgement is not completed deactivation or physical stopping. Cold startup still homes on a stand; physical self-righting and FC power-on-to-ready remain unproved. |

The existing [finite trial client](behaviors/ROBOT_RUNBOOK.md#direct-deadline-qualification-candidate)
now supports the direct-deadline path without field poses. A device-free rehearsal
connected its real pulse, camera cancellation, fresh IMU turn and Stop to the actual
native controller with stock policy and memory interfaces. The gameplay bridge
also reached native execution and refused an old-source restart. These qualify
software connections, not physical movement or a complete game.

The [offline practice viewer](pupper_fc/README.md) now uses the real shared executor
for offense/defense, expiring permission, camera interruptions and deliberate goal
resume. Its importable rehearsal includes one naturally simulated goal and return;
the display identifies the observation used by each decision. Run
`.venv-api/bin/python -m final_project.pupper_fc` from the active repository.
Its contact/motion model is not physical validation.

The bounded [self-righting experiment](../artifacts/pupper-fc/2026-11-17/recovery-side-feasibility/experiment-summary.json)
closed after five physics runs. Standing and belly-to-standing controls passed;
both side attempts failed to establish the prescribed starting posture and did
not recover. No side-to-stand capability or native recovery policy is available.

The [build/test handoff](ROBOT_BUILD_AND_TEST_HANDOFF.md) distinguishes the Ubuntu test container from Pi-compatible deployment binaries; source synchronization does not select the running release. [Policy comparison](../foundation/results/movement-readiness-20261116.json) and [Pi reconciliation](PI_RECONCILIATION.md) retain candidate/provenance details without assigning new work.

Current processes, authorization, pending recording archives and next actions belong only in the [checkpoint](../foundation/results/interruption-recovery.json). The [delivery plan](PLAN.md) holds the course deadlines and accepted progression and acceptance targets. The [demo and recording script](DEMO.md) contains the 2:50 video cut, five-minute presentation, footage priorities and live fallback procedure. These are production drafts; filming and submission remain open.

Reusable robot components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources.

From the repository root, run `python3 foundation/scripts/session_status.py` for dated task, branch and access state in [the session checkpoint](../foundation/results/interruption-recovery.json). Verify current ownership and authorization before hardware work.

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the [November 10 snapshot](https://github.com/ShrishPremkrishna/mts_123/blob/acd705d9207a024cdaf98c4dd368feba41a6ed4e/final_project/README.md).
