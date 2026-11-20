# Pupper FC

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. Overhead perception estimates robot and ball positions in a calibrated field with known goal locations. The immediate demo needs one to two minutes of responsive play, useful goal-directed shots and automatic offense/defense. A bounded locomotion run-through is the first shot candidate; neither that nor a leg kick has accepted physical contact evidence, and no bumper is fitted.

## Current agreement and owner steering — November 19, 2026

The goals have arrived. The intended ball and second Pupper are still outstanding.
On November 19 the stock walker completed four one-second forward pulses at
0.2 m/s. Two from the same mark appeared to travel the same distance. The
operator taped one at about 8 in (20 cm). Another looked normal and had no
individual measurement. A two-second pulse toward 40 cm traveled an estimated
15 in (38 cm). Lateral offsets were unmeasured, so endpoint spread remains
unknown. The [straight-trial procedure](ROBOT_BUILD_AND_TEST_HANDOFF.md#first-camera-free-straight-trial)
includes independent x/y and heading measurements for repeats.

A relative +20° turn timed out after six seconds. The IMU reported +15.6°
at cutoff. The turn briefly entered tolerance but did not hold it for the
required 0.3 s. Combined turn-and-translate trials await a repeatable turn.
Shrish supported the robot while the driver deactivated, and the recorder
closed. [Physical trial receipts](../foundation/results/staged-field-session-20261119.json).

Ryan’s overhead producer and the adapter that converts its capture times to the
robot’s clock are integrated. On a direct local network, 9 of 16 clock probes
met the 20 ms uncertainty limit and 148 of 229 synthetic frames met the capture
clock budget. The earlier routed campus test admitted 2 of 16 probes. The
mounted camera still needs a sustained timing check on the arena network.
[Pi and network measurements](../foundation/results/staged-field-session-20261119.json).
The [camera setup and tag check](ROBOT_BUILD_AND_TEST_HANDOFF.md#mounted-overhead-camera-motors-inactive)
can proceed with a loose tag before a robot is available. The [tracked navigation test](PLAN.md#current-sprint--november-19)
uses the same target with measured robot poses once the mounted camera, timing,
field dimensions and Pi runtime have been qualified.

The newly supplied `tej/gemini-ball-kick` branch (`07ed0fd`) does contain kick
assets and operator tools, correcting the earlier branch inventory. Its broad
gate removal and reproduced motion/freshness/result defects prevent wholesale
integration. The extracted operator-only kick in place uses the existing
0.5-second initialization and 1.0-second fade. The staged Pi source exposed both
clips only under explicit skill selection. The [kick procedure](ROBOT_BUILD_AND_TEST_HANDOFF.md#operator-kick-after-the-straight-baseline)
covers the separate runtime selection, course clearance, witnessed playback
and walker restoration needed before ball contact. [Pi source verification](../artifacts/pupper-fc/2026-11-19/pi-deployment/verified.json).

- Shrish currently owns movement/sprinting, locomotion-based headbutts, gameplay and integration. Tej iterates on the foot-kick policy. Ryan owns overhead perception, localization and proposed actions/shot targets; our harness translates those into responsive robot execution. Marcus’s existing gait exports remain available for comparison; the active ownership checkpoint governs robot access. Shrish builds the 8 × 12-foot arena with approximately 2-foot goals; no bumper is fitted yet.
- The owner accepted manual ball replacement and one deliberate resume cue after goals for the first working game; robots reposition themselves with software still running. Physical self-righting gets a bounded feasibility investigation before a demo commitment. Fall stopping, ordinary standing and physical get-up remain distinct capabilities.
- Two-robot soccer remains the target, with penalty shootout as fallback. The owner prioritized coordinate/heading navigation, explicit offense/defense selection, measured latency, gait qualification and onboard obstacle avoidance before further contact trials with the wrong ball. The intended ball is pink, size 5, with flower patterns. Front-camera stop/hold/resume is sufficient. Mounted-field tracking and supervised camera-stop trials remain open. Current process/access state is in the checkpoint.
- Power-on should prepare the selected stack automatically with minimal setup. A supported startup pose and one deliberate start/arm action are accepted; their implementation needs verification. Other game-rule decisions can wait.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposals, integration responsibilities and acceptance evidence. Working notes cover [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and evidence — through November 19, 2026

Active development follows the dedicated `main` worktree. Use `git worktree list` to locate it when other projects share the repository. The sibling `cs123-foundation` checkout preserves earlier engineering work and its November 4 checkpoint; it is not the current FC handoff.

| Capability | What the evidence establishes | Remaining acceptance |
| --- | --- | --- |
| Overhead integration | Incoming source merged through `65f9f37`. Calibration faults latch; explicit measurement timestamps distinguish observation from coasting. A measured-clock UDP adapter emits normalized frames into the existing executor. Local rendered-image/UDP checks admitted frames and revoked after a camera bump. Two oblique tag cases failed in the earlier OpenCV 4.11 run. The full 70-test perception suite passed in the Mac camera environment with OpenCV 5.1. [Integration and kick review](../foundation/results/perception-harness-integration-20261119.json). | Sustained camera-to-Pi timing, selected ROS execution, mounted tag coverage, measured geometry and capture latency, and real coordinate arrival. Field/goal/ball placeholders currently differ across configs. |
| Movement | In the November 17 floor session after removing per-step controller logging, three-second left/right tests at 0.15 m/s were stable and stopped cleanly; a marked 0.45 m/s, two-second forward repeat traveled roughly 30 inches. Earlier timed turn pulses produced an IMU yaw change in the commanded direction. [Trials and fall diagnosis](../foundation/results/gait-gate-20261116.json). | The original 1.32-second interruption's exact cause remains provisional. Long lateral endurance, repeatability, braking distance and exact-angle turns remain unaccepted. |
| Shooting | [Recovered behaviors](behaviors/README.md) include stock-gait run-through, onboard detection and an experimental foot-kick candidate. The latest contact attempt did not reach the ball; placement was uncertain, and a subsequent marked no-ball run confirmed 30-inch travel. | Reliable contact and shot direction remain unproved. Further contact trials await the intended ball and calibrated geometry. [Earlier trial evidence](../foundation/results/stock-shot-session-20261116.json). |
| Gameplay | [Executor](pupper_fc/README.md) connects leased automatic offense/defense, ball-facing guard motion, retained goal events and explicit goal resume. A deterministic two-robot replay exercises tracking/lease/camera loss, goals, resume and Stop. A resident per-robot bridge exercises actual ROS routing, expiry and holds against fake native feedback; concurrent domain tests kept one robot moving when the other stopped, expired or lost its guard. Separate mirrored ROS rehearsals naturally scored in the simulator, returned home and resumed with one cue. They use idealized one-robot physics, not a shared-ball match. [Overnight evidence](../foundation/results/overnight-execution-20261117.json). | Run the bridge and measured-clock ingress together on the selected Pi runtime, qualify the mounted producer, then demonstrate physical defense and two-robot play. |
| Defensive positioning | Ball-clear guard targets and transit correct reproduced goalward ball pushing, bumper contact during retreat, and sideline blocking. Independent replay verified 28 reachable cases without moving the ball; mirrored ROS rehearsals reached guard in about ten seconds using only recorded final velocity commands. Missing-ball and unreachable deep-ball cases hold. | This is an approximate-motion result. Physical defense, moving-ball interception and opponent clearance remain open. Some longer approaches exceed 15 seconds; the heuristic does not guarantee every reachable route. |
| Shared tracking and command age | Required-participant mode preserves original captures and holds on missing/stale tracks. A reproduced 491 ms stale-command tail led to an explicit direct FC command mode that preserves original deadlines through the camera guard to the native controller. Real guard/controller failure tests enforce expiry despite relay failure and replay; ROS bridge checks cover startup, native source binding, tracking loss and Stop. | The legacy Twist release retains receipt-based expiry. The direct mode requires a target-compatible release, loaded identity checks and physical acceptance. [Target build and staging](../foundation/results/staged-field-session-20261119.json). Track freshness and software expiry do not establish opponent clearance or physical braking. |
| Navigation and local vision | The stock walker accepted the updated IMU turn client on the Pi. Its first physical turn timed out. Capture-stamped camera guards have offline coverage. The optional pinned launch routes the mux through the camera guard; source/process checks and cancellation behavior distinguish camera holds from native faults. Isolated ROS tests reach the actual controller timeout with memory hardware. Raw camera review exposed a missed tower obstacle. A larger offline model recognized it better but exceeded the live deadline on all 245 frames and introduced carpet false holds at lower confidence. | The legacy guarded walk is assembled on the Pi and awaits runtime and physical acceptance. Direct-deadline release selection is tracked in the [staging report](../foundation/results/staged-field-session-20261119.json). Physical pose control, turn accuracy and camera-based stopping remain unaccepted. Detector coverage, frame timing, thresholds and braking geometry need independent validation. The larger model and a tested camera crop are not selected for live use. An explicit, model-bound ball-contact guard mode is an offline candidate; pink-ball/person coverage remains open. |
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

The [build/test handoff](ROBOT_BUILD_AND_TEST_HANDOFF.md) covers Pi release selection, physical trial setup and the environment for camera calibration. [Policy comparison](../foundation/results/movement-readiness-20261116.json) and [Pi reconciliation](PI_RECONCILIATION.md) retain candidate/provenance details without assigning new work.

Current processes, authorization, pending recording archives and next actions belong only in the [checkpoint](../foundation/results/interruption-recovery.json). The [delivery plan](PLAN.md) holds the course deadlines and accepted progression and acceptance targets. The [demo and recording script](DEMO.md) contains the 2:50 video cut, five-minute presentation, footage priorities and live fallback procedure. These are production drafts; filming and submission remain open.

[Overhead perception](overhead_perception/README.md) estimates robot poses and ball motion in field coordinates. Goals come from configured geometry. The November 16 desk test read tags on a live iPhone feed and fitted the ball outline in one frame. The November 19 loopback exercised camera-bump revocation with rendered images. The real mount still needs whole-field tag coverage, measured geometry, lens and latency checks, and a pink-ball color range verified against the intended ball.

Reusable robot components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources.

From the repository root, run `python3 foundation/scripts/session_status.py` for dated task, branch and access state in [the session checkpoint](../foundation/results/interruption-recovery.json). Verify current ownership and authorization before hardware work.

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the [November 10 snapshot](https://github.com/ShrishPremkrishna/mts_123/blob/acd705d9207a024cdaf98c4dd368feba41a6ed4e/final_project/README.md).
