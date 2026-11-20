# Pupper FC

## Pupper FC appliance release · November 20, 2026

Open **Pupper FC.command** and connect the field camera and a team. A cold team connection prepares the session and prompts for supported Home. Use the physical Stop gamepad and fresh standing cue to activate the walker. A warm connection skips Home. Start match admits Blue alone, Red alone or both. End match keeps healthy walkers standing.

The Pi release includes a reversible headless boot selection, a measured-corner alternative to the six floor tags, signed team-color tracking with physical patches and selectable onboard pink-ball telemetry. Strict floor sessions and the supported damping experiment have different motor behavior after IMU loss. The release record describes the recordings, compiled tests, limits and first physical check. [Operator release](pupper_fc/RELEASE.md) · [Pi installation and rollback](pupper_fc/PACKAGING.md)

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. Overhead perception estimates robot and ball positions in a calibrated field with known goal locations. The immediate demo needs one to two minutes of responsive play, useful goal-directed shots and automatic offense/defense. The demo shot uses Tej’s leg-kick policy. The combined single-robot release steers using a measured field pose, hands the legs to the selected kick, and restores walking. Both feet have scored in simulation. The Pi completed walk–kick–walk for each foot on memory-backed joints, then latched Stop and deactivated both leg controllers. The physical robot homed on the stand with the verified build loaded. At standby both motion controllers were inactive and the gamepad, camera and IMU were live. A later supervised session observed walker activation and Stop on the stand. The rebuild needs the same check with its own target binary. Tej owns physical kick qualification, and mounted field tracking remains open. [Pi qualification](../foundation/results/pi-combined-walk-kick-20261119.json)

On November 20, Pupper completed stand recovery after a stale balance-sensor reading disabled its driver. During the final west-goal attempt, the overhead camera produced 950 paired robot-and-ball observations. Intermittently stale front-camera images kept the offense runner from arming. The recording contains only zero-velocity requests and no kick. The operator supported Pupper for shutdown. [Final field trial](../foundation/results/field-offense-final-20261120.json)

The final ROS rehearsal verified solo Blue, solo Red and paired kickoff, standby, scoring and deliberate Resume. On the earlier matchday release, warm kickoff reached the first nonzero executor command in 0.35 s for Blue, 0.41 s for Red, and 2.69 s for the pair in individual ROS rehearsals. The field robot still needs its updated driver, mounted-camera and motion acceptance. [Current release evidence](../foundation/results/fc-matchday-release-20261120.json) · [Native driver evidence](../foundation/results/fc-native-reliability-20261120.json)

## Current agreement and owner steering — November 19, 2026

Shrish and the project lead own the single-robot software loop. Tej owns physical kick and recovery trials. Ryan’s mounted camera and the second robot are later blocks. The [current software rehearsal](pupper_fc/README.md#one-robot-software-rehearsal) exercises continuous metric position and heading targets, speed changes, slowing into either paw, single-shot handoff and resumption of walking. [Watch the simulated approach at three speeds](pupper_fc/media/three-speed-approach.mp4).

The goals and pink ball have arrived. Physical qualification of the second Pupper remains open.
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
uses the same target with measured robot poses once the mounted camera, timing
and field dimensions have been qualified.

Tej’s `tej/gemini-ball-kick` branch advanced to `30b84a0`. The Pi logged u5 walk–kick–walk jobs of 5.76/5.63 s for left/right, then
4.98/5.03 s with the shorter settings. Ball contact was unmeasured.
The combined release uses the conservative transition timings. Tej owns qualifying the faster return, ball contact and a real step after each kick. The [shooting handoff](handoffs/SHOOTING.md) records the
timings, evidence limits and the integration decision. Mounted tracking follows when the field is ready. [Testing order](PLAN.md#current-sprint--november-19).

- Shrish currently owns movement, shot positioning, gameplay and integration. Tej iterates on the foot-kick policy. Ryan owns overhead perception, localization and proposed actions/shot targets; our harness translates those into responsive robot execution. Marcus’s existing gait exports remain available for comparison; the active ownership checkpoint governs robot access. Shrish builds the 8 × 12-foot arena with approximately 2-foot goals; no bumper is fitted yet.
- The owner accepted manual ball replacement and one deliberate resume cue after goals for the first working game; robots reposition themselves with software still running. Physical self-righting gets a bounded feasibility investigation before a demo commitment. Fall stopping, ordinary standing and physical get-up remain distinct capabilities.
- Two-robot soccer remains the target, with penalty shootout as fallback. The owner prioritized coordinate/heading navigation, explicit offense/defense selection, measured latency, gait qualification and onboard obstacle avoidance before further contact trials with the wrong ball. The intended ball is pink, size 5, with flower patterns. Front-camera stop/hold/resume is sufficient. Mounted-field tracking and supervised camera-stop trials remain open. Current process/access state is in the checkpoint.
- Power-on should prepare the selected stack automatically with minimal setup. A supported startup pose and one deliberate start/arm action are accepted; their implementation needs verification. Other game-rule decisions can wait.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposals, integration responsibilities and acceptance evidence. Working notes cover [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and evidence — through November 19, 2026

Active development follows the dedicated `main` worktree. Use `git worktree list` to locate it when other projects share the repository. The sibling `cs123-foundation` checkout preserves earlier engineering work and its November 4 checkpoint; it is not the current FC handoff.

| Capability | What the evidence establishes | Remaining acceptance |
| --- | --- | --- |
| Overhead integration | Incoming source merged through `65f9f37`. Calibration faults latch; explicit measurement timestamps distinguish observation from coasting. A measured-clock UDP adapter emits normalized frames into the existing executor. Local rendered-image/UDP checks admitted frames and revoked after a camera bump. Two oblique tag cases failed in the earlier OpenCV 4.11 run. The full 70-test perception suite passed in the Mac camera environment with OpenCV 5.1. [Integration and kick review](../foundation/results/perception-harness-integration-20261119.json). | Sustained camera-to-Pi timing, mounted tag coverage, measured geometry and capture latency, and real coordinate arrival. Field/goal/ball placeholders currently differ across configs. |
| Movement | In the November 17 floor session after removing per-step controller logging, three-second left/right tests at 0.15 m/s were stable and stopped cleanly; a marked 0.45 m/s, two-second forward repeat traveled roughly 30 inches. Earlier timed turn pulses produced an IMU yaw change in the commanded direction. [Trials and fall diagnosis](../foundation/results/gait-gate-20261116.json). | The original 1.32-second interruption's exact cause remains provisional. Long lateral endurance, repeatability, braking distance and exact-angle turns remain unaccepted. |
| Shooting | The combined release selects one stock walker and Tej’s left/right leg-kick clips. MuJoCo measures which paw first touches the ball and how far the restored walker moves. A separate Linux ROS test checks command ownership through native deactivation and reactivation. [Physics results](../foundation/results/combined-walk-kick-physics-20261119.json) · [Native wire results](../foundation/results/combined-walk-kick-native-20261119.json). | Tej qualifies physical kicks with both feet, measures the real contact window, and checks the first step after each kick. The faster transitions from his branch remain a separate candidate. |
| Gameplay | [Executor](pupper_fc/README.md) connects leased automatic offense/defense, ball-facing guard motion, retained goal events and explicit goal resume. A deterministic two-robot replay exercises tracking/lease/camera loss, goals, resume and Stop. A resident per-robot bridge exercises actual ROS routing, expiry and holds against fake native feedback; concurrent domain tests kept one robot moving when the other stopped, expired or lost its guard. Separate mirrored ROS rehearsals naturally scored in the simulator, returned home and resumed with one cue. They use idealized one-robot physics, not a shared-ball match. [Overnight evidence](../foundation/results/overnight-execution-20261117.json). | Run the bridge and measured-clock ingress together on the selected Pi runtime, qualify the mounted producer, then demonstrate physical defense and two-robot play. |
| Defensive positioning | Ball-clear guard targets and transit correct reproduced goalward ball pushing, bumper contact during retreat, and sideline blocking. Independent replay verified 28 reachable cases without moving the ball; mirrored ROS rehearsals reached guard in about ten seconds using only recorded final velocity commands. Missing-ball and unreachable deep-ball cases hold. | This is an approximate-motion result. Physical defense, moving-ball interception and opponent clearance remain open. Some longer approaches exceed 15 seconds; the heuristic does not guarantee every reachable route. |
| Shared tracking and command age | Required-participant mode preserves original captures and holds on missing/stale tracks. A reproduced 491 ms stale-command tail led to an explicit direct FC command mode that preserves original deadlines through the camera guard to the native controller. Real guard/controller failure tests enforce expiry despite relay failure and replay; ROS bridge checks cover startup, native source binding, tracking loss and Stop. | The legacy Twist release retains receipt-based expiry. The Pi loaded the verified direct-mode build. Physical braking and camera-stop response still need measurement. [Pi qualification](../foundation/results/pi-combined-walk-kick-20261119.json). Track freshness and software expiry do not establish opponent clearance or physical braking. |
| Navigation and local vision | The stock walker accepted the updated IMU turn client on the Pi. Its first physical turn timed out. Capture-stamped camera guards have offline coverage. The optional pinned launch routes the mux through the camera guard; source/process checks and cancellation behavior distinguish camera holds from native faults. Isolated ROS tests reach the actual controller timeout with memory hardware. Raw camera review exposed a missed tower obstacle. A larger offline model recognized it better but exceeded the live deadline on all 245 frames and introduced carpet false holds at lower confidence. | The legacy guarded walk is assembled on the Pi and awaits runtime and physical acceptance. The combined direct-deadline release is loaded on the Pi. [Pi qualification](../foundation/results/pi-combined-walk-kick-20261119.json). Physical pose control, turn accuracy and camera-based stopping remain unaccepted. Detector coverage, frame timing, thresholds and braking geometry need independent validation. The larger model and a tested camera crop are not selected for live use. The combined release uses the model-bound ball-contact guard. Turning off preview masks kept camera data fresh for all 286 guard checks in a stationary minute. Pink-ball/person coverage and physical stopping distance remain open. |
| Recovery/startup | The persistent client has explicit software Start with no homing or automatic retry. Fable 5.2 and independent review exposed a native-only Stop that left gateway Start uncancelled. Both clients now request the gateway's latch as well as native damping. Actual ROS tests verify old-ticket rejection and corrective deactivation after a delayed activation; backend lifecycle is synthetic. | The combined release verified both gateway latching and controller deactivation on Pi memory joints. On Nov 20 the physical robot homed on its stand, activated the walker alone, and acknowledged Stop. Both leg controllers became inactive. Physical self-righting remains unproved. |

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

The mounted iPhone calibrated from the corner tags and simultaneously measured the striker tag and the intended pink ball. The real-time preview also exposed a focus failure that hid all six floor tags. Tape-referenced scale, goal placement, motion accuracy and capture latency remain open. [Mounted field observation](../foundation/results/field-camera-live-20261120.json)

Reusable robot components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources.

From the repository root, run `python3 foundation/scripts/session_status.py` for dated task, branch and access state in [the session checkpoint](../foundation/results/interruption-recovery.json). Verify current ownership and authorization before hardware work.

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the [November 10 snapshot](https://github.com/ShrishPremkrishna/mts_123/blob/acd705d9207a024cdaf98c4dd368feba41a6ed4e/final_project/README.md).
