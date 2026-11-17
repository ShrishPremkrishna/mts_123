# Pupper FC demo and recording

Working delivery script, November 17, 2026. The [delivery plan](PLAN.md#course-deliverables)
owns deadlines and acceptance targets. This is the production plan for the narrated
video and five-minute presentation. It is not a record of completed filming or
submission. The [brief](README.md#implementation-and-evidence--november-17-2026)
owns current capability claims.

The story is a complete soccer decision cycle: observe the field, choose a role,
reach a useful pose, act, stop when evidence or permission expires, and restart
after a goal. Show two opposing robots when qualified. Penalty shootout is the
agreed fallback; it still needs physical demonstration. An offline replay can
explain the controller but cannot replace the physical demo requirement.

## Narrated video cut

Target **2:50**, leaving ten seconds below the three-minute limit. Record narration
after the final footage and results are selected. Keep the accepted play sequence
continuous and at real speed; use labels for simulation, replay and any speed-up.
Do not splice a human repositioning out of a claimed autonomous sequence.

| Time | Picture | Spoken purpose |
| --- | --- | --- |
| 0:00–0:12 | Best continuous physical play excerpt, with both robots visible; title and team names | State the soccer objective. This clip is still needed. |
| 0:12–0:34 | Actual arena view plus a simple field-to-command diagram | Explain Ryan's tracking, Shrish's execution and Tej's kick work. |
| 0:34–0:56 | Real role/action display alongside field view; labeled offline rehearsal until physical integration is recorded | Explain offense versus defense and the pose target. |
| 0:56–1:30 | Continuous approach, alignment, shot, goal return, visible ball replacement and resume | Demonstrate the complete play. If it takes longer, borrow time from explanation; do not conceal an intervention. |
| 1:30–1:54 | Qualified front-camera interruption with wide external view and corresponding hold/zero evidence | Show actual stopping and resumption. Current stationary camera footage alone cannot fill this claim. |
| 1:54–2:18 | Short failure/repair explanation with native timing evidence | Explain the logging stall reproduction and why source-age deadlines matter. |
| 2:18–2:38 | Measured trial results: attempts, goals, interventions, falls, stopping/heading error | Report the full trial set, including misses. Physical numbers are pending. |
| 2:38–2:50 | Team/contributions, repository location, one concrete remaining limitation | Close without claiming get-up or collision avoidance beyond the demonstrated coverage. |

The following narration is accurate for the **current engineering draft**. After
physical qualification, replace the explicitly pending-results paragraph with
the measured result and retain the footage's scope. Do not read editing notes.

> Pupper FC is our project to make two quadruped robots play soccer. Each robot
> should choose offense or defense, move into position, and react when the field changes.
>
> Ryan is developing overhead tracking of the robots, ball and goals. Shrish owns
> movement and integration: turning that field state into short, replaceable motion
> commands. Tej is developing a leg-kick policy. Our initial shooting path uses locomotion.
>
> On offense, the controller approaches a point behind the ball, aligns toward the
> goal, makes a bounded strike, then brakes and reassesses. On defense, it seeks a
> goal-side position facing the ball. Different switching thresholds keep the role
> from changing on every observation.
>
> A role does not automatically permit movement. Old observations, expired
> instructions, an obstacle hold, or Stop can interrupt the action. After a goal,
> the robot returns home and waits while a person replaces the ball and gives a
> deliberate resume cue.
>
> One major engineering problem was delayed control. We reproduced a logging path
> that could block native controller updates, removed it, and observed stable short
> lateral repeats. We also built a command path that preserves the original expiry
> through the guard and controller. Its software tests do not measure physical braking.
>
> Today, short physical forward and lateral trials are verified. The shared executor
> has also completed a labeled simulation of attack, defense, interruptions, scoring
> and restart. Physical camera stopping, coordinate-controlled play and the two-robot
> match still need qualification. Our bounded self-righting experiment did not establish
> side recovery.
>
> The final demonstration must show those connections working together, with every
> intervention counted. Our contribution is the integration of soccer decisions with
> responsive, interruptible robot execution.

## Five-minute presentation

Seven screens, including the live view. The five-minute limit includes the live
demo. Full acceptance trials happen beforehand; three two-minute bouts cannot fit
inside this slot. The names below indicate current ownership, not an imposed
speaking assignment.

| Slot | Screen title and on-screen content | Presenter cue |
| --- | --- | --- |
| 0:00–0:25 | **Pupper FC** — Two robots, opposing goals, autonomous roles | State the objective and allowed human ball reset. Open with the best qualified physical clip if staging is not already visible. |
| 0:25–1:05 | **Field state to motion** — Overhead tracking → role/pose controller → expiring commands → gait. Front camera interrupts motion. | Explain Ryan/Shrish/Tej boundaries. Explain that Tej's leg kick is a separate candidate unless selected and demonstrated. |
| 1:05–1:45 | **Roles and interruptions** — Offense: approach → align → strike → reassess. Defense: guard goal-side, face ball. Goal: home → reset cue. | Use the [implemented state diagram](pupper_fc/README.md#shared-state-executor-and-deliberate-game-resume). Distinguish role from permission. |
| 1:45–2:20 | **Keeping commands current** — Original expiry survives relays. Stop cancels authority. Observation age stays visible. | Explain one reproduced failure and its verified repair. Never turn a software expiry bound into a braking claim. |
| 2:20–4:10 | **Live play** — Field view plus role/action/hold reason | Run one rehearsed complete play and restart. Show defense or a qualified person interruption only if the time/space budget supports it. |
| 4:10–4:45 | **Measured results** — Attempts, goals, interventions, falls; motion/stopping error | Show the full predeclared trial set and physical-versus-simulation labels. Do not substitute a passing-test count for robot performance. |
| 4:45–5:00 | **What Pupper gained** — Reusable gameplay execution; tested interfaces; remaining limitation | Credit contributors and point to installation/run instructions. End at five minutes. |

## Minimum footage to collect

The [offline executor replay video](../artifacts/pupper-fc/2026-11-17/deliverable-prep/replay-video/pupper-fc-offline-executor-replay.mp4)
is ready as an explanatory asset: 1080p, 20.45 seconds, including a labeled
one-second ending hold. It preserves the recorded simulation time, visible manual
inputs, role/action/reason and input age. It is not physical footage. The
[source and timing record](../artifacts/pupper-fc/2026-11-17/deliverable-prep/replay-video/metadata.json)
documents the nine superseded zero-duration records and exact frame attribution.

Use one wide camera showing the robot, ball, goal, floor markers and supervisor's
interventions. Preserve the continuous original with audio. Align it to the
existing run log with a visible/verbal trial identifier; record the selected
release/profile in the run receipt. No new capture framework is needed.

| Priority | Capture and observation | What it establishes |
| --- | --- | --- |
| 1 | Marked forward/backward/lateral travel, measured heading target and stop; include failures | The usable movement envelope on the actual surface. Current forward/lateral telemetry lacks an external-view film. |
| 1 | Fixed onboard camera at measured walking height/pitch: clear floor, person at marked ranges/offsets, intended pink ball, representative obstacle; separate holdout sequence | Detector coverage and blind spots. This is stationary perception evidence until moving stopping tests pass. |
| 1 | Qualified motion with the person/obstacle already in a tested stopping corridor; film stop, clear condition and fresh resumption | Physical front-camera stop/hold behavior and measured stopping margin. Plan the range from prior braking measurements. |
| 1 | One continuous pose-controlled approach → alignment → useful ball contact; goal → return/reset/resume, miss → brake/reassess | First complete physical play, with the actual ball and perception feed. Keep failed attempts. |
| 2 | Two opposing robots in a shared field, visible role changes and defense, one operator goal-resume cue | The target match behavior. Separate one-robot mirrored clips do not establish it. |
| 2 | Three full two-minute bouts after qualification, all interventions counted | Demo reliability under the agreed acceptance target. Extract an excerpt for the short video without discarding the full bouts. |
| 3 | Tej's kick or recovery only after its own qualification | Optional capability, separately identified from the base gait/headbutt demo. No side-recovery footage is currently supported. |

Retain the existing motor-inactive onboard recording as perception-development
material. It shows the room from the camera, not Pupper avoiding a person. The
[artifact evidence inventory](../artifacts/pupper-fc/2026-11-17/deliverable-prep/evidence-inventory.json)
links source media and plots; do not re-upload personal
room footage or publish raw logs merely to prepare the presentation.

## Live demo sequence and fallback

Before the presentation, select and qualify the complete release, home with support,
confirm the actual camera/field feed, and establish a known safe starting pose.
The [build handoff](ROBOT_BUILD_AND_TEST_HANDOFF.md) and
[movement runbook](behaviors/ROBOT_RUNBOOK.md) own operational commands.
Do not use the five-minute slot for dependency repair, build selection or homing.

1. Announce the active mode and allowed human reset. Show the field and role/action view.
2. Give one deliberate play activation; let the controller navigate and act.
3. After a goal, show return home, human ball replacement and one deliberate cue.
   **Open integration item:** each bridge currently accepts its own resume. A single
   human cue reaching both robots, with visible per-robot acceptance/refusal, still
   needs implementation/rehearsal. Do not describe two manual cues as one.
4. If a hold occurs, identify its actual reason. Wait only within the pre-rehearsed
   slot. Do not clear Stop or repeat Start to hide an unresolved fault.
5. If live execution cannot continue, stop the owned session and show a previously
   qualified physical clip. Name the live failure plainly. A penalty shootout is
   available only after that mode is physically qualified; it is not yet a ready fallback.

At the final rehearsal, choose the longest live sequence that repeatedly fits
the 110-second segment. If it does not fit, shorten the explanation or demonstrate
one shot and use recorded evidence for return/defense. Keep the full soccer goal
and its separate acceptance trials intact.

## Results and publication

Use the existing per-trial receipts and operator observations to populate results;
record attempts as well as successes. For a match, count goals for each robot,
misses, unexpected contacts, falls, operator interventions, and reset duration.
For motion/camera qualification, report commanded versus measured displacement,
heading error and stopping distance with capture conditions. No percentage is
credible without the denominator and trial setup.

The December 9 write-up should explain the problem, team contributions, architecture,
one consequential failure/repair, experiment methods/results, limitations and how
to reproduce the selected release. The course has not specified a more exact
wrap-up format. Final repository URL, credits, results and submissions remain to
be confirmed; preparing this script does not complete those deliverables.
