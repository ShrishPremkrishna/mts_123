# Pupper FC delivery plan

Updated November 17, 2026. Confirmed outcomes guide this plan. Rules and implementation proposals below remain open to discussion. The November 17 owner steering below supersedes the immediate contact-first progression from November 16. Use this as project context when choosing work with the current user, not as an automatic task assignment. Current activity and robot access are in the [checkpoint](../foundation/results/interruption-recovery.json). This plan does not claim soccer implementation or hardware acceptance.

## Outcome and ownership

Two Puppers sustain offensive and defensive play using overhead vision and a 2D field model. The working base includes locomotion, headbutt shooting, tracking, autonomous positioning and goal restarts. The owner accepted manual ball replacement and one deliberate resume cue for the first working game. Self-righting receives a bounded feasibility investigation before commitment to the demo. Foot kicking and a special goalie pose are stretch goals. Ball-in-simulation training is a later experiment, not a prerequisite for the first working game.

| Owner | Outcome |
| --- | --- |
| Shrish — [movement notes](handoffs/MOVEMENT.md) | Taking over controllable forward/lateral motion, turning, braking, stability and locomotion-based headbutts. Existing teammate gait exports remain candidates; measured limits are still needed. |
| Tej — [shooting](handoffs/SHOOTING.md) | Foot-kick policy, with known entry conditions, contact and exit behavior. Headbutt baseline now belongs to Shrish. |
| Ryan — [perception](handoffs/PERCEPTION.md) | iPhone ingestion, field coordinates, persistent robot identities/headings, ball position/velocity, goals, shooting geometry and observation age/validity. |
| Shrish, assisted by agents — gameplay/integration | Offense/defense selection, approach execution, command ownership, interruption, recovery coordination, startup, operator view and whole-game validation. |
| Shrish — arena | Camera mounting, boundaries/goals, markers, ball, surface and bumper arrangement for repeatable trials. Coordinate dimensions and markers with Ryan. |

These are the team's current areas, not fixed roles for incoming agents. Get-up was previously P0; the accepted takeover recommendation is a bounded feasibility investigation before committing it to the demo. Physical-motion ownership remains to be assigned. Shrish and assisting agents own its gameplay integration. The first playable prototype can advance while recovery is open; it must not be described as having solved recovery. Technical boundaries may change jointly without dropping an outcome.

## Course deliverables

Checked against the [live fall-26 directions](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/final_project.html) on November 16, 2026. Dates below use the course's local Pacific time. Submission/checkoff completion needs owner confirmation; source-code progress does not establish submission.

| Due | Deliverable | Known status |
| --- | --- | --- |
| November 6 | Proposal (20%) | Completion and TA feedback not recovered. |
| November 16 | Progress checkpoint with TA/instructor (20%) | Completed, confirmed by owner during takeover. |
| December 2, 11:59 p.m. | Narrated demo video under 3 minutes (20%) | Not produced; capture representative successes and failures during development. |
| December 3, 6:30 p.m., CoDA B90 | Five-minute presentation including live demo (30%); documented GitHub repository/install instructions | Open. Reserve rehearsal and two-robot access; retain a runnable, filmed fallback. |
| December 9, 11:59 p.m. | Code/blog/other wrap-up (10%) | Open; exact linked wrap-up/rubric details still need verification. |

No late days. The saved rubric emphasizes innovation, reliable execution, presentation and contribution/collaboration. Do not invent observations, personal contributions or checkoffs.

## Execution agreement — November 16, 2026

The owner confirmed two-robot soccer as the target and a complete live play before further gait optimization, conditional on proving the existing gait is adequate. Penalty shootout remains fallback. For the first working game, a person replaces the ball and gives one deliberate resume cue while robots reposition themselves. Physical self-righting gets a bounded feasibility investigation before a demo commitment; fall stopping remains required. A complete camera-to-robot play is the next major result. More speed, new policies and temporary onboard perception are valuable only when they remove a measured obstacle to that result.

Owner accepted the recommendations and progression during the takeover discussion. The targets below guide acceptance; they are not achieved results:

- Repeatable core: measured forward/brake, turn-to-heading and lateral trials; choose usable limits from observed error, wobble and stopping distance on the actual arena surface.
- One-robot play: useful contact in at least 8 of 10 predefined arrangements; five consecutive goal/reset/resume sequences without restarting software or repositioning the robot by hand.
- Full demo: three consecutive two-minute two-robot bouts with autonomous offense/defense and only agreed ball resets. Report all interventions, goals, failed shots and falls.
- Recovery, if retained: demonstrate self-righting and fresh-play resumption from agreed representative postures, retaining failed attempts and duration. Set the success threshold after a bounded feasibility trial, before acceptance.
- Delivery: reproducible startup, a tested fallback release, recorded evidence, narrated video, concise presentation, installation instructions and honest contributions/results write-up.

Work advances by evidence: resolve the observed persistent-client idle freshness failure offline; connect a synchronized perception sample to gameplay and replay; measure the smallest physical envelope needed for a complete play; then extend defense and second-robot integration. Recovery feasibility can run independently after its scope is agreed. Start defense with goal-side ball tracking and bounded lateral positioning; add prediction only if measured shots expose a need. Ryan supplies field state and candidate actions/targets; our harness owns execution, interruption and low latency, while Tej develops foot kicking. An unavailable camera or robot routes work to recorded/synthetic inputs, interface integration, recovery simulation or deliverables rather than idle waiting.

Each experiment states its question, comparison, observable result and stopping limit before it runs. Prepare dependencies, builds and rehearsals before requesting physical time. Aim for less than a minute between prepared trials in one homed session; measure the interval and fix a recurring delay once. Stop a failing path after it no longer separates plausible causes. Keep one owner per file/robot; use bounded independent reviews for uncertain faults and verify the integrated checkout after edits freeze.

Use the existing checkpoint for active owners, exact next action, verified commands, failures and pending archives. Use this plan for outcomes/decisions and the brief for capabilities; replace superseded claims. The current chat's active goal and bounded delegates support continued execution. No scheduled task, cloud compute, remote robot monitor or background motion is implied. A chat/platform interruption still requires a resumable checkpoint; repository edits alone cannot cure UI latency.

## Game behavior and proposed rules

Offense selects a target, approaches the useful side of the ball, aligns the bumper, strikes, brakes and reassesses. Defense predicts a threatened goal crossing and seeks a position the robot can actually reach. Selection considers threat, opportunity and reachability, with enough persistence to prevent oscillation. Proximity alone is an initial heuristic, not a complete strategy.

Manual ball replacement and one deliberate resume cue are accepted for the first working game; detailed game rules remain adjustable. A missed shot, moving ball or blocked route causes reassessment rather than stale timed actions. Geometry and heuristics can establish a baseline; learned decision-making remains an option. Proposed placement: local gameplay handles time-sensitive control, while voice sets intent and explains state.

- Fixed opposing goals and robot identities. Both robots can attack and defend. Shared observations need not expose each robot's intended action to its opponent.
- Define goal-mouth crossing using measured goal geometry and ball radius; count each goal once.
- After a goal, cancel the old play, reach valid restart positions and wait while a person replaces the ball. Resume without restarting software or homing. One deliberate resume signal after the field is clear is accepted for the first version; the precise interaction can be chosen downstream.
- Replan for stationary and boundary-adjacent balls. Decide how to handle out-of-bounds or physically trapped balls; routine human resets beyond goals are not agreed.
- Avoid deliberate robot collisions. Boundary and opponent clearance must accommodate measured braking and recovery space.
- Tracking loss, controller faults and physical obstruction are explicit states. Exceptional human interventions count against reported autonomy.

## Physical recovery feasibility

If the feasibility study supports committing physical get-up, the required chain is: detect a fall, cancel the old action, acquire exclusive recovery control, physically self-right, verify stable standing and localization, then select a fresh play. Ordinary stand, controller reactivation or clearing a tilt latch does not satisfy it. Stop must interrupt recovery; reconnect must not replay old attacks. Distinguish a recoverable fall from operator Stop, stale sensors, low power and hardware faults.

The owner accepted a bounded feasibility investigation before committing physical get-up to the demo. Assign a physical-motion owner and compare a small set of representative postures; do not promise broad coverage from an upright restart. Catalogue expected postures on the arena surface with the bumper, including both sides and any reachable back/belly configurations. Compare suitable motion sequences or learned policies in simulation, then supervised physical trials. Record successes, duration, retries and failure postures. Unsolved expected falls keep the recovery deliverable open; do not redefine them away after failure. Entrapment or damaged hardware may require exceptional assistance. Unbounded retries do not establish recovery.

## Minimum integration agreement

Agree these meanings at the first connection and version changes with affected owners. Schemas, libraries and transport remain open.

- Field axes, metric units, heading convention, body-to-field conversion, robot IDs and goal ownership. Convert explicitly at existing ROS/tool boundaries.
- Capture time, clock basis and observation age; measured, predicted and missing states. Include robot footprint, bumper contact offset and ball radius. Camera distortion and elevated marker effects need calibration even with 2D gameplay.
- Skill entry conditions, limits, interruption and completion meanings. Navigation targets must be replaceable rather than accumulating in a queue.
- One effective motion owner per robot across gameplay, strike, recovery, voice and joystick; Stop has priority. Commands identify their robot.
- Robot-local expiration of stale motion intent covering lost camera, laptop, network or publisher. An upstream timeout cannot cover that process dying. Reconnect establishes fresh authority.

Perception supplies state and candidate shooting geometry. Gameplay chooses and executes an approach considering opponent, boundaries and uncertainty. Shooting declares acceptable entry states. Movement supplies achievable motion/braking. Connect these early with the existing gait before polishing components independently.

## Power-on experience

Target: power on, the selected software prepares automatically and reports ready or a concrete missing dependency; one deliberate start/arm action begins play. No routine SSH recipes or repeated service setup. Goals and ordinary skill changes keep the session alive.

Automatic software preparation and physical initialization have different effects. Supported startup followed by one deliberate start action is accepted for the first version. Existing `start-native` may home and assumes supervised placement; automated orchestration still needs implementation and verification. Missing camera, credentials, models, network or controller must produce actionable status. Pin the runnable release, retain rollback and avoid automatic updates from changing branches. Mac/iPhone readiness matters too; Pi power alone does not establish the overhead feed.

Preserve each teammate's active runtime and experiments. The [build/test handoff](ROBOT_BUILD_AND_TEST_HANDOFF.md) separates shared source, isolated builds, selected releases and physical acceptance. Current ownership and authorization come from the checkpoint and user, not historical assignments in this plan. FC automatic startup is still unimplemented.

## Working prototype ladder and evidence

The first [offline approach-and-shot sandbox](pupper_fc/README.md) implements one-robot approach/alignment/strike and simulated goal restart, with interactive disturbances and replay. It establishes an executable gameplay baseline for integration; the physical and two-robot outcomes below remain open.

These are integration outcomes, not a serial allocation of teammate time. Every increment leaves a demonstrable baseline.

1. **See the field.** Live overlay and recorded replay show identities/headings, ball, goals and observation age. Check known positions, motion, occlusion and reacquisition.
2. **Reach a pose.** Existing gait reaches marked positions with chosen heading and stops. Measure position/heading error, response delay and stopping distance.
3. **Complete a play.** Find, approach, shoot, reacquire and repeat from varied arrangements. Record attempts, useful shots/goals, elapsed time and interventions. Compare specialized shooting against this baseline before adding training complexity.
4. **Defend and recover.** Intercept rolled balls with measured lateral reach; demonstrate get-up/resume and temporary perception loss. Recovery development begins earlier; this increment integrates it.
5. **Play against an opponent.** Use replay, simplified simulation and physical surrogates before robot two is available. Then commission its own profile/calibration and run real matches. Robot one's acceptance does not validate robot two.
6. **Sustain the demo.** Repeat match-length trials with shots, defense, misses, goal restart and representative recovery. Separately exercise camera/command loss, Stop and restart. Set numerical thresholds before acceptance and retain failures.

Simulation of game decisions does not establish physical contact or sim-to-real performance. Preserve compact evidence with source/policy identities, conditions, trial counts, timing and limiting failures, plus relevant clips. The live operator view should show observed state, selected behavior/target, motion owner, tracking age, release and fault reason. Measure end-to-end delay, not FPS alone. Reserve two-robot access before the final integration window.

## Owner steering and architecture proposal — November 17, 2026

The owner now requests native object detection, explicit offense/defense roles, coordinate and angle targets, rigorous directional/combined movement tests, latency measurement and gait improvement. Person entry should make both Puppers avoid the person. Contact trials using the temporary ball are deferred: the intended ball is a pink size-5 soccer ball with flower patterns. Overhead perception remains authoritative for field localization and shot geometry; onboard ball perception supplements it where useful. The project goal is paused for this alignment discussion, with hardware closed. The following implementation recommendation is proposed, not yet accepted as a revised execution agreement.

- Separate persistent **role** (offense/defense), **action phase** (navigate, align, strike, guard) and **motion authority** (execute, yield/replan, hold for stale tracking, stopped/fallen). These are two opponents, so roles are selected independently; do not force one global attacker/defender assignment. Start with a deterministic threat/opportunity rule and hysteresis.
- Ryan supplies timestamped robot poses/headings, ball state, goals and validity, plus optional staging/shot targets. Agree one owner for shot geometry; the existing controller already computes it. A target alone cannot close the navigation loop. Build against synthetic/replayed state now; physical coordinate acceptance needs independently measured live pose, even if only an early AprilTag feed.
- Reuse pose navigation and the bounded strike controller. Add a small reactive goalie that stays goal-side on a clamped guard line and faces/tracks the ball. Replace blind long run-throughs with current-state checks for ball corridor, heading, boundaries, obstacles and braking. Person interruption must cancel/revalidate a strike before resumption.
- Reuse the existing Hailo detector without activating person-following as a competing motion owner. First qualify frontal detection and stopping, then detour and resume. Recommended full-field coverage combines overhead person keep-out regions with an onboard veto for nearby hazards. Unknown clearance should hold rather than choose a blind lateral escape. Detection alone is not avoidance; stopping alone does not complete the requested detour demo.
- Qualify a frozen gait on forward, backward, left, right, turns, direction changes and combined translation/turns, then under camera load and warm operation. Measure tracking error, slip, tilt, stopping distance and end-to-end latency distributions. Tune/compare existing candidates for a demonstrated deficit before committing to new training. The short passing floor trials do not establish endurance or coordinate accuracy.
- Calibrate field/goal/ball/contact geometry and live movement limits before connecting the sandbox to hardware. Current simulation defaults include 0.82 m goals and 0.06 m ball radius, and allow speeds beyond the recent physical tests. Measure capture-to-decision-to-command-to-native-feedback timing separately from actual physical response; acknowledgement is not movement.
- For the real ball, compare the existing detector and a pink-color baseline on robot-view footage, then fine-tune only if measured failures justify it. Flower patterns, lighting, partial views and other pink objects belong in evaluation. Ball-specific training is independent of initial person avoidance.

Independent checks: a source audit and a context-blind Claude Code review using exact model `claude-fable-5-2` agree on the localization dependency, competing-planner risk, uncalibrated limits/geometry and front-camera coverage limit. Root does not adopt the review's proposed serial schedule: role/executor logic and person perception can advance in parallel with motion qualification. Any missed person intervention fails its planned acceptance scenario; an aggregate success percentage must not hide it.

Decisions for this discussion: whether full-field person avoidance may combine overhead and onboard sensing, and how soon Ryan can expose pose/headings separately from completed soccer perception. Proposed next integrated result: reach a commanded pose/heading, yield or detour for a person, resume with fresh state, then enter the selected offense/defense action. Real-ball contact and two-robot play build on that result.

## Existing gaps and decisions to settle

The local source audit found tilt stopping and supervised restart, not self-righting: [recovery logic](../gemini_pupper/robot/robot_api.py), [upright-injected tests](../gemini_pupper/tests/test_fall_recovery.py). The maintained [native velocity path](../pupper_gait_deploy/src/neural_controller.cpp) now expires silent gait-velocity input after 250 ms. [Movement evidence](../foundation/results/movement-live-20261116.json) records partial physical acceptance of the Pi-native release; the brief separates observed capabilities from remaining tests. This expiry does not implement target-policy cancellation, hardware-watchdog protection or measured braking. [Session helpers](../foundation/hardware/persistent_session.py) and the [velocity mux](../pupperv3-monorepo/ros2_ws/src/cmd_vel_mux/src/cmd_vel_mux_node.cpp) remain reusable but do not establish FC boot-to-ready or gameplay.

Settle when the relevant work begins: surface/ball/bumper/goal dimensions; mount and whole-field coverage; out-of-bounds/trapped-ball rules; recovery ownership/postures/clearance; match duration/success thresholds; second robot access and identity. Supported startup, manual ball replacement and one deliberate resume cue are accepted; self-righting commitment follows its feasibility investigation. Implementation can progress with explicit reversible assumptions. The adversarial review identified recovery, limited arena braking space, late robot-two access, occlusion, and boot/restart behavior as the principal risks; the plan includes evidence for each.
