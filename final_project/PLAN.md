# Pupper FC delivery plan

Updated November 16, 2026. Confirmed outcomes guide this plan. Rules and implementation proposals below remain open to discussion. Use this as project context when choosing work with the current user, not as an automatic task assignment. Current activity and robot access are in the [checkpoint](../foundation/results/interruption-recovery.json). This plan does not claim soccer implementation or hardware acceptance.

## Outcome and ownership

Two Puppers sustain offensive and defensive play using overhead vision and a 2D field model. The base includes locomotion, bumper shooting, tracking/prediction, autonomous positioning, goal restarts and physical fall recovery. Foot kicking and a special goalie pose are stretch goals. Ball-in-simulation training is a later experiment, not a prerequisite for the first working game.

| Owner | Outcome |
| --- | --- |
| Shrish — [movement notes](handoffs/MOVEMENT.md) | Taking over controllable forward/lateral motion, turning, braking, stability and locomotion-based headbutts. Existing teammate gait exports remain candidates; measured limits are still needed. |
| Tej — [shooting](handoffs/SHOOTING.md) | Foot-kick policy, with known entry conditions, contact and exit behavior. Headbutt baseline now belongs to Shrish. |
| Ryan — [perception](handoffs/PERCEPTION.md) | iPhone ingestion, field coordinates, persistent robot identities/headings, ball position/velocity, goals, shooting geometry and observation age/validity. |
| Shrish, assisted by agents — gameplay/integration | Offense/defense selection, approach execution, command ownership, interruption, recovery coordination, startup, operator view and whole-game validation. |
| Shrish — arena | Camera mounting, boundaries/goals, markers, ball, surface and bumper arrangement for repeatable trials. Coordinate dimensions and markers with Ryan. |

These are the team's current areas, not fixed roles for incoming agents. Get-up is a separate P0 work item with physical-motion ownership still open. Shrish and assisting agents own its gameplay integration. The first playable prototype can advance while recovery is open; it must not be described as having solved recovery. Technical boundaries may change jointly without dropping an outcome.

## Game behavior and proposed rules

Offense selects a target, approaches the useful side of the ball, aligns the bumper, strikes, brakes and reassesses. Defense predicts a threatened goal crossing and seeks a position the robot can actually reach. Selection considers threat, opportunity and reachability, with enough persistence to prevent oscillation. Proximity alone is an initial heuristic, not a complete strategy.

A missed shot, moving ball or blocked route causes reassessment rather than stale timed actions. Geometry and heuristics can establish a baseline; learned decision-making remains an option. Proposed placement: local gameplay handles time-sensitive control, while voice sets intent and explains state.

- Fixed opposing goals and robot identities. Both robots can attack and defend. Shared observations need not expose each robot's intended action to its opponent.
- Define goal-mouth crossing using measured goal geometry and ball radius; count each goal once.
- After a goal, cancel the old play, reach valid restart positions and wait while a person replaces the ball. Resume without restarting software or homing. One deliberate resume signal after the field is clear is accepted for the first version; the precise interaction can be chosen downstream.
- Replan for stationary and boundary-adjacent balls. Decide how to handle out-of-bounds or physically trapped balls; routine human resets beyond goals are not agreed.
- Avoid deliberate robot collisions. Boundary and opponent clearance must accommodate measured braking and recovery space.
- Tracking loss, controller faults and physical obstruction are explicit states. Exceptional human interventions count against reported autonomy.

## Autonomous get-up is separate P0 work

Required chain: detect a fall, cancel the old action, acquire exclusive recovery control, physically self-right, verify stable standing and localization, then select a fresh play. Ordinary stand, controller reactivation or clearing a tilt latch does not satisfy it. Stop must interrupt recovery; reconnect must not replay old attacks. Distinguish a recoverable fall from operator Stop, stale sensors, low power and hardware faults.

The user has separated this priority from the immediate environment and baseline work. Assign a physical-motion owner when it is taken up. Catalogue expected postures on the arena surface with the bumper, including both sides and any reachable back/belly configurations. Compare suitable motion sequences or learned policies in simulation, then supervised physical trials. Record successes, duration, retries and failure postures. Unsolved expected falls keep the recovery deliverable open; do not redefine them away after failure. Entrapment or damaged hardware may require exceptional assistance. Unbounded retries do not establish recovery.

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

## Existing gaps and decisions to settle

The local source audit found tilt stopping and supervised restart, not self-righting: [recovery logic](../gemini_pupper/robot/robot_api.py), [upright-injected tests](../gemini_pupper/tests/test_fall_recovery.py). The maintained [native velocity path](../pupper_gait_deploy/src/neural_controller.cpp) now expires silent gait-velocity input after 250 ms. The [Pi-native movement release](../foundation/results/movement-release-20261116.json) passed offline/target checks and is staged; physical acceptance remains open. This expiry does not implement target-policy cancellation, hardware-watchdog protection or measured braking. [Session helpers](../foundation/hardware/persistent_session.py) and the [velocity mux](../pupperv3-monorepo/ros2_ws/src/cmd_vel_mux/src/cmd_vel_mux_node.cpp) remain reusable but do not establish FC boot-to-ready or gameplay.

Settle when the relevant work begins: surface/ball/bumper/goal dimensions; mount and whole-field coverage; out-of-bounds/trapped-ball rules; recovery ownership/postures/clearance; match duration/success thresholds; second robot access and identity. Supported startup and one start/resume action are agreed. Implementation can progress with explicit reversible assumptions. The adversarial review identified recovery, limited arena braking space, late robot-two access, occlusion, and boot/restart behavior as the principal risks; the plan includes evidence for each.
