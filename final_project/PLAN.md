# Pupper FC delivery plan

November 12, 2026. Confirmed outcomes guide this plan. Rules and implementation proposals below remain open to discussion. This is local planning while a teammate uses Pupper for Lab 6; it does not claim soccer implementation or hardware acceptance.

## Outcome and ownership

Two Puppers sustain offensive and defensive play using overhead vision and a 2D field model. The base includes locomotion, bumper shooting, tracking/prediction, autonomous positioning, goal restarts and physical fall recovery. Foot kicking and a special goalie pose are stretch goals. Ball-in-simulation training is a later experiment, not a prerequisite for the first working game.

| Owner | Outcome |
| --- | --- |
| Marcus — [movement](handoffs/MOVEMENT.md) | Controllable forward/lateral motion, turning, acceleration, braking and stability; measured limits. Physical get-up capability jointly developed with gameplay recovery. |
| Tej — [shooting](handoffs/SHOOTING.md) | Useful ball motion toward a target, known approach/entry conditions, strike and exit behavior. Start with existing locomotion. |
| Ryan — [perception](handoffs/PERCEPTION.md) | iPhone ingestion, field coordinates, persistent robot identities/headings, ball position/velocity, goals, shooting geometry and observation age/validity. |
| Shrish + lead agent — gameplay/integration | Offense/defense selection, approach execution, command ownership, interruption, recovery coordination, startup, operator view and whole-game validation. |
| Shrish — arena | Camera mounting, boundaries/goals, markers, ball, surface and bumper arrangement for repeatable trials. Coordinate dimensions and markers with Ryan. |

The movement/get-up split is proposed ownership within the confirmed recovery requirement. Technical boundaries may change jointly without dropping an outcome or leaving it unowned.

## Game behavior and proposed rules

Offense selects a target, approaches the useful side of the ball, aligns the bumper, strikes, brakes and reassesses. Defense predicts a threatened goal crossing and seeks a position the robot can actually reach. Selection considers threat, opportunity and reachability, with enough persistence to prevent oscillation. Proximity alone is an initial heuristic, not a complete strategy.

A missed shot, moving ball or blocked route causes reassessment rather than stale timed actions. Geometry and heuristics can establish a baseline; learned decision-making remains an option. Proposed placement: local gameplay handles time-sensitive control, while voice sets intent and explains state.

- Fixed opposing goals and robot identities. Both robots can attack and defend. Shared observations need not expose each robot's intended action to its opponent.
- Define goal-mouth crossing using measured goal geometry and ball radius; count each goal once.
- After a goal, cancel the old play, reach valid restart positions and wait while a person replaces the ball. Resume without restarting software or homing. The resume signal and human-clear determination remain open.
- Replan for stationary and boundary-adjacent balls. Decide how to handle out-of-bounds or physically trapped balls; routine human resets beyond goals are not agreed.
- Avoid deliberate robot collisions. Boundary and opponent clearance must accommodate measured braking and recovery space.
- Tracking loss, controller faults and physical obstruction are explicit states. Exceptional human interventions count against reported autonomy.

## Autonomous recovery is required new motion work

Required chain: detect a fall, cancel the old action, acquire exclusive recovery control, physically self-right, verify stable standing and localization, then select a fresh play. Ordinary stand, controller reactivation or clearing a tilt latch does not satisfy it. Stop must interrupt recovery; reconnect must not replay old attacks. Distinguish a recoverable fall from operator Stop, stale sensors, low power and hardware faults.

Start this work alongside the first gameplay prototype. Catalogue expected postures on the arena surface with the bumper, including both sides and any reachable back/belly configurations. Compare suitable motion sequences or learned policies in simulation, then supervised physical trials. Record successes, duration, retries and failure postures. Unsolved expected falls keep base acceptance open; do not redefine them away after failure. Entrapment or damaged hardware may require exceptional assistance. Unbounded get-up retries do not establish recovery.

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

Automatic software preparation and physical initialization have different effects. Existing `start-native` may home and assumes supervised placement. Determine whether startup requires a supported pose or a new method enables floor boot; do not blindly enable current homing at power-on. Missing camera, credentials, models, network or controller must produce actionable status. Pin the runnable release, retain rollback and avoid automatic updates from changing branches. Mac/iPhone readiness matters too; Pi power alone does not establish the overhead feed.

Preserve the teammate's Lab 6 setup until handover. Shrish's go is required before Pi deployment/verification. No boot service or runtime is changed by this planning checkpoint.

## Working prototype ladder and evidence

These are integration outcomes, not a serial allocation of teammate time. Every increment leaves a demonstrable baseline.

1. **See the field.** Live overlay and recorded replay show identities/headings, ball, goals and observation age. Check known positions, motion, occlusion and reacquisition.
2. **Reach a pose.** Existing gait reaches marked positions with chosen heading and stops. Measure position/heading error, response delay and stopping distance.
3. **Complete a play.** Find, approach, shoot, reacquire and repeat from varied arrangements. Record attempts, useful shots/goals, elapsed time and interventions. Compare specialized shooting against this baseline before adding training complexity.
4. **Defend and recover.** Intercept rolled balls with measured lateral reach; demonstrate get-up/resume and temporary perception loss. Recovery development begins earlier; this increment integrates it.
5. **Play against an opponent.** Use replay, simplified simulation and physical surrogates before robot two is available. Then commission its own profile/calibration and run real matches. Robot one's acceptance does not validate robot two.
6. **Sustain the demo.** Repeat match-length trials with shots, defense, misses, goal restart and representative recovery. Separately exercise camera/command loss, Stop and restart. Set numerical thresholds before acceptance and retain failures.

Simulation of game decisions does not establish physical contact or sim-to-real performance. Preserve compact evidence with source/policy identities, conditions, trial counts, timing and limiting failures, plus relevant clips. The live operator view should show observed state, selected behavior/target, motion owner, tracking age, release and fault reason. Measure end-to-end delay, not FPS alone. Reserve two-robot access before the final integration window.

## Existing gaps and decisions to settle

The local source audit found tilt stopping and supervised restart, not self-righting: [recovery logic](../gemini_pupper/robot/robot_api.py), [upright-injected tests](../gemini_pupper/tests/test_fall_recovery.py). The inspected [native velocity path](../pupper_gait_deploy/src/neural_controller.cpp) retains the last Twist without receipt-age checking; verify and implement controller-local expiration. [Session helpers](../foundation/hardware/persistent_session.py) and the [velocity mux](../pupperv3-monorepo/ros2_ws/src/cmd_vel_mux/src/cmd_vel_mux_node.cpp) are reusable but do not establish FC boot-to-ready or gameplay.

Settle before physical acceptance: surface/ball/bumper/goal dimensions; mount and whole-field coverage; boot placement and start/resume signal; out-of-bounds/trapped-ball rules; expected recovery postures and clearance; match duration/success thresholds; second robot access and identity. Implementation can progress with explicit reversible assumptions. The adversarial review identified recovery, limited arena braking space, late robot-two access, occlusion, and boot/restart behavior as the principal risks; the plan includes evidence for each.
