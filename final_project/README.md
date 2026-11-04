# Pupper soccer

Our first field demo uses one Pupper in two roles. On offense it finds the ball, aligns its nose behind it relative to the attacking goal, then drives through the ball and brakes. On defense it starts squarely in front of its own goal and moves sideways to block a rolling shot. Two Puppers playing against each other is the later team demo.

## Team handoffs

| Owner | Current work | Start here |
| --- | --- | --- |
| Perception | Finish Optional Labs 1 and 3. Detect and track the ball, identify the two colored goals, and approach the ball around a stationary obstacle. | [Perception handoff](PERCEPTION_HANDOFF.md) |
| Gait | Finish Optional Lab 2. Measure sprinting, lateral reach, braking and stable transitions. Evaluate jumping and use the real ball to characterize front contact. | [Gait handoff](GAIT_HANDOFF.md) |
| Harness | Finish Lab 6 and verify compound commands, handoff between behaviors and Stop on the staged release. Use replayed ball observations to exercise a soccer tool before field integration. | [Lab 6 handoff](../foundation/WORKSTREAMS.md) · [Tool registry](../gemini_pupper/robot/tool_registry.py) |

The fourth teammate can join whichever experiment needs a second set of hands. Rolling repeatable shots and filming from a fixed position lets the owner concentrate on the robot. A clear floor, tape, the intended ball, two colored goal markers and a soft obstacle are enough for the first trials.

## How the pieces fit

Perception supplies the ball’s acquisition time, frame, bearing and track validity. Where calibration supports a metric estimate, include position in metres and velocity in m/s. Motion supplies achieved speeds, stopping distance and lateral reach versus time. For a goalie trial, compare the predicted goal-line crossing against that measured reach. The harness starts or cancels the play. A local controller uses fresh observations to steer and requests zero travel when pursuit loses its target.

Put soccer-specific tracking and replay in `final_project/perception/`, measured motion profiles and skill adapters in `final_project/motion/`, and offense and defense tools in `final_project/behaviors/` as code lands. Keep reusable class components in their existing packages. Agree on one owner for motion, camera and detector during each robot block. [Robot session checklist](../foundation/HARDWARE_CHECKLIST.md)
