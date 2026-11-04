# Pupper soccer

The first offensive demo is to approach a stationary ball from behind, kick toward goal and recover. The defensive demo is to guard a marked goal against rolling shots. Agree on the ball, surface, goal width and trial markings. Both plays still need physical evidence.

## Workstreams and evidence

| Owner | Deliverable | How we’ll judge it |
|---|---|---|
| Perception | After the optional labs, deliver a ball track for both plays. Test the existing `sports ball` detector on the actual ball. Reuse camera projection and target association. | Measure misses, false tracks, delay and reacquisition on held-out clips of near, far, rolling, hidden and absent balls. Compare crossing position and time predictions with filmed rolls. Fine-tune for measured failures. |
| Gait and skills | Approach, brake, turn and sidestep on the playing surface. Add a kick and lateral block with stable exits. Advance sprinting or jumping when it improves a play. | Measure stopping distance by speed, lateral reach by time and the range of ball positions the paw can kick. Film transitions, kick distance, direction and recovery. Count blocks across repeated shots. |
| Harness | Finish the required Lab 6 demonstrations. Expose tested soccer skills as tools and let the local controller consume fresh tracks during a play. Build on the existing tool registry, ownership handoff and Stop. | Replay a good track, a disappearing ball and a Stop mid-play. Check that cancelled work stays cancelled. Film the outcome of each play on the field. |

The Lab 6 follower defaults to people and uses the bottom of the target box for its stopping distance. Keep those class defaults intact. Start from the [label list](../follow_me_pupper/robot/coco.txt), [camera and detector service](../follow_me_pupper/robot/follow_service.py) and [target lock and approach rule](../follow_me_pupper/lab/target_lock.py).

Before training, mark a fixed goal direction and move the ball through the camera view with Pupper stationary. Include the last few centimetres of approach: compare the closest reliable detection with the paw’s measured kicking window. A ball location alone does not tell Pupper which way to shoot.

## Interface to agree before integration

- Perception supplies acquisition time and clock, track ID, validity, ball position `(x, y)` in metres and velocity in m/s in a named frame, with uncertainty. The consumer computes age from acquisition; preserve the input [ROS image timestamp](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg) through inference. Use `base_link` with x forward and y left for approach. Defense needs a fixed goal frame or a stationary goalie with a measured transform. Account for the robot’s motion before interpreting apparent ball velocity.
- Motion supplies supported forward and lateral speeds in m/s, yaw rate in rad/s, measured braking and lateral reach, kick-ready position tolerance and Stop semantics. Use these to set maximum track age and decide whether a block is reachable. A stale or lost track ends pursuit with zero commanded travel. The harness can report loss and request a fresh play.

## Where code goes

Add soccer code here as it lands. Use `perception/` for track estimation and replay evaluation, `motion/` for soccer skill definitions and measured profiles, and `behaviors/` for offense, defense and tool adapters. Reuse the [existing gait environments](../pupper-mjlab/src/mjlab/tasks/pupper_gait/) and [tool registry](../gemini_pupper/robot/tool_registry.py). The [Lab 6 handoff](../foundation/WORKSTREAMS.md) tracks submission separately.

One person owns each physical robot block. Other teammates can inspect saved logs and prepare code while that owner tests. Before switching owners, agree on the active release and who controls motion, the camera and the inference accelerator. Use the [robot session checklist](../foundation/HARDWARE_CHECKLIST.md) for the handoff.
