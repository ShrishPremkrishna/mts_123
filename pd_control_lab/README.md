# CS 123: PD Control Lab

Student code for the PD control lab. You will implement a PD controller for a single Pupper joint (`leg_front_l_3`, the last link of the front-left leg).

## Files

- `pd_control.py` — ROS 2 node that subscribes to `/joint_states` and publishes torque commands. Fill in `KP`, `KD`, `get_target_joint_info`, and `calculate_torque`.
- `pd_control.launch.py` — launches `ros2_control_node`, the joint state / IMU broadcasters, and the forward command controller.
- `pd_control.yaml` — controller configuration (which joint is commanded, update rate, broadcasters).

## Running

```bash
ros2 launch pd_control.launch.py   # in one terminal
python3 pd_control.py               # in another
```

Press Ctrl+C to stop; the node sends zero torque before shutting down.
