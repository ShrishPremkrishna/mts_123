# CS 123: Forward Kinematics Lab

Student code for the forward kinematics lab. You will implement the forward kinematics of
all four of Pupper's legs (`leg_front_l`, `leg_front_r`, `leg_back_l`, `leg_back_r`, each with
joints `_1`, `_2`, `_3`) and visualize the end-effector (foot) positions, either in your
browser (viser) or in RViz2.

## Files

- `forward_kinematics.py` — ROS 2 node that subscribes to `/joint_states`, computes the foot
  position of every leg, and publishes them on `leg_<leg>_end_effector_position` (one topic
  per leg) plus one colored sphere per leg on `marker`.
  Fill in `rotation_y`, `rotation_z`, `translation`, then the front-left leg
  (`fk_front_left`: `T_1_2`, `T_2_3`, `T_3_ee`, `T_0_ee`, `end_effector_position`; `rotation_x`
  and `T_0_1` are given), and finally the other three legs (`fk_front_right`, `fk_back_left`,
  `fk_back_right`). The hip joint origins you need for `T_0_1` of each leg are listed in the
  comments of `forward_kinematics.py` and come from `pupper_v3.urdf.xacro`. A leg whose function still returns `None` is skipped, so you can check
  each leg as you go.
- `forward_kinematics_viser.py` — browser-based 3D viewer (robot model from `/joint_states`
  and the spheres from `marker`). Started automatically by the launch file.
- `forward_kinematics.launch.py` — launches `ros2_control_node`, the robot state publisher,
  the joint state / IMU broadcasters, the kp/kd forward command controllers, the Foxglove
  bridge, and the viser viewer.
- `forward_kinematics.yaml` — controller configuration (update rate, joints, broadcasters).
- `forward_kinematics.rviz` — RViz2 config with the robot model and the end-effector markers
  already added, if you would rather use RViz2 over a forwarded X session.
- `forward_kinematics_layout.json` — Foxglove layout, if you prefer Foxglove.

The node holds the legs limp by continuously commanding zero kp and kd, so you can move the
legs by hand and watch the markers follow the feet. The numeric positions are shown in the
viewer's side panel (or `ros2 topic echo /leg_front_l_end_effector_position`).

Marker colors: front-left **green**, front-right **red**, back-left **blue**, back-right
**yellow**.

## Running

```bash
ros2 launch forward_kinematics.launch.py   # in one terminal
python3 forward_kinematics.py              # in another
```

### Viewing in the browser (viser)

The launch file starts the viewer, and both it and `forward_kinematics.py` print the link to
open when they start. On the same network as the Pupper, open

```
http://<pupper-ip>:8080
```

in a browser (several people can have it open at once). If you can only reach the robot over
SSH, forward the port from your laptop and browse to `http://localhost:8080` there:

```bash
ssh -N -L 8080:localhost:8080 pi@<pupper-ip>
```

Use `viser:=false` to skip the viewer or `viser_port:=8081` to change the port.

### Viewing in RViz2

RViz2 still works exactly as before (all four markers are on the same `marker` topic):

```bash
rviz2 -d forward_kinematics.rviz           # in a third terminal
```
