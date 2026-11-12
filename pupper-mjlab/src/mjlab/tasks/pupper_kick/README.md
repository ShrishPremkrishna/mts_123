# Pupper header kick (`Mjlab-HeaderKick-Flat-Pupper-v3`)

This task trains one joint-level policy that runs at a stationary ball, hits it with a head bumper, sends it along a given aim direction, and then stops upright. Goal finding, aim selection, and the walk to a start point stay outside the policy. The policy reads the ball position and the aim in the command slot of the standard 36-value Pupper frame, so it deploys with the existing `neural_controller` and needs no C++ change.

## Scope

| In the policy | Outside the policy (soccer layer) |
| --- | --- |
| Sprint to the ball, header contact, aim through body heading, brake and stay upright | Ball detection and floor position, goal localization, choice of aim, approach to a start pose, handoff to and from the walking policy |

## Interface (deploy contract)

The policy observes the standard 20-frame history of 36 values: `[ang_vel(3), projected_gravity(3), command(3), desired_world_z(3), joint_pos - default(12), last_action(12)]`. The command slot holds the kick target, not a velocity:

| Slot | `/cmd_vel` field | Meaning | Unit | Clip on robot |
| --- | --- | --- | --- | --- |
| `command[0]` | `linear.x` | Ball center x, robot heading frame (yaw only, origin at `base_link`) | m | -0.5 to 2.0 |
| `command[1]` | `linear.y` | Ball center y, same frame (left is +) | m | -1.5 to 1.5 |
| `command[2]` | `angular.z` | Aim: wanted ball travel direction minus robot yaw, wrapped | rad | -pi to pi |

The soccer node must publish this `Twist` at 20 Hz or more while the kick policy runs, and start a kick only when the ball is 0.45 to 1.5 m away, its bearing is within ±0.5 rad, and the aim is within ±0.9 rad of the heading. Hold the last estimate when the camera loses the ball. The export writes the clips into `command_clip`. The policy has no gait reference, so the controller keeps heading hold off.

Robot-side items for a hardware trial (not done, see `pupper_gait_deploy/config.yaml`):

- Add a new controller block for this policy. Do not copy `neural_controller_default`: its `max_vx_forward: 0.49` would clamp `ball_x` to 0.49 m.
- Shorten `init_duration` and `fade_in_duration` (default 2 s each) for the switch from the walker; do not set the fade to 0. Bring the robot to a stand with the walker first.
- Set `max_body_angle` just above the training limit of 35 degrees (approximately 0.65 rad).
- Make sure that nothing else publishes `/cmd_vel` while the kick runs (`teleop_twist_joy` publishes at 50 Hz with a gamepad connected, and the walkers read `/cmd_vel` as velocity).

## Scene

- Ball (entity `ball`): solid sphere, nominal radius 0.075 m and mass 0.10 kg (constants `BALL_RADIUS`, `BALL_MASS` in `kick_env_cfg.py`). Change these two constants when the real ball is measured. The ball has collision priority 2, so its friction and contact softness apply to all ball contacts. Contact `solref = (0.01, 0.5)` gives a drop rebound of e ≈ 0.43.
- Rolling resistance: the robot's solver settings (2 iterations, pyramidal cone, impratio 10) over-damp MuJoCo rolling friction by an order of magnitude (a 1 m/s ball stopped in under 1 s). Thus the ball uses condim 3, and the kick command applies a constant deceleration force (0.1 to 0.6 m/s², per episode) against the ball's horizontal velocity.
- Head bumper: the stock model has no collision geometry on the torso. The task adds a foam bumper box on the front face (`head_bumper`, 0.02 x 0.114 x 0.12 m, center at x = 0.161 m, z = -0.03 m in `base_link`), the front 4 cm of the chassis (`head_block`), and the rest of the chassis (`torso_shell`). At the standing height of 0.145 m, the bumper bottom edge is 0.055 m above the floor, below the nominal ball center (0.075 m), so the contact normal stays near horizontal. It reaches the floor at approximately 20 degrees of nose-down pitch. The boxes collide with the ball and the floor (contype 2, conaffinity 1|2), and body-pair excludes keep them off the leg spheres. **The physical robot needs a matching bumper** (leave a cutout for the camera).
- A header is a contact between the ball and `head_bumper`. A contact with the legs (knee and foot spheres), `head_block` or `torso_shell` before the header is a foul and ends the attempt.

## Episode

1. Reset: the robot stands at a random yaw, and in 50% of episodes it already moves forward at up to 0.25 m/s (handoff from walking). The ball is at distance d and bearing b from `base_link`. The aim is b plus a cut angle c.
2. The policy runs at the ball. The first header sets `kicked`.
3. The episode ends 1.0 s after the header (`kick_done`), on a foul (`foul`), on a tilt above 35 degrees (`fell_over`), when the chassis or bumper touches the floor (`body_ground`), when the robot passes or loses the ball (`missed_ball`), or at 5 s (`time_out`).

An adaptive curriculum raises one difficulty level from 0 to 1 when the success rate of the last `num_envs` episodes is above 60%, and lowers it below 25%. Success means a header, no foul, ball speed along the aim of at least 0.3 m/s, and an aim error of at most 20 degrees, both measured 0.3 s after contact.

| Parameter | Level 0 | Level 1 |
| --- | --- | --- |
| Distance d | 0.45 to 0.8 m | 0.45 to 1.5 m |
| Bearing b | ±0.15 rad | ±0.5 rad |
| Cut angle c | 0 | ±0.4 rad |

## Rewards

Weights are per second (the reward manager multiplies by dt). Event terms return 1/dt on one step, so their weight is the total reward of the event.

| Term | Weight | Purpose |
| --- | --- | --- |
| `approach_velocity` | 2.0 | Base velocity toward the ball, capped at 1.5 m/s, before the header |
| `kick_alignment` | 1.0 | Near the ball: heading on the aim and the ball centered on the bumper |
| `header` | 3.0 | Event: first bumper contact |
| `kick_quality` | 5.0 | Event, 0.3 s after the header: ball speed along the aim (0 to 3 m/s) minus 0.6 x speed across it |
| `head_recontact` | -5.0 | Bumper still on the ball more than 0.1 s after the header (pushing) |
| `leg_ball_contact` | -5.0 | Leg contact with the ball after the header |
| `post_kick_stop` | 3.0 | Low base speed from 0.2 s after the header |
| `foul` | -3.0 | Event: `foul` termination |
| `fell` | -10.0 | Event: `fell_over` or `body_ground` termination |
| `missed` | -2.0 | Event: `missed_ball` termination |
| `upright` | 1.5 | Body upright |
| `lin_vel_z_l2`, `ang_vel_xy_l2` | -0.2, -0.05 | Body bounce and roll/pitch rate |
| `action_rate_l2`, `joint_torques_l2` | -0.02, -2e-4 | Smooth, low-torque commands |
| `foot_slip` | -0.1 | Foot speed while in contact (walking-task term) |
| `knee_ground_contact`, `self_collision_{r,l}` | -1.0 each | Hardware safety |
| `low_base` | -40.0 | Metres of base height below 0.12 m |
| `joint_speed` | -0.02 | Sum of (\|qd\| - 20 rad/s)² above 20 rad/s: the PD model has no torque-speed curve |
| `abduction_deviation` | -1.0 | Squared abduction (`_2` joint) offset from the default pose |

Speed is rewarded through the kick: momentum transfer makes the ball faster when the head is faster at contact, and the discount (gamma 0.99) prefers an earlier header. The integral of `approach_velocity` is the distance closed, so a loop gives no net reward. The ball reward is paid once: a per-step version (first design) paid more for pushing the ball through the whole window than for a strike followed by braking.

## Domain randomization

Inherited from the Pupper walking tasks: foot friction 0.6 to 1.4, base COM offset, encoder bias, kp x0.6 to 1.1, kd x0.8 to 1.5, link mass and inertia x0.9 to 1.3, fixed 16 ms action latency, IMU latency, and observation noise. Added for this task:

| Quantity | Range | When |
| --- | --- | --- |
| Ball mass (with consistent inertia) | x0.5 to 2.0 (50 to 200 g) | each reset |
| Ball radius | x0.85 to 1.15 | each reset |
| Ball sliding friction | 0.3 to 1.0 | each reset |
| Ball rolling deceleration | 0.1 to 0.6 m/s² | each reset |
| Ball restitution (contact damping ratio) | 0.5 to 0.8 (e ≈ 0.43 to 0.18) | each reset |
| Motor effort limit | x0.8 to 1.0 | startup |
| Initial ball drift | ±0.05 m/s | each reset |
| Ball observation latency | 1 to 10 steps (20 to 200 ms) | each reset |
| Ball observation rate | every 1 to 5 steps (50 to 10 Hz), 0 to 20% dropouts | each reset |
| Ball observation noise | std 0 to 1 cm + 0 to 3 cm per m of range, bias ±2 cm, scale ±10% | each reset |
| Camera blind zone | ball center closer than 0.30 to 0.45 m: estimate frozen | each reset |
| Aim noise | bias ±0.03 rad, std 0.02 rad | each reset / step |

The critic receives the true ball state, the kick phase, and the ball mass and radius (privileged). The actor receives only the deploy frame.

## Train, evaluate, export

Run from `pupper-mjlab/` with an environment that has `mjlab` from this checkout.

```sh
# GPU (Colab or a CUDA host): the real run.
python -m mjlab.scripts.train Mjlab-HeaderKick-Flat-Pupper-v3 \
  --env.scene.num-envs 4096 --agent.max-iterations 3000 --agent.logger tensorboard

# Mac CPU: approximately 3,000 to 4,000 env steps per second, for smoke tests only.
python -m mjlab.scripts.train Mjlab-HeaderKick-Flat-Pupper-v3 \
  --env.scene.num-envs 256 --agent.max-iterations 20 --agent.logger tensorboard --gpu-ids None

# Scored evaluation at full difficulty (CPU), optional video of env 0.
python -m mjlab.tasks.pupper_kick.evaluate --checkpoint <run>/model_<N>.pt --episodes 512 --video kick.mp4
```

The runner writes `policy.json` (deploy format) next to each checkpoint. Simulation results are not hardware evidence. A new policy needs the team's approval before its first hardware run, and the physical bumper must match the simulated one.

## Known limits

- Restitution and rolling resistance of the real ball are not measured; the randomized ranges are estimates.
- The actuator model gives 3 Nm at any joint speed. The `joint_speed` term keeps speeds near 20 rad/s, but the real torque-speed limit is not modeled.
- mujoco_warp's `netforce` reduction read 0 N for the chassis-on-floor box contacts while native MuJoCo reported hundreds of N. Contact checks therefore use `found` OR the force history (`mdp.contact_any`).
- The policy does not see the ball velocity, so the ball must be still at the start of the kick.
- The perception ranges are estimates. The proposal assumes 0.25 s latency at 10 Hz before any odometry correction; measure the pipeline while trotting and narrow or widen the ranges.
- Robot yaw for the aim comes from the IMU. Yaw drift during a 2 s kick is small, but the soccer node must recompute `angular.z` from the current yaw at each publish.
