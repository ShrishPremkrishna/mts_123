"""Lab 4, Step 4 (from scratch) settings used by group 1. Final run: W&B tejk-stanford-university/mjlab/vt55ode8."""

TASK = "Mjlab-VelocityFS-Flat-Pupper-v3"
NUM_ENVS = 4096
ITERATIONS = 3000
KP_MULTIPLIER_RANGE = (0.6, 1.1)  # notebook defaults, unchanged
KD_MULTIPLIER_RANGE = (0.8, 1.5)
FRICTION_RANGE = (0.6, 1.4)

# Run 2, final (vt55ode8): error_vel_xy 0.096, error_vel_yaw 0.287, episode length 498/500 at 3,000 iterations.
REWARD_WEIGHTS = {
  "track_linear_velocity": 4.0, "track_yaw_velocity": 2.0, "upright": 1.0, "air_time": 0.25, "base_height": 0.5,
  "pose": 0.0, "gait_tracking": 0.0, "track_angular_velocity": 0.0,
  "termination": -10.0, "orientation_l2": -5.0, "lin_vel_z_l2": -2.0, "ang_vel_xy_l2": -0.05, "foot_slip": -0.1,
  "action_rate_l2": -0.02, "joint_torques_l2": -0.0002, "joint_acc_l2": 0.0,
  "stand_still_pose": -0.5, "stand_still_joint_velocity": -0.1, "knee_ground_contact": -1.0, "abduction_angle": -0.1,
  "self_collision_l": -1.0, "self_collision_r": -1.0,
}

# Run 1 (dgx39kes), stopped at ~1,300 iterations: stood still (error_vel_xy flat at 0.47). Differences from run 2:
REWARD_WEIGHTS_RUN1 = dict(REWARD_WEIGHTS, track_linear_velocity=1.5, track_yaw_velocity=0.8, air_time=0.1,
                           action_rate_l2=-0.01, joint_acc_l2=-2.5e-7)
