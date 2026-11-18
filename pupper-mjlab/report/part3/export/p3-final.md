# Export round-trip: p3-final -- **PASS**

Checkpoint `/home/ubuntu/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt` (sha256 `ce97712166fb`), reference `/home/ubuntu/wt/w4/pupper-mjlab/configs/opt-lab-2/p2/s42/r2-lag`. Simulation/offline checks only; nothing was deployed.

| comparison (same recorded observations) | n | max abs action err | p99 | tolerance |
|---|---|---|---|---|
| exported policy.json (json_forward, robot order) vs torch actor | 9600 | 5.54e-06 | 4.28e-06 | 0.001 |
| rollout inference actions vs actor.mlp(normalizer(x)) (obs capture check) | 9600 | 4.05e-06 | 2.38e-06 | 0.001 |
| run dir auto-exported policy.json vs torch | 9600 | 5.54e-06 | 4.28e-06 | 0.001 |
| run ONNX (rp-p2-s42-r2-cont-1500.onnx) vs torch | 2000 | 3.46e-06 | 2.38e-06 | 0.0001 |

Observations: 9600 (every 4 steps over 16.0 s, coverage {'vx_min': -1.5, 'vx_max': 1.5, 'wz_absmax': 1.5, 'vy_absmax': 0.4000000059604645, 'frac_fast_slot': 0.22750000655651093}). Gait-reference golden residual vs the training term: 1.09e-05 (< 1e-4).

run_dir_policy_json: field diffs vs fresh export = {}
wandb_policy_json: field diffs vs fresh export = CommError('HTTP 404: file wmhcp24q/policy.json not found while downloading file')

## Limit audit (reviewed values)

| check | ok | detail |
|---|---|---|
| joint_lower_limits == shipped default_policy.json | yes | max |diff| 0.00e+00 rad |
| joint_upper_limits == shipped default_policy.json | yes | max |diff| 0.00e+00 rad |
| kp == shipped default_policy.json | yes | 5.5 vs 5.5 |
| kd == shipped default_policy.json | yes | 0.25 vs 0.25 |
| action_scale == shipped default_policy.json | yes | 0.75 vs 0.75 |
| default_joint_pos == shipped default_policy.json | yes | max |diff| 0.00e+00 rad |
| kp <= driver kp_max (robot description) | yes | kp 5.5 vs kp_max [10.0] |
| kd <= driver kd_max (robot description) | yes | kd 0.25 vs kd_max [1.0] |
| effort_max (current/torque limit) == 3.0 Nm == sim PUPPER_EFFORT_LIMIT | yes | xacro [3.0], sim 3.0 |
| class driver limits JSON == robot description position_min/max | yes | mismatched: [] |
| exported joint limits contain the driver limits (driver clamps tighter on hardware) | yes | model-minus-driver margin min 0.100 rad |
| robot description sha256 == the one the driver limits were reviewed against | yes | dd3768478610 vs ['dd3768478610'] |
| command_clip vx == +-1.5 m/s (spec: the trained clip ships in the export) | yes | {'vx': [-1.5, 1.5], 'vy': [-0.5, 0.5], 'wz': [-2.0, 2.0]} |
| gait_reference present with fast-forward, fast-backward and lift tables | yes | keys ['gallop_back_table', 'gallop_table', 'lift_table', 'trot_table'] |
| gait_reference switch constants: gallop_speed 0.5, dir_threshold 0.05 | yes | gallop_speed 0.5, gallop_freq_mult 2.25 |
| gait_reference clamp == exported joint limits | yes |  |
| heading_hold contract stamped | yes | {'kp': 1.0, 'clip': 0.3, 'yaw_threshold': 0.1, 'walk_threshold': 0.05} |
| reviewed files unchanged vs origin/main | yes | pupper_gait_deploy/policies/default_policy.json, pupperv3-monorepo/ros2_ws/src/pupper_v3_description/description/components.xacro, foundation/configs/class-driver-joint-limits.json, pupper-mjlab/src/mjlab/asset_zoo/robots/pupper_v3/pupper_constants.py |
| pupper_gait_deploy download_policy.py --validate-only | yes | rc 0: Policy looks consistent with the controller config. |
| pupper_gait_deploy test_gait_reference.cpp (host build) | yes | rc 0: OK: 48 golden cases matched (worst max_abs_err = 5.960e-08) + 5 property checks |

NOT deployed. Deployment needs a human operator and TA approval; see report/part3_notebook.md.
