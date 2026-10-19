# Lab 4, Step 4 and 5 package (group 1, from-scratch policy)

Summary. Everything a teammate needs to reproduce, replay, compare against, or deploy the from-scratch Pupper walking policy: the exact notebook settings, the W&B runs, the result numbers, and the deploy command. Trained 2026-10-19 by Tej.

## Runs

| Run | W&B | What | Result |
| --- | --- | --- | --- |
| 1 | https://wandb.ai/tejk-stanford-university/mjlab/runs/dgx39kes | diagnostic, stopped at ~1,300 iterations | stood still: error_vel_xy flat at 0.47, episode length 490 |
| 2, final | https://wandb.ai/tejk-stanford-university/mjlab/runs/vt55ode8 | 3,000 iterations, `model_2999.pt`, `policy.json` on the run | error_vel_xy 0.096, error_vel_yaw 0.287, episode length 498 (last-50-iteration means) |

Task `Mjlab-VelocityFS-Flat-Pupper-v3`, 4,096 envs, 3,000 iterations, domain randomization at the notebook defaults. Weights: `reward_weights_step4.py` (run 2 final and the run 1 differences). Metrics every 10 iterations: `metrics_dgx39kes.csv`, `metrics_vt55ode8.csv`.

## Files

- `CS123_Pupper_step4_from_scratch.ipynb`: the course notebook with sections 3 and 5 set to the run 2 values, plus a replay cell (section 10) and a run-numbers cell (section 11). Open in Colab, G4 runtime, paste your own W&B key in section 2.
- `reward_weights_step4.py`: the settings as Python constants.
- `metrics_*.csv`: iteration, error_vel_xy, error_vel_yaw, mean_reward, mean_episode_length, falls per iteration.

## Replay in the viewer

Run notebook sections 1, 2, 3, then section 10. Open the printed link. In the Controls tab tick **Enable** under Commands → Twist, otherwise the sliders do nothing and the robot follows random training commands. Type values in the boxes beside the sliders and press Enter; Zero resets them.

## Deploy on the robot

```bash
python3 -m wandb login --relogin        # once, any account that can read the run (see access)
cd ~/pupper_gait_deploy && ./deploy.sh tejk-stanford-university/mjlab/vt55ode8
```

Press Square on the gamepad. Relaunch without rebuild: `ros2 launch neural_controller launch.py`.

## Access

The runs live under the W&B entity `tejk-stanford-university`. To read them from another account, Tej either sets the `mjlab` project to public (project settings → visibility) or adds the teammate to the entity. Until then, use Tej's API key on the robot.

## Sim videos

Drive `Lab 4 Drive Folder/lab4/sim`: `sim_forward_0.3.mp4`, `sim_backward_-0.3.mp4`, `sim_sidestep_+0.25.mp4`, `sim_sidestep_-0.25.mp4`, `sim_stand_zero_command.mp4`, `sim_full_take_2026-10-19.mp4`. Turn-in-place clip pending.
