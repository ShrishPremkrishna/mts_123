# CS 123 Lab Workspace

All my CS 123 labs, one folder per lab.

- `pd_control_lab/` — week 1
- `forward_kinematics_lab/` — week 2
- `ik_heuristic_walking_lab/` — week 3
- `pupper-mjlab/` — week 4
  - [Shrish’s Steps 6–7 training handoff](pupper-mjlab/lab4_runs/shrish_stablegait/) — settings, Colab, W&B results, and videos.
- `follow_me_pupper/` — week 5
- `gemini_pupper/` — week 6

Supporting repos (not weekly labs):

- `pupper_rebase/` — Pupper base software
- `pupper_gait_deploy/` — gait deployment
- `pupper-gait-tuner/` — gait tuning tool
- `UniMate/` — UniMate motion generation
- `ros2_cheats_sheet/` — ROS 2 reference
- `pupperv3-monorepo/` — Pupper v3 full monorepo (ROS 2 workspace, infra, recordings)

## Adding a new lab

Clone it, delete its `.git` so it becomes a plain folder, then commit:

```bash
git clone https://github.com/cs123-stanford/<lab_name>
rm -rf <lab_name>/.git
git add <lab_name>
git commit -m "Add <lab_name>"
git push
```
