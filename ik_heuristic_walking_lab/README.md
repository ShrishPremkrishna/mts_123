# IK & Heuristic Walking Lab

CS 123 lab code: inverse kinematics on one leg, then a heuristic (Raibert-style)
trotting gait on all four. Follow the lab spec on the course website — this repo is
where you write the code.

## Layout

| File | Purpose |
|------|---------|
| `kinematics.py` | Your lab 2 FK for all four legs, plus gradient-descent IK, shared by `ik.py` and `walking.py`. `python3 kinematics.py` checks it without the robot |
| `ik.py` | Handout Part 3: triangle trajectory tracking on the front right leg |
| `ik.yaml` / `ik.launch.py` | controller stack for `ik.py` (commands 3 joints) |
| `walking.py` | Handout Parts 4–5: trot keyframes and the gait loop on all four legs |
| `walking.yaml` / `walking.launch.py` | controller stack for `walking.py` (commands all 12 joints) |
| `extension/` | Part 6: live gait tuning with `pupper-gait-tuner` |

## Running

Each program needs two terminals. Put Pupper on its stand before running anything.

**IK on one leg (handout Part 3)**

```bash
cd ~/ik_heuristic_walking_lab
ros2 launch ik.launch.py     # terminal 1
python3 ik.py             # terminal 2
```

**Walking (handout Parts 4–5)**

```bash
cd ~/ik_heuristic_walking_lab
ros2 launch walking.launch.py     # terminal 1
python3 walking.py        # terminal 2
```

`walking.py` solves IK for the whole gait cycle at startup, which takes a few seconds.
Once your gait works you can save that result and skip the solve:

```bash
python3 walking.py --save-cache   # solve, then write joint_positions_cache.npz
python3 walking.py --use-cache    # load it back and start immediately
```

The cache records its keyframes, kinematics and timing contract. `--use-cache`
refuses a stale or incompatible gait; regenerate it after changing those inputs.

Only one process may drive the motors at a time. Stop `ik.py` before starting
`walking.py`, and stop both before running the gait tuner.

## Replay recorded foot motion

The [Lab 2 recorder](../forward_kinematics_lab/README.md#record-and-export-foot-motion)
produces a validated `pupper_recorded_foot_v1` JSON file. With the single front-right
leg controller already active in the supported test setup:

```bash
python3 ik.py --recorded-motion /path/to/front-right-motion.json
```

This **commands the robot** and requires a supervised test. It enters from the
measured foot position over two seconds, follows the recording once, then holds
its final position. Coordinates are already in `base_link` metres. Other legs are
rejected because `ik.yaml` exposes only the front-right leg. Continuous IK-branch
selection, interpolated path checks, fresh feedback, and an online 15 rad/s target
step bound cover entry and replay; delayed callbacks do not permit a larger step.
These command checks do not establish physical tracking or stability.

The saved `wave_01` bag contains an essentially stationary front-right foot. Its
moving back-right leg includes six positions beyond the replay model's knee
bound, so that motion is refused. It has not been clipped or substituted as a
successful moving demonstration. See the [recording review](../foundation/results/lab2-recorded-motion-review.json).

## TODOs

TODOs are numbered in handout order:

| TODOs | File |
|-------|------|
| 1 | `kinematics.py`: paste in your four-leg FK from lab 2 |
| 2–4 | `kinematics.py`: gradient-descent IK |
| 5–6 | `ik.py`: triangle trajectory |
| 7–8 | `walking.py`: trot keyframes and interpolation |
| 9–11 | extension (see `extension/README.md`) |

`ik.py` and `walking.py` both import FK and IK from `kinematics.py`, so anything you fix there fixes both.
