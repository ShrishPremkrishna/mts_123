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

Regenerate the cache whenever you change the keyframes — `--use-cache` will happily
replay a stale gait.

Only one process may drive the motors at a time. Stop `ik.py` before starting
`walking.py`, and stop both before running the gait tuner.

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
