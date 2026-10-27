# 🐕 Pupper Live Gait Tuner

An interactive, browser-based tool for tuning Pupper V3's walking gait **live** and
watching the real robot (and a 3D model) respond instantly. Built for a ~20-minute
high-school workshop activity.

Drag a slider → the gait is regenerated (fast Gauss-Newton IK, ~25–65 ms) → the 3D
foot trajectories + mesh robot update, and (optionally) the real motors follow.

## What's here

| File | Role |
|------|------|
| `gait.py` | Pure-numpy gait generator: parameters → foot trajectories → joint cache. Vectorized FK/IK (exact match to lab 4's kinematics, ~380× faster). |
| `viser_app.py` | Viser web UI: sliders, 3D foot paths, live foot markers, optional URDF mesh robot. Visualize-only by itself. |
| `ros_bridge.py` | Streams the joint cache to `/forward_command_controller/commands` at 200 Hz with smooth ramp-in + stand pose. Drop-in replacement for `lab_4.py`'s publishing. |
| `urdf_robot.py` | Loads the real Pupper URDF/meshes into the 3D view (graceful fallback if unavailable). |
| `main.py` | Workshop entry point — bridge + UI together. |

## Running it

### Visualize-only (no robot, great for dev / projector)
```bash
cd ~/gait_tuner
python3 viser_app.py
# open http://<pi-ip>:8080
```

### With the real robot
1. Bring up the ros2_control stack the **same way you do for lab 4** so that
   `/forward_command_controller` exists, e.g.:
   ```bash
   ros2 launch ~/lab_4_fall_2025_solutions/lab_4.launch.py
   ```
   (Run this instead of, not alongside, `lab_4.py` — the tuner is what publishes commands.)
2. In another terminal:
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/gait_tuner
   python3 main.py
   ```
3. Open `http://<pi-ip>:8080`. **"Send to robot" is OFF by default.** Verify the gait
   in the 3D view, put the robot on its stand / hold it, then tick "Send to robot".

## Multiple groups, one robot

Several groups can tune at the same time and deploy one-by-one:

```bash
python3 workshop_multi.py 3        # 3 groups on ports 8080, 8081, 8082
```

Each group gets its own URL (`http://<robot-ip>:8080`, `:8081`, `:8082`) and tunes
**independently**. The robot can only be driven by one group at a time:

- The **Robot** panel shows a shared status: `🟢 FREE`, `🟢 YOU are driving`, or
  `🔴 OCCUPIED by Group N`.
- When another group is driving, your **"Send to robot"** box is **disabled** — you
  can't take the robot out from under them. It frees up (within ~0.5 s) when they
  untick "Send to robot" or hit **Stand**.
- If an *external* process is driving the motors (e.g. the normal robot stack is still
  running), every group shows `🔴 OCCUPIED — external process` and deploy is blocked.

So the flow is: everyone tunes in visualize-only → a group waits for `🟢 FREE` →
deploys → watches their gait → unticks to release → next group goes.

> Note: separate **ports**, not separate IPs — the robot has one IP, so groups are
> distinguished by port (`:8080`, `:8081`, …). Multiple devices on the *same* port
> share one set of sliders (good for collaborating within a group).

## The sliders

| Slider | What it changes | Tradeoff to discover |
|--------|-----------------|----------------------|
| **Body height** | how tall it stands | too tall = tippy, too low = scrapes |
| **Step length** | stride reach | longer = faster but less stable |
| **Step height** | foot clearance | higher clears bumps, costs stability/speed |
| **Stance width** | how wide the feet plant | wider = stabler, narrower = tippier |
| **Frequency** | steps per second | faster cadence vs. stumbling |
| **Duty** | % of cycle a foot is planted | high = walk-like/stable, low = run-like |
| **Preset** | trot / walk / pace / bound | loads a starting set of leg timings |
| **Leg timing (×4)** | each leg's phase offset, 0–1 of a cycle | *when* each foot steps — the heart of a gait |

| **Per-leg step tuning** | each leg's step length/height × the global value | asymmetric legs → veers, limps, leans |

**Per-leg step tuning:** a collapsible section with a folder per leg, each holding
"step length ×" and "step height ×" multipliers (0–2, default 1.0 = follow the global
slider). Set front_r to 1.5× and it reaches further on that corner; drop one leg's
height to 0.3× and it barely lifts. Great for showing how asymmetry makes a quadruped
turn or limp — and that a *symmetric* gait is what walks straight.

**Leg timing sliders:** one per leg (front_r, front_l, back_r, back_l). The value is
*when in the cycle that leg starts its step*, as a fraction (0–1). Pick a **Preset** to
load a known-good set, then drag any leg to explore — the preset switches to "custom".
Examples: trot = diagonal legs together `(0, .5, .5, 0)`; bound = front pair then back
pair `(0, 0, .5, .5)`; all four equal = a hop. Big offset mismatches can be unstable —
that's part of the lesson.

**Estimated body speed** = step length × frequency (ideal, no slip).

## Safety

- Start with defaults; change one slider at a time.
- Keep "Send to robot" off until you've seen the gait in 3D.
- "Stand (reset pose)" smoothly returns to a neutral stand and stops the gait.
- All parameters are clamped to safe ranges in `GaitParams.clamped()`.
- The first frame after enabling ramps in over ~1.2 s from the measured pose — no jerk.

## How it works (for the curious students)

1. Each **slider** sets a gait parameter.
2. `foot_trajectories()` turns those into a looping 3D path for each foot
   (stance: foot pushes back on the ground; swing: foot lifts in a sine arc and resets).
3. `ik_batch()` solves **inverse kinematics** — "what motor angles put the foot
   *there*?" — for all 50 frames of all 4 legs at once with Gauss-Newton.
4. The bridge plays that 12-motor cache on a loop at the chosen frequency.

The original lab precomputed this offline because IK took **9.3 s**. Vectorizing the
math and switching gradient descent → Gauss-Newton got it to **~0.03 s and exact**,
which is what makes live tuning possible.

## Install

```bash
pip install -r requirements.txt     # numpy, viser, (optional) yourdfpy
# rclpy comes from your ROS 2 Jazzy install, not pip
```

## Credits

The leg forward-kinematics and the keyframe foot-trajectory idea are derived from the
**Stanford Pupper V3** lab 4 materials and the
[`pupperv3-monorepo`](https://github.com/Nate711/pupperv3-monorepo). This project adds a
parameterized, vectorized gait generator (fast Gauss-Newton IK), a Viser front end for
live tuning, and a ROS 2 bridge with multi-group arbitration.
