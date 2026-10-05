# Extension: Live Gait Tuning

The exercises for this section are written up in the lab spec (Part 6). This folder
holds the one piece of code you write locally; everything else is done inside your
clone of the gait tuner.

## Setup

The tuner is a separate repository:

```bash
cd ~
git clone https://github.com/ankushDhawan5812/pupper-gait-tuner
cd pupper-gait-tuner
pip install -r requirements.txt      # numpy, viser, yourdfpy (rclpy comes from ROS)
```

## Running it

**Visualize only** (no motors — always start here):

```bash
cd ~/pupper-gait-tuner
python3 viser_app.py
```

Open the URL it prints (`http://<pi-ip>:8080`).

**With the robot.** Put Pupper on its stand first. Bring up the same controller
stack you used for Part 5, but run the tuner *instead of* `walking.py` — both
publish to `/forward_command_controller/commands`, so only one may run at a time.

```bash
# terminal 1
ros2 launch ~/ik_heuristic_walking_lab/walking.launch.py
# terminal 2
cd ~/pupper-gait-tuner && python3 main.py
```

"Send to robot" is off by default. Verify the gait in the 3D view before ticking it,
and use "Stand (reset pose)" to stop.

## What you edit where

| TODO | File | What |
|------|------|------|
| 9 | `~/pupper-gait-tuner/gait.py` | add your own entries to `GAIT_PATTERNS` (they appear in the Preset dropdown automatically) |
| 10 | `~/pupper-gait-tuner/gait.py` | change the swing arc in `foot_position()` |
| 11 | `extension/tuned_params_to_keyframes.py` (here) | convert the gait you tuned back into walking.py keyframes |

Check your work on TODO 9/10 with the tuner's self-test, which reports IK accuracy:

```bash
cd ~/pupper-gait-tuner && python3 gait.py
# cache (50, 12) in 27 ms, max EE error 0.0000 mm
```

Then run TODO 11 and paste its output into `walking.py`:

```bash
cd ~/ik_heuristic_walking_lab/extension
python3 tuned_params_to_keyframes.py --pattern trot --step-length 0.12 --frequency 2.5
```
