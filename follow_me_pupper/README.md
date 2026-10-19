# Follow Me Pupper

The Pupper finds an object with its camera, stays on the *same* person when
there are several, and walks up to them and stops at a comfortable distance.
Two optional steps go further: telling people apart by the color of their
clothes, and stepping around anything in the way. Everything runs on the
robot; you watch it in a browser.

This lab is self-contained: no API keys, no LLM, no voice. The `KarelPupper`
tracking API (`begin_tracking()` / `end_tracking()`) is what the robot
foundation model lab will drive later in the quarter.

## Layout

```
lab/       YOUR WORK: everything you write is in here, and the tests for it
robot/     provided: camera, detector, viewer, launch file, Karel API (read, don't rewrite)
scripts/   download the model, run everything at once, clean up
```

Run every command below from the repo root (`~/follow_me_pupper`).

## Setup (once)

```bash
./scripts/download_model.sh      # fetches robot/yolov8n_seg.hef (~11 MB)
```

## Running

Three terminals on the Pupper. Keeping them separate means you can restart your
state machine without restarting the robot.

```bash
# 1) robot + camera + viewer + detection
ros2 launch robot/follow_me.launch.py

# 2) your tracking state machine
python3 lab/follow_me.py

# 3) pick something to track
python3 robot/test_tracking.py
```

Then open the viewer at `http://<pupper-ip>:8080`. Over SSH, forward the port
from your laptop first and browse to `localhost:8080`:

```bash
ssh -N -L 8080:localhost:8080 pi@<pupper-ip>
```

`scripts/run_tracking.sh` does all three at once for a quick demo.
`scripts/cleanup.sh` stops everything and frees the Hailo accelerator.

To skip terminal 3 and start tracking immediately:

```bash
python3 lab/follow_me.py --ros-args -p target:=person     # any COCO class
```

Everything you write can be tested with no robot at all:

```bash
python3 lab/test.py           # Steps 2-4, the required lab
python3 lab/test.py step2     # just Step 2 (or step3, step4)
python3 lab/test.py step5     # Optional Step 5 (or step6)
```

A check whose code you have not written yet is skipped (`--`), not failed. A
failing check prints what your code returned and an example that passes. The
lab handout lists every check.

## Viewer controls

Two settings in the browser GUI matter when the link is poor:

- **Lock aspect ratio** (Camera, on by default) — viser stretches the background
  image across the whole window, which at the default 220° FOV squashes the
  picture sideways by ~45%. On, the frame keeps its true proportions and pads
  with bars instead.
- **Drop frames when behind** (Camera, on by default) — on weak wifi, sending
  every frame just grows a queue the viewer has to chew through, so the picture
  drifts further behind real time the longer you watch. On, a client that has not
  kept up simply misses frames. You lose smoothness, never currency. The Status
  folder counts what was dropped.

## Files

Your work, in `lab/`. Every function you write has a card in its docstring:
what each input is, what to return (with an example), what uses it, and the
test to run.

| File | What it does |
|---|---|
| `lab/follow_me.py` | The state machine (Steps 2–4): `process_detections`, `choose_state`, `motion_command`; `ready_to_avoid` (Optional Step 6). |
| `lab/boxes.py` | Reading a detection: `box_offset` (Step 2), `box_bottom` (Step 4). |
| `lab/target_lock.py` | Staying on the same person (Step 3: `pick_target`, `remember`), the stop distance (Step 4: `approach_speed`), and telling people apart by color (Optional Step 5). |
| `lab/avoidance.py` | Optional Step 6: `find_blocking_obstacle`; the rest (which way around) is provided, to read. |
| `lab/test.py` | Desk-side tests for all of it, one section per step. |

Provided, in `robot/`:

| File | What it does |
|---|---|
| `follow_me.launch.py` | Brings up motor control, the camera, and the viser viewer. |
| `viser_camera.py` | Camera → browser. Runs YOLOv8n-seg on the Hailo and publishes `/detections`. |
| `yolo_seg.py` | Hailo YOLOv8n-seg wrapper: inference, NMS, mask decoding, drawing. |
| `fisheye_converter.py` | Fisheye → equirectangular. Detection runs on the undistorted image. |
| `karel.py` | `KarelPupper` robot API: `move_forward()`, `bark()`, `begin_tracking()`, … |
| `test_tracking.py` | Interactive prompt for choosing what to track. |
| `follow_service.py` | Your `follow_me.py` as an on/off service, for the robot foundation model lab. |
| `lab_checks.py` | The small harness both test scripts share (fake detections, skips, failure printouts). |
| `coco.txt` | The 80 COCO class names, in the id order detections use. |
| `camera_params.yaml` | Double-sphere lens model for the fisheye undistortion, and the camera's fixed white balance. |
| `sounds/` | Bark, wiggle, bob, and dance audio. |

## How the pieces talk

```
camera_ros ──/camera/image_raw/compressed──> viser_camera.py ──/detections──> follow_me.py
                                                    ^                            |
                                                    |                         /cmd_vel
                              karel.py ──/tracking_control──┘                     |
                          (test_tracking.py)                                      v
                                                                          neural_controller
```

`follow_me.py` starts in IDLE and only begins tracking when something publishes
to `/tracking_control` — either `KarelPupper.begin_tracking()` or the `target`
parameter above.

## Notes

- Only one process can hold the Hailo accelerator at a time. If you see a
  device-in-use error, run `./scripts/cleanup.sh`.
- Detections are published in a fixed 700×572 equirectangular frame, so the
  pixel thresholds in `avoidance.py` mean the same thing at any camera
  resolution.
- Each detection also carries the mean color of its segmentation mask (in the
  message's `id` field, as `"r,g,b"`) — what Optional Step 5 uses to tell
  people apart. The camera runs a fixed white balance (`camera_params.yaml`)
  so those colors do not drift as auto white balance hunts.

## In the robot foundation model lab

Gemini Pupper's "follow me" runs *your* follow-me: its launch file starts
`robot/follow_service.py` from `~/follow_me_pupper`, which wraps the state machine in
`lab/follow_me.py` (with your target lock and stop distance) behind two services,
`activate_person_following` and `deactivate_person_following`. It needs no
viewer, keeps the Hailo off until it is asked to follow, and leaves obstacle
avoidance off. So keep this repo on the robot, with the model downloaded.

```bash
python3 robot/follow_service.py                             # by hand, for a try
python3 robot/follow_service.py --ros-args -p always_detect:=true   # keep detecting (eyes)
```
