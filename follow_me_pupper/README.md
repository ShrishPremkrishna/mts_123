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

Run the commands below from the `follow_me_pupper/` subdirectory of this repo.

## Setup (once)

```bash
./scripts/download_model.sh      # fetches robot/yolov8n_seg.hef (~11 MB)
```

The downloader checks the course version's SHA-256 before installing the Hailo8L
model and refuses an existing file with different contents. This verifies file
identity, not the installed accelerator/runtime or camera performance. Preserve
the robot's current model before replacing it.

## Running

This is the standalone Lab 5 procedure. It drives the motors, camera and Hailo directly. One operator must own the robot and these resources, so follow the [hardware checklist](../foundation/HARDWARE_CHECKLIST.md) first. Use three terminals so the state machine can restart independently.

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

In the shared foundation runtime, the launcher resolves the configured `FOLLOW_ME_DIR` and requires the detector model. It then starts `follow_service.py` in Lab 5 mode. The service wraps `lab/follow_me.py` through `activate_person_following` and `deactivate_person_following`. It runs without a viewer and starts the detector only on demand. Obstacle avoidance is disabled by default.

```bash
python3 robot/follow_service.py                             # by hand, for a try
python3 robot/follow_service.py --ros-args -p always_detect:=true   # keep detecting (eyes)
```

## Shared runtime integration

Optional color recovery and timed detours are implemented but remain off by
default. Enable them separately, after camera/stop-distance validation:

```bash
python3 lab/follow_me.py --ros-args -p use_color:=true
python3 lab/follow_me.py --ros-args -p avoid_obstacles:=true
```

The same parameters work on `robot/follow_service.py`; Gemini exposes them as
`FOLLOW_USE_COLOR` and `FOLLOW_AVOID_OBSTACLES`. Camera frames are consumed once,
duplicate stamps are rejected, and detections older than one second are discarded.
A frozen camera or stalled detector cannot keep refreshing the follower's motion
watchdog. A target entering the stopping zone cancels a timed detour; these
detours still are not collision-free navigation or person identity recognition.

Only the inference worker closes its Hailo device. If an inference cannot stop,
service reactivation is refused until that worker exits; do not start another
detector to work around the error. Viewer and service compete for the same Hailo,
so choose one operating mode. A controller acknowledgement or zero velocity
publication still needs physical observation of stopping.

Robot prerequisites include the built course ROS workspace, Hailo driver/runtime,
the matching `robot/yolov8n_seg.hef`, OpenCV/NumPy/PyYAML, camera calibration, and
`viser` for the viewer. The service's optional eye feed additionally uses pyzmq.
This laptop's passing offline tests do not install or validate those robot
dependencies. The combined repository layout is supported by Gemini's explicit
component paths; do not copy a laptop virtual environment onto the Raspberry Pi.

## FC onboard ball and obstacles

Pupper FC starts one camera and one headless detector. Each team’s onboard-ball setting changes what the operator sees. **Pink** tracks a round pink region and reports when the same Hailo frame also identifies a sports ball. A colour-only acquisition needs two nearby measurements. **Confirmed** requires both pink pixels and a matching COCO sports-ball box. **Off** disables the ball search. The same inference continues to publish obstacle evidence first in all three modes. Stop, walking clearance and the existing model-bound arena contact rule operate independently. No onboard ball observation commands a walk or kick.

The coordinates belong to the calibrated 220° camera panorama. A positive bearing means camera-right. The angular estimate uses the projection’s longitude rays. Lens error and camera-to-body yaw remain unmeasured. The dashboard removes coordinates on a missed, ambiguous or expired observation. A capture expires after 350 ms on the Pi. The ROS settings topic is `/fc/onboard_ball_mode` with String values `pink`, `confirmed` and `off`. Per-frame JSON appears on `/fc/onboard_ball` and is included in full recording. The headless startup option `--fc-ball-mode` takes the same values and defaults to `pink`.

| Stage | Course Lab 5 | Pupper FC |
| --- | --- | --- |
| Capture | Classroom launch starts 1400 × 1050 RGB camera_ros with fixed white balance. The viewer can also acquire the sensor with Picamera2. Its ROS callback decodes every delivered JPEG with depth 10. The viewer can read the same image repeatedly. | One camera_ros owner with fixed gains [1.4, 3.3] and a 100 ms frame-duration request. A depth-one best-effort subscriber keeps the latest compressed frame, its original timestamp and receipt clocks. The detector wakes on arrival and rejects expired work before JPEG decode and inference. |
| Projection | The camera model caches double-sphere remaps. Viewer FOV, scaling and colour controls can change the image presented to the detector. | Fixed 220° equirectangular input, normally 700 × 572. The new headless path converts channel order after projecting. The projected pixels were identical across all 490 replayed JPEGs. |
| Model and decode | YOLOv8n-seg v2.14.0 Hailo8L HEF. 640 × 640 RGB letterbox, 80 COCO classes, confidence 0.4 by default, DFL boxes, class-wise NMS and optional instance masks. The GUI can change confidence and mask construction. | The class decoder, lens math, intrinsics and label file match Stanford’s source byte-for-byte at `0bb12a114ee7c2e7c033413c2b3d28e0fd6435f8`. FC checks the HEF and canonical label digests, fixes confidence at 0.4 and omits CPU mask construction. The compiled network still produces its segmentation heads. |
| Output and behavior | A Viser worker displays boxes/masks and publishes all classes to `/detections` with publication time. Optional per-person mean colour and the Lab 5 state machine support following. | FC omits the viewer, person-following service, mask-colour averaging and legacy detections. It publishes capture-stamped normalized boxes on `/fc/obstacle_detections` and the separate pink-ball result from the same frame. Selecting a different ball mode cannot change the obstacle boxes. |

The course identifies the supplied model and its handoff to the student state machine. Hailo’s reference distinguishes accelerator execution from host NMS and mask reconstruction. The ROS sensor-data guidance favors recent readings and allows a best-effort subscriber to connect to either a reliable or best-effort camera publisher. These informed the queue and instrumentation choices. [Stanford Lab 5](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/lab-5.html) · [Exact Stanford source](https://github.com/cs123-stanford/follow_me_pupper/tree/0bb12a114ee7c2e7c033413c2b3d28e0fd6435f8) · [Hailo instance segmentation](https://github.com/hailo-ai/hailo-apps/blob/main/hailo_apps/python/standalone_apps/instance_segmentation/README.md) · [ROS 2 Jazzy QoS](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

### What interrupted the field attempt

On November 20, release `fc-walk-kick-3bd1bc9f54db` ran an attached Viser process on the Pi. Its log shows a connected viewer and a Hailo8L HEF on a Hailo8 device. The model produced 2,616 consecutively numbered valid results over 275.3 seconds. No result contained class 32 (sports ball). The bag omitted camera pixels, so visibility and model recall on the pink ball cannot be established from that run. The 39 frames already older than 350 ms at the recorder include a burst of six. Median capture-to-recorder age was 93 ms and p95 was 240 ms. Recorder arrival includes middleware and recording delay.

During the 57.2-second offense window the guard sampled the age of its **latest available** detection, including a 50 ms clock reserve on that release. Its 276 samples included 46 expired and 62 awaiting the required clear-frame streak. Median sampled age was 257 ms and p95 was about 480 ms. The ready path also required a new zero command. No nonzero walk or kick reached the robot. An earlier 8-second stationary test had all 38 guard samples clear and a detector CPU observation of 66.7% of one core. The later headless FC release already removed Viser, and the current candidate adds wake-on-arrival, expiry checks and a smaller colour conversion. These changes need a new Pi measurement.

The CPU peaked at 84.8°C while the fan reported 9,331–9,686 RPM. Firmware showed active throttling in three of 28 ten-second samples. Of 39 frames already late at bag receipt, 32 were within 5.1 seconds of a sample with the current-throttle bit clear; seven lacked a nearby sample. All 273 frames near an active-throttle sample arrived within 350 ms. Coarse samples and changing scene/load leave the thermal contribution unresolved. The final bag has no camera-stage timing or Hailo temperature. Raspberry Pi documents progressive throttling in this temperature range and separate bits for current and historical throttling. [Pi thermal control](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#frequency-management-and-thermal-control) · [Firmware flags](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled)

The independent November 17 recording contains 490 onboard JPEGs of a person and a moving tower. A timing replay of the older GUI’s 10 Hz admission rule reproduced much of its measured 290-publication cadence with zero simulated inference cost. That identifies an earlier scheduling loss; the November 20 continuous sequence and host replay do not isolate Hailo latency in the later field attempt.

On the 490 JPEGs, the current colour tracker reported one false track. An inserted optical patch of the actual pink field ball was localized within its radius on 418 frames with zero wrong locations. The longest gap was 31 frames. When the test supplied a correct semantic box, all 490 patches were localized. These are composited images, and the test supplied the semantic boxes. The full unaltered recording has no pink-ball presentation. A Pi view under field lighting is still needed to measure live recognition, occlusion recovery and sustained capture-to-publication age. [Source identities, timings and test results](../foundation/results/fc-onboard-ball-20261120.json) · [Recorded-image replay](../foundation/scripts/replay_fc_onboard_ball.py)

During the next camera-only qualification, keep one camera owner and compare static, moving, hidden and substituted balls before and after warm-up. Pair the original camera stamp with each obstacle/ball result and the thermal log. Full recording produces `detector-timings.jsonl` with receipt, JPEG decode, projection boundary, HailoRT call duration, total inference including host decode, obstacle publication and optional colour-stage completion. Mailbox overwrites count deliveries superseded inside this process. Record the existing compressed image topic separately for visual ground truth when disk and recording load permit.
