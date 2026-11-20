# Overhead perception

One overhead iPhone camera becomes a shared field state for Pupper FC: where
each Pupper is and which way it faces, where the ball is and how fast it is
moving, the goals, and the geometry of a shot. [Project brief](../README.md) ·
[perception notes](../handoffs/PERCEPTION.md).

## Status — November 20, 2026

The mounted iPhone calibrated from four corners and showed all six floor tags,
the striker tag and the actual pink ball together. Loss of focus later stopped
fresh detections. The camera-estimated dimensions and goal placement still
need checking against tape. [Mounted field observation](../../foundation/results/field-camera-live-20261120.json)

## How it works

Everything tracked sits on a horizontal plane of known height, so one camera
is enough and nothing is triangulated:

1. Floor AprilTags (or clicked floor points) give the camera's position.
2. Each pixel is a ray from the camera; intersect it with the right plane:
   robot tag at `tag_height`, ball centre at `radius`, floor at 0.
3. Kalman filters give velocities and bridge short dropouts.
4. State goes out as JSON over UDP every frame, with an `age_s` on every object.

## Setup (once)

Run everything below from this folder (`final_project/overhead_perception`).
It has its own small environment; it does not use the repository's `.venv-api`.

```bash
uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python -r requirements.txt
```

## Try it with no hardware

```bash
.venv/bin/python scripts/run_perception.py --source sim
```

```bash
.venv/bin/python -m pytest tests -q -s
```

`--source sim` and the tests use the frozen `config/sim_field.yaml` (a 3.0 × 2.0 m
arena), so editing the real `config/field.yaml` never breaks them.

## Live camera

Any phone that the Mac can see as a camera works, and a **USB cable is the
dependable way** to connect it. The config's default `camera.source: phone`
finds the phone by name every time, whatever number macOS gives it that day,
and never falls back to the laptop's own camera.

| Phone | How it becomes a camera over a cable | Tested here |
| --- | --- | --- |
| iPhone (iOS 16+, macOS 13+) | Plug in, unlock once and tap Trust, then lock it and leave it still. It appears as "... iPhone Camera" (Continuity Camera; no app). | Wireless only: found by name, 1080p30. The cable path has not been run. |
| Android with a built-in webcam mode (Android 14+ on recent Pixels and others) | Plug in, pull down the USB notification, choose Webcam. It appears as an ordinary USB camera. | No |
| Any other phone | A webcam app that offers a USB mode and a Mac companion (Camo, Iriun, DroidCam and similar), which adds a camera to macOS. Check the app's free-tier resolution limit. | No |
| A phone app that serves video over the network | `--source http://...` or `rtsp://...`; over USB tethering this is wired too. | No |

The first run from a terminal triggers the macOS camera prompt. The script
prints which camera it chose and whether a phone is on a USB cable.

Wireless (iPhone only) needs the same Apple ID on both, Wi-Fi and Bluetooth on
for both (no internet needed, Personal Hotspot off), and the phone locked,
still and near the Mac. Picking the phone up or unlocking it pauses the feed.
A cable needs none of that and keeps the phone charged.

What the code does about an unreliable link:

- **Finds the phone by name**, again on every reconnect, because its number
  changes as cameras come and go. `--source iphone` (part of a name) or
  `--source 1` (a number) override the default.
- **Waits for the phone at start** instead of failing, and lists the cameras macOS sees.
- **Reports a lost feed.** No frame for 2 s shows CAMERA FEED LOST, stops publishing
  (consumers see their data go stale, the dashboard says STALE), and keeps trying to
  reopen the camera. Feed loss or reconnect now withholds calibration until an
  explicit recalibration; reconnect never grants motion again automatically.
- **Always releases the camera on exit**, including Ctrl-C. Before this, stopping the
  program left a wireless phone unavailable to the next run.

This lists the cameras by name, says which one is the phone and whether it is
on a cable, and saves a frame from each:

```bash
.venv/bin/python scripts/find_camera.py
```

```bash
.venv/bin/python scripts/run_perception.py --corners --dashboard
```

`--corners` is for a marked-out field with a tag on each corner; use `--survey`
to try things at a desk with whatever tags are in view.

To measure a recorded run, pass its video path as `--source`. Each replay state
reports `replay_clock: video_presentation`. Its timestamps and measured
velocities use seconds on the video's timeline. Playback speed and processing
load do not change the intervals. A decoder that supplies missing, duplicate
or backward timing stops the replay. For a known constant-rate recording,
`--replay-fps 30` selects frame-index timing explicitly and the output records
that assumption. Use the original clip for ordinary real-time motion analysis;
an edited or slow-motion export can have different timing from the actual run.

That opens one window: the camera feed with overlays on the left, the top-down
field view on the right, and the readouts underneath (described below). The
script says where calibration stands, in the terminal and as a banner in
the window: **CALIBRATION NEEDED → CALIBRATING… (n of m tags) → CALIBRATION
DONE**, and **CALIBRATION WITHHELD** if a visible floor reference drifts more
than 8 px, the image dimensions change or the feed reconnects. This is a latch:
moving the camera back does not clear it. Verify the setup and recalibrate.

Two things calibration does by itself on a live camera (November 20, after the first
corner calibration on a phone looking down at an angle kept being withheld):

- **It waits for a steady picture.** A phone feed that has just opened is still
  focusing and framing. Calibrating on one of those frames gave a camera position
  the next second's picture no longer matched, and the drift check withheld it at
  once. Automatic calibration now waits until every floor tag in view has stayed
  within 2 px for a second (the banner says so). Pressing a calibration key is not held back.
- **It fits the lens angle when there is no lens file.** The config's 69° guess left
  the far tags 7 to 8 px from where the model put them, right at the 8 px limit; the
  same frames fit at 66° with 0.5 px. With no `config/intrinsics.yaml`, calibration
  searches the angle within 15° of `camera.hfov_deg` and keeps the result only if it
  at least halves the error; the calibration message reports the fitted angle. Looking
  straight down the angle cannot be told from the camera's height, so the configured
  value stays. A real lens calibration (`scripts/calibrate_intrinsics.py`) is still
  better: this fits one number and no distortion.

If every tag it asked for is in view and it still has not calibrated, the banner
turns red (**CALIBRATION BLOCKED**) and says why, instead of looking as if it were waiting.

### Four ways to calibrate

| Key | Needs | Field frame |
| --- | --- | --- |
| `f` (automatic with `--corners`) | Tags 0, 1, 2, 3 on the four corners of the field, nothing measured | **The tags define the field:** its centre is the middle of the four, +x runs from the 0–3 side to the 1–2 side (east goal between tags 1 and 2), and its length and width are measured from the tags and replace the config's. Seen from above the tags go counter-clockwise. Other floor tags in view help but do not change the field. `tags.corner_inset` covers sheets that sit inside the lines. |
| `c` (automatic at start) | 4+ floor tags at the measured positions in `config/field.yaml` | The real field |
| `t` (automatic with `--survey`) | Any 2+ floor tags, nothing measured; 3+ spread apart is better | **With all four corner tags in view, pressing `t` does the same as `f`.** Otherwise: lowest-numbered tag is the field centre; its right edge points at +x, top edge at +y. **The field outline and goals are then only drawn around that tag from the sizes in the config; the tags do not mark the field's edges.** Good for trying things at a desk, not for the real arena. |
| `m` | Click the floor points listed under `manual_points` (default: the four field corners) | The real field |

A tag counts only when its whole black square and white border are visible.
After calibration the camera position is frozen, so tags may be covered.
Robot tags (ids 10, 11) never count for calibration.
Covered floor tags retain the last transform but cannot reveal a hidden camera
bump; `calibration_checked_at` records the last check. Click-only calibration
has no live floor-reference check. These limits need actual arena qualification.

### Other keys

Mouse = floor position under the cursor · left click = pin a point (consecutive
pins show the distance) · right click or `x` = clear pins · `g` = floor grid ·
`p` = save every pixel's floor position to `out/pixel_map.npy` · `q` = quit.
`b` = set the ball colour from the pixels under the cursor (see below).
A pixel's position is where its line of sight meets the floor, so it is only
right for things lying on the floor.

## Fake approach: line up, then run at the ball (no camera)

```bash
.venv/bin/python scripts/fake_approach.py
```

A pretend Pupper and ball on a pretend field. Each step the fake field hands the
approach logic just two things, both relative to the robot: the ball (metres
ahead, metres left) and the goal (the same, plus which way it faces). From those
it works out the preparatory moves and a velocity command, and the dashboard
plays it out:

1. **APPROACH**: walk to a point behind the ball on the line from the goal
   through the ball (about 0.95 m back), going round the ball if it is in the
   way, and turn to face the goal.
2. **ALIGN**: creep forward along that line, shuffling sideways onto it.
3. **RUN**: the moment it is on the line (within 4 cm) and facing along it
   (within 5°), anywhere from 0.45 to 1.2 m behind the ball, it goes to run
   speed without stopping, and keeps correcting until the bumper reaches the ball.
4. **BRAKE**, then start again from wherever the ball ended up.

It is written for a robot that takes only speed commands, follows them
imprecisely, strafes slowly and cannot walk backwards: it is closed-loop on what
the camera sees, never commands backward motion, and does not need to stop on an
exact spot (only on the line and facing along it).

The terminal prints the two relative inputs, the phase, the command and the
moves in words ("turn 48 deg left, walk 0.87 m to get behind the ball, turn 72 deg
right to face the goal, creep forward onto the shot line, run through the ball").
The dashboard shows the path, the line-up point, the run arrow, the phase and the steps.

Left click puts the ball somewhere, right click the robot; `space` pauses, `r`
gives a random scene, `g` attacks the other goal. `--headless --seconds 30`
prints without a window. Settings are under `approach_plan` in `config/field.yaml`.

```bash
.venv/bin/python scripts/fake_approach.py --sloppy --delay 0.15 --noise 0.01
```

`--sloppy` makes the fake robot follow commands badly (80% of the forward speed
asked for, 70% sideways, veers left, creeps forward while strafing); `--delay`
and `--noise` make the planner see positions late and jittery. The 80% is the
shooting code's own `speed_efficiency` assumption; the rest are guesses.

The logic is `fieldcam/approach.py` (`plan()` for the geometry and the list of
moves, `ApproachController.tick(ball, goal, now)` for the phases and command).
It takes only the two relative points, so the camera's `relative` output can
feed it unchanged; that connection, and sending the command to a robot, are not
made yet. The fake field (`fieldcam/fakeworld.py`) is a cartoon for trying the
logic: it does not predict a real Pupper's motion or a real ball's bounce. The
team's gameplay controller in `../pupper_fc` covers similar ground with its own
approach, align and strike phases.

## Where the ball is going

On the `perception/ball-prediction` branch (November 20). Simulation only: nothing
here has been checked against a real rolling ball, and nothing moves a robot.

**The physics.** A rolling ball slows at a roughly constant rate along a straight
line: from speed v it rolls for v / decel seconds and covers v² / (2 · decel)
metres. It ends by stopping, crossing a goal line between the posts, or reaching
the field edge, where it is "out" or (with `prediction.walls: true`) bounces.

**Every roll is fitted on its own, every frame.** Hand rolls on the real carpet
gave scattered slow-down rates, so no single measured number is trusted. From the
first frames of a roll the tracker fits that roll's own start speed and slow-down
rate, and refits on every frame after:

- For the first 0.6 s the curve is too short to show the slow-down, so the rate
  stays at the starting guess (`prediction.roll_decel`, then the rates of finished
  rolls) and only the newest frames set the speed.
- After that the roll's own data takes over. The roll is taken to start where the
  ball was fastest, so frames where a hand was still pushing it do not count.
- The prediction comes with a **stopping area**: an ellipse of about 2 sigma that
  starts large ("somewhere over there") and shrinks to a spot as the roll goes on.

**Knocked off course.** A divot or seam can turn the ball. A sharp knock shows as
a jump and a gentle one as the newest frames sitting to one side of the roll's
line; either way the direction is refitted from there while the slow-down learned
so far is kept. Because that can happen at any point still ahead, the area is
widened across the roll by `prediction.wander_deg` per metre still to go, and
rolls that really are knocked off line widen it further for the next ones.

**The plans** (`fieldcam/intercept.py`), per robot that is in view:

- `strike` (run through the ball): line up behind where the ball **will** stop, on
  the line to the goal it attacks. While the ball rolls the line-up point sits
  further back by the size of the stopping area and `hold` is true: get lined up
  now, run only once the ball has stopped.
- `kick` (leg kick): the pose that puts the ball in front of one paw with the
  swing aimed at the goal, and the timeline walk → settle → wind-up → contact.
  The wind-up takes about 2.3 s, so it may **start while the ball is still
  rolling**, timed so the paw arrives as the ball does; that is allowed once the
  stopping area is under `intercept.kick_window`. Until then the robot waits
  behind the pose by the size of the area. `action` is `go_to_pose`, `wait` or
  `wind_up`, and `saves_s` says how much sooner contact happens than waiting for
  the ball to stop first.
- `block`: only when the ball is predicted to roll into the robot's own goal:
  where to stand so its body is across the ball's path, and the time to spare.

Routes are the quickest of a few candidates for a robot that walks forward,
turns, shuffles sideways slowly and never reverses. Speeds are the limits of the
combined walk-and-kick release (0.22 m/s forward, 0.18 m/s sideways, 0.65 rad/s),
not measured travel, and turning on the spot before each leg is pessimistic. The
kick offsets and timing are the team's current figures, not measured with the match ball.

**Where it shows.** Published as `prediction` and `intercepts` (see
[Interfaces](#interfaces)). On the dashboard and the camera view, while the ball rolls:
the predicted path (pink dashes, a dot each second), the stopping area (shaded
ellipse), a ring where it should stop, and white dots for what each earlier frame
predicted, so you can watch the prediction settle. For a few seconds after a roll
the dots stay and a line says how far off the first guess was. Each robot gets its
line-up point, a green outline at the kick pose (thin = where to wait, thick = the
pose itself, "KICK NOW" when the wind-up should start) and, when its goal is
threatened, a red cross at the block point.

**Try it with no camera:**

```bash
.venv/bin/python scripts/fake_approach.py --rolling     # k rolls the ball again
```

```bash
.venv/bin/python scripts/fake_approach.py --compare 200 --sloppy --noise 0.01 --delay 0.15
```

**Measuring the roll rate** is now optional (it only sets the starting guess):
`scripts/fit_ball_roll.py`, run next to a calibrated live view, prints each hand
roll's rate and the median.

**What the numbers say (simulated rolls, November 20).** Each roll slows at its
own rate, drawn from 0.2 to 0.6 m/s², is pushed by hand for 0.35 s first, and is
measured with 1 cm noise. Error in the predicted stopping point, median (90th percentile):

| When in the roll | Fixed rate 0.35 | Per-roll fit, 7.5 frames/s | Per-roll fit, 30 frames/s |
| --- | --- | --- | --- |
| first 0.5 s | 29 cm (86) | 32 cm (97) | 35 cm (107) |
| 0.5 to 1 s | 21 cm (63) | 24 cm (132) | 14 cm (63) |
| middle | 14 cm (48) | 7 cm (45) | 3 cm (20) |
| last 30 % | 3 cm (10) | 1 cm (4) | 1 cm (2) |

| Question | Result |
| --- | --- |
| Real stopping point inside the drawn area | 83 to 97 % of frames |
| Every roll knocked 5 to 25° off course once: last 30 % | 2 cm (6); inside the area 76 to 93 % of frames |
| Area under 4 cm (wind-up may start) | 0.45 s before the ball stops at 7.5 frames/s (in two thirds of rolls), 0.7 s at 30 frames/s |
| A still ball reported as moving | 0.1 % of frames |
| Striker reaches a rolling ball sooner by predicting (fake field, 100 scenes) | 0.1 s median out of about 12 s |
| Keeper 30 cm in front of its goal can reach a block (travel model only) | 73 % of shots on target; 60 % when the ball arrives in under 1.5 s |

So: early in a roll nothing beats a rough guess, because the frames so far cannot
tell a ball that will slow quickly from one that will not; that is what the large
area is for. From the middle of the roll on, the per-roll fit is two to five times
closer than a fixed rate, and at 2 cm noise or 7.5 frames/s the gain is smaller.
For a walking robot the practical uses are getting lined up before the ball has
stopped, starting the kick's wind-up up to about half a second early, and defence.

Known limits: the simulated ball follows exactly the constant-rate rule, so these
figures show the fitting works, not that a real ball behaves this way. A ball that
curves steadily (spin, a slope) is followed as a series of straight pieces. The
plans do not check that the waiting robot is clear of the ball's path. The approach
logic itself is unchanged, so it can still place its line-up point outside the
field when the ball is near an edge.

## Dashboard

A top-down drawing of the field: goals, floor tags (filled when readable in the
current frame, hollow when covered or out of view), the ball with its velocity,
each Pupper with its heading, the line from each Pupper to the ball and the
shot line from the ball to the goal. The side panel gives calibration state,
positions, ages, each robot's view of the ball and whether it is in kick
position. It shows only what perception currently reports: a ball or robot that
is not seen disappears and is listed as "not seen", and the view dims and says
STALE if updates stop.

- In the live script it shares one window with the camera feed: `--dashboard` at
  start, or `d` to switch between that and the camera feed alone; `z` zooms the
  field view between the whole field and "fit to what is visible" (useful at a
  desk). The mouse tools (position, pins, `b`) work on the camera part.
- From another terminal, or another machine that receives the stream:

```bash
.venv/bin/python scripts/dashboard.py
```

It is drawn from the published state alone (`fieldcam/topdown.py`), with no
camera and no config, so what it shows is exactly what consumers receive.

## Ball

The detector takes a list of colour ranges (`ball.colors`) to find roughly where
the ball is, then fits a circle to its outline for the exact centre. The config
is set for a **pink** ball with a generic pink range that has not been tuned on
the real ball. To set it under the real lights: run the live view, put the
cursor on the ball and press `b`. The colour takes effect at once, and the
terminal prints the lines to paste into `config/field.yaml`, plus an estimate
of the ball's radius once the camera is calibrated. Measure the radius properly
and put it in `ball.radius`; a wrong value still finds the ball but costs
accuracy (see the table below).

## Tags

`tags/pupper_fc_tags.pdf` is made for **double-sided printing**: 16 US Letter
pages, each tag followed by its own guide page. Printed double-sided, the guide
lands directly behind the tag on the same sheet (it is centred, so flipping on
the long or the short edge both work); printed single-sided it is simply the
next sheet. There are six floor tags (17 cm, ids 0–5) and two robot tags (8 cm,
ids 10 striker, 11 keeper). Print at 100% on matte paper.

The whole field on one sheet is separate: [tag_layout.png](tags/tag_layout.png)
(also `tags/tag_layout.pdf`).

| Tag | Where it goes |
| --- | --- |
| 0 | Corner: west goal end, south side |
| 1 | Corner: east goal end, south side |
| 2 | Corner: east goal end, north side |
| 3 | Corner: west goal end, north side |
| 4, 5 | Ends of the halfway line, south and north (optional: they sharpen the camera fit) |
| 10, 11 | On the striker's and keeper's backs |

Each floor tag's guide page names it, shows a map with its spot marked, and has
a + at the tag's centre to set on that spot. The printed side also carries one
line naming the spot, so a taped-down tag can be checked from above.
Seen from above the corner tags read 0, 1, 2, 3 counter-clockwise, and the east
goal (the one the striker attacks) is between tags 1 and 2. East, west, north
and south are only names: choose which goal is east once and the rest follows.

Tape floor sheets down whole, tag centre on its spot, any rotation. Cut robot
tags at the corner marks, mount flat on a stiff card with the FRONT edge toward
the nose, then measure `tag_height` (and `tag_offset` if it is not over the body
centre) into the config. A robot tag's guide page fits inside the cut-out, so
the card says which robot it is on its back. Keep the printed layout sheet out
of the camera's view: its miniature tags are readable and carry the same ids as
the real ones.

`scripts/make_tags.py` regenerates everything from the config (`--pdf` for the PDF).

## Remaining bring-up on the arena

1. Measure floor tag centres into `config/field.yaml` (current values are placeholders).
2. `scripts/calibrate_intrinsics.py` with the lens and resolution actually used. Until then the lens is a guess (`hfov_deg`).
3. `scripts/measure_latency.py` → `camera.latency_s` (0.12 s is a guess).
4. Press `b` on the ball under the arena lights → `ball.colors` (`scripts/tune_hsv.py` for fine adjustment).
5. Measure the pink ball (`ball.radius`, 0.08 m is a placeholder) and each robot's standing tag height.

**Camera coverage is an open problem.** At 1080p 16:9 the iPhone main lens sees
about 69° × 42°. Fitting the 3.66 × 2.44 m arena straight down needs the phone
roughly 3.8 m up, and at that height an 8 cm robot tag is under 4 pixels per
cell, where detection starts to fail. Options to test: the 0.5× lens, a slanted
view, a 4K mode, larger robot tags, or two cameras. None is tested yet.

## Interfaces

Field frame: metres, origin at the field centre, +x toward the east goal, +y
left of it, yaw counter-clockwise from +x. This is the same frame as
[the gameplay model](../pupper_fc/model.py), and `config/field.yaml` carries the
same field size; change both together.

**UDP JSON**, one datagram per frame (`net.host`, `net.port`; `scripts/listen.py` prints it):

```
seq, stamp, captured_at, frame_age_s, calibrated, calib_mode, calib_err_px
calibration_valid, calibration_revision, calibration_status, calibration_reason, calibration_checked_at
field:   {length, width, ball_radius}
prediction: null | {moving, x, y, vx, vy, speed, speed_sigma, heading, decel, decel_sigma, samples,
                    ends: still|stops|goal_east|goal_west|out|horizon, end:[x, y], end_in_s,
                    bounces, path:[[t, x, y], ...],
                    area: null | {x, y, along, across, heading},      # where it should stop, ~2 sigma half-axes
                    trail: [[age_s, x, y], ...],                      # what earlier frames of this roll predicted
                    last_roll: null | {ago_s, stopped:[x, y], trail, first_guess_error, decel}}
intercepts: {<name>: {strike: {feasible, kind: at_rest|moving, ball:[x, y], pose:[x, y, yaw],
                               path:[[x, y], ...], travel_s, ball_there_in_s, contact_in_s, wait_s, hold, note},
                      kick:   {feasible, paw, ball:[x, y], pose:[x, y, yaw], stage:[x, y, yaw], path, travel_s,
                               commit_in_s, contact_in_s, ball_ready_in_s, ball_speed_at_contact, sure,
                               at_pose, action: go_to_pose|wait|wind_up, saves_s, note},
                      block:  {feasible, point:[x, y], path, travel_s, ball_there_in_s, spare_s,
                               enters_in_s, enters_at_y}}}     # block only when its own goal is threatened
floor_tags: {<id>: {x, y, visible}}
ball:    {visible, x, y, vx, vy, age_s, measured_at, measured_this_frame}
robots:  {<name>: {visible, x, y, yaw, vx, vy, wz, age_s, measured_at, measured_this_frame}}
goals:   {east|west: {x, y_min, y_max}}
relative: {<name>: {ball:{x, y, distance, bearing}, goal:{x, y, distance, bearing},
                    shot_line:{ball_to_goal_yaw, turn_to_shot_line, behind_ball, left_of_line},
                    age_s}}
shots:   {<name>: {target, aim_yaw, staging:[x,y,yaw], ball_to_target,
                   error:{along, lateral, dist, yaw}, ready, cmd:{vx, vy, wz}}}
```

**`relative` is the scene from each robot's point of view** (`fieldcam/relative.py`):

| Field | Meaning | Sign |
| --- | --- | --- |
| `ball.x`, `ball.y` | Ball in the robot's body frame, metres | x forward, y to the robot's left |
| `ball.distance`, `ball.bearing` | Range to the ball, and the turn that faces it | bearing + = turn left |
| `goal.*` | The same four numbers for the centre of the goal this robot attacks | |
| `goal.facing` | Direction a ball must travel to go into that goal, as a turn from the robot's heading | + = left |
| `shot_line.ball_to_goal_yaw` | Field direction of the line ball → goal centre | counter-clockwise from +x |
| `shot_line.turn_to_shot_line` | Turn that points the robot along that line | + = turn left |
| `shot_line.behind_ball` | How far the robot is behind the ball along the line, metres | + = ball between robot and goal |
| `shot_line.left_of_line` | How far the robot is off the line, metres | + = left of it, looking at the goal |

All angles are radians in (−π, π]. A robot is lined up for a straight shot when
`left_of_line` and `turn_to_shot_line` are near zero and `behind_ball` is the
standoff the kick wants. `ball` and `shot_line` are absent when the ball is not
seen; the whole entry is absent when the robot is not seen. The body frame is
the yaw-only frame the shooting code calls `base_link_yaw`, so `ball.x, ball.y`
are the numbers its robot-mode `/soccer/ball` input expects; nothing publishes
them to ROS with a robot-clock stamp yet.

Positions are extrapolated to `stamp`; `age_s` is how old the newest real
measurement behind each one is. `visible` stays true while a track coasts
(up to `tracking.lost_after_s`). `stamp` is `time.time()` on the perception Mac.

**Gameplay controller.** `fieldcam/gameplay.py` `to_observation(state, robot, robot_id)`
returns a `final_project.pupper_fc.model.Observation`: pose at capture time,
the capture time itself, and `None` for anything not measured in that frame.
`fallen` and `goal` are not observed from overhead. This helper's `clock_offset`
is only for controlled same-clock use. For live delivery, use the measured
clock adapter below rather than guessing an offset or stamping on receipt.

**Maintained live ingress.** [overhead_ingress.py](../behaviors/overhead_ingress.py)
maps the source into the existing same-Pi `/fc/normalized_events` contract.
`StatePublisher` adds `_transport` identity, live/simulation mode and paired
wall/monotonic samples. A bounded nonce clock exchange measures the mapping to
each Pi; delayed frames retain their original age. Source process, calibration
revision, field geometry and robot mapping are pinned. Missing measurements stay
missing, and invalid calibration/clock/session evidence withholds motion. It
publishes only frames; it cannot arm, start motors or renew action permission.

Run the ingress without `--publish` for bounded camera-only discovery. Live
publication requires the pinned arguments shown by `--help` and the same
calibrated field JSON used by the FC bridge. Names `striker`/`keeper` map to
identities `pupper-1`/`pupper-2`, not permanent tactical roles. Camera/video
simulation is explicitly rejected by live ingress. The camera's `latency_s`
remains an unmeasured estimate until the physical capture-latency test; network
clock synchronization does not determine sensor/driver latency.

**ROS 2.** `ros2/field_state_bridge.py` is an untested sketch (UDP → topics);
topic names are placeholders and it has never run against ROS. Its `--drive`
option is not the integration path; use the maintained ingress and FC executor.

`shots` and `cmd` are an earlier, simpler approach planner kept for the
simulator loop. The gameplay controller in `pupper_fc` is the one to drive robots.

## What is verified

Simulation, 1080p, camera 2.6 m up (and a slanted view), 39 tests, plus 7 capture tests against a fake camera and 13 approach tests on the fake field:

| Check | Result |
| --- | --- |
| Robot position / heading, measured floor tags | ≤ 0.5 cm / ≤ 2.3° |
| Ball position (red/blue/grey ball, 8 cm radius) | ≤ 0.7 cm |
| Pink ball, 6 cm radius, default pink range | ≤ 0.5 cm |
| Same pink ball with the config radius wrong (8 cm) | ≤ 1.4 cm overhead, ≤ 3.1 cm slanted |
| Ball radius estimated from the image | 5.9 cm for a 6.0 cm ball |
| Robot-relative ball position / bearing | within 3 cm / 4° of truth |
| Calibration from 3 unmeasured tags | robot ≤ 1.2 cm, ball ≤ 1.1 cm |
| Field from 4 corner tags (true 2.80 × 1.80 m) | measured 2.81–2.82 × 1.81 m; robot ≤ 0.9 cm, ball ≤ 0.8 cm |
| Corner tags in the wrong order, or one hidden | refused, with the tag to move or uncover named |
| Same, with the lens guessed badly (60° vs 69°) | robot ≤ 3.6 cm, ball ≤ 3.3 cm |
| Calibration from 4 clicks, each up to 4 px off | ≤ 0.8 cm |
| Tag-to-tag distance from 3 tags, over 1.4–3.3 m | ≤ 2.2 cm |
| 8 cm robot tag read rate / 5 cm on a soft image | 100% / 96% |
| Dashboard | drawn from the UDP state; ball and tags land on their field positions; unseen objects are not drawn |
| Fake approach, 4 set starts | every first run starts on the line (≤ 1 cm off, ≤ 5° off) 0.75–0.90 m behind the ball, never touches the ball on the way in, never commands reverse, and scores |
| Same with a sloppy fake robot, positions 0.15 s late and jittery (8 scenes × 60 s) | 48 runs, no scene without a hit; at contact heading 2.3° (90th pct 3.2°), ball 1.4 cm off-centre (90th pct 2.2 cm); 8 goals |
| Timed moves with no camera feedback, same sloppy robot (experiment, not in the tests) | 38 cm and 20° off at the line-up point |
| Gameplay bridge | real `Controller.tick` accepts the observation and commands motion |

Live iPhone at a desk, November 16 (no physical reference measurements taken):

- 1920 × 1080 at 30 fps through Continuity Camera into the pipeline.
- November 18, wireless (no USB): four consecutive starts at 30 fps, including a
  restart straight after Ctrl-C, which previously lost the phone every time.
- November 19, wireless: `source: phone` chose "…iPhone Camera" by name and ran. A phone
  on a USB cable, an Android phone and a webcam app have not been tried.
  A mid-run dropout and automatic reconnection has only been exercised against
  a fake camera in `tests/test_capture.py`, not on the real link.
- Printed tags were read; three tags gave distances steady to about 0.1 cm
  (28.6, 30.1, 31.3 cm), not checked against a tape measure.
- The ball detector's fitted outline matched the real ball in one saved frame.
- Click calibration registered clicks in the real window.

Open field checks include focus stability, tape-referenced geometry, true capture latency, lens calibration, moving-ball tracking, physical approaches and two-robot play. The stationary ball was detected with the existing colour range. The live colour picker and rolling-ball accuracy still need evaluation.

## Layout

- `fieldcam/camera_model.py` pixel ↔ field geometry
- `fieldcam/detect.py` AprilTag and ball detectors
- `fieldcam/filters.py` Kalman / yaw filters
- `fieldcam/pipeline.py` frame → state, three calibration modes
- `fieldcam/tagmeasure.py` tag layout from the tags alone
- `fieldcam/clickcal.py` calibration from clicked points
- `fieldcam/probe.py` floor position of any pixel, grid, pins
- `fieldcam/gameplay.py` state → gameplay `Observation`
- `fieldcam/relative.py` ball and goal from each robot's point of view, shot-line angles
- `fieldcam/colorpick.py` ball colour ranges from pixels under the cursor
- `fieldcam/topdown.py`, `scripts/dashboard.py` top-down dashboard from the published state
- `fieldcam/approach.py` line-up-then-run plan and phase controller, from a robot-relative ball and goal
- `fieldcam/fakeworld.py`, `scripts/fake_approach.py` fake field that drives it, with no camera
- `fieldcam/ballistics.py` ball velocity fit, roll model and predicted path; `scripts/fit_ball_roll.py` measures the roll rate
- `fieldcam/intercept.py` where each robot should line up or block, and the route there
- `fieldcam/planner.py` simple shot geometry and approach (simulator loop)
- `fieldcam/sim.py` synthetic arena for tests and `--source sim`
- `fieldcam/net.py`, `fieldcam/capture.py` UDP and camera capture
- `scripts/` run, camera finder, tag pages, calibration and tuning tools
- `config/field.yaml` real arena · `config/sim_field.yaml` frozen simulator arena
