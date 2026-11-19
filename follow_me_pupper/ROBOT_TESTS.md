# Follower robot tests

Stream 3 evidence plan. Part A needs the camera and Hailo only, so that the robot cannot move. Part B walks. Part A comes first because each result changes a constant that Part B uses.

Offline, `robot/follow_replay.py` passes every scripted scene, but its camera geometry is assumed. It does not establish a physical distance, speed or detector behavior. Only the steps below can do that.

## Recovery policy

The follower never replaces a lost target with a guess. All constants are in `lab/follow_me.py` and `lab/target_lock.py`.

| Situation | Color off | Color on |
|---|---|---|
| No sighting for up to `COAST_TIME` (0.3 s) | Keep the last command | Same |
| No sighting for 0.3 s to `TIMEOUT` (1.0 s) | Stand still and watch the last spot | Same |
| No sighting for more than 1.0 s | `LOST`: stand still until a new `activate` | `SEARCH` for one turn (6.3 s), accept a color match only, then `LOST` and keep watching for a color match |
| A person we cannot tell apart overlaps the target by `AMBIGUOUS_OVERLAP` | `LOST` | Different shirt: keep the target. Same shirt: `LOST` |
| A box wider than `MAX_WIDTH_RATIO` (1.5) times the target's lands on its spot | Not the target | Same |
| Anybody inside the stop distance, within `IN_THE_WAY_BAND` of straight ahead | No forward motion | Same |
| No fresh frames for 1.0 s | Stand still, then as for a lost target | Same |

Known limits: a stranger in the same shirt passes the color check. A crossing can follow the wrong person for a few frames before it stops (1 run in 30 in replay, 0.6 s). Color off cannot resume by itself, so `FOLLOW_USE_COLOR=1` is the intended acceptance setting if Part A shows that color separates people.

## Before you start

1. Agree who owns the robot for this block. Only one program may own the camera and the Hailo.
2. Copy this branch to a new folder, so that you do not change the staged session in `~/pupper-foundation-session-20261029`:

```bash
rsync -a --exclude '__pycache__' follow_me_pupper/ pi@mts-pi:~/stream3/follow_me_pupper/
```

3. On the Pupper, reuse the pinned model and make a log folder:

```bash
cp ~/pupper-foundation-session-20261029/software/follow_me_pupper/robot/yolov8n_seg.hef ~/stream3/follow_me_pupper/robot/
mkdir -p ~/stream3/logs
```

## Part A: no motion

Put the robot on its stand. Make sure that the motor stack is off. This command must print nothing:

```bash
pgrep -af 'ros2_control_node|controller_manager'
```

Terminal 1, the camera with the fixed white balance:

```bash
source /opt/ros/jazzy/setup.bash; source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash; export ROS_LOCALHOST_ONLY=1
ros2 run camera_ros camera_node --ros-args -p format:=RGB888 -p width:=1400 -p height:=1050 -p AwbEnable:=false -p ColourGains:='[1.4, 3.3]'
```

Terminal 2, the follower. Change the log name and `use_color` for each test:

```bash
source /opt/ros/jazzy/setup.bash; source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash; export ROS_LOCALHOST_ONLY=1
cd ~/stream3/follow_me_pupper
python3 robot/follow_service.py --ros-args -p record_path:=$HOME/stream3/logs/NAME.jsonl -p use_color:=true
```

Terminal 3, the controls:

```bash
source /opt/ros/jazzy/setup.bash; source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash; export ROS_LOCALHOST_ONLY=1
ros2 service call /activate_person_following std_srvs/srv/Trigger
ros2 service call /person_following_status std_srvs/srv/Trigger
ros2 service call /deactivate_person_following std_srvs/srv/Trigger
```

| Log name | What to do | What it decides |
|---|---|---|
| `calib_distance` | One person stands still for 5 s each at 0.5, 0.75, 1.0, 1.5, 2.0 and 3.0 m from the camera, measured with a tape | The true distance of `STOP_BOTTOM = 414`, and the commanded speed at range |
| `calib_color` | One person walks the whole room for 60 s, then a person in a different shirt does the same | `REID_MAX_DIST`, and if color can separate people here |
| `short_occlusion` | Walk behind a person-sized object for less than 1 s, three times | The lock stays |
| `long_occlusion` | Stay hidden for 3 s while a bystander stands in view, three times | The bystander never becomes the target |
| `leave_return` | Leave the view for 5 s and come back, three times | Color on resumes, color off stays `LOST` |
| `cross_diff` | Two people in different shirts cross, three times each way | Color on keeps the target |
| `cross_same` | Two people in similar shirts cross, three times | The follower stops, it does not swap |
| `legs_hidden` | Walk behind a table so that it hides your legs | The target stays |
| `lifecycle` | Run activate and deactivate 20 times | Read `detector ready in X s` each time. X must stay well below 3 s, which is what Lab 6 waits |
| `stall` | While following, stop terminal 1, wait 5 s, start it again. Then stop terminal 2 and start it again | Status shows the stale frames, and the service recovers |

Do `long_occlusion`, `leave_return` and `cross_diff` once with `use_color:=true` and once with `use_color:=false`. Then press Ctrl-C in terminals 1 and 2, copy `~/stream3/logs` to the Mac, and read each log:

```bash
python3 robot/follow_replay.py --log logs/calib_distance.jsonl --table
python3 robot/follow_replay.py --log logs/cross_diff.jsonl --color both
```

## Part B: walking

Do Part B only after Part A sets `STOP_BOTTOM` and the color choice. Stream 1 owns the Lab 6 session, so start it with the Stream 1 procedure in `foundation/HARDWARE_CHECKLIST.md`. Put the robot on the floor, with clear space, and keep your hand on the e-stop (right stick). A hot motor stops the session.

| Test | Pass | Reps |
|---|---|---|
| Walk in from 1, 2 and 3 m, straight and at 45 degrees | Stops within 15 cm of the chosen distance, no contact | 3 each |
| Lead it on a curve at a walk | Keeps the target, stops when you stop | 3 |
| Repeat `long_occlusion`, `leave_return` and `cross_diff` | Never follows the wrong person | 3 each |
| A person steps into the path | No forward motion toward them | 3 |
| Say "follow me", ask a question, then give a motion command | Keeps following through the question, stops before the motion | 3 |
| Stop while following: R3 | The robot stops, and following is off | 3 |

For each pass, record the commit, the settings, the lighting, the distance range and the speed. Keep the log and a video. A failed run is evidence too, so keep it.
