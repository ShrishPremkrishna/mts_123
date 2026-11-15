# Header kick robot trials

Step-by-step procedure for the first physical trials of both kicks: the walker-based run-through kick (`run_through_kick*.py`) and the learned header kick (`learned_kick*.py` with `policies/header_kick_cpu_run2.json`). Neither kick has run on hardware. Do the stages in order, stop at the first failure, and record every attempt in `trial_log.csv`.

**R3 (right-stick press) is the stop.** Ctrl-C, a service call or a crashed program can leave the last command on the walker, because `neural_controller` keeps the last `/cmd_vel` with no age limit (see "Remaining risks").

## What is verified before the robot

| Check | Run-through kick | Learned kick |
| --- | --- | --- |
| Controller logic, kinematic walker/camera model | 16 tests; 200/200 random setups reach the ball | not applicable |
| ROS node logic, offline replay with stand-in rclpy | 14 tests: /cmd_vel ownership, latched gateway, manual bounds, bad stamps, travel budget on a re-stamped stale ball, stop by service/R3/button/latch/teleop | 14 tests: entry window, ownership, latched gateway, confirmed switches with no target before them, exit with targets until confirmed, refused or lost exit → emergency stop, square, IMU loss, frozen target |
| Real ROS run | not done (no ROS on the Mac) | not done |
| Physics simulation | not done (shipped walker not reproducible in mjlab) | 84.6% success at full difficulty with the exported JSON (PyTorch: 82.6%) |
| Deploy validator (`download_policy.py --validate-only`) | not applicable | consistent |
| Hardware | not run | not run |

## Before the session (both kicks)

1. Get Shrish's go. The Pi has a handover hold, and these steps change what runs on it.
2. One operator owns motion, with R3 in reach the whole time. Follow `foundation/HARDWARE_CHECKLIST.md`.
3. Clear a 3 m lane. People stand behind the start line or at least 1.5 m to the side, never in front of the robot or the ball.
4. Use a ball of at least 15 cm diameter: the chassis front is about 11 cm above the floor and the front legs are about 16 cm apart, so a smaller ball can go under the body.
5. Use a stand for every stand test. Do not hold the robot by hand while a kick runs.
6. Learned kick only:
   - Get the course approval for a new policy.
   - Fit the bumper from the simulation: a 2 × 11.4 × 12 cm foam pad on the front face, bottom edge 5.5 cm above the floor at standing height, with a cutout for the camera.
   - Have live ball estimates on `/soccer/ball` at 10 Hz or more, stamped with the robot's clock (synchronize the clocks if perception runs on another machine). There is no manual mode for this kick.

## 0. Files on the robot

From the Mac (`<pupper>` is the robot address). This copies only the kit; the Lab 6 release stays as it is:

```sh
rsync -a --delete final_project/behaviors/ pi@<pupper>:~/kick/
```

## 1. Course stack

Start the course stack **with joystick teleop off**. Teleop defaults to on and would publish `/cmd_vel` at 50 Hz over the kick; both kick nodes refuse to start while another node publishes `/cmd_vel`.

```sh
ros2 launch neural_controller launch.py teleop:=False
```

The e-stop gateway starts latched, so the face buttons do nothing yet. Press **Options**: the X walker activates and stands. Both kick nodes refuse to start while the gateway is latched.

## 2. Run-through kick (manual mode, no perception)

Start the mux and the kick node. The node publishes zero on `/cmd_vel` when it shuts down, and the mux sends zero 150 ms after the node goes quiet.

```sh
ros2 launch ~/kick/launch/kick_trial.launch.py kick:=run_through input_mode:=manual charge_speed:=0.45
```

Each kick needs a new measured ball position: the centre of the ball from the centre of the body, x forward (0.9 to 1.5 m), y left (|y| ≤ 0.5 m). Always type decimals (`1.0`, not `1`). Each activation consumes the measurement and resets `manual_ball_y` to 0.0 and `manual_aim_deg` to straight through. Other parameters (for example `speed_efficiency`) set with `ros2 param set` apply from the next activation.

```sh
ros2 param set /run_through_kick manual_ball_x 1.0
ros2 param set /run_through_kick manual_ball_y 0.0
ros2 service call /activate_run_through_kick std_srvs/srv/Trigger
ros2 topic echo /run_through_kick/state
```

The node stops on R3, on any face button, on the deactivate service, when the gateway latches, and when another node starts to publish `/cmd_vel`. After R3, check that the state shows `aborted`, then press Options.

| Stage | Setup | Pass |
| --- | --- | --- |
| A. Stand test | Robot on the stand, feet free; ball "at" 1.0 m | Legs walk, then stop after about 4 s; R3 and the deactivate service each stop them; with `ros2 topic echo /cmd_vel` running in another terminal, the last message after Ctrl-C of the trial launch is all zero |
| B. Floor, no ball | Ball "at" 1.0 m; mark the start and the 1.0 m point | The body centre passes the 1.0 m mark (the command drops to zero there), then the robot brakes and stays up. If it stops before the mark, the dead reckoning overestimates progress: lower `speed_efficiency` in proportion, for example 0.8 × 0.9 m / 1.0 m = 0.72 (`ros2 param set /run_through_kick speed_efficiency 0.72`) |
| C. Ball, slow | Real ball 1.0 m ahead, 0.45 m/s, 5 trials | Contact, ball rolls forward, robot stays up |
| D. Speed ladder | Relaunch the course stack with the raised cap (below). For `charge_speed` 0.7, then 1.0: repeat stage B to recalibrate `speed_efficiency` (the walker changes gait above 0.5 m/s), then 5 trials with the ball | Same, with longer ball travel |
| E. Aim | Ball at (1.0, ±0.2); `manual_aim_deg` 0.0, then 15.0 and -15.0 | Ball direction within about 10° of the aim |

Raised walker cap, for stage D only (check first that the launch has `config_file` with `ros2 launch neural_controller launch.py --show-args`):

```sh
python3 ~/kick/tools/make_course_config.py --speed 1.0 --out /tmp/kick_speed_config.yaml
ros2 launch neural_controller launch.py teleop:=False config_file:=/tmp/kick_speed_config.yaml
```

Afterwards, relaunch with the plain command from section 1. Never use the raised cap in a learned-kick session.

Dead reckoning drifts. Move to `input_mode:=field` (overhead tracking) or `robot` (onboard detection) when the perception topics exist. A travel budget (2.5 m while aligning, about 1.35 m for the charge) ends a kick whose ball estimate looks fresh but is wrong; a goal straight behind a far ball can also use up the align budget.

## 3. Learned kick

Stop everything from sections 1 and 2 first: R3, robot on the stand, Ctrl-C both launches, and check that `ros2 node list` shows no course or kick nodes.

Install the policy in the square-button slot on the robot. The script validates the policy, shows `install.py --dry-run`, and asks before it changes anything. The install replaces the course config, launch file, controller sources and policies in the monorepo with the ones in the team checkout (`/home/pi/mts_123/pupper_gait_deploy` by default) and rebuilds; the previous slot policy is kept as a backup.

```sh
~/kick/tools/install_learned_policy.sh ~/kick/policies/header_kick_cpu_run2.json
```

Make a config copy for the kick slot: e-stop at 0.65 rad of tilt (training ended episodes at 35°; the course value is 86°), and 0.5 s init plus 0.5 s fade-in (the course values are 2 s each, during which the policy acts only partly):

```sh
python3 ~/kick/tools/make_course_config.py --kick-max-body-angle 0.65 --kick-init 0.5 --kick-fade 0.5 --out /tmp/learned_kick_config.yaml
ros2 launch neural_controller launch.py teleop:=False config_file:=/tmp/learned_kick_config.yaml
```

Press Options so the walker stands, and check that the kick controller is loaded and inactive:

```sh
ros2 control list_controllers
```

Start the bridge (with the fade-in that matches the config copy), then activate. Do not press square to start: the bridge switches the controllers through the e-stop gateway. During a kick, square ends it normally and any other face button aborts it.

```sh
ros2 launch ~/kick/launch/kick_trial.launch.py kick:=learned input_mode:=robot fade_in_s:=1.0
ros2 service call /activate_learned_kick std_srvs/srv/Trigger
ros2 topic echo /learned_kick/state
```

A kick refuses to start unless the newest ball estimate is at most 0.3 s old, the ball is 0.45 to 1.5 m away and within ±29° of straight ahead, and the aim is within 23° of the ball line. The bridge then checks the controllers, switches to the kick policy, and sends targets only after the gateway confirms (zero until then). The kick window starts when the ball enters 0.45 m (the camera blind zone) and lasts 1.25 s; then the bridge keeps sending the newest target until the walker is confirmed back, and sends zero. It aborts on a lost estimate (0.25 s far from the ball), a ball more than 57° off the nose, a target that stops changing, IMU loss, or 3 s after the fade-in. If the gateway refuses or does not answer the way back within 2 s, the bridge publishes an emergency stop.

| Stage | Setup | Pass |
| --- | --- | --- |
| A. Stand test | Robot on the stand. In a second terminal, start `python3 ~/kick/tools/replay_ball.py --x0 1.2 --speed 0.12`, then activate within 1 s | State goes checking → switching_in → running → exiting → zero_hold → idle; legs move; the walker is active again at the end. Repeat once with R3 during the run: the bridge aborts and publishes zero |
| B1. Floor, short | Real ball and live estimates, 0.6 m ahead, robot standing | Header contact, ball moves forward, robot returns to the walker upright |
| B2. Floor, normal | Ball 1.0 m ahead | Same |

After an e-stop: the bridge has already stopped and published zero. Options re-activates the controller of the last *face button* press, which is the X walker unless someone pressed square by hand; the walker receives no old target.

## 4. After every session

1. Record every attempt (successes, misses, falls, aborts) in `trial_log.csv`, with video file names.
2. Restore the square slot (it would run the kick policy on a stray square press, and joystick teleop would feed it targets): copy the newest `policies/test_policy.json.bak.*` in the team checkout back to `policies/test_policy.json`, run `python3 install.py` there, and restart the stack.
3. Note the measured `speed_efficiency` per speed, the closest reliable detection (`blind_range`) and the body or bumper front (`contact_x` minus the ball radius) in the README.

## Remaining risks (not fixable in this kit)

- `neural_controller` keeps the last `/cmd_vel` forever. If a kick program is killed hard (SIGKILL, power loss of the laptop running perception, a crash before its cleanup), the walker can keep the last velocity and the learned policy keeps chasing the last target until R3. A command-age limit in `neural_controller` (the team plan lists "controller-local expiration") would remove this.
- The walker's 2 s re-initialisation after a learned kick has not been tested at speed. The bridge exits only after the trained 1 s post-kick window, in which the policy learned to brake.
- Nothing here has run on ROS or hardware; the stand tests are the first real checks.
