# Pupper BCI

The Ultracortex/Cyton provides two inputs. Tilt your head toward either shoulder to turn Pupper in that direction. Center your head to stop turning. A calibrated focus score gates a fixed 0.15 m/s forward command. A tilt and a forward command can be active together. Steering uses a 5° deadband around your calibrated center and tops out at 0.35 rad/s by 25°. Turn commands start above the stock walker's heading-hold threshold so small tilts can steer while walking. The Cyton USB dongle feeds one Python process.

This adaptation derives its tilt and EEG feature from [Thryambak Ganapathy's telekinetic-car at 6645143](https://github.com/thryambakg/telekinetic-car/tree/66451431136bfb7aa075c312eeec8027b29131fe). The detector measures 12–15 Hz power from two EEG channels using 1.024 s windows. It rejects gross spikes, flat traces, amplifier clipping and overlapping calibration scores. Blinks and muscle activity can still affect the score. Releasing the enable button requests zero on both axes independently of the focus score. [Forehead muscle artifact study](https://pubmed.ncbi.nlm.nih.gov/12948787/)

## Run without a headset or robot

From this folder in the CS123 checkout, use the existing numerical environment.

```sh
../.venv/bin/python -m pupper_bci replay
PYTHONPATH=.. ../.venv/bin/python -m pytest -q tests
```

The built-in replay runs synthetic rest, focus, left and right tilts, and headset dropout. Look for turn commands with zero forward speed during relaxed tilts, fixed forward speed during focus, and zero on both axes at the end. It simulates the enable button and never opens a device connection. The unit tests also check malformed samples, input freshness, enable rearming, and command ownership.

If the worktree has no numerical environment, use the interpreter from your existing checkout or create a local environment. With Python 3.11 or 3.12, run `python3 -m venv .venv` and `.venv/bin/python -m pip install -r requirements.txt`. Replace `../.venv/bin/python` in the examples with `.venv/bin/python`. Install `pytest` in that environment to run the tests.

Offline verification also captured and replayed 9,000 emulated Cyton packets through a pseudo-terminal on ARM Linux. The stock gait walked straight and turned at the selected commands in nominal MuJoCo simulation. Headset calibration and floor behavior await hardware checks.

The [ROS smoke test](tests/ros_smoke.py) verifies the actual stock mux with simulated robot feedback and a gamepad. From the Pupper checkout root, use the [existing offline ROS image](../foundation/README.md#local-numerical-setup).

```sh
docker run --rm --network none --cap-drop ALL --security-opt no-new-privileges \
  -v "$PWD:/repo:ro" -e PUPPER_ISOLATED_ROS_TEST=1 \
  -e ROS_DOMAIN_ID=166 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e PYTHONPATH=/repo/telekinetic-car -e PYTHONDONTWRITEBYTECODE=1 \
  cs123-ros-jazzy-slew-check:latest bash -ec '
    source /opt/ros/jazzy/setup.bash
    cmake -S /repo/pupperv3-monorepo/ros2_ws/src/cmd_vel_mux -B /tmp/mux -DBUILD_TESTING=OFF
    cmake --build /tmp/mux --parallel 2
    timeout 25s python3 /repo/telekinetic-car/tests/ros_smoke.py \
      --mux /tmp/mux/cmd_vel_mux_node --policy /repo/pupper_gait_deploy/policies/default_policy.json
  '
```

## Record and preview when the headset arrives

Use a stock eight-channel Cyton in its default accelerometer mode at 250 Hz. Mount it so the X acceleration changes as you tilt toward a shoulder. Use the matching electrode indices with `--channels FP1 FP2` if your Fp1/Fp2 leads are on channels other than 1 and 2. Plug in the Cyton's dongle and supply its serial port, commonly `/dev/cu.usbserial-…` on macOS or `/dev/ttyUSB0` on Linux. Close other apps using the port. [OpenBCI Cyton serial format](https://docs.openbci.com/Cyton/CytonDataFormat/)

```sh
.venv/bin/python -m pupper_bci capture --port "$CYTON_PORT" --output /tmp/cyton-test.csv
.venv/bin/python -m pupper_bci replay --recording /tmp/cyton-test.csv
.venv/bin/python -m pupper_bci monitor --port "$CYTON_PORT"
```

For capture, start with your head centered. Stay relaxed for the first 12 seconds and focus with your head still for the next 12 seconds. In the test segment, tilt both ways while relaxed, then alternate focus and rest. Preview the resulting forward and yaw commands on screen. Confirm relaxed tilts keep forward at zero, both signs match the intended turns, and forward release feels usable. Use `--flip-tilt` on preview and drive if the mounting reverses the steering. If calibration refuses the signal, check contact and repeat with your head still. Longer calibration stages are available through `--stage-seconds`; use the same value when capturing and replaying a file.

Capture writes elapsed seconds, Cyton packet number, two EEG channels in µV, and optional XYZ acceleration in g. Keep actual EEG recordings local unless the wearer chooses to share them.

## Supervised Pupper session

Use the selected plain stock **walk** release with the private movement mux and Stop gateway. The [stock runbook](../final_project/behaviors/ROBOT_RUNBOOK.md#select-the-runtime-once) explains release selection, supported startup, and feedback checks. BCI uses the existing private mux input, named `/kick_cmd_vel` in this profile, as its sole motion source. The program verifies the pinned running session and controller feedback. It cannot start or switch controllers.

Run on the Pi with the dongle attached and the same sourced ROS environment as that session. If setting up a dedicated Python environment there, use the ROS-compatible system Python with `python3 -m venv --system-site-packages .venv`, then install [the pinned dependencies](requirements.txt). Confirm that the gamepad reports the intended hold button in `/joy`. Choose an unused shoulder button and keep R3 available for operator Stop.

```sh
.venv/bin/python -m pupper_bci drive --port "$CYTON_PORT" \
  --runtime-receipt "$PUPPER_RUNTIME_RECEIPT" --enable-button "$HOLD_BUTTON_INDEX"
```

Calibrate with the robot stationary and the enable button released. After the selected walker is active and the program shows readiness, release and then hold the button to drive. Headset dropout or invalid data requires a new release and press after input recovers. A serial-port disconnect, an operator Stop, or a change in the motion path ends the session. Ctrl-C exits. Qualify the actual installation with the robot supported before the first short floor trial.
