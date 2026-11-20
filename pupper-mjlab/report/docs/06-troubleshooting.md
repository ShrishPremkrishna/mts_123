# 6 · Troubleshooting

## IMU (BNO055)
Symptom: `[BNO055] failed to read header for receivePacket: Connection timed out`, or `Remote I/O error`, in the launch log. `./check_imu.sh` shows no device at 0x28/0x29.
- The IMU is on `/dev/i2c-1`, and the driver keeps running with stale data. **Do not activate a policy**: the policy and the tilt e-stop both read the IMU.
- Fix: stop the launch, power-cycle the whole robot (battery off for about 10 s), then rerun `./check_imu.sh`. Relaunching alone did not fix it on the loaner.

## Gamepad does nothing
- Not connected: `ls /dev/input/js0`. Press PS. Pair it again if needed ([01](01-pi-setup.md)).
- E-stop latched: press **Options**.
- Another program holding the pad: `fuser -v /dev/input/js0`.

## Build
- `./deploy.sh` or `install.py` fails on `test_neural_controller_safety.cpp`: use `./install.sh`, which builds with tests off ([03](03-install.md)).
- The monorepo `build.sh` exits 0 even when colcon fails. Read `~/pupperv3-monorepo/ros2_ws/log/latest_build/` instead.

## W&B
- `API key must be 40 characters`: wandb is older than 0.22.3. Upgrade it ([01](01-pi-setup.md)).
- `404 policy.json`: the run never uploaded it. Ask the trainer to upload it, or copy it with scp ([02](02-get-policy.md)).

## Two stacks
Only one `ros2_control_node` may run at a time. Check with `pgrep -af "ros2 launch|ros2_control_node"`.
