# 4 · Run on the robot

## Before launch
- Battery charged. Robot **on its stand, feet off the ground**. Clear floor. Never test on a table.
- Gamepad connected (`ls /dev/input/js0`).
- `./check_imu.sh` with nothing running. It must print `IMU OK`. See [06](06-troubleshooting.md#imu-bno055) if it doesn't.

## Launch

```bash
./launch.sh 2>&1 | tee ~/launch.log
```

Wait for `Finished homing!`. All policy controllers start **inactive**, so nothing moves until you press a button.
Abort if `[BNO055] failed …` lines appear: the IMU is down, and the policy and the fall e-stop rely on it.

## Gamepad

| Button | Controller |
|---|---|
| X | `neural_controller_default` (course default walk) |
| **Square** | `neural_controller_student` = **your installed policy** |
| Triangle / Circle | legacy walk / three-legged |
| Left stick / right stick | velocity / turn |
| **R3** (right-stick press) | **e-stop**: relaxes the motors |
| Options | release the e-stop, re-activate the last controller |

The e-stop is latched at startup. If a button does nothing, press **Options** first.
Shell equivalent of Options: `./start_motion.sh`. It reads one live stop status and calls `/estop_controller/start_motion` within its 500 ms window. It needs the gamepad connected, cannot clear a fault, and moves the robot.

## Test order (stop at the first bad sign)
1. On the stand: **X** (baseline), then **Square**. Expect a sane stance and roughly symmetric legs.
2. On the floor, standing, stick at zero: watch the **left back leg** (bumpy's sim defect). Optionally run `./stand_check.py` ([05](05-measure.md)).
3. Slow walk under 0.3 m/s, forward and back.
4. Turns and sidesteps (lift gait).
5. Gallop above 0.5 m/s **last**, in an open hallway, thumb on R3.

Stop if the robot is hot or a motor faults. Ctrl+C the launch to shut it down, then check that the motors went limp.

`ros2_control` logs `Could not enable FIFO RT scheduling` without real-time limits. That is harmless, but watch `/neural_controller_student/policy_inference_latency_seconds`; the budget is about 19 ms.
