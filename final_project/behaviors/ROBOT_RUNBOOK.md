# Stock run-through shot: supervised baseline

Updated November 16, 2026. This replaces the recovered experimental procedure;
its original bytes remain in the archive listed in [RECOVERY.json](RECOVERY.json).
See [the reconciliation report](../PI_RECONCILIATION.md) for the actual Pi trials
and policy inventory. Software completion is not evidence of ball contact.

## Before motion

- One operator owns the robot, confirms readiness and watches with R3/power
  available. Use [the hardware procedure](../../foundation/HARDWARE_CHECKLIST.md)
  for supported startup and shutdown. The current user has requested offline
  preparation; this document does not authorize activation.
- Use the stock default actor (`ecf3a4ab…`), a healthy battery and a clear lane.
  The current temporary ball is about seven inches across, soft/partly deflated,
  with no bumper. Whether the chassis makes clean contact remains to be measured.
- Start from a verified native configuration with `game_buttons=false`, forward
  cap 0.49 m/s and backward cap 0.3 m/s. Keep the native configuration and model
  fixed for the session. The native controller caches parameters at initialization:
  changing its ROS parameter store does **not** establish applied settings.
  A changed configuration needs a fresh controller session.
- Select `neural_controller_default`; the installed student slot contains the
  unaccepted learned header policy. Do not select it or assume zero velocity is
  a stop for that target-conditioned policy. No policy installer is needed here.

Preserve `/home/pi/kick` as the recovered experiment. Run updated source from
`/home/pi/mts_123/final_project/behaviors`; do not overwrite the old directory.
Source synchronization does not install a native runtime. The currently installed
native launch ignores the historical `config_file` override because it does not
declare that argument, although the monorepo's newer source supports it.

## One session, successive measured trials

The plain course stack must have joystick **teleop off**, while retaining R3 and
the e-stop gateway. Its startup can home the legs: only launch once the operator
is ready on the stand. Do not start this private mux alongside Gemini's own
motion owner. Keep the same homed session between trials.

Once the approved stack is running and standing, start only the shot adapter:

```sh
ros2 launch /home/pi/mts_123/final_project/behaviors/launch/kick_trial.launch.py kick:=run_through input_mode:=manual charge_speed:=0.45
```

Activation requires fresh controller/parameter evidence, the pinned stock policy
file, a released gateway and the expected publisher count. Unsupported settings
are refused, not silently clipped. The node's alignment limit is 0.3 m/s; requested
charge speed may not exceed the smaller of the native cap and 0.49 m/s.

Measure the ball centre from the body centre: x forward, y left, in metres. Manual
mode accepts x from 0.9 to 1.5 and |y| at most 0.5. Each accepted shot consumes the
measurement; remeasure before the next shot. Decimal values are required.

```sh
ros2 param set /run_through_kick manual_ball_x 1.0
ros2 param set /run_through_kick manual_ball_y 0.0
ros2 service call /activate_run_through_kick std_srvs/srv/Trigger
```

| Trial | Observable result |
| --- | --- |
| Marked advance, no ball | Record actual displacement, elapsed time, braking distance and stability at 0.45 m/s. Compare with the command integral; calibrate `speed_efficiency` from measured travel, not a `done` receipt. |
| Same approach with ball | Record contact/no contact, ball travel/direction, robot stability and stopping. Repeat up to five times if controlled; a miss or fall is useful evidence, not a reason to repeat unchanged. |
| Offset/aim trial, after straight contact works | Measure lateral offset and requested heading, then compare the ball's actual direction. |

If contact is poor, inspect ball/body geometry and measured motion before changing
policies. Try one documented alternate only when its measured behavior addresses
the failure. Faster profiles require a separate reviewed configuration/guard
update; the recovered speed-ladder and learned-kick installer recipes are not the
current procedure.

## Stop and evidence

R3/power remain available throughout. `/deactivate_run_through_kick` requests a
normal stop when stock authority is known. Loss of stock authority during a shot
requests `/emergency_stop`, then aborts; it never automatically resumes. This is
software intent, not proof that the robot stopped. Support the robot and follow
the hardware shutdown procedure before disconnecting the driver.

Record source revision, initialized native config/model identities, ball/bumper,
requested speed, measured displacement/contact, result and any video reference.
The existing `trial_log.csv` is a compact starting point; raw evidence belongs
outside onboarding docs. Keep failures as well as successful shots.

Remaining boundaries: sampled controller evidence is not exclusive ownership;
publisher counts do not identify a publisher; native command expiry is still
missing if the mux itself fails. The camera path also needs real ball/range
calibration and consumer frame/timestamp validation before integration. These
limits do not disappear when offline tests pass.
