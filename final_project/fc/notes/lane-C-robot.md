# Lane C — robot side (2026-11-23)

## Profiles

**`tej_u6`** runs Tej's kit as is: `kickseen6-prepare.sh` then `kickseen6-session.sh start-native`.
- The scripts are copied with only `RUN_DIR` changed, to `~/fc-v1/runs/<UTC>-tej_u6/session`. The copy is refused if the kit's sha256 changed.
- Underneath: persistent_session → `lab6_session.launch.py`, domain 168, driver `1dd92b65`, native `37938659`, gateway, `cmd_vel_mux`, camera and recorder.
- No `start-api`.

**`stock`** runs `ros2 launch neural_controller launch.py teleop:=False` (monorepo, domain 0), plus teleop remapped to `/teleop_cmd_vel` and `cmd_vel_mux_node` (500 ms).

Why stock:
- It is the simplest stack known to walk: 25 min on 2026-11-19 (`~/launch_p3_pupper1.log`) with no driver fault.
- It uses the pristine driver `01813d58` and the same policy as u6 (`ecf3a4ab`).
- The FC Nov 16–19 releases use u6's driver (same stale-IMU kill) and add more layers, so they are not simpler.
- Caveat: the monorepo neural_controller has local changes. It is "as installed", not pristine.

Why the mux: the stock launch publishes teleop straight to `/cmd_vel`, and the walker holds the last `/cmd_vel` forever. The mux supplies the dead-man and gamepad priority. tej_u6 gets the same teleop node (`FC_TELEOP=0` disables it).

Standing up: X activates `neural_controller_default` in both profiles (tej_u6 needs Options first). R3 is the e-stop.

## Mux topic

`/llm_cmd_vel` (priority 1):
- Only robot_api publishes it, and robot_api runs in neither profile.
- It outranks the follower's `/person_following_cmd_vel`.
- The gamepad counts only outside the deadband.

Set with `--topic`.

## Verified

- 32 pure tests pass.
- Domain-77 loopback on mts via `fcctl bridge start`:
  - 50.0 Hz output.
  - One zero 20 ms after ttl expiry, then silence.
  - Malformed and out-of-order packets rejected; a 9 m/s packet clamped.
  - IMU summary present in the acks.
  - LAN acks 20/20, median RTT 9.7 ms.
  - SIGINT mid-stream publishes a zero.
- Bridge → mux → `/cmd_vel` in domain 77: the mux forwards at 20 Hz, holds the bridge zero about 0.5 s, then goes to `none`.
- fcctl refusals checked.
- The tej_u6 script copy differs from the kit only in RUN_DIR.
- **No profile was started or stopped.**

**Incident:** a bare `fcctl.sh imu-test` ran the IMU lane's `imu_test.py`, which read the BNO085 over I2C for 60 s (no motors, no stack). Output: `~/fc-v1/imu/runs/20261123T105100Z-bno-unlabeled/`. `imu-test` now requires seconds.

## Preservation

- `vendor/tej_u6/`: 280 text files, 3.0 MB. Release, kit, driver source and gateway source.
- `pupper-foundation-artifacts/tej_u6_20261120.tar.gz`: 279 MB, sha256 `be1cd1e2…c38781`. Contents:
  - The u6 release (72 MB) and the kit in full.
  - The pinned dependency installs/sources: imu-startup-trace, static-idle-native, gateway-overlay-v4, imu-diagnostic, lab6-deployment-r6, pupper_gait_deploy.
  - No build/, log/ or venv.
  - The driver and native .so files match the kit's hashes.

## First floor test

1. Pi below 65 °C. Robot on the stand: `start stock --confirm`.
2. On the floor: X, then `bridge start`.
3. From the laptop: 0.1 m/s for 2 s, then stop sending. It should stop in about 0.3 s.
4. Kill the sender mid-walk.
5. Stick override.
6. R3.
7. `stop`.
8. Repeat with `tej_u6` and compare gait and IMU acks.

## Risks

- The stock driver never flags a frozen IMU: watch `imu.stale_ms`. u6 kills the hardware after one receipt older than 100 ms.
- `stop` deactivates the legs, so the robot drops.
- The Nov-19 stock shutdown aborted in the driver destructor: check that the motors go limp.
- tej_u6 also runs the camera, recorder and follower, which adds heat.
- `status` takes about 2 s while a stack runs.
- Only `mts` was checked, and paths are hard-coded.
