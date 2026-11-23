# fc/robot — Pi side of Pupper FC

Laptop `link.py` → UDP :9870 → `drive_bridge.py` → `/llm_cmd_vel` → `cmd_vel_mux` (500 ms timeout,
gamepad `/teleop_cmd_vel` wins) → `/cmd_vel` → walker. Contract: [../SPEC.md](../SPEC.md).

## Files

| File | What |
| --- | --- |
| `drive_bridge.py` | rclpy node. 50 Hz Twist while the newest packet is younger than its ttl (0.25 s default, 0.5 s cap), then one zero and silence. Accepts only HMAC-signed packets for its `--robot-id` (key `link.key`, 0600), rejects malformed/replayed packets (replay state survives restarts), clamps to \|vx\|≤0.5, \|vy\|≤0.3, \|wz\|≤2.0. Holds zero on a bad IMU or a missing/stalled mux (below). Signed ack (seq, `imu`, profile, `hold`, `imu_hold`, `mux_ok`, stats) to accepted packets only. |
| `fcctl.sh` | Runs on the Pi from `~/fc-v1/robot/`: `status` (incl. `phase`/`light`, `mux_alive`, `mux_ok`, `ros.mux`), `ready [profile] [--confirm]`, `down [--keep-stack]`, `start <profile> --confirm`, `stop`, `bridge start\|stop`, `imu-test <seconds>`, plus `walk`/`rest` and `boot` (service only). |
| `readiness.py` | Pure: the `status` phase (`off/starting/homed/standing/ready/fault`) and the controller check `ready` uses. |
| `boot/` | Power-on service, prepared and **not installed**: `fc-boot.service`, `install.sh` (install/uninstall/status/print). What it does, rollback and the homing-posture evidence: [boot/README.md](boot/README.md). |
| `profiles/stock.sh` | Class platform: monorepo `ros2 launch neural_controller launch.py teleop:=False` + teleop→`/teleop_cmd_vel` + `cmd_vel_mux_node`. Domain 0. |
| `profiles/tej_u6.sh` | Tej's `software-ballkick-20261120-u6` via his `kickseen6` kit (prepare + start-native), fresh run dir, no HTTP API; + teleop→`/teleop_cmd_vel`. Domain 168. |
| `profiles/stock_hh0.sh`, `profiles/tej_u6_hh0.sh` | The same stacks with the walker's heading hold off: set `FC_HEADING_HOLD=0` and source the base profile. Its `profile_start` first runs `fc_hh_overlay` (`imu/heading_hold.py`, lane D): a fresh kp=0 policy overlay in `<run>/hh0/`, re-read and verified (only `heading_hold.kp` and the policy path differ from the sources; refuses to write outside `~/fc-v1`). Any failure refuses the start before motion. stock: `ros2 launch <run>/hh0/launch_hh0.py` (src and installed launch/config/policy must be identical). tej_u6: kit scripts still checked against their sha pins; only prepare's `--launch/--config/--urdf` point at `<run>/hh0/foundation/hardware/`. `status` reports `heading_hold` (`on`/`off`) and `heading_hold_overlay` (policy sha). |
| `stage.sh <host>` | Laptop: rsync this dir (minus `vendor/`, `provision/`) to `<host>:~/fc-v1/robot/`, print changes; copy `fc/config/link.key` (0600) and write `robot_id` from the `robots.json` entry for `<host>` (or `FC_ROBOT_ID`). |
| `provision/` | Laptop: `provision.sh [--check] <host> <id> <team> <tag>` takes a new Pi to game-ready (read-only probe + readiness report; stages via `stage.sh`; writes only `~/fc-v1/` unless `--fix-clone-identity`). Second-robot plan, clone vs class card, and per-robot calibration: [provision/PROVISION.md](provision/PROVISION.md). Not staged to the Pi. |
| `keygen.py` | Laptop, once: `python -m fc.robot.keygen` writes `fc/config/link.key` (0600, gitignored). |
| `vendor/tej_u6/` | Text sources/configs/manifests of u6, the kit, its driver source and gateway source (3.5 MB). Full archive with binaries: `~/workspace/pupper-foundation-artifacts/tej_u6_20261120.tar.gz` (+ `.sha256`). |

## Velocity topology (one mux, one `/cmd_vel` publisher)

Checked on mts 2026-11-23 by reading the installed launch files (no stack started):

| Profile | Who starts the mux | `/cmd_vel` publishers | mux inputs, priority order (500 ms timeout) |
| --- | --- | --- | --- |
| `stock` | `stock.sh` (`ros2 run cmd_vel_mux cmd_vel_mux_node`). The Pi's installed `neural_controller/launch/launch.py` (identical to its `src/` copy) starts **no** mux, and `teleop:=False` really drops its teleop node. A laptop monorepo copy that starts a mux is a different revision. | `cmd_vel_mux` only | 0 `/teleop_cmd_vel` ← our `teleop_twist_joy` (gamepad); 1 `/llm_cmd_vel` ← `fc_drive_bridge` only; 2 `/person_following_cmd_vel` ← nobody |
| `tej_u6` | Tej's `lab6_session.launch.py` (a "required" node: its exit shuts the whole session down). `tej_u6.sh` starts no second one. | `cmd_vel_mux` only | 0 `/teleop_cmd_vel` ← our `teleop_twist_joy` (`FC_TELEOP=0` drops it); 1 `/llm_cmd_vel` ← `fc_drive_bridge` only (his HTTP API, the other publisher, is not started); 2 `/person_following_cmd_vel` ← his Lab 5 follower |

The joystick input needs |value| > 0.05 to take over, so the gamepad's idle zeros do not mask the bridge.
`start` reports READY only when the live graph (rclpy, `fcctl.sh status` → `ros.mux`) shows exactly
`/cmd_vel ← [cmd_vel_mux]`, the mux subscribed to `/llm_cmd_vel`, nothing but the bridge publishing
`/llm_cmd_vel`, and at most one `/teleop_cmd_vel` publisher; `stock` also requires the launch, teleop and
mux processes alive. Otherwise the start fails with the problem in `starter.log`.

## Bridge holds (defence in depth; the laptop also stops)

While held, the bridge publishes zero instead of the command (so the mux input stays alive) and acks
carry `hold` (reason), `imu_hold`, `mux_ok`, and `bridge.mux` (`why`, publisher/subscriber counts).

- **IMU** (`--imu-stop`, default on): ImuHealth `bad` with `bad_cause` `stale` or `tilt` → `hold:
  "imu_stale"|"imu_tilt"`, `imu_hold: true`, released after 1 s of `ok`/`warn`. Zero velocity does not cure a
  false tilt: lay the robot down with R3.
- **Mux** (`--mux-guard`, default on): `mux_unseen` (not yet discovered), `cmd_vel_publishers=N` (N≠1),
  `mux_not_subscribed`, `no_cmd_vel_publisher`, or `mux_stalled` (bridge publishing ≥0.5 s with no `/cmd_vel`;
  the mux republishes at 20 Hz). The last two mean the mux was lost: a dead mux leaves the walker on its
  last command, so the bridge then also publishes zero **directly on `/cmd_vel`** for 2 s (a temporary
  second publisher, created only then). A SIGKILLed mux stays in the ROS graph for a while (DDS lease);
  the stall check catches it in about 0.5 s. Its ghost also means a mux restarted within ~20 s shows as
`cmd_vel_publishers=2` (held, and `start` refuses READY) until the lease expires (seen on mts 2026-11-23,
domain 77). Without a bridge, R3 is still the only stop for a dead mux.
- `fcctl.sh bridge start --no-imu-stop` / `--no-mux-guard` turn them off (floor escape hatch; note it).

## Replay state

The bridge saves the newest accepted laptop `t`, the current sid + seq and up to 64 retired sids to
`~/fc-v1/run/bridge_state.json` (0600, at most 1 Hz and on exit) and reloads it on start, so a restart does
not reopen old sessions (a SIGKILL can lose the last second). Only laptop times are compared, never the Pi
clock. If the laptop clock steps back more than 2 s, or the file is damaged, packets are refused / the
bridge will not start: `fcctl.sh bridge stop; fcctl.sh bridge start --reset-replay`.

## Operating

```bash
python -m fc.robot.keygen                   # laptop, once (from final_project/)
fc/robot/stage.sh mts                       # laptop
ssh mts bash ~/fc-v1/robot/fcctl.sh status
ssh mts bash ~/fc-v1/robot/fcctl.sh start stock --confirm   # ROBOT ON THE STAND: homes the legs
#   move the robot to the floor, then:
ssh mts bash ~/fc-v1/robot/fcctl.sh ready   # stands the walker (tej_u6: waits for Options), starts the bridge
ssh mts bash ~/fc-v1/robot/fcctl.sh down    # bridge zero -> leg controllers off (robot lies down) -> stack down
```

`ready stock --confirm` with nothing running does the start (homing) itself and then stops at `homed`
(`"next":"ready"`): move the robot to the floor and run `ready` again (`--stand-now` stands it right away,
on the stand, for in-air tests). Manual equivalent of `ready`:
DualSense X (tej_u6: Options) or `fcctl.sh walk`, then `bridge start`. R3 = e-stop. Between matches `down --keep-stack` lies it down
and keeps it homed, so the next `ready` needs neither the stand nor `--confirm`.

## `ready` / `down` / `status` phase (interface for the laptop UI)

`ssh <host> bash ~/fc-v1/robot/fcctl.sh ready <profile> [--confirm [--stand-now]] [--no-bridge]` prints one
JSON object per line on stdout (logs go to stderr), for example (stack already homed, e.g. by the boot service):

```
{"phase":"begin","ok":true,"elapsed":0.0,"msg":"profile stock","profile":"stock","walker":"neural_controller_default"}
{"phase":"homed","ok":true,"elapsed":0.9,"msg":"homed (already running)","run_dir":"/home/pi/fc-v1/runs/..."}
{"phase":"standing_up","ok":true,"elapsed":2.6,"msg":"activating neural_controller_default (software stand: ...)"}
{"phase":"standing","ok":true,"elapsed":4.4,"msg":"neural_controller_default active; settling 3 s ..."}
{"phase":"bridge","ok":true,"elapsed":9.1,"msg":"bridge up","bridge":{"ok":true,"bridge_pid":1234,...}}
{"phase":"ready","ok":true,"elapsed":11.2,"msg":"ready to play","profile":"stock","run_dir":"...","done":true}
```

- Every line: `phase`, `ok`, `elapsed` (s since the command began), `msg` (show it). The **last line has
  `"done":true`**; success = last line `phase == "ready"` (or `standing` with `--no-bridge`) and `ok`.
- Phases in order: `begin`, `starting` (repeats every 5 s while homing), `homed`, `standing_up`,
  `waiting_operator` (repeats every 10 s; has `action`, e.g. "press Options on the gamepad ..."; it resumes by
  itself once the walker is active, up to `FC_READY_OPERATOR_S`, default 180 s), `standing`, `bridge`, `ready`.
- `needs_confirm` (exit 3, `done`): the stack is not running, so starting it would **home the legs**. Ask
  "robot on the stand?" and rerun with `--confirm`. Never add `--confirm` automatically.
- `homed` with `"next":"ready"` (exit 0, `done`): it just homed on the stand. Tell the operator "move it to
  the floor, then press Ready" and run `ready` again (no `--confirm`).
- `error` (exit 1, 2 bad arguments, 4 operator timeout; `done`): `at` names the failed step, `msg` says what
  to do. Another profile running → `at:"profile"` (`down` first, which means homing again).
- Exit code 0 only on success. Already-done steps are skipped ("already standing", "bridge already
  running"), so pressing Ready twice is harmless; a second concurrent `ready`/`down` is refused.

`down [--keep-stack]`: `begin`, `bridge_stopped`, `lying_down` (leg controllers off: damped collapse, the same
as `stop`), `down` (`done`; `"stack":"kept"` with `--keep-stack`, else the `stop` result in `stop`).

`status` adds `phase`, `light`, `phase_detail`, `walker`, `walker_active`, `active_legs`, `boot`:

| phase | light | meaning |
| --- | --- | --- |
| `off` | grey | no fcctl profile (or only state from an earlier boot) |
| `starting` | amber | homing / loading controllers |
| `homed` | amber | READY, no leg controller active (on the stand, lying, or after R3): Ready stands it |
| `standing` | amber | a leg controller active, bridge down |
| `ready` | green | standing + bridge up + mux ok |
| `fault` | red | start failed, stack died, foreign stack, wrong mux topology, manager not answering |

Blind spot: the stock walker's internal fall e-stop keeps the controller "active", so a fallen robot can
still read `standing`/`ready` until R3 (which deactivates) or `down`.

**Software stand, per profile** (source evidence 2026-11-23; not yet run on the robot):

- `stock`, `stock_hh0`: yes. The stock `estop_controller` X handler sends a plain BEST_EFFORT
  `SwitchController` (activate `neural_controller_default`, deactivate the other `controller_names`) to
  `/controller_manager/switch_controller` (monorepo `joy_utils/src/estop_controller.cpp` at `13299fa`);
  `ready` sends the same request (`rosq switch`, already used by `walk`). Activation blends from the
  current pose to the stance (`init_duration`), then fades the policy in. Assumption to confirm read-only:
  the Pi's installed `estop_controller` is that stock binary, e.g.
  `strings ~/pupperv3-monorepo/ros2_ws/install/joy_utils/lib/joy_utils/estop_controller | grep -c switch_controller_backend`
  = 0. If it were Tej's gateway instead, `ready` falls back to `waiting_operator` (no harm).
- `tej_u6`, `tej_u6_hh0`: no, by design. Tej's Stop gateway owns `/controller_manager/switch_controller`,
  starts **latched** and refuses every activation until a physical Options press (vendor
  `joy_utils/include/joy_utils/stop_arbiter.hpp` `submit()`; `estop_controller.cpp` constructs it latched;
  u6 has no software Start/release service). `ready` tries once (a refused request has no side effect),
  then shows "press Options" and continues by itself: Options alone activates `controller_names[0]` =
  `neural_controller_default`. Faking a `/joy` Options press would defeat the latch, so `ready` does not.
- After R3: stock `ready` re-stands it like X would (Ready is a deliberate operator action; nothing
  re-stands automatically). tej_u6 stays latched until Options.

Power-on autostart and the homing-posture finding: [boot/README.md](boot/README.md).

State lives in `~/fc-v1/run/` (pidfiles with process start times, `active` = profile + run dir, `lock`,
`bridge_state.json`); each start
gets `~/fc-v1/runs/<UTC>-<profile>/` with all logs. `start` refuses if any `ros2_control_node`,
`persistent_session.py` or `ros2 launch` is already running. `stop` only signals processes fcctl recorded
(tej_u6: Tej's own `session.sh shutdown` on our run dir) and never SIGKILLs. `FC_TELEOP=0` drops the added
teleop node; `FC_STATUS_BATTERY=1` makes `status` read the battery ADC (shared I2C bus, off by default).
The `lock` (fd 9) serialises `start`/`stop`/`bridge start`; spawned processes never inherit it, and `start`
releases it once the detached starter runs, so `stop` works during homing (it stops the starter first).
`ready`/`down` hold their own `ready.lock` (fd 8, also never inherited) and call those commands as children.
`active` line 3 is the boot id: state from an earlier boot reads as `off`, not as a dead stack.

Bridge-only test without a robot: `FC_ROS_DOMAIN_ID=77 fcctl.sh bridge start --profile stock` (with no
mux in domain 77 every command is held as `mux_unseen`; start a lone `cmd_vel_mux_node` there to see
`/cmd_vel`). `bridge start` also takes `--robot-id ID` (default: `robot_id` file), `--allow-from IP[,IP]`,
`--reset-replay`, `--no-imu-stop`, `--no-mux-guard`.

Tests (no ROS): `.venv/bin/python -m pytest -q fc/robot/tests` from `final_project/`. `test_fcctl.py` (fake
profile: lock, stop during homing, topology gate, heading-hold overlay gate, `ready`/`down` incl. a refusing
gateway and operator timeout, status phases, boot modes) needs Linux and is skipped on the Mac; run it with (`--init` reaps zombies; without it pytest is PID 1 and `stop` waits on dead pids)
`docker run --rm --init --network none -v "$PWD/fc/robot:/src:ro" ros:jazzy-ros-base bash -c 'cd /src && python3 -m pytest -q -p no:cacheprovider tests/test_fcctl.py'`.
