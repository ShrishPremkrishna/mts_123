# fc/robot — Pi side of Pupper FC

Laptop `link.py` → UDP :9870 → `drive_bridge.py` → `/llm_cmd_vel` → `cmd_vel_mux` (500 ms timeout,
gamepad `/teleop_cmd_vel` wins) → `/cmd_vel` → walker. Contract: [../SPEC.md](../SPEC.md).

## Files

| File | What |
| --- | --- |
| `drive_bridge.py` | rclpy node. 50 Hz Twist while the newest packet is younger than its ttl (0.25 s default, 0.5 s cap), then one zero and silence. Accepts only HMAC-signed packets for its `--robot-id` (key `link.key`, 0600), rejects malformed/replayed packets (replay state survives restarts), clamps to \|vx\|≤0.5, \|vy\|≤0.3, \|wz\|≤2.0. Holds zero on a bad IMU or a missing/stalled mux (below). Signed ack (seq, `imu`, profile, `hold`, `imu_hold`, `mux_ok`, stats) to accepted packets only. |
| `fcctl.sh` | Runs on the Pi from `~/fc-v1/robot/`: `status` (incl. `mux_alive`, `mux_ok`, `ros.mux`), `start <profile> --confirm`, `stop`, `bridge start\|stop`, `imu-test <seconds>`, plus `walk`/`rest`. |
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
ssh mts ~/fc-v1/robot/fcctl.sh status
ssh mts ~/fc-v1/robot/fcctl.sh start stock --confirm   # ROBOT ON THE STAND: homes the legs
#   floor: DualSense X stands (tej_u6: Options first, then X). R3 = e-stop.
ssh mts ~/fc-v1/robot/fcctl.sh bridge start
ssh mts ~/fc-v1/robot/fcctl.sh stop         # bridge zero -> deactivate leg controllers (robot goes limp) -> stack down
```

State lives in `~/fc-v1/run/` (pidfiles with process start times, `active` = profile + run dir, `lock`,
`bridge_state.json`); each start
gets `~/fc-v1/runs/<UTC>-<profile>/` with all logs. `start` refuses if any `ros2_control_node`,
`persistent_session.py` or `ros2 launch` is already running. `stop` only signals processes fcctl recorded
(tej_u6: Tej's own `session.sh shutdown` on our run dir) and never SIGKILLs. `FC_TELEOP=0` drops the added
teleop node; `FC_STATUS_BATTERY=1` makes `status` read the battery ADC (shared I2C bus, off by default).
The `lock` (fd 9) serialises `start`/`stop`/`bridge start`; spawned processes never inherit it, and `start`
releases it once the detached starter runs, so `stop` works during homing (it stops the starter first).

Bridge-only test without a robot: `FC_ROS_DOMAIN_ID=77 fcctl.sh bridge start --profile stock` (with no
mux in domain 77 every command is held as `mux_unseen`; start a lone `cmd_vel_mux_node` there to see
`/cmd_vel`). `bridge start` also takes `--robot-id ID` (default: `robot_id` file), `--allow-from IP[,IP]`,
`--reset-replay`, `--no-imu-stop`, `--no-mux-guard`.

Tests (no ROS): `.venv/bin/python -m pytest -q fc/robot/tests` from `final_project/`. `test_fcctl.py` (fake
profile: lock, stop during homing, topology gate, heading-hold overlay gate) needs Linux and is skipped on the
Mac; run it with (`--init` reaps zombies; without it pytest is PID 1 and `stop` waits on dead pids)
`docker run --rm --init --network none -v "$PWD/fc/robot:/src:ro" ros:jazzy-ros-base bash -c 'cd /src && python3 -m pytest -q -p no:cacheprovider tests/test_fcctl.py'`.
