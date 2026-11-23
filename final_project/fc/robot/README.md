# fc/robot — Pi side of Pupper FC

Laptop `link.py` → UDP :9870 → `drive_bridge.py` → `/llm_cmd_vel` → `cmd_vel_mux` (500 ms timeout,
gamepad `/teleop_cmd_vel` wins) → `/cmd_vel` → walker. Contract: [../SPEC.md](../SPEC.md).

## Files

| File | What |
| --- | --- |
| `drive_bridge.py` | rclpy node. 50 Hz Twist while the newest packet is younger than its ttl (0.25 s default, 0.5 s cap), then one zero and silence. Accepts only HMAC-signed packets for its `--robot-id` (key `link.key`, 0600), rejects malformed/replayed packets, clamps to \|vx\|≤0.5, \|vy\|≤0.3, \|wz\|≤2.0. Signed ack (seq, `imu`, profile, stats) to accepted packets only. |
| `fcctl.sh` | Runs on the Pi from `~/fc-v1/robot/`: `status`, `start <profile> --confirm`, `stop`, `bridge start\|stop`, `imu-test <seconds>`, plus `walk`/`rest`. |
| `profiles/stock.sh` | Class platform: monorepo `ros2 launch neural_controller launch.py teleop:=False` + teleop→`/teleop_cmd_vel` + `cmd_vel_mux_node`. Domain 0. |
| `profiles/tej_u6.sh` | Tej's `software-ballkick-20261120-u6` via his `kickseen6` kit (prepare + start-native), fresh run dir, no HTTP API; + teleop→`/teleop_cmd_vel`. Domain 168. |
| `stage.sh <host>` | Laptop: rsync this dir (minus `vendor/`) to `<host>:~/fc-v1/robot/`, print changes; copy `fc/config/link.key` (0600) and write `robot_id` from the `robots.json` entry for `<host>` (or `FC_ROBOT_ID`). |
| `keygen.py` | Laptop, once: `python -m fc.robot.keygen` writes `fc/config/link.key` (0600, gitignored). |
| `vendor/tej_u6/` | Text sources/configs/manifests of u6, the kit, its driver source and gateway source (3.5 MB). Full archive with binaries: `~/workspace/pupper-foundation-artifacts/tej_u6_20261120.tar.gz` (+ `.sha256`). |

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

State lives in `~/fc-v1/run/` (pidfiles with process start times, `active` = profile + run dir); each start
gets `~/fc-v1/runs/<UTC>-<profile>/` with all logs. `start` refuses if any `ros2_control_node`,
`persistent_session.py` or `ros2 launch` is already running. `stop` only signals processes fcctl recorded
(tej_u6: Tej's own `session.sh shutdown` on our run dir) and never SIGKILLs. `FC_TELEOP=0` drops the added
teleop node; `FC_STATUS_BATTERY=1` makes `status` read the battery ADC (shared I2C bus, off by default).

Bridge-only test without a robot: `FC_ROS_DOMAIN_ID=77 fcctl.sh bridge start --profile stock`.
`bridge start` also takes `--robot-id ID` (default: `robot_id` file) and `--allow-from IP[,IP]`.
If the laptop clock steps back more than 2 s, packets are refused as stale until `bridge stop`/`start`.

Tests (no ROS): `.venv/bin/python -m pytest final_project/fc/robot/tests`.
