# fc/robot — Pi side of Pupper FC

Laptop `link.py` → UDP :9870 → `drive_bridge.py` → `/llm_cmd_vel` → `cmd_vel_mux` (500 ms timeout,
gamepad `/teleop_cmd_vel` wins) → `/cmd_vel` → walker. Contract: [../SPEC.md](../SPEC.md).

## Files

| File | What |
| --- | --- |
| `drive_bridge.py` | rclpy node. 50 Hz Twist while the newest packet is younger than its ttl (0.25 s default, 0.5 s cap), then one zero and silence. Accepts only HMAC-signed packets for its `--robot-id` (key `link.key`, 0600), rejects malformed/replayed packets (replay state survives restarts), clamps to \|vx\|≤0.5, \|vy\|≤0.3, \|wz\|≤2.0. Holds zero on a bad IMU or a missing/stalled mux (below). Signed ack (seq, `imu`, profile, `hold`, `imu_hold`, `mux_ok`, `estopped`, stats) to accepted packets only. vx ceiling = the stack's speed cap (`--max-vx`, at most 1.0). E-stop + fallen cutoff: [below](#e-stop-and-fallen-cutoff-spec-v11). |
| `rosq.py` | fcctl's rclpy queries (no ros2 daemon): `probe` (velocity topology + controllers + host DDS participant count), `controllers`, `switch` (BEST_EFFORT, or `strict` for the lie-down handover). `probe` waits until the graph settles (2-8 s) and names the reason when the mux is missing (process running but undiscovered vs no process). |
| `fcctl.sh` | Runs on the Pi from `~/fc-v1/robot/`: `status` (incl. `phase`/`light`, `mux_alive`, `mux_ok`, `ros.mux`), `ready [profile] [--confirm]`, `down [--keep-stack]`, `start <profile> --confirm`, `stop`, `bridge start\|stop`, `imu-test <seconds>`, plus `walk`/`rest` and `boot` (service only). |
| `readiness.py` | Pure: the `status` phase (`off/starting/homed/standing/ready/fault`) and the controller check `ready` uses. |
| `boot/` | Power-on service, prepared and **not installed**: `fc-boot.service`, `install.sh` (install/uninstall/status/print). What it does, rollback and the homing-posture evidence: [boot/README.md](boot/README.md). |
| `profiles/stock.sh` | Class platform: monorepo `ros2 launch neural_controller launch.py teleop:=False` + teleop→`/teleop_cmd_vel` + `cmd_vel_mux_node`. Domain 0. |
| `profiles/tej_u6.sh` | Tej's `software-ballkick-20261120-u6` via his `kickseen6` kit (prepare + start-native), fresh run dir, no HTTP API; + teleop→`/teleop_cmd_vel`. Domain 168. |
| `profiles/stock_hh0.sh`, `profiles/tej_u6_hh0.sh` | Aliases of the base profile with `--heading-hold off` (set `FC_HEADING_HOLD=0`, source the base; `--heading-hold on` with an alias is refused). |
| `stack_overlay.py` | The one per-start overlay builder (generalises `imu/heading_hold.py`, whose kp=0 policy helper it reuses): `--speed-cap`, `--heading-hold off`, and (stock) the lie-down controller, in `<run>/overlay/`, re-read and verified (YAML parsed and compared with the source plus exactly the intended edits; launch text compared; policies compared; gains/limits never wider). Any failure refuses the start before motion. Details: [Stack options](#stack-options-spec-v11). |
| `sim/liedown_sim.py` | Laptop only (not staged): MuJoCo check of the gentle lie-down. [Gentle lie-down](#gentle-lie-down-stock-down-and-stop). |
| `stage.sh <host>` | Laptop: rsync this dir (minus `vendor/`, `provision/`, `sim/`) to `<host>:~/fc-v1/robot/`, print changes; copy `fc/config/link.key` (0600) and write `robot_id` from the `robots.json` entry for `<host>` (or `FC_ROBOT_ID`). |
| `provision/` | Laptop: `provision.sh [--check] <host> <id> <team> <tag>` takes a new Pi to game-ready (read-only probe + readiness report; stages via `stage.sh`; writes only `~/fc-v1/` unless `--fix-clone-identity`). Second-robot plan, clone vs class card, and per-robot calibration: [provision/PROVISION.md](provision/PROVISION.md). Not staged to the Pi. |
| `camstream.py` | Optional, not run by any script yet: front camera as 320 px MJPEG on :8090 (`/stream.mjpg`, `/frame.jpg`, `/health`) for the laptop Play screen (`cam_url` in robots.json). Default source subscribes read-only to camera_ros `/camera/image_raw/compressed` (same ROS env/domain as the profile); `--source picamera2` opens the sensor only when no camera node runs. Not yet run on a Pi. |
| `keygen.py` | Laptop, once: `python -m fc.robot.keygen` writes `fc/config/link.key` (0600, gitignored). |
| `vendor/tej_u6/` | Text sources/configs/manifests of u6, the kit, its driver source and gateway source (3.5 MB). Full archive with binaries: `~/workspace/pupper-foundation-artifacts/tej_u6_20261120.tar.gz` (+ `.sha256`). |

## Velocity topology (one mux, one `/cmd_vel` publisher)

Checked on mts 2026-11-23 by reading the installed launch files (no stack started):

| Profile | Who starts the mux | `/cmd_vel` publishers | mux inputs, priority order (500 ms timeout) |
| --- | --- | --- | --- |
| `stock` | `stock.sh` (`ros2 run cmd_vel_mux cmd_vel_mux_node`). The Pi's installed `neural_controller/launch/launch.py` (identical to its `src/` copy) starts **no** mux, and `teleop:=False` really drops its teleop node. A laptop monorepo copy that starts a mux is a different revision. | `cmd_vel_mux` only | 0 `/teleop_cmd_vel` ← our `teleop_twist_joy` (gamepad); 1 `/llm_cmd_vel` ← `fc_drive_bridge` only; 2 `/person_following_cmd_vel` ← nobody |
| `tej_u6` | Tej's `lab6_session.launch.py` (a "required" node: its exit shuts the whole session down). `tej_u6.sh` starts no second one. | `cmd_vel_mux` only | 0 `/teleop_cmd_vel` ← our `teleop_twist_joy` (`FC_TELEOP=0` drops it); 1 `/llm_cmd_vel` ← `fc_drive_bridge` only (his HTTP API, the other publisher, is not started); 2 `/person_following_cmd_vel` ← his Lab 5 follower |

**Graph queries need time (2026-11-23, mts).** Each `rosq` call is a new DDS participant and discovers a
stack-sized graph in random order over seconds. The old fixed 1.5 s spin produced `ready`'s verify failure
"/cmd_vel publishers []" with the mux running (a fresh-shell `ros2 node list` also shows a partial graph
right after it spawns its daemon). Repro in domain 42 on mts, 10 stand-in nodes with stack-like endpoint
counts and the Pi loaded: 3 of 12 fixed-1.5 s probes saw no mux; with the settling probe 12 of 12 did. Not
the cause (checked): DDS islands. Under `ROS_LOCALHOST_ONLY=1` rmw_fastrtps (Jazzy) uses SHM + UDP with
`maxInitialPeersRange = 32` and 127.0.0.1 as the only initial peer (no multicast), the live stack used
participant ids 0-11, and long-lived participants all saw each other in the repro. Remaining limit: Fast DDS
allocates participant ids host-wide (UDP port 7000+id, across domains); two participants with ids >= 32 would
never meet, so `status` reports `dds.participants`/`max_id` and warns from id 28. Two robots on one Wi-Fi do
not cross-talk in this mode (only 127.0.0.1 is ever contacted); profiles keep it unchanged (Tej's kit forces
`ROS_LOCALHOST_ONLY=1` itself, so all processes share one mode). `ready`'s verify step now rechecks the
topology for up to 15 s before failing and reports the probe's problem text.

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
ssh mts bash ~/fc-v1/robot/fcctl.sh start stock --confirm --speed-cap 0.80   # ROBOT ON THE STAND: homes the legs
#   move the robot to the floor, then:
ssh mts bash ~/fc-v1/robot/fcctl.sh ready --confirm  # stands the walker (tej_u6: waits for Options), starts the bridge
ssh mts bash ~/fc-v1/robot/fcctl.sh down    # bridge zero -> lowered to its belly (stock) -> legs off -> stack down
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

`down [--keep-stack] [--force]`: `begin`, `bridge_stopped`, `lowering`, `lying_down`, `down` (`done`;
`"stack":"kept"` with `--keep-stack`, else the `stop` result in `stop`). `lowering` has `"gentle":true` (lowered
under position control first) or `"gentle":false` with a "WILL DROP" message (no lie-down controller: u6,
`FC_LIEDOWN=0`, or the STRICT handover refused); it is omitted when no leg controller is active.
`ready` adds `rearmed` after an e-stop (below) and `homed` carries the running `speed_cap`/`heading_hold`.

## Gentle lie-down (stock `down` and `stop`)

Deactivating the leg controllers (= stock R3: `neural_controller::on_deactivate` writes position/velocity/kp = 0,
kd = `estop_kd` 0.1) **drops** the robot (seen on mts 2026-11-23). Stock starts now also load an inactive
`neural_controller_liedown`: a second instance of the installed `neural_controller/NeuralController` plugin
(no rebuild) whose policy is `legacy_walk_policy.json` with only `default_joint_pos` replaced by the lying pose
(per leg abduction/hip/knee 0 / ±1.6 / ±0.3, left legs mirrored, front = back), `init_duration` 2.5 s and a
100000 s fade-in (the policy never acts). `down`/`stop` hand the legs to it in **one STRICT switch** (walker off
+ lie-down on in the same controller-manager cycle, or nothing changes); its own init blend (the code path every
stand-up uses, reversed) moves the joints linearly from the measured pose to the lying pose at
init_kps/init_kds 7.5/0.25, then holds at the JSON kp/kd 5.0/0.25 (below the walker's 5.5). After 2.5 + 1.0 s
(`FC_LIEDOWN_HOLD_S`) the usual confirmed deactivation follows; the body already rests on its belly. R3 still
works during the lowering (the instance subscribes `/emergency_stop` like every neural controller). Not for u6
(pinned kit, latched Stop gateway): `down` drops as before with the warning. `FC_LIEDOWN=0 fcctl.sh start stock
--confirm` starts without it (with no other option, the stock launch is then unchanged).

Sim evidence (2026-11-23, `FORCE_CPU=1 .venv/bin/python final_project/fc/robot/sim/liedown_sim.py`, pupper-mjlab
full-hull model at 520 Hz, the real gains; 14 cases: blend 1.5/2.5/3.5 s, friction ×0.5/×1.5, +0.3 kg, kp ×0.85,
±3° pitch, random ±0.1 rad start poses, combined harsh): body 15.8 → 3.1 cm resting on the belly hull in every
case, max tilt 3.8°, peak descent 0.18 m/s, torque ≤ 1.8 Nm (limit 3), **0.0 cm** further drop after
deactivation, re-stand from that pose tilt ≤ 4°. Baseline R3 from standing: 7.7 cm drop, impact 0.37 m/s. A
crouch pose (knee at its limit) also tested well but relies on the knee stop for support after deactivation,
which the sim cannot vouch for; the belly pose does not. Not modelled: motor latency/backlash, the walker
stepping while standing, the real knee mechanics. **Floor test before relying on it** (below).

`down` and `stop` never stop a stack whose leg controllers are not **confirmed** inactive by a fresh
`list_controllers` after the deactivation (the switch reply alone is not trusted). Not confirmed (walker
still active, or the manager does not answer while `ros2_control_node` runs after READY): `down` ends with
`error`, `at:"lying_down"`, `"stack":"running"`, exit 1, and `stop` returns `ok:false`,
`left_running:"stack"`; the message says "use R3 or support the robot". `--force` overrides (UNSAFE: a
standing robot then drops with the motors off). Before 2026-11-23 `down` always reported "deactivation not
confirmed": it matched `"ok":true` against rosq's spaced JSON (`"ok": true`); rosq now prints compact JSON.

`status` adds `phase`, `light`, `phase_detail`, `walker`, `walker_active`, `active_legs`, `boot`, and (SPEC v1.1)
`speed_cap` (effective walker cap), `speed_cap_source` (`overlay`|`default`), `heading_hold`, `liedown` (lie-down
controller loaded), `stack` (the run's `stack.json`), `estopped`, `estop` (latch file content):

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

## Stack options (SPEC v1.1)

`fcctl.sh start <profile> --confirm [--speed-cap X] [--heading-hold on|off]` and the same options on `ready`
(any order, also after `--confirm`). Values outside 0.30..1.00 / on|off are refused before anything starts.
`stack_overlay.py` builds one verified overlay in `<run>/overlay/`; `<run>/stack.json` records the options either
way (no overlay = stock defaults: cap 0.49, hold on).

- **Speed cap** (fast-gait.md: the walker clips to the policy's ±1.5, then `max_vx_forward` 0.49, then the
  jump-slot cap = `speed_cap` if > 0, else the JSON walk cap 0.49, on vx and vy): sets `max_vx_forward` and
  `speed_cap` to X. `max_vx_backward` (0.3) and wz (±2.0) are unchanged. stock: `neural_controller_default` and
  its chord partner `_default_alt` in the overlay config.yaml. tej_u6: `neural_controller_default` and
  `gemini_walk`, set in the launch file's in-memory baseline (inserted before `session_skill_controllers`, which
  raises if `max_vx_forward` is missing). **u6 limits:** `lab6-session.yaml` stays byte-identical (Tej's launch
  pins its sha), the kit scripts stay sha-checked, his trick controllers derived from that baseline inherit the
  cap, and the Stop gateway is untouched.
- **Bridge ceiling**: `bridge start` passes `--max-vx` = the run's effective cap (0.49 by default, 0.5 for runs
  without `stack.json`), so a 0.75 command reaches the walker only when the stack was started with
  `--speed-cap ≥ 0.75`. vy 0.3 and wz 2.0 ceilings unchanged.
- **Heading hold off**: the kp=0 policy copy (lane D), walkers pointed at it.
- `ready` on a running stack with different options fails at `profile` ("down first": a new start re-homes).
- stock: src and installed launch/config/policies must be identical (the overlay is built from `src/`).

## E-stop and fallen cutoff (SPEC v1.1)

The bridge keeps a persistent `/controller_manager/switch_controller` + `list_controllers` client and an
`/emergency_stop` publisher, and polls the controller states at 2 Hz. A signed, admitted packet with
`"estop": true` (same auth/robot/replay rules), or a quaternion tilt > `--fallen-tilt-deg` (45) held
`--fallen-hold-s` (0.5 s), triggers the stock R3 action once: publish `std_msgs/Empty` on `/emergency_stop` (every
active neural controller zeroes kp, kd = 0.1, at its next update) and send `SwitchController(deactivate = active
leg controllers + walker, BEST_EFFORT)` (only profile leg names, so u6's gateway accepts it), re-listing at
20 Hz until no leg controller is active (resend after 1 s, 3 tries). Then it **latches**: zero only, acks
`"estopped": true` (`hold: "estopped"`, `bridge.estop_reason` `link`|`fallen`), repeated estop packets are acked
and change nothing. `run/estop.json` (0600; reason, tilt, R3 timings) survives bridge restarts (a restarted
bridge starts latched; unreadable = latched). Only `fcctl.sh ready ... --confirm` re-arms: it stops the latched
bridge, moves the latch to `logs/estop-<UTC>.json`, deactivates a leg controller that still reads active (its
estop flag clears only on activation), stands and restarts the bridge. `ready` without `--confirm` returns
`needs_confirm` (exit 3) naming the reason. `down`/`stop` leave the latch alone.

Latency (2026-11-23, `tests/test_estop_ros.py`, real rclpy Jazzy in docker on the Mac, fake manager, 5 trials):
packet sent → `/emergency_stop` received median 1.2 ms (max 1.8), → SwitchController received median 7.3 ms
(max 7.5; one 5 ms main-loop spin). Fallen: SwitchController 511 ms after the tilt passed 45°. Not yet measured
on the Pi. A carried robot tilted past 45° for 0.5 s with the bridge up also trips it (harmless: it goes limp;
re-arm with Ready).

## Floor tests still needed (operator)

1. Gentle lie-down on mts: `ready stock --confirm` (new start, loads the lie-down controller; `status.liedown`
   true), stand, `down --keep-stack`, watch: slow lowering to the belly (~2.5 s), no tip, no drop at the end; then
   `ready --confirm` stands again. Hand under the robot the first time; R3 aborts.
2. Link e-stop from the UI while walking slowly: legs off within a frame, UI shows estopped, Ready re-arms.
3. Fallen cutoff: tip the standing robot over by hand (bridge up) -> legs off after ~0.5 s, `status.estop.reason`
   `fallen`.
4. `--speed-cap 0.75` bursts per fast-gait.md (stock first; tej_u6 overlay launch not yet run).

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

Tests (no ROS): `.venv/bin/python -m pytest -q fc/robot/tests` from `final_project/` (incl. `test_stack_overlay.py`;
the e-stop ROS test `test_estop_ros.py` skips without rclpy: run it in `cs123-ros-jazzy-slew-check`, command in its
docstring). `test_fcctl.py` (fake
profile: lock, stop during homing, topology gate, stack-overlay gate and options, e-stop latch/re-arm, gentle
lie-down and its fallback, `ready`/`down` incl. a refusing
gateway and operator timeout, `down`/`stop` refusing to stop a robot not confirmed lying, status phases, boot
modes) needs Linux and is skipped on the Mac; run it with (`--init` reaps zombies; without it pytest is PID 1 and `stop` waits on dead pids)
`docker run --rm --init --network none -v "$PWD/fc/robot:/src:ro" ros:jazzy-ros-base bash -c 'cd /src && python3 -m pytest -q -p no:cacheprovider tests/test_fcctl.py'`.
