# fc — Pupper FC v1 contract

North star: two Puppers play soccer against each other, each switching between
offense and defense, moving smoothly. Demo Dec 3, 2026. Everything here serves that.

Principles: small modular primitives; the same brain drives sim and robots; every
physical test is a pipeline run from the UI that logs itself; cheap knobs are
parameters, not forks. Iterate: sim → floor → adjust.

## Pipeline

```
camera (fieldcam) ─┐
                   ├─> World ─> brain.decide() per robot ─> Cmd ─> link (UDP) ─> Pi drive_bridge ─> cmd_vel mux ─> walker
sim ───────────────┘                                          └──> sim (same Cmd)
```

One laptop process (`python -m fc.app`) owns the loop at 30 Hz and serves the UI.

## Conventions

- SI units: metres, radians, seconds. Angles wrapped to (-pi, pi].
- Field frame: origin at field centre, +x toward the east goal, +y left (north),
  matching `overhead_perception/fieldcam`. Robot body frame: +x forward, +y left.
- Time: laptop `time.monotonic()`. Observations carry their capture time.
- Teams: `blue` defends the west goal (x<0) and attacks east; `red` the reverse.

## Shared types (`fc/core.py`, the only cross-lane contract)

- `Pose(x, y, yaw)`; `Ball(x, y, vx, vy, t)`; `RobotObs(id, pose, t)`.
- `Field(length, width, goal_width, ...)` loaded from `fc/config/field.json` — the
  single geometry source.
- `World(t, ball: Ball|None, robots: dict[str, RobotObs])`.
- `Cmd(vx, vy, wz, tag)` in body frame; `Cmd.stop()` is zero.
- `Params`: flat dict of named knobs loaded from `fc/config/params.json`, editable
  live from the UI. Every tunable lives here (speeds, gains, contact mode, role
  hysteresis, heading-hold flag, ...).

## Modules and owners

| Module | Role |
| --- | --- |
| `core.py`, `config/` | contract (overseer) |
| `brain.py` | `Brain(cfg, field).decide(world, params) -> (Cmd, debug)`; roles attack/defend/auto; contact `push`/`headbutt` |
| `sim.py` | 2 robots + ball kinematics with injected real errors; consumes `Cmd`, produces `World` |
| `eye.py` | fieldcam camera → `World` |
| `link.py` | `Cmd` → UDP packet to each Pi; heartbeat/ack status |
| `app.py`, `ui/` | loop, run logging, HTTP UI (Play screen `/`, tests `/ops`), test pipelines |
| `play.py`, `media.py` | match flow, ticker, banners, health lights; onboard feeds, run video |
| `robot/` | Pi side: `drive_bridge.py`, profile start/stop scripts, IMU health |

## Wire protocol (laptop → Pi), UDP port 9870, JSON per datagram

`{"v":2,"robot":id,"sid":hex,"seq":int,"t":float,"vx":m/s,"vy":m/s,"wz":rad/s,"ttl":s,"mac":hex}`

`mac` = HMAC-SHA256 with the shared key over the canonical JSON of all other fields
(sorted keys, `,`/`:` separators). Key: laptop `fc/config/link.key` (gitignored; made
by `python -m fc.robot.keygen`), Pi `~/fc-v1/robot/link.key` (0600; placed by `stage.sh`,
the bridge refuses to start without it). `sid` is random per laptop process; `seq`
counts from 1 per robot within it; `t` is laptop wall time.

The bridge (`--robot-id`, optional `--allow-from`) drops, counts and never acks: bad
MAC, other `robot`, `seq` ≤ last accepted in the session, a retired `sid`, or `t` more
than 2 s behind the newest accepted `t` (Pi and laptop clocks are never compared). A
new `sid` with a valid MAC starts a new session (laptop restart). It publishes a Twist
at 50 Hz while the latest packet is younger than `ttl` (default 0.25 s), then zero once
and silence. Signed reply to accepted packets:
`{"v":2,"robot":id,"sid":..,"ack":seq,"ok":true,"imu":{...},"profile":name,"bridge":{stats},"mac":..}`;
the laptop ignores acks with a bad MAC, other robot/sid, or old seq and shows `auth`
(`ok|missing_key|bad_ack`) per robot. Residual risk: a bridge restart forgets retired
sessions, so a recorded session could be replayed until the laptop sends again.

## Safety (never relaxed)

Gamepad has mux priority; mux 500 ms timeout; bridge ttl dead-man; UI Stop and
keyboard space stop all robots; speed caps in params with hard ceilings in
`link.py`; bridge accepts only authenticated packets for its own robot id;
joint/current limits untouched.

## Runs

Each run writes `fc/runs/<UTC timestamp>-<test>/` (gitignored): `params.json`,
`events.jsonl` (world, cmds, acks at loop rate), `summary.json`.

## Python interfaces (fixed for v1 lanes)

- `fc.brain.Brain(cfg: RobotCfg, field: Field)`;
  `.decide(world: World, params: dict) -> tuple[Cmd, dict]`. Debug dict keys:
  `role` (attack|defend|idle), `phase` (str), `target` ([x, y] or null), free extras.
- `fc.sim.Sim(field: Field, robots: list[RobotCfg], params: dict, seed: int = 0)`;
  `.reset(scenario: str = "kickoff")`, `.step(cmds: dict[str, Cmd], dt: float)`,
  `.observe(now: float) -> World` (delayed, noisy), `.truth() -> dict` (exact
  poses/ball for drawing), `.events: list[dict]` (goals, out of bounds).
- `fc.link.Link(robots: list[RobotCfg], params: dict)`; `.send(cmds: dict[str, Cmd])`,
  `.status() -> dict[str, dict]` (last ack age, imu health, profile), `.stop_all()`.
- `fc.eye.Eye(field: Field, robots: list[RobotCfg], params: dict)`; `.start()`,
  `.latest() -> World | None`, `.jpeg() -> bytes | None` (annotated frame).
- `fc.robot.imu_health.ImuHealth(params: dict)`;
  `.update(t, quat_xyzw, gyro_xyz, cmd_zero: bool)`, `.summary() -> dict` with
  `tilt_deg, pulse_density, bias_x, stale_ms, level` (ok|warn|bad).
- `RobotCfg.cam_url` (optional, default ""): onboard camera MJPEG URL for the Play screen.
- Pi control (run by the app over ssh): `~/fc-v1/robot/fcctl.sh` with
  `status` (prints one JSON line, incl. `phase` off|starting|homed|standing|ready|fault),
  `ready [profile] [--confirm]` (JSON lines; homed → standing → bridge; homes only with `--confirm`),
  `down [--keep-stack]`, `start <profile> --confirm` (may home the legs),
  `stop`, `bridge start [--robot-id ID]|stop`, `imu-test <seconds>` (format: `robot/README.md`). Staged by
  `fc/robot/stage.sh <host>` (rsync into `~/fc-v1/` only, plus `link.key` and `robot_id`).

## v1.1 additions (Nov 24)

- **E-stop over the link**: a signed packet may carry `"estop": true` (all motion fields zero). The bridge
  then immediately does the stock R3 action locally (deactivate the leg controllers; legs damp, robot
  drops) and latches: it ignores motion until `fcctl.sh ready ... --confirm` re-arms. Acks carry
  `"estopped": bool`. The laptop sends an estop burst (e.g. 5 packets, 20 ms apart).
- **Fallen-robot cutoff**: the bridge also does the R3 action if ImuHealth reports a sustained tilt beyond
  `fallen_tilt_deg` (default 45°) — a fallen robot must not keep driving its legs.
- **Laptop loss**: unchanged — bridge ttl and mux timeout zero the command; the robot stands still.
- **Stack options** for `fcctl.sh start|ready <profile>`: `--speed-cap <0.30..1.00>` (walker speed caps,
  default stock 0.49) and `--heading-hold on|off` (default on), applied by one per-start overlay under the
  run dir. `stock_hh0`/`tej_u6_hh0` stay as aliases. `status` reports `speed_cap` and `heading_hold`.
  stock also takes `--lean on|off` (default on: no foxglove_bridge, no onboard camera node) and `--camera on|off`
  (default off); `status` adds `lean`, `camera`, `not_launched` and `heat` (governor, MHz, fan, top 5 processes).
- Presets may set `_stack: {"speed_cap": 0.8, "heading_hold": "off"}`; the app passes them to `ready`.
