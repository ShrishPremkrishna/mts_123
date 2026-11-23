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
| `app.py`, `ui/` | loop, run logging, HTTP UI, test pipelines |
| `robot/` | Pi side: `drive_bridge.py`, profile start/stop scripts, IMU health |

## Wire protocol (laptop → Pi), UDP port 9870, JSON per datagram

`{"v":1,"seq":int,"t":float,"vx":m/s,"vy":m/s,"wz":rad/s,"ttl":s}`

The Pi publishes a Twist at 50 Hz while the latest packet is younger than `ttl`
(default 0.25 s), then publishes zero once and goes silent. Reply datagram to the
sender: `{"v":1,"ack":seq,"imu":{...health...},"profile":name}`.

## Safety (never relaxed)

Gamepad has mux priority; mux 500 ms timeout; bridge ttl dead-man; UI Stop and
keyboard space stop all robots; speed caps in params with hard ceilings in
`link.py`; joint/current limits untouched.

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
- Pi control (run by the app over ssh): `~/fc-v1/robot/fcctl.sh` with
  `status` (prints one JSON line), `start <profile> --confirm` (may home the legs),
  `stop`, `bridge start|stop`, `imu-test <seconds>`. Staged by
  `fc/robot/stage.sh <host>` (rsync into `~/fc-v1/` only).
