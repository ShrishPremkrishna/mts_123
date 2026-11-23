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
| `brain.py` | `Brain.decide(world, me, params) -> (Cmd, debug)`; roles attack/defend/auto; contact `push`/`headbutt` |
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
