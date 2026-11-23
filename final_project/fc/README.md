# fc — Pupper FC v1 laptop app

Contract: [SPEC.md](SPEC.md). Run everything from `final_project/`.

```bash
# sim: no camera, no robots (stdlib Python is enough)
python -m fc.app --source sim                      # http://127.0.0.1:8800/

# camera, robots listed in config/robots.json (needs OpenCV: see requirements.txt)
python -m fc.app --source camera --robots blue     # live phone camera ("phone")
python -m fc.app --source camera --video synthetic --no-link --dry-run-ops   # offline rehearsal
python -m fc.app --source camera --video clip.mov  # or a folder of jpg/png
```

Flags: `--port 8800`, `--host <LAN IP>` (UI from another device; default 127.0.0.1), `--camera NAME|INDEX`,
`--no-link` (compute commands, send nothing), `--dry-run-ops` (robot buttons print their ssh command),
`--udp-host blue=192.168.x.y` (control packets to the Pi's LAN address; overrides robots.json `udp_host`).

Floor day: follow [RUNBOOK.md](RUNBOOK.md).

The app imports the real `fc.brain` and `fc.sim`; if either fails to import, the app does not start
(no fallback controller).

UI: **space = stop all, always**. Pick a robot, press a test button; each run writes
`fc/runs/<UTC>-<test>/{params.json,events.jsonl,summary.json}` and shows in the Runs list.

| Test | What it does | Summary |
| --- | --- | --- |
| Go (`play`) | brain on every active robot with current params | stopped |
| `drive` | hold WASD (move) / QE (turn), scaled by `max_*`, ramped by `accel_*`; keys idle 0.4 s -> zero | — |
| `goto` | click a field point; the brain's own `role_mode=goto` (the play controller) | `pos_err_m`, `heading_err_deg`, `pass` (< 8 cm) |
| `step` | open loop: still 0.7 s, hold (vx, vy, wz) for `step_s`, zero for `step_settle_s` | `gain_lin`/`gain_wz` (actual / commanded), `latency_s` (fit), `onset_s`, `heading_bias_deg` (walk direction minus tag yaw), `lateral_m`, `yaw_drift_deg_per_m`, `pass` |
| `deadman` | walk at `deadman_vx` for `deadman_drive_s`, then send **nothing** for `deadman_mute_s` | `stop_time_s` (last packet -> at rest, from the camera; hinge fit), `coast_m`, `pass` (≤ 0.5 s). Sim emulates the bridge ttl |
| `push_ball` | `role_mode=attack` until a goal or `test_timeout_s` | `goal`/`own_goal`/`timeout`, `time_s` |
| `goalie` | `role_mode=defend`; in sim a new shot every `goalie_shot_s` | `conceded`, `saved` |
| `match` | both robots, `role_mode=auto`, for `match_s` | goals during the match |
| `sim_eval` | runs `python -m fc.sim_eval <args>` | output table |

Every summary also has `preset` and `metrics`: run time, loop overruns and per robot `seen_frac`, `path_m`,
`stale_stops`, `wz_reversals`, `fast_reversals` (< 1 s apart), `spin_frac`, `imu_worst`, `sent`/`acked`/`loss`,
`rtt_p95_ms`, `final` pose. A step or deadman that loses the tag for `stale_stop_s` ends as `lost_tag`, one that
leaves the field as `edge`.

Output stage: every app-made command (drive, step, deadman, stale stop) goes through `brain.shape` (the
brain's clamp + `accel_*` rate limit); an all-zero command passes at once. After an override the app calls
`Brain.sent` so the brain ramps from what was really sent. Stop, test end, `imu_bad`, `mux_lost` are hard zero.

Robot health from the bridge acks: IMU `level: bad` -> zero to that robot, the test ends `imu_bad`;
`mux_ok: false` -> the same with `mux_lost`. New tests on that robot are refused until it clears. Link/IMU
health (`imu`, `hold`, `imu_hold`, `mux_ok`, auth, RTT, loss, bridge stats) and eye fps go to `events.jsonl`
at 2 Hz (`lk`, `eye` records), with the full brain debug dict every tick.

Params: every knob (params.json + brain/sim defaults + `app.PARAM_FALLBACKS`) is editable live; limits are
enforced (speed ceilings 0.5 / 0.3 / 2.0 also in `link.py`). The `eye_*` mount knobs apply on the next frame
(a new `eye_floor_size` recalibrates).

Presets (`config/presets/<name>.json`, committed) hold only the keys they change. **Load** = base params
(params.json + defaults at app start) + the preset's keys; presets never stack, **Base** returns to base,
**Save current** writes only keys that differ from base. `_profile` names the robot profile `start` will use;
`_note` is shown. The header shows the active preset (`*` = edited since), and each run's `params.json`
records `preset`, `preset_modified`, `preset_profile`, `profile_running` (from the acks) and the link address.
Shipped: `slow`, `fast`, `learn`, `push`, `headbutt`, `stock-hh-on`, `stock-hh-off`, `u6-hh-on`, `u6-hh-off`.

Network: `link.py` sends to robots.json `udp_host` (or `--udp-host`) when set, else the ssh alias's address.
`mts` resolves to a Tailscale 100.x address; the UI warns and shows address, RTT, p95 and loss per robot.
`udp_host` needs `udp_host: str = ""` in `core.RobotCfg` before `sim_eval` can read such a robots.json
(the app already tolerates it).

Robot ops run `ssh <host> bash ~/fc-v1/robot/fcctl.sh ...` or `fc/robot/stage.sh <host>` in the
background; `start` asks for confirmation because it homes the legs. The host comes only from
`config/robots.json`; profile must be a `robot/profiles/*.sh` name (picked per robot in the UI; a preset's
`_profile` is the default), `imu_test` seconds an int in 1–900 with optional `mode` (bno|ros|auto) and
`condition` (a-z0-9_), and each remote argument is shell-quoted. `stop` is never refused by a running op;
`start` waits up to 180 s. POSTs need `Content-Type: application/json`
and, when the browser sends `Origin`, this server's own address (else 403).

Eye offline check (error vs. truth on rendered frames):
`python -m fc.eye --source synthetic --seconds 8 --save /tmp/eye.jpg`

Tests: `python -m pytest -q fc/tests`
