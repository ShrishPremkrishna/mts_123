# fc — Pupper FC v1 laptop app

Contract: [SPEC.md](SPEC.md). Run everything from `final_project/`.

```bash
# launcher: makes/repairs <repo>/.venv-fc from requirements.txt (Python 3.12), then runs fc.app
fc/run.sh                                          # sim: Play screen http://127.0.0.1:8800/, tests at /ops
fc/run.sh camera --robots blue --udp-host blue=<LAN IP>

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

## Play screen (`/`, for the projector)

Overhead view (camera frame with field lines, belief rings, target crosses and 4 s trails drawn through the
calibrated camera's floor/tag-height homographies; a rendered field in sim, with solid = truth and dashed =
what the brain sees), Blue vs Red score, match clock, per robot: health light, role chip, plain-words
action, what it sees (own tag age, ball distance, target distance, speed), job (auto/attack/goalie), optional
front camera, **Ready**. A ticker narrates real brain decisions (role flips with the travel-time reason,
debounced phase changes); banners say in plain words why a playing robot is stopped or held (tag lost, IMU
bad, mux lost, bridge hold, no link, ball not seen). Nothing is shown as ready without evidence: the light
says "ready" only when fcctl reports phase `ready` (else "link ok", "not seen by camera", ...).

Keys: **space stop** (always), K kickoff, P play/pause (or start now), R walk back to kickoff spots,
T / shift+T referee touch +1/-1, V cycle who plays (both, blue alone, red alone; no restart), 1/2 cycle
the first/second robot's job, C camera/field view, Esc hide the end card. `/ops` is the test page below.

Match flow (`play.py`, one `match` run with `flow`): **kickoff** (brain `role_mode=goto` to spots at
∓`kickoff_x_frac`·length, facing the other goal; waits for both in position and the ball still at the
centre; the sim referee re-centres the ball) → **countdown** (`countdown_s`, robots hold) → **play**
(clock runs only here; jobs from the Play screen, a lone robot defaults to attack) → goal: **goal**
(`goal_pause_s`, robots hold, score + flash) → **restart** (play resumes once the ball is seen still,
`ball_settle_s` within `ball_rest_m`, anywhere in the field; `restart_mode=kickoff` walks back to the spots
instead) → … → **full_time** at `match_s`. Pause holds everyone (zero at once) and freezes the clock. A
robot the bridge halts (IMU/mux) is zeroed and bannered but the match goes on for the other one. Each
tick logs `m` (phase, score, clock, touches) and new ticker lines `tk` to `events.jsonl`.

Recording: with the camera source and `record: 1`, every run writes `<run>/overhead.mp4` (clean frames,
`rec_fps`, ≤1280 px, real time; side thread, `summary.json` → `video`). Onboard feeds: optional
`cam_url` per robot in robots.json (MJPEG from `robot/camstream.py`); a side thread per feed, served as
`/cam/<id>.jpg`; the card says "not connected" when it stalls. Feeds never touch the loop.

## Ops page (`/ops`)

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
background; `start` and `ready` ask for confirmation because they home the legs. `ready <profile>` streams
its JSON progress lines (`phase`, `elapsed`, `ok`, `error`) into the op state and the Play card (timeout
300 s); `phase` from `ready`/`status` feeds the health light. `stop` and `down` are never refused. The host comes only from
`config/robots.json`; profile must be a `robot/profiles/*.sh` name (picked per robot in the UI; a preset's
`_profile` is the default), `imu_test` seconds an int in 1–900 with optional `mode` (bno|ros|auto) and
`condition` (a-z0-9_), and each remote argument is shell-quoted. `stop` is never refused by a running op;
`start` waits up to 180 s. POSTs need `Content-Type: application/json`
and, when the browser sends `Origin`, this server's own address (else 403).

Eye offline check (error vs. truth on rendered frames):
`python -m fc.eye --source synthetic --seconds 8 --save /tmp/eye.jpg`

Tests: `python -m pytest -q fc/tests`
