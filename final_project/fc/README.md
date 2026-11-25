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

# drive a running app from a shell or an agent: terse output, exit 0 ok / 1 failed / 3 health stop / 4 paused
python -m fc.ctl status                            # one line per robot (--url, default http://127.0.0.1:8800)
python -m fc.ctl test step vx=0.2 --wait           # also ready/down/estop/pi/preset/set/drive/stop/last/check/sleep: --help
python -m fc.ctl batch fc/sessions/S1.batch        # first floor session (RUNBOOK) as one batch; pauses wait for Enter
python -m fc.ctl rollcal --apply                   # roll the ball a few times: fits ball_decel (brain prediction), sets it live
python -m fc.ctl ballcal [X Y]                     # ball colour from the ball (or pixel X Y); --show; also Shift-click on /ops
```

Flags: `--port 8800`, `--host <LAN IP>` (UI from another device; default 127.0.0.1), `--camera NAME|INDEX`,
`--no-link` (compute commands, send nothing), `--dry-run-ops` (robot buttons print their ssh command),
`--udp-host blue=192.168.x.y` (control packets to the Pi's LAN address; overrides robots.json `udp_host`).

Floor session: follow [RUNBOOK.md](RUNBOOK.md). Calibrations: `rollcal --apply` and `/params` edits are live only; to keep
a value, put it in `config/params.json` (`ball_decel` is global there) or a preset. `ballcal` saves to
`config/ball_color.json` (loaded at eye start). Field length/width come from the corner-tag calibration
(`field_from_camera`, default on; field.json is the fallback); goal mouth and ball radius come from `config/field.json`.

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

Keys: **space stop** (always), **Esc E-STOP** (always), K kickoff, P play/pause (or start now), R walk back to
kickoff spots, T / shift+T referee touch +1/-1, V cycle who plays (both, blue alone, red alone; no restart), 1/2
cycle the first/second robot's job, C camera/field view, H hide the end card. `/ops` is the test page below.
The overhead view also draws the predicted ball path (the brain's rolling model, `ball_decel`, 3 s) and the
brain's committed plan (`plan` 1, athletic preset). A frozen camera feed (byte-identical frames) shows
CAMERA PAUSED; observations go stale, so robots stop. STOP ends a match (end card with the score); Pause keeps it.
Goals count only in the play phase; on camera a goal is the observed ball crossing a goal line (a ball lying in
the goal scores once). Screenshots (Nov 24): [notes/play-screenshots/](notes/play-screenshots/); UI notes: [notes/ui-handoff.md](notes/ui-handoff.md).

**Stop vs E-STOP** (both screens). STOP (space) = zero speed to every robot; it stands still with its legs on.
E-STOP (Esc, the striped header button = all robots, the card/robot button = that robot) = `Link.estop`: a signed
burst of 5 `"estop": true` packets 20 ms apart (SPEC v1.1), then one e-stop packet per tick until an ack reports
`estopped` (at most 3 s). The bridge does the R3 action (legs off, damped drop) and latches. The app latches too
(`state.estop`): zero commands to that robot, its test ends `estop`/`estopped`, new tests are refused, the light
and a pitch banner say `E-STOPPED · re-arm = Ready`. Nothing clears it by itself: only a Ready on that robot that
ends in phase `ready` and started after the e-stop (sim: "clear (sim)" stands in for Ready). An ack with
`estopped: true` that the operator did not cause (fallen-robot cutoff, another laptop) latches it the same way
("E-STOPPED by the robot"). No confirming ack within 3 s: "E-STOP NOT CONFIRMED by the robot: use R3".
A `down` that exits 1 ("still standing", lie-down not confirmed) turns the light and a banner red: "use R3 or
support the robot". The app never passes `--force`.

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

UI: **space = stop all, Esc = E-STOP all, always** (per-robot E-STOP on each card). Pick a robot, press a test button; each run writes
`fc/runs/<UTC>-<test>/{params.json,events.jsonl,summary.json}` and shows in the Runs list.

| Test | What it does | Summary |
| --- | --- | --- |
| Go (`play`) | brain on every active robot with current params | stopped |
| `drive` | hold WASD (move) / QE (turn), scaled by `max_*`, ramped by `accel_*`; keys idle 0.4 s -> zero | — |
| `goto` | click a field point; the brain's own `role_mode=goto` (the play controller) | `pos_err_m`, `heading_err_deg`, `pass` (< 8 cm) |
| `step` | open loop: still 0.7 s, hold (vx, vy, wz) for `step_s`, zero for `step_settle_s` | `gain_lin`/`gain_wz` (actual / commanded), `latency_s` (fit), `onset_s`, `heading_bias_deg` (walk direction minus tag yaw), `lateral_m`, `yaw_drift_deg_per_m`, `pass` |
| `deadman` | walk at `deadman_vx` for `deadman_drive_s`, then send **nothing** for `deadman_mute_s` | `stop_time_s` (last packet -> at rest, from the camera; hinge fit), `coast_m`, `pass` (≤ 0.5 s). Sim emulates the bridge ttl |
| `push_ball` | `role_mode=attack` until a goal or `test_timeout_s` | `goal`/`own_goal`/`timeout`, `time_s` |
| `goalie` | `role_mode=defend`; in sim a new shot every `goalie_shot_s`; on camera shots come from the ball track (`shot_*` params) | `conceded`, `saved`, `shots` (camera also `missed`, `unclear`, `shot_log`) |
| `match` | both robots, `role_mode=auto`, for `match_s` | goals during the match |
| `sim_eval` | runs `python -m fc.sim_eval <args>` | output table |

Every summary also has `preset` and `metrics`: run time, loop overruns and per robot `seen_frac`, `path_m`,
`stale_stops`, `wz_reversals`, `fast_reversals` (< 1 s apart), `spin_frac`, `imu_worst`, `sent`/`acked`/`loss`,
`rtt_p95_ms`, `final` pose. A step or deadman that loses the tag for `stale_stop_s` ends as `lost_tag`, one that
leaves the field as `edge`.

Output stage: every app-made command (drive, step, deadman, stale stop) goes through `brain.shape` (the
brain's clamp + `accel_*` rate limit); an all-zero command passes at once. After an override the app calls
`Brain.sent` so the brain ramps from what was really sent. Stop, test end, `imu_bad`, `mux_lost` are hard zero.

Stale pose: a robot whose observation is older than `stale_stop_s` gets zero (not in `drive`). The link clamps
`ttl_s` to 0.05–0.5 s (the bridge's `MAX_TTL`).

Robot health from the bridge acks: IMU `level: bad` -> zero to that robot, the test ends `imu_bad`;
`mux_ok: false` -> the same with `mux_lost`. New tests on that robot are refused until it clears. Link/IMU
health (`imu`, `hold`, `imu_hold`, `mux_ok`, auth, RTT, loss, bridge stats) and eye fps go to `events.jsonl`
at 2 Hz (`lk`, `eye` records), with the full brain debug dict every tick.

Params: one dict, `app.PARAM_FALLBACKS` < `brain.DEFAULTS` < `sim.SIM_DEFAULTS` (sim source) < params.json; every
knob is editable live, unknown keys are rejected, strings are checked against `CHOICES`; limits are
enforced (speed ceilings vx 0.75 / vy 0.3 / wz 2.0 = `link.CEIL`, the hard cap on every packet; `max_vx`,
`push_vx`, `contact_vx`, `goalie_vmax` share the vx ceiling). The bridge clamps again with its own `MAX_VX`. The `eye_*` mount knobs apply on the next frame
(a new `eye_floor_size` recalibrates).

Presets (`config/presets/<name>.json`, committed) hold only the keys they change. **Load** = base params
(params.json + defaults at app start) + the preset's keys; presets never stack, **Base** returns to base,
**Save current** writes only keys that differ from base. `_profile` names the robot profile `start` will use;
`_note` is shown. `_stack` (SPEC v1.1) = walker options Ready/start pass to fcctl, allowlisted: `speed_cap` 0.30–1.00
(`--speed-cap 0.80`), `heading_hold` on/off (`--heading-hold off`). The header shows the active preset (`*` =
edited since) and its stack; each robot card shows the running `speed_cap`/`heading_hold` from `fcctl status`
(refreshed after a successful Ready) and warns when `max_vx` exceeds the robot's speed cap (unknown cap: stock
0.49) or the running options differ from the preset's. Each run's `params.json` records `preset`,
`preset_modified`, `preset_profile`, `preset_stack`, `profile_running` (acks), `stack_running`, `estops` and the
link address. Shipped: `slow`, `fast`, `learn`, `push`, `headbutt`, `fast-play` (sim/brain), `fast-play-robot`
(fast-play + `_stack.speed_cap` 0.8), `fast-light` (fast-play-robot with the Nov 24 push tuning, light
ball), `athletic` (fast-light + committed plan, coast, `fast_yaw_bias_deg` −8; sim only so far), `stock-hh-on`, `stock-hh-off`, `u6-hh-on`, `u6-hh-off` (the `*-hh-off`
presets now use the base profile + `_stack.heading_hold: off`; the `*_hh0` profiles remain aliases).

Network: `link.py` sends to robots.json `udp_host` (or `--udp-host`) when set, else the ssh alias's address.
`mts` resolves to a Tailscale 100.x address (p95 up to 1.1 s on Nov 24 vs ~8 ms on the LAN), so set
`udp_host`; the UI warns and shows address, RTT, p95 and loss per robot.

Robot ops run `ssh <host> bash ~/fc-v1/robot/fcctl.sh ...` or `fc/robot/stage.sh <host>` in the
background; `start` and `ready` ask for confirmation because they home the legs or stand the robot up, and
the app then passes `--confirm` (`ready <profile> --confirm [stack options]`: stack off → homes and stops;
homed → stands, starts the bridge and re-arms an e-stop). `ready` and `down` stream their JSON progress lines
(`phase`, `elapsed`, `ok`, `msg`/`error`) into the op state and the Play card (timeout 300 s / 90 s); `phase`
from `ready`/`status` feeds the health light. `stop` and `down` are never refused, and never get `--force`. The host comes only from
`config/robots.json`; profile must be a `robot/profiles/*.sh` name (picked per robot in the UI; a preset's
`_profile` is the default), `imu_test` seconds an int in 1–900 with optional `mode` (bno|ros|auto) and
`condition` (a-z0-9_), and each remote argument is shell-quoted. `stop` is never refused by a running op;
`start` waits up to 180 s. POSTs need `Content-Type: application/json`
and, when the browser sends `Origin`, this server's own address (else 403).

Eye offline check (error vs. truth on rendered frames):
`python -m fc.eye --source synthetic --seconds 8 --save /tmp/eye.jpg`

Tests: `python -m pytest -q fc/tests fc/robot/tests`
