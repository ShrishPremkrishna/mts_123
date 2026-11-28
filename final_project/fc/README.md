# fc — Pupper FC v1 laptop app

Contract: [SPEC.md](SPEC.md). Run everything from `final_project/`.

```bash
# launcher: makes/repairs <repo>/.venv-fc from requirements.txt (Python 3.12), then runs fc.app
fc/run.sh                                          # http://127.0.0.1:8800/: home (Play / Simulation / Test bench)
fc/run.sh camera --robots blue --udp-host blue=<LAN IP>

# sim: no camera, no robots (stdlib Python is enough)
python -m fc.app --source sim                      # http://127.0.0.1:8800/

# camera, robots listed in config/robots.json (needs OpenCV: see requirements.txt)
python -m fc.app --source camera --robots blue     # live phone camera ("phone")
python -m fc.app --source camera --video synthetic --no-link --dry-run-ops   # offline rehearsal
python -m fc.app --source camera --video clip.mov  # or a folder of jpg/png

# drive a running app from a shell or an agent: terse output, exit 0 ok / 1 failed / 3 health stop / 4 paused
python -m fc.ctl status                            # one line per robot (--url, default http://127.0.0.1:8800)
python -m fc.ctl ops                               # every operation the app accepts, enabled now or why not
python -m fc.ctl op match.kickoff                  # run any one of them by name: op <name> k=v ... (JSON reply)
python -m fc.ctl test step vx=0.2 --wait           # also ready/down/estop/pi/preset/set/drive/stop/last/check/sleep: --help
python -m fc.ctl batch fc/sessions/S1.batch        # first floor session (RUNBOOK) as one batch; pauses wait for Enter
python -m fc.ctl preset play                       # the demo brain (or `slow` for the first motion after a change)
python -m fc.ctl show warmup [loops=N] [--wait]    # warm-up at the kickoff spots (W); again = walk to the spots and end
python -m fc.ctl show kickoff --wait               # K, then follow the match: 3-2-1, goals, full time (show watch / show go = P)
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

## Screens (`/` home, `/play` match, `/ops` test bench, `/play-old` the previous Play screen until the new one is floor-verified)

One look and one set of building blocks: [ui/kit/](ui/kit/) (`tokens.css` every colour/size/motion value,
`ui.css` one class per component, `kit.js` DOM helpers + the /state poller + the operations client,
`pitch.js` overhead renderer, `robots.js` robot/camera cards, `feeds.js` robot cameras). No build step.

- **Home `/`**: arrow keys + Enter. **Play** shows the live field camera and a card per robot whose one button
  does the next setup step (Check → Home legs → Stand up → Start link → Ready; Re-arm after an E-stop; Stop
  stack on a fault). The button always runs `fcctl status` first and decides on that fresh phase; a `ready`
  without link acks shows Check / No link, never Ready. **Simulation** shows the sim field; **Test bench** opens
  `/ops`. The selection starts on the running mode; choosing the other one asks, then restarts the app in place
  with the other `--source` and the loaded preset (`app.source confirm=true`; refused mid-match; live param
  edits are lost).
- **Play `/play`**: field camera with the overlays (C: drawn map), player tiles (robot camera on top when it
  has one: `cam_url` on a robot, a rendered robot's-eye view in sim; 1/2 swap it into the big view), the
  decision ticker, one banner per fact, a ready check before kickoff, and the Robots drawer (Tab: jobs, setup,
  lie down, per-robot E-stop). Keys: **Space stop** and **Esc E-STOP** always (even over a dialog), K kickoff,
  P play / start now / pause / resume (whichever applies), R back to kickoff spots, V who plays, T / shift+T
  referee touch, W show warm-up (again: to the spots), C field view, 1/2 robot cameras, Tab robots, H hide the end card.

Every button is an operation in [ops.py](ops.py): `GET /api/ops` lists them (label, key, safety, typed
params), `POST /api/op {"op", "args"}` runs one (`{ok, op, ...}` or `{ok: false, code, error}`), and
Safety rules: `stop` / `estop` with bad or missing args act on every robot (with a warning), never a 400;
`robot.*` ops are unavailable in a sim without `--dry-run-ops`; `robot.ready` needs an fcctl phase at most
15 s old and `home=true` to home from phase `off`. Space / Esc are bound inline to `/action stop` / `estop`, so
they work even if `/api/ops` fails. `/state` carries `avail[op]` (enabled, reason, label, next args) and `play.robots[id].setup` (the robot's next
step). The UI, `fc.ctl op` and agents use the same list, so a key never does something different from its
button. `/state?lite=1` leaves out static config (`GET /api/config`). Nothing is shown as ready without
evidence: `ready` needs a green light and, on a real robot, fcctl phase `ready`. The overhead view draws the
predicted ball path (`ball_decel`, 3 s). A frozen camera feed shows CAMERA
PAUSED; observations go stale, so robots stop. STOP ends a match (end card); Pause keeps it. Goals count only in
the play phase. UI notes: [notes/ui-handoff.md](notes/ui-handoff.md).

**Stop vs E-STOP** (both screens). STOP (space) = zero speed to every robot; it stands still with its legs on.
E-STOP (Esc or the striped key-bar prompt = all robots; the robot card button = that robot) = `Link.estop`: a signed
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
(clock runs only here; two playing robots: the brain's Coach assigns their tasks; jobs from the Play screen) → goal: **goal**
(`goal_pause_s`, robots hold, score + flash; with `celebrate_style` spin|wiggle instead **celebrate**: the scorer
turns on the spot for `celebrate_s` at `celebrate_wz`, after stepping inside the fence margin (`wall_margin_m`, else
0.28 m), the other robot holds or walks slowly to its spot per `celebrate_other`; in sim the robots keep their
places) → **restart** (play resumes once the ball is seen still,
`ball_settle_s` within `ball_rest_m`, anywhere in the field; `restart_mode=kickoff` walks back to the spots
instead) → … → **full_time** at `match_s`. Pause holds everyone (zero at once) and freezes the clock. A
robot the bridge halts (IMU/mux) is zeroed and bannered but the match goes on for the other one. Each
tick logs `m` (phase, score, clock, touches) and new ticker lines `tk` to `events.jsonl`. The show settings (90 s
`match_s`, 25/75 % kickoff spots, 3-2-1, spin celebration, kickoff restarts) are plain params in `config/params.json`;
the show runs with whatever brain preset is loaded (`play`).

Show warm-up (`show.warmup`, W, `play.Warmup`, its own run `warmup`): the playing robots loop trot in place → small
circle → side shuffle → quick spin at their kickoff spots (app-made commands through `brain.shape`; walks back by the
brain's goto). A move that would leave the field shrunk by `wall_margin_m` (else 0.28 m), stray 0.5 m from the spot
or come within 0.7 m of the other robot ends early (`recentres`). `loops=0` runs until W again; then both walk to
their spots facing the field and it ends `done`. Stop / E-stop / IMU / mux end it at once; K replaces it.

Recording: with the camera source and `record: 1`, every run writes `<run>/overhead.mp4` (clean frames,
`rec_fps`, ≤1280 px, real time; side thread, `summary.json` → `video`). Onboard feeds: optional
`cam_url` per robot in robots.json (MJPEG from `robot/camstream.py`); a side thread per feed, served as
`/cam/<id>.jpg`; the card says "not connected" when it stalls. Feeds never touch the loop.

## Ops page (`/ops`)

UI: **space = stop all, Esc = E-STOP all, always** (per-robot E-STOP on each card). Pick a robot, press a test button; each run writes
`fc/runs/<UTC>-<test>/{params.json,events.jsonl,summary.json}` and shows in the Runs list.

| Test | What it does | Summary |
| --- | --- | --- |
| Go (`play`) | brain on every active robot with current params (two playing robots: the Coach assigns tasks) | stopped |
| `drive` | hold WASD (move) / QE (turn), scaled by `max_*`, ramped by `accel_*`; keys idle 0.4 s -> zero | — |
| `goto` | click a field point; the brain's own `role_mode=goto` (the play controller) | `pos_err_m`, `heading_err_deg`, `pass` (< 8 cm) |
| `step` | open loop: still 0.7 s, hold (vx, vy, wz) for `step_s`, zero for `step_settle_s` | `gain_lin`/`gain_wz` (actual / commanded), `latency_s` (fit), `onset_s`, `heading_bias_deg` (walk direction minus tag yaw), `lateral_m`, `yaw_drift_deg_per_m`, `pass` |
| `deadman` | walk at `deadman_vx` for `deadman_drive_s`, then send **nothing** for `deadman_mute_s` | `stop_time_s` (last packet -> at rest, from the camera; hinge fit), `coast_m`, `pass` (≤ 0.5 s). Sim emulates the bridge ttl |
| `push_ball` | `role_mode=attack` until a goal or `test_timeout_s` | `goal`/`own_goal`/`timeout`, `time_s` |
| `goalie` | `role_mode=defend`; in sim a new shot every `goalie_shot_s`; on camera shots come from the ball track (`shot_*` params) | `conceded`, `saved`, `shots` (camera also `missed`, `unclear`, `shot_log`) |
| `match` | both robots, `role_mode=auto`, for `match_s` | goals during the match |
| `sim_eval` | runs `python -m fc.sim_eval <args>` | output table |
| `autoplay` | hands-off solo demo loop (`play.AutoPlay`, op `autoplay.start`, key A on Play): attack the ball wherever it is, score, then bring the ball back to the centre spot slowly with the brain's goto (line up behind it, push at `reset_vx`), back off 0.4 m, attack again; `alternate` attacks the other goal each cycle (the reset leaves the robot behind the ball facing it). A ball left in a net / off the field / unseen shows "please roll it out" after 4 s and waits without a time-out; a ball it cannot get behind (wall, corner, goal line) is attacked toward the nearer goal, and if that fails too it asks for help. Args `minutes` (0 = until Stop), `alternate`, `reset` centre/none, `after_roll` centre/attack, `reset_vx`. Sim: goals leave the ball in the net (`sim_goal_bounce` = chance it bounces back) and a scripted person rolls it out 3 s after the banner | `goals`, `own_goals`, `cycles`, `time_per_goal_s` and `reset_s` (medians), `interventions`, `attack_timeouts`, `cycle_log` |

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

Params: one dict, `app.PARAM_FALLBACKS` < `brain.DEFAULTS` (the ~13 brain settings that matter, listed in the
brain.py doc; every other brain number is a fixed constant in `brain.TUNE`) < `sim.SIM_DEFAULTS` (sim source) <
params.json; every key is editable live, unknown keys are rejected, strings are checked against `CHOICES`; limits
are enforced (speed ceilings vx 1.5 / vy 0.3 / wz 2.0 = `link.CEIL`, the hard cap on every packet). The bridge
clamps again with its own `MAX_VX`. The `eye_*` mount knobs apply on the next frame
(a new `eye_floor_size` recalibrates).

Presets (`config/presets/<name>.json`, committed) hold only the keys they change. **Load** = base params
(params.json + defaults at app start) + the preset's keys; presets never stack, **Base** returns to base,
**Save current** writes only keys that differ from base. `_profile` names the robot profile `start` will use;
`_note` is shown. `_stack` (SPEC v1.1) = walker options Ready/start pass to fcctl, allowlisted: `speed_cap` 0.30–1.50
(`--speed-cap 0.80`), `heading_hold` on/off (`--heading-hold off`). The header shows the active preset (`*` =
edited since) and its stack; each robot card shows the running `speed_cap`/`heading_hold` from `fcctl status`
(refreshed after a successful Ready) and warns when `max_vx` exceeds the robot's speed cap (unknown cap: stock
0.49) or the running options differ from the preset's. Each run's `params.json` records `preset`,
`preset_modified`, `preset_profile`, `preset_stack`, `profile_running` (acks), `stack_running`, `estops` and the
link address. Shipped (two; older ones are in git history before the Nov 27 simplification):
- `play`: the demo brain at max_vx 0.75 under a walker cap of 1.5 (`_stack.speed_cap`), so the speed ladder runs
  live: `ctl set max_vx=0.9 wall_brake=0.5` (= the old `dribble-fast`), then 1.0. `stale_stop_s` 1.6 (the brain dead
  reckons through short tag dropouts).
- `slow`: the same brain at walking speed (0.15 m/s, stock walker cap) for the first motion after a change.

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
