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
`--no-link` (compute commands, send nothing), `--dry-run-ops` (robot buttons print their ssh command).

UI: **space = stop all, always**. Pick a robot, press a test button; each run writes
`fc/runs/<UTC>-<test>/{params.json,events.jsonl,summary.json}` and shows in the Runs list.

| Test | What it does | Summary |
| --- | --- | --- |
| Go (`play`) | brain on every active robot with current params | stopped |
| `drive` | hold WASD (move) / QE (turn), scaled by `max_*`; keys idle 0.4 s -> zero | — |
| `goto` | click a field point; simple turn-and-walk controller | `pos_err_m`, `heading_err_deg`, `time_s` |
| `push_ball` | `role_mode=attack` until a goal or `test_timeout_s` | `goal`/`own_goal`/`timeout`, `time_s` |
| `goalie` | `role_mode=defend`; in sim a new shot every `goalie_shot_s` | `conceded`, `saved` |
| `match` | both robots, `role_mode=auto`, for `match_s` | goals during the match |
| `sim_eval` | runs `python -m fc.sim_eval <args>` | output table |

Params: every knob (params.json + brain/sim/app defaults) is editable live; limits are enforced
(speed ceilings 0.5 / 0.3 / 2.0 also in `link.py`). Presets save/load to `config/presets/<name>.json`.

Robot ops run `ssh <host> bash ~/fc-v1/robot/fcctl.sh ...` or `fc/robot/stage.sh <host>` in the
background; `start` asks for confirmation because it homes the legs. The host comes only from
`config/robots.json`; profile must be a `robot/profiles/*.sh` name and `imu_test` seconds an int
in 1–30, and each remote argument is shell-quoted. POSTs need `Content-Type: application/json`
and, when the browser sends `Origin`, this server's own address (else 403).

Eye offline check (error vs. truth on rendered frames):
`python -m fc.eye --source synthetic --seconds 8 --save /tmp/eye.jpg`

Tests: `python -m pytest -q fc/tests`
