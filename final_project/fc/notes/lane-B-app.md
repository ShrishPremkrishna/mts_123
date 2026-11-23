# Lane B: laptop app, link, eye, run logs (2026-11-23)

## Run
From `final_project/`: `python -m fc.app --source sim` (stdlib venv works), open http://127.0.0.1:8800/.
Camera: `/Users/shrish/.codex/worktrees/fc-teleop/cs123/.venv-fc/bin/python -m fc.app --source camera --robots blue`.
Offline camera rehearsal: add `--video synthetic --no-link --dry-run-ops`. Details in [../README.md](../README.md).

## Works (verified)
- `pytest fc/tests` (test_link, test_app): 12 pass under both venvs, with the real brain.py and sim.py.
- Real app over curl and in a headless browser (gstack browse), sim source: Go, Stop, space bar,
  click-to-goto (reached, 4-5 cm error), held-key drive, push_ball (goal in 11-45 s), goalie (3/3 saved),
  15 s match, sim_eval `--quick` table, param edit + ceiling rejection, preset save/load, dry-run robot ops.
  Each run wrote params.json, events.jsonl (~500 B per tick, about 4 MB per 5 min match), summary.json.
- Link packets parse through `fc/robot/drive_bridge.parse_packet` unchanged; ack fields (imu, profile, ok,
  bridge stats) are shown per robot. Ack round trip tested against a local UDP echo.
- Eye on fieldcam-rendered frames: corner calibration with lens fit, median position error 0.1 cm and yaw
  0.1 deg vs truth, both robot tags read; annotated JPEG at /frame.jpg; app ran push_ball on it with link off.

## Not verified / stubbed
- No real camera footage was found (overhead_perception has none; audit folder has notes only), so the Eye
  has not seen a real frame in this lane. Tej's fixes are ported, not re-measured.
- No robot was contacted. Live UDP to the Pi, ssh ops and stage.sh ran only as dry runs.
- Stub brain/sim in app.py remain only as a fallback if `fc.brain`/`fc.sim` fail to import (shown in the header).
- Goal detection on camera uses the observed ball crossing a goal line (3 s cooldown); sim uses sim.events.

## Decisions
- One params dict for everything: app defaults < `brain.DEFAULTS` < `sim.SIM_DEFAULTS` (sim source) < params.json.
  Unknown keys are rejected; numbers are range-checked (`LIMITS`), strings checked against `CHOICES`.
- Tests force `role_mode` (push_ball=attack, goalie=defend, match=auto) and pick sim scenarios
  (attack_solo, shot_on_goal, kickoff); brains reset at every run start.
- App stops a robot whose observation is older than `stale_stop_s` (0.5 s), except in drive.
- Link clamps to min(params cap, hard ceiling), ttl to [0.05, 0.5] s (matching the bridge's MAX_TTL).

## Proposed contract changes
1. SPEC module table says `decide(world, me, params)`; the interface section says `decide(world, params)`. Code uses the latter. Fix the table.
2. Add the `eye_*` knobs (tag height 0.24 m, tag offset 0.065 m, yaw offset, floor tag size 0.15875 m, latency) to
   params.json or per robot in robots.json; they are per-robot physical facts. Today they live in `app.APP_DEFAULTS`.
3. Add app test knobs (`test_timeout_s`, `match_s`, `goalie_shot_s`, `goto_*`, `stale_stop_s`, `teleop_timeout_s`) to params.json so presets are complete.
4. robots.json: red has no host; set it before 2-robot floor tests.
5. `config/presets/` will be created by the UI; decide whether presets are committed.

_Update (review fixes): the stub brain/sim fallbacks were removed; an import failure now stops the app._
