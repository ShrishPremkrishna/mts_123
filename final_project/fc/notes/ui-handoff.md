# UI handoff (Nov 24, 2026)

For the next chat continuing the Pupper FC screens. Usage and keys also live in [../README.md](../README.md) (Play screen section).

## Principles (read first)
- **North star:** a 2-minute live demo (Dec 3) of two Puppers playing soccer, switching offense/defense, moving fast and
  deliberately. The UI serves that demo and the operator; everything else is secondary. Plan: [../DEMO.md](../DEMO.md);
  status: [../STATUS.md](../STATUS.md).
- **Iterate, don't one-shot.** Small, working, visible increments; check each one in sim (`fc/run.sh sim`) and in the
  browser before the next. Keep the current screen working while you improve it.
- **The UI is a client of backend operations, never the place where logic lives.** Every action (Ready = home/stand/bridge,
  down, E-STOP, tests like push_ball/goalie/step, ballcal, rollcal, presets) is ONE backend operation exposed by
  `fc/app.py` and usable identically by the UI, by `python -m fc.ctl`, and by agents. Example: "Ready" is a single
  `fcctl ready` that homes, stands and starts the bridge — nobody (human or agent) should need to chain several commands.
  When the UI needs something new, add (or extend) a backend operation and its `ctl` command first, then a button.
- **Don't break the robot path.** The control loop, link, brain and robot side are owned by the main FC session;
  the UI chat owns `fc/ui/*` (and this note). If a backend change is needed, keep it small, add a test, and say so in
  the commit. Never block the 30 Hz loop from the UI (no heavy work in request handlers; feeds/recording stay off the hot path).
- **Honest state.** Never show a robot as ready/healthy/standing unless `/state` says so; missing feeds show "not connected".
- **Safety keys are sacred:** space = STOP and Esc = E-STOP must work everywhere, always.
- **Before pushing:** `/Users/shrish/workspace/cs123/.venv-fc/bin/python -m pytest -q fc/tests` (from `final_project/`),
  check both screens in sim, commit only your files, push `fc-v1` (main is fast-forwarded from it).

## How it is served
- `fc/app.py` (stdlib HTTP, one process): `/` and `/play` → `ui/play.html` (projector), `/ops` → `ui/index.html`
  (tests, params, teleop), `GET /state` (JSON, the UI polls it every 100 ms), `/view.jpg` (clean overhead frame, the
  Play screen polls it at ~15 fps in camera mode), `/cam/<id>.jpg` (onboard feed when `cam_url` is set). POSTs need
  `Content-Type: application/json` from the page's own origin: `/action` (stop, estop, estop_clear, kickoff, play,
  pause, reposition, touch, variant, job), `/robot` (ready with `confirm`, down), `/params`, `/preset`, `/ballcal`.
- `/state` keys the Play screen uses: `t` (server monotonic), `source`, `mode`, `field`, `robots`, `active`,
  `world` (robots x/y/yaw/age, ball x/y/vx/vy/age), `truth` (sim only), `debug[rid]` (`role`, `phase`, `target`,
  optional `plan` [[x,y]…], `my_cost`/`opp_cost` seconds to the ball), `cmds`, `params` (`match_s`, `ball_decel`,
  `stale_stop_s`, `kickoff_x_frac`), `estop`, `ops`, `stack`, `preset`, `eye`, `link`, `dry_run_ops`, and `play`
  (`flow` phase/score/clock/goals/roles/waiting, `last_match`, `playing`, `robots[rid]` action/role/health/phase/cam,
  `ticker`, `banners`, `overlay` homographies, `recording`). Health text comes from `fc/play.py health()`.
- Keys (Play): **space = STOP** and **Esc = E-STOP** always (capture phase, even with a dialog open), K kickoff,
  P play/pause, R reposition, T/shift+T touch, V who plays, 1/2 job, C camera/field view, H hide end card.

## Files and structure
- `ui/play.html`, one file, no build. CSS tokens on `:root`; JS sections in order: server/post, keys, confirm +
  Ready, team cards (`buildCards` once per robot set, `renderCards` per poll), header/clock, centre overlays
  (`renderCenter`: goal, countdown, kickoff, restart, paused, end card, banners, legend), ticker, **motion**
  (`feed`/`sample`: snapshot interpolation, each object's playhead ~0.11 s behind its newest observation, ≤ 0.15 s
  extrapolation then hold), pitch drawing (`mapping` world→px, affine in sim or homography on the camera image;
  static pitch cached offscreen), `frame()` at rAF, `poll()`. Render errors are logged and never show "App not reachable".
- `ui/index.html`: ops page, unchanged in this pass (older dark style, its own `draw()`/`renderRobots()`/`poll()`).

## Design direction (this pass)
Floodlit night match, broadcast scorebug. Navy `#0b1a2d`/panel `#13283f`, chalk `#f3f6ef`, grass `#1f6b3e/#237546`,
kits blue `#2f80ff` (soft `#a6cbff`) and red `#ff4a3d` (soft `#ffb4ab`), volt-yellow ball `#ffd43b`. Type: DIN
Condensed (display: score, names, roles, GOAL; ships with macOS, falls back to Avenir Next Condensed / Arial Narrow)
and Avenir Next (body). Signature device: the -12° skew (scorebug plates, role chips, canvas name tags). Motion is
reserved for meaning: role chip flips on a change, action line slides, goal band + ripple, target ping, plan/aim dashes march.
`prefers-reduced-motion` turns the decorative motion off.

## Done (verified in sim and the synthetic camera, not on a projector or the real camera)
Scorebug + live dot, player cards (role, plain action, health light, "to the ball" times), tokens with nose and
comet trail, dashed "what it sees" ring (sim), grey "NOT SEEN" ring past `stale_stop_s`, target reticle, plan curve,
ball prediction ribbon with stop ring, goal moment, kickoff/restart cards moved low so the centre spot stays visible,
restyled banners/E-STOP, narrow-screen stacking. Screenshots: [play-screenshots/](play-screenshots/)
`two-robots-live.png`, `one-robot-live.png`, `camera-synthetic.png`, `goal-moment.png` (flow forced to goal in the
browser), `plan-path.png` (plan and ball velocity injected in the browser: the brain rarely sends `plan` in a sim match).

## Known gaps
- Not seen on the real projector: check contrast of soft kit colours and the 0.76 em legend from the back row.
- Label tags can cover the ball or target when robots crowd it. Trails are hidden behind tokens at low speed.
- Overlay drawn ~0.11 s behind the newest observation; on the camera image it may trail the picture slightly.
- Onboard camera slot only renders when `cam_url` is set (unchanged logic); no layout for it was tested with a live feed.
- `index.html` still has the old look; the ops canvas has no interpolation.

## Owner backlog
1. **Robot's-eye panel**: `robot/camstream.py` exists, never run on a Pi. The lean stack turns `camera_node` off
   (`--camera on` keeps it); measure Pi heat with it on (`robot/heat.py`) before relying on it.
2. Nicer field animations, a stronger goal moment (kept brief; DEMO.md says no celebrations work before the demo).
3. Public website "play against Pupper": brain and sim are stdlib Python, so Pyodide could run them in the browser.
   Never expose real-robot control publicly (no `/action`, `/robot` or UDP link reachable from the internet).

## Rules
- Never show a robot as ready or healthy unless `/state` says so (`health.level`, `phase == "ready"`).
- Keep space = STOP and Esc = E-STOP working on every screen, with no confirmation for either.

## Test in sim
`fc/run.sh` (sim, http://127.0.0.1:8800/) or `cd final_project && ../.venv-fc/bin/python -m fc.app --source sim --port 8815`.
Camera overlay without hardware: `--source camera --video synthetic --no-link --dry-run-ops`. Tests:
`../.venv-fc/bin/python -m pytest -q fc/tests` (timing tests in `test_eye`/`test_ctl` can flake under heavy CPU load).
