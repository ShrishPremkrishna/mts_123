# UI handoff (Nov 24, 2026, night)

For the next chat continuing the Pupper FC screens. Usage and keys: [../README.md](../README.md) (Screens section).

## Principles (read first)
- **North star:** a 2-minute live demo (Dec 3) of two Puppers playing soccer. The UI serves that demo and the
  operator. Plan: [../DEMO.md](../DEMO.md); status: [../STATUS.md](../STATUS.md).
- **Iterate, don't one-shot.** Small, working, visible increments; check each in sim (`fc/run.sh`) and on the
  synthetic camera (`fc/run.sh camera --video synthetic --no-link --dry-run-ops`) in a browser before the next.
- **The UI is a client of operations, never the place where logic lives.** Every button is one operation in
  [../ops.py](../ops.py), usable identically by the UI, `python -m fc.ctl op <name>` and agents. Need something
  new? Add the operation (with a test) first, then the button. Availability, labels, next arguments and the
  robot's next setup step are computed on the server (`/state` `avail`, `play.robots[id].setup`).
- **Build from the kit.** [../ui/kit/](../ui/kit/): `tokens.css` (every value), `ui.css` (one class per
  component, state in `data-*`), `kit.js`, `pitch.js`, `robots.js`, `feeds.js`. Screens only add layout. A new
  look means a new primitive in the kit, not a one-off style in a screen.
- **Don't break the robot path.** Control loop, link, brain and robot side belong to the main FC session; keep
  backend changes in the request layer, small and tested, and say so in the commit. Never block the 30 Hz loop.
- **Honest state.** Never show a robot as ready unless `/state` says so; a stale feed shows NO SIGNAL.
- **Safety keys are sacred:** Space = STOP and Esc = E-STOP work on every screen, over any dialog, no confirm.
- **No explanatory copy.** Key prompts, icons and one-line states; no sentences describing what things do.
- **Before pushing:** `../.venv-fc/bin/python -m pytest -q fc/tests` from `final_project/`, all three screens in sim
  and on the synthetic camera, commit only your files.

## How it fits together
- `fc/app.py` serves `/` (ui/home.html), `/play` (ui/play.html), `/ops` (ui/index.html, the old test bench, not
  restyled), `/kit/<name>.(css|js)`, `GET /state[?lite=1]`, `GET /api/ops`, `GET /api/config`, `POST /api/op`,
  `/view.jpg`, `/cam/<id>.jpg`. The legacy POST routes (`/action`, `/robot`, `/params`, `/preset`, `/ballcal`)
  still work for `/ops` and the older `fc.ctl` commands.
- `kit.js` `createOps()` loads the manifest; `ops.prompt(key)` makes a key-bar button whose label and enabled
  state follow `avail`; `ops.bindKeys()` runs whichever op of a key is enabled (P is play-here / start-now /
  pause / resume; a test enforces exactly one). Screen-only view actions (C, 1, 2, Tab, H, arrows) are
  `ops.local(...)` entries in the same registry.
- Mode switch: `app.source` re-execs `fc.app` with the same flags and the other `--source` (refused while a
  match, test or robot op runs). Play mode runs one `fcctl status` per robot at start.

## Verified (sim + synthetic camera + 201 tests; not on a robot, a real camera or a projector)
Home in both modes, mode round trip (same PID), ready check, kickoff/countdown/play/pause/end card, merged
ALL ROBOTS E-STOPPED banner, Robots drawer, setup steps with injected phases, step-specific confirm dialogs,
sim robot's-eye view and the 1/2 swap. Screenshots: [play-screenshots/](play-screenshots/).

## Known gaps / next
1. Real robot setup flow on mts: Check → Home legs → Stand up → Ready, and Re-arm after an E-stop.
2. Projector check from the back row (contrast of soft kit colours, type sizes).
3. Onboard cameras: `robot/camstream.py` never run on a Pi; measure heat (`robot/heat.py`) and link RTT with it
   on before setting `cam_url`.
4. `/ops` still has the old look; restyle with the kit only after the demo.
5. Health text has a few CLI-flavoured strings from the backend (e.g. "link off (--no-link)").
6. Never expose real-robot control publicly (no `/api/op`, `/action`, `/robot` or UDP link on the internet).
