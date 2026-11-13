# Offline approach-and-shot prototype

One gameplay controller approaches behind a ball, aligns the bumper toward the opposing goal, makes a bounded run-through, brakes and replans. After a goal it returns home and waits for ball replacement. The interactive field and headless trials run the same Python controller.

This is a **2D kinematic/contact sandbox**, not MuJoCo, a gait rollout or evidence that a real Pupper can execute these commands. It contains no robot transport, ROS, camera access or cloud calls.

## Run

From the repository root, using the existing API environment:

```sh
.venv-api/bin/python -m final_project.pupper_fc
```

Open <http://127.0.0.1:8765>. The server binds loopback only; all tabs share one practice session. `--port 8766` selects another port. Stop the server with Ctrl-C. Python changes require a server restart; refresh the page after frontend changes. The [root setup instructions](../../README.md) cover the existing environment; headless runs need only Python's standard library.

Choose a scenario and attack direction and press **Reset practice**, then **Start**. Drag the ball or robot, or use the reposition buttons. Add camera delay/noise, hide the ball, freeze observations or nudge the ball while it plays. **Pause** freezes simulated time; **Stop** commands bounded braking and stays latched until Start. An injected fall stops play and requests recovery; clearing that checkbox is an operator injection, not simulated physical get-up.

After a goal, **Replace ball & resume** becomes available once Pupper is home. It puts the ball at centre and starts a fresh play without resetting the robot or score. Reset practice starts a new trial and retains the selected tracking/speed settings.

**Export trace** saves the latest 2,400 frames (roughly two minutes, less with edits), including truth, observations, decisions, settings, metrics and source hashes captured at process startup. **Open trace** plays those frames locally; it does not rerun the controller or send recorded commands. Live controls are disabled during replay except Stop. Replay does not itself pause the background simulation. Truncated exports say so. Exports are opt-in; raw traces are not automatically added to Git.

```sh
python3 -m final_project.pupper_fc --headless --scenario all --seconds 40
.venv-api/bin/python -m pytest -q final_project/pupper_fc/tests
```

Headless JSON reports each outcome, including failures. `--seed 12` changes seeded sensor noise; changing the seed alone does not randomize scenario geometry. `--output /tmp/fc-trial.json` saves a summary to a new file.

## Small integration seams

| Component | Current boundary | Teammate integration |
| --- | --- | --- |
| Perception | `model.Observation`: capture time, robot pose/ID, optional ball position/velocity, fallen flag and goal event | Ryan's adapter supplies calibrated field coordinates and capture age. Missing data stays missing. The controller never receives simulator truth. |
| Movement | `model.Command`: body-frame forward/lateral/yaw velocity, expiry and robot ID; `model.Limits` | Marcus's measured limits and a robot adapter replace simulated motion. Enforce expiry at the robot, exclusive command ownership and Stop before hardware use. |
| Shooting | `Controller` stages/aligns, then produces a bounded velocity strike; decision exposes target pose and phase | Tej's strike can replace that phase once its entry, cancellation and exit conditions are known. No policy loading is built into this sandbox. |
| Gameplay | `Controller.tick(observation, now) -> Decision`; `reset()` clears play intent | Same controller can run against recorded or live observations. The runtime owns operator Stop, reset and metrics. |

Field coordinates are metres, origin at centre, +x right and +y up in the viewer. Heading is counterclockwise radians from +x. Body commands are +vx forward, +vy left and +wz counterclockwise radians/sec. The opponent goal is at `attack_sign * length/2`; tests exercise both attack directions. All timestamps currently use one simulation clock. A real adapter must reconcile camera and controller clocks rather than stamp old frames as new.

`Observation.goal` is a referee event relative to this robot (+1 opponent goal, -1 own goal); the simulator currently supplies it. `fallen` will need a real robot status source. `Controller` returns zero for missing/stale/malformed input, requests recovery for a fall, and aborts a strike if the ball leaves its corridor. Its default freshness threshold is a prototype assumption, not a validated camera budget.

## Files and limits

- `model.py`: plain data types and coordinate contract.
- `controller.py`: gameplay geometry/state, no I/O.
- `simulation.py`: acceleration, friction, simplified bumper/body/wall contact and delayed/noisy observations.
- `runtime.py`: one practice session, operator controls and bounded trace.
- `server.py`, `static/`: local API and dependency-free browser viewer.
- `tests/`: geometry, faults, simulator invariants, closed-loop plays and API checks.

Centered, offset, wrong-side and moving-ball baseline scenarios are covered by score-and-return tests in both attack directions. The [saved validation](validation.json) records 143 passing tests plus 48 subtests, 16 score-and-return trials (including bounded delay/noise trials), two explicitly blocked near-wall trials, source hashes and browser checks. A wall can make the required shot pose unreachable; the controller reports **blocked** instead of attempting extraction. Larger noise/delay can also prevent a useful shot. Treat these as experiments, not guaranteed successes.

The simulator uses assumed footprint, bumper, speed, acceleration, friction and restitution. It omits legs, balance, motor dynamics, real contact variability, opponents and obstacles. Defense, physical recovery, robot adapters, real perception and boot-to-ready remain separate work. The existing Lab 6 timed action queue is not used for this continuously replaced motion intent.
