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
| Perception | `model.Observation`: capture time, robot pose/ID, optional ball position/velocity, fallen flag and goal event | The perception adapter supplies calibrated field coordinates and capture age. Missing data stays missing. The controller never receives simulator truth. |
| Movement | `model.Command`: body-frame forward/lateral/yaw velocity, expiry and robot ID; `model.Limits` | Measured movement limits and a robot adapter replace simulated motion. Enforce expiry at the robot, exclusive command ownership and Stop before hardware use. |
| Shooting | `Controller` stages/aligns, then produces a bounded velocity strike; decision exposes target pose and phase | A dedicated strike can replace that phase once its entry, cancellation and exit conditions are known. No policy loading is built into this sandbox. |
| Gameplay | `Controller.tick(observation, now) -> Decision`; `reset()` clears play intent | Same controller can run against recorded or live observations. The runtime owns operator Stop, reset and metrics. |

The [project brief](../README.md) records current owners and agreed outcomes for these interfaces.

Field coordinates are metres, origin at centre, +x right and +y up in the viewer. Heading is counterclockwise radians from +x. Body commands are +vx forward, +vy left and +wz counterclockwise radians/sec. The opponent goal is at `attack_sign * length/2`; tests exercise both attack directions. All timestamps currently use one simulation clock. A real adapter must reconcile camera and controller clocks rather than stamp old frames as new.

`Observation.goal` is a referee event relative to this robot (+1 opponent goal, -1 own goal); the simulator currently supplies it. `fallen` will need a real robot status source. `Controller` returns zero for missing/stale/malformed input, requests recovery for a fall, and aborts a strike if the ball leaves its corridor. Its default freshness threshold is a prototype assumption, not a validated camera budget.

## Shared-state executor and deliberate game resume

`integration.py` provides a transport-free two-robot executor and JSONL replay. It starts stopped; an explicit operator `arm` selects a never-used session epoch and named monotonic clock. Each robot has one expiring intent, capped at 0.5 seconds. `attack`, `shoot`, `move` and `defend` use explicit ownership; an optional `auto` intent lets the existing `Tactics` choose offense/defense from fresh shared field state. Role appears separately from execution phase in each decision. Explicit replacement cancels the previous action, and expired intents never fall back to autonomous play. Defense preserves the guard target's heading during repositioning; ordinary movement faces travel on long moves.

Goal pulses latch only after an entire valid field frame is admitted, even during a lease or camera hold. Motion still requires current authority and fresh evidence. Every action yields to the goal return before using its action target or opponent-goal geometry. Stop, tracking loss, expired intent and a configured camera hold continue to stop motion. Camera configuration remains optional for camera-free replay; configured gates start closed.

After the robot has completed its goal return and the operator has replaced the ball, send one deliberate cue per robot:

```json
{"event":"resume","now":15.0,"epoch":"game-1","clock":"replay","robot_id":"pupper-1","frame_seq":301}
```

`frame_seq` must identify the exact current admitted field frame, with a fresh pose at the robot's home position **and heading**, `fallen=false`, and `goal=0`. A completed goal latch, a live intent, and a clear configured camera gate are required; after a camera hold the field observation must also be new enough for resumption. Refusal returns `accepted=false` and a concrete reason. Resume does not move the ball, arm the executor, renew an intent or switch its owner. It consumes the reset immediately, with no queued request that could erase a later goal. A frame reference can be consumed only once per robot; stale/repeated cues and old epochs cannot clear another goal. This event belongs to the operator adapter, not an upstream perception producer.

```sh
.venv-api/bin/python -m final_project.pupper_fc.integration --replay INPUT.jsonl
.venv-api/bin/python artifacts/pupper-fc/2026-11-17/overnight-execution-review/two_robot_replay.py
```

The second command runs the bounded [two-robot fault experiment](../../artifacts/pupper-fc/2026-11-17/overnight-execution-review/two-robot-summary.json) from the active repository root. Its 30 simulated seconds include delayed/dropped/reordered field frames, lease loss, camera holds, goal pulses, manual replacement/resume cues, a later goal and Stop. It uses ideal kinematic bodies and scripted ball/referee events, with no physical contact or collision model. The saved event stream replays identically. Host execution latency is measured around the software calls and is separate from simulated timestamps and unmeasured camera/transport/robot latency.

## Files and limits

- `model.py`: plain data types and coordinate contract.
- `controller.py`: gameplay geometry/state, no I/O.
- `integration.py`, `tactics.py`, `obstacles.py`: leased shared-state execution, optional role selection, camera hold gate and JSONL replay.
- `simulation.py`: acceleration, friction, simplified bumper/body/wall contact and delayed/noisy observations.
- `runtime.py`: one practice session, operator controls and bounded trace.
- `server.py`, `static/`: local API and dependency-free browser viewer.
- `tests/`: geometry, faults, simulator invariants, closed-loop plays and API checks.

Centered, offset, wrong-side and moving-ball baseline scenarios are covered by score-and-return tests in both attack directions. The [saved validation](validation.json) records 143 passing tests plus 48 subtests, 16 score-and-return trials (including bounded delay/noise trials), two explicitly blocked near-wall trials, source hashes and browser checks. A wall can make the required shot pose unreachable; the controller reports **blocked** instead of attempting extraction. Larger noise/delay can also prevent a useful shot. Treat these as experiments, not guaranteed successes.

The simulator uses assumed footprint, bumper, speed, acceleration, friction and restitution. It omits legs, balance, motor dynamics, real contact variability, opponents and obstacles. Offline defense and camera-gate logic do not establish physical acceptance; physical recovery, robot adapters, real perception and boot-to-ready remain separate work. The existing Lab 6 timed action queue is not used for this continuously replaced motion intent.
