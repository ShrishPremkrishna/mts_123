# Pupper FC demo plan (Thu Dec 3, 2026)

Updated Nov 28 (after the Nov 27 evening floor session and robot 2's clone). Current state:
[STATUS.md](STATUS.md). Next floor session: [RUNBOOK.md](RUNBOOK.md). Numbers marked sim are targets until measured on
the floor. On stage the robot **shoots by running through the ball**: no kick policy in this demo.

## The vision: a 1v1 exhibition match

Two Puppers play a short, watchable game. Nobody drives; the Play screen shows what each robot sees and why.

1. **Warm-up (W)** while Shrish talks: each robot trots, circles, shuffles and spins at its kickoff spot (inside the
   fence margin, away from the other). **W** again: both walk to their spots facing the field.
2. **Kickoff (K)**: robots on the 25 % / 75 % lines, referee puts the ball on the centre spot, big **3-2-1**, play.
3. **90 s match**, the Coach assigning tasks (one on the ball, the other shadows goal-side or keeps goal).
4. **Goal**: the scorer spins on the spot; both walk back to their kickoff spots; referee re-spots the ball; 3-2-1.
   **P** starts play if the flow waits on a misread ball or spot. **Space** stops everything at once.
5. Full time: end card (score, time played, referee touches).

**Target: ≥ 2 goals per 90 s game.** Floor, one robot vs Shrish (Nov 27): 5–1 in a 90 s show-flow match. Sim 1v1 with preset `play` (Nov 28 rehearsal, the
app's show flow, sim referee for traps): **1.0 goals per 90 s, 7 of 25 games reach 2**; `sim_eval --floor25` 3.3 goals
/ 3 min. `aggression` 1.0 gave 4.0 / 3 min with harder bumps (closing max 0.64 m/s): floor-check anti-ram first
([notes/brain.md](notes/brain.md#1v1-coach-2026-11-27-sim-only)). The biggest lever is fewer corner traps, not the Coach. Setup: `ctl preset play`; show settings (90 s, spots, 3-2-1, spin) are plain params in `config/params.json`.
Robots wear their AprilTags styled as jerseys (blue 10, red 11).

What the show must prove: **nobody is driving** (hands visible), **glass box** (target, path, task on screen),
**it adapts** (ball moved → re-plan), **it fails safe** (cover a tag: it stops; referee touches counted on screen).

## Gates (on the floor, counted)

| Gate | Pass | If not |
|---|---|---|
| Speed | **Done Nov 27:** 1.0 and 1.2 clean, 1.4 fell → play at `max_vx` 1.0–1.1 | — |
| Fence | Wall and corner balls: no tip; a trapped corner ball = referee touch (open: the fence is bent) | Larger `wall_margin_m` |
| Solo show (1 robot) | Show flow W → K → goal → celebration → spots → 3-2-1 runs clean ×3 (1 of 3 Nov 27: 5–1; calibration and camera drops needed restarts) | Fix before robot 2 time |
| Robot 2 G0 | Stops, step test, heading bias < 5° | Solo fallback below |
| 1v1 (at `max_vx` ≤ 1.0 until anti-ram is floor-checked) | ≥ 2 goals in ≥ 3 of 5 90 s games; 0 falls; bumps at walking pace only | Solo fallback below |

**Fallback (decide Wed Dec 2, 8 pm; never upgrade on the day):** one robot plays the same show vs Shrish or the
referee (keeper + dribble, `play` alone uses the vs-human rules), then the 1v1 as film.

## Robot 2 (critical path)

Card done Nov 28: a live clone of mts onto a USB drive with an identity reset (hostname `pupper-red`, robot_id red, tag
11; [PROVISION.md path A](robot/provision/PROVISION.md#path-a-clone-mts-used-nov-28)). It boots, answers `ssh pupper-red`
(LAN 10.5.6.37) and `provision.sh --check` passes except the gamepad. Remaining, in order
([RUNBOOK robot 2 block](RUNBOOK.md#robot-2-red-first-motion--shrish-present)):

1. Its own DualSense paired (each robot needs its own R3 stop).
2. `fcctl.sh imu-level` on the stand (the bias is per chip; the clone has no `imu_cal.json`).
3. A tag-11 board in a **different colour** from blue's, then `ctl robotcal red` on the live camera.
4. Step test facing east and west. Eye offsets are global today, so red's tag mount must copy blue's until per-robot
   `eye_*` is wired (open item).
5. First motion with Shrish present: stops, then the solo show once, then the 1v1 at `max_vx` ≤ 1.0.

## Who does what

- **Shrish (owner)**: robot 2, the video (due Dec 2), the stage script, the venue Wi-Fi (RTT p95 ~75–80 ms on CS.Test.PSK Nov 27; campus Wi-Fi was ~200–250 ms),
  the UI pass, rigid jersey tags, balls, room access.
- **Operator** at the laptop (space, Esc). **Referee**: ball, stick, gamepad R3. Robots homed, standing, health green
  before the slot.

## Calendar

Next floor session: [RUNBOOK](RUNBOOK.md). Robot 2 G0 by Sat 28 night (card done Nov 28; motion steps above remain). Sun 29 feature freeze, film
day. Mon 30 counted games. Tue Dec 1 param freeze, 5 run-throughs, venue drill (setup ≤ 8 min). Wed 2 video in,
rehearse ×5, no code, charge. Thu 3 two warm-up goals, demo. **Do not build:** kick policy, commentator, retraining, self-righting.

## Failures on stage

| Event | Action | Line |
|---|---|---|
| Fall | Fallen cutoff drops it (or R3). Lift it out; play on solo or show film | "No get-up yet. It cut its own motors." |
| `imu_bad` / tracking lost | App already zeroed it. Wait 3 s, referee checks the tag | "It lost sight of itself, so it stopped." |
| Ball stuck in a corner / robots tangled | Referee stick, counter +1 | "Referee touch, that's one." |
| App crash | Bridge stops the robot within 0.25 s; one relaunch (< 10 s), else film | "Laptop goes quiet, robot stops." |
| E-STOP not confirmed | R3 | — |

Never debug on stage. Never press Play twice to hide a fault.
