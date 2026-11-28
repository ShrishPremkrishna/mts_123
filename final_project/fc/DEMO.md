# Pupper FC demo plan (Thu Dec 3, 2026)

Updated Nov 27 (exhibition plan replaces the staged Shootout → Keeper → Switch → Match gates). Current state:
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

**Target: ≥ 2 goals per 90 s game.** Sim: 59 % of 90 s games reach 2 goals (Coach, `aggression` 0.75, fenced
field; [notes/brain.md](notes/brain.md#1v1-coach-2026-11-27-sim-only)). The biggest lever is fewer corner traps, not
the Coach. Setup: `ctl preset play`; show settings (90 s, spots, 3-2-1, spin) are plain params in `config/params.json`.
Robots wear their AprilTags styled as jerseys (blue 10, red 11).

What the show must prove: **nobody is driving** (hands visible), **glass box** (target, path, task on screen),
**it adapts** (ball moved → re-plan), **it fails safe** (cover a tag: it stops; referee touches counted on screen).

## Gates (on the floor, counted)

| Gate | Pass | If not |
|---|---|---|
| Speed | Highest clean step on the ladder 1.0 / 1.2 / 1.4 (1.5 optional): no fall, no stumble in bursts and arcs | Play at the last clean `max_vx` (0.75 is floor-proven) |
| Fence | Wall and corner balls: no tip; a trapped corner ball = referee touch | Larger `wall_margin_m` |
| Solo show (1 robot) | Show flow W → K → goal → celebration → spots → 3-2-1 runs clean ×3 | Fix before robot 2 time |
| Robot 2 G0 | Stops, step test, heading bias < 5° | Solo fallback below |
| 1v1 (at `max_vx` ≤ 1.0 until anti-ram is floor-checked) | ≥ 2 goals in ≥ 3 of 5 90 s games; 0 falls; bumps at walking pace only | Solo fallback below |

**Fallback (decide Wed Dec 2, 8 pm; never upgrade on the day):** one robot plays the same show vs Shrish or the
referee (keeper + dribble, `play` alone uses the vs-human rules), then the 1v1 as film.

## Robot 2 (critical path)

Card: use the robot's own card and run `provision.sh --apply-os pupper-red red red 11`
([PROVISION.md path B](robot/provision/PROVISION.md)). It stages `~/fc-v1` and applies mts's OS/driver changes: Hailo off,
fan curve, `pupper-gui` off, GT911 touch overlays off, and the v4 IMU driver built from the card's own source. It prints each
change with its undo and waits for the owner to confirm, then a reboot finishes it. `--check` lists each item as ok/missing.
This is tested on a fake Pi only. The driver build refuses if our patch does not apply to that card's source; the fallback is
cloning mts (path A: two readers, a larger card, mts offline ~1.5–2 h, an identity reset). Then per robot:

1. Identity (path A clone only): `provision.sh --fix-clone-identity pupper-red pi@<ip> red red 11` (hostname, machine-id,
   ssh keys, Tailscale state, `~/fc-v1/run`; robot_id red, tag **11**).
2. `robots.json` red entry (`host`, `udp_host`, `cam_url`), then `stage.sh` + `sha_check.sh`.
3. A tag board in a **different colour** from blue's, then `ctl robotcal red` on the live camera.
4. `fcctl.sh imu-level` on the stand (the bias is per chip; the clone's `imu_cal.json` is cleared with `run/`).
5. Step test facing east and west (heading bias → `eye_yaw_offset_deg`), stops, then the solo show once.

## Who does what

- **Shrish (owner)**: robot 2, the video (due Dec 2), the stage script, the router (campus Wi-Fi RTT p95 ~200–250 ms),
  the UI pass, rigid jersey tags, balls, room access.
- **Operator** at the laptop (space, Esc). **Referee**: ball, stick, gamepad R3. Robots homed, standing, health green
  before the slot.

## Calendar

Next floor session: [RUNBOOK](RUNBOOK.md). Robot 2 G0 by Sat 28 night (Nov 23 target). Sun 29 feature freeze, film
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
