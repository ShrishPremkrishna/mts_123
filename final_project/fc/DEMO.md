# Pupper FC demo plan (Dec 3, 2026)

Updated Nov 24 (gate status and owner to-do only). Current state: [STATUS.md](STATUS.md). Floor steps: [RUNBOOK.md](RUNBOOK.md). Gate numbers are
targets until measured on the floor. Wording on stage: the robot **shoots by running through the ball**. There is
no leg kick in this demo (maybe a later iteration).

## What the demo must prove

1. **Nobody is driving.** Hands empty; one button on the laptop, then nobody touches it.
2. **Glass box.** The Play screen shows what each robot sees, wants (target cross, path) and why (role, phase,
   ticker). The audience can predict the next move.
3. **It adapts.** A judge moves the ball; the robot re-plans (and, from stage 3, switches role by itself).
4. **It fails safe, and we count our help.** Cover a tag: it stops within 0.5 s. A referee-touch counter stays on screen.

Facts the script relies on: fast gait **0.75 m/s** commanded (walker speed cap raised to 0.8 by preset
`fast-play-robot`; ~0.73 m/s in sim; floor: 0.47 m in a 0.75 burst, gain 0.88, no falls), trot near the ball, shot = contact at speed
(`contact_vx` 0.4 → 0.6 → 0.75, highest that passes). Field 2.63 × 1.93 m, **1.0 m goal mouths**, fence on all
sides. Safety: STOP (space), E-STOP over the link (Esc), gamepad R3, fallen-robot cutoff at 45°, gentle lie-down on `down`.

## Stages and gates

Each stage is its own demo. Film the hero take the day a stage passes. Live variant = the highest stage whose gate
holds in counted sets on Dec 1–2. Decide **Wed Dec 2, 8 pm**; never upgrade on the day.

| Stage | Pass (on the floor) | Target date | If not |
|---|---|---|---|
| G0 loop | Five stops pass; `step` `latency_s` ≤ 0.35; heading bias < 5° | Tue 24 (S1) | 0.35–0.55: set `act_lag_s`, lower `k_turn`, carry on. Above: a bug; fix first |
| G-fast | 0.75 bursts ×6, no fall; real ball at contact 0.4 → 0.6 → 0.75, 0 falls in 10 | Wed 25 (S2) | Use the highest clean contact speed |
| **1. Shootout** | Static ball, 3 spots: ≥ 8/10 goals, median ≤ 20 s | Wed 25 (S2) | Fix the top failure; no keeper work below 6/10 |
| **2. Keeper** (vs human) | ≥ 7/10 hand-rolled shots saved; midfield → on the line ≤ 4 s | Wed 25 (S3) | Narrow to penalty saves |
| **3. Switch** (solo, vs human) | Right role on ≥ 8/10 ball moves, moving within 1.5 s, ≤ 1 wrong flip/min; rolling-ball goal ≥ 6/10 | Fri 27 (S4) | Show Shootout and Keeper as two acts |
| Robot 2 | In hand Wed 25; passes G0 by Sat 28 night | Sat 28 | Lock stage 3 or lower; stop robot-2 work |
| **4. Match** (two robots) | Goal within 90 s in ≥ 4/5; 0 falls in 10 match-minutes | Sun 29 | Stage 3 live, match as film |

**Gate status (Nov 24, robot unless noted; details in [STATUS.md](STATUS.md)):**
- G0: mostly met. Dead-man 0.32 s, link E-STOP and gentle lie-down confirmed; `latency_s` ≤ 0.14; slow heading bias
  fixed by the +7° offset. Open: fast gait walks ~8° right of slow (speed-dependent offset), five-stop set not counted.
- G-fast: partial. Fast gait stable at cap 0.8, 0 falls in 8 fast pushes; contact ladder not run (`contact_vx` 0.3 only).
- 1. Shootout: not met. `fast-light` 3/7 goals (15.9, 21.7, 16.4 s); failures track robot-tag visibility (bigger tag next).
- 2. Keeper: 1 run, 0 conceded; saves not counted in camera mode. Not gated yet.
- 3. Switch: sim only. Robot 2: not available.

Stage 3 uses the solo role rule (defend while the ball threatens our goal, attack otherwise) and the interception
lead, both in the brain since `bc8e68a` and sim-only so far. Fallback: red's tag (id 11) on a box a human moves.

Calendar: Thu 26 sim retune, no floor. Sat 28 hardware freeze. Sun 29 feature freeze, film day. Mon 30 counted sets,
timed run-throughs. Tue Dec 1 param freeze, 5 run-throughs, venue drill. Wed 2 video in by noon, rehearse ×5, no
code, charge. Thu 3 two warm-up goals, demo. Between sessions build only the last session's top failure and the next
gate's test. **Do not build:** leg kick, headbutt tuning, commentator, retraining, corner extraction, self-righting,
celebrations, more UI.

## Run of show (120 s each)

People: **Shrish** talks, hands free. **Referee** has the ball, a stick and the gamepad (R3). **Operator** at the
laptop (space, Esc). Robots homed, standing, bridge up, health green *before* the slot. Every variant opens and closes the same way:

- **0:00–0:10** Operator presses Play. "One laptop, an overhead camera, nobody driving. My hands are empty."
- **1:45–2:00** Operator hits space. End card: 3 measured numbers (attempts as well as goals), repo QR. "Same
  brain runs the sim and the robot: a few hundred lines of Python on the stock walking policy. I'm Shrish. Thanks."

| Time | 1. Shootout | 2. Keeper | 3. Switch | 4. Match |
|---|---|---|---|---|
| 0:10–0:40 | Ball at a marked spot. Robot runs behind it and shoots through it. "The camera finds robot and ball 30 times a second; that cross is where it wants to be." | Robot on its line. Referee rolls a shot; robot crabs across and blocks. "It projects the ball's path to the goal line." | Human rolls at our goal: robot defends. Ball rests: robot attacks. "Nobody picks its job. It reads the ball." | Play from start spots. Blue attacks, red defends. "Each robot estimates who reaches the ball first. Watch the labels." |
| 0:40–1:10 | Judge places the ball anywhere, including the wrong side. Robot orbits and scores. | Two more shots, one from a judge. Score the saves on screen. | Judge rolls and places the ball. Role chips flip on screen. | Judge moves the ball mid-play. Roles flip. |
| 1:10–1:25 | Card over the robot's tag 2 s. Banner: "can't see itself, stopped." Remove; it carries on. | same | same | same, on the attacker |
| 1:25–1:45 | Numbers line: "This week: N of M goals, median T s. Sim with our measured lag predicted P." Referee touch if a ball sticks. | Same line with saves. | Same line with role accuracy. | Play to goal 2 or a wall ball; referee touch, counter +1. |

Rules: no goal by 0:40, move on anyway. Behind time, cut the tag cover first. Stop at 1:55 whatever happens.

## Failures on stage

| Event | Action | Line |
|---|---|---|
| Fall | The fallen cutoff drops it (or referee R3). Lift it out; other robot plays on, else drop a stage or play film. Re-arm with Ready only if rehearsed. | "No get-up yet. It cut its own motors." |
| `imu_bad` / tracking lost | App has already zeroed it. Wait 3 s hands off; referee checks the tag for shadow or tilt. | "It lost sight of itself, so it stopped." |
| Ball stuck in a corner or robots tangled | Referee stick, counter +1, announced as a rule. | "Referee touch, that's one." |
| App crash | Bridge stops the robot within 0.25 s. One scripted relaunch (< 10 s), else film. | "Laptop goes quiet, robot stops. You just saw it." |
| E-STOP not confirmed | R3. | — |

Never debug on stage. Never press Play twice to hide a fault. Keep one key each to drop a robot and to play the film.

## Video (due Dec 2 noon, ≤ 2:50, real speed, sim labelled)

Uncut autonomous goal with hands visible; the same clip beside the Play screen; ball moved, robot re-plans; keeper
saves; role switch; safety (tag cover, laptop killed); sim and real side by side; results card with attempts; one
failure and the limit it shows. Record every session: fixed wide phone plus screen capture.

## Owner to-do (Shrish)

- [ ] **Staff, by Wed 25 evening** (last day reachable): robot 2 (Wed 25 → Dec 3, or fixed checkout slots; ours
      installs into `~/fc-v1` only) and a replacement BNO085 IMU or control board for `mts`.
- [ ] If robot 2 comes: collect it and a second DualSense; run `provision.sh --check` (no motion) while help exists ([PROVISION.md](robot/provision/PROVISION.md)).
- [ ] **Tags (top failure)**: rigid, flat robot tags **≥ 15 cm** ×2 (blue 10, red 11), same mount; measure tag height and offset.
- [ ] Ask for **Coda access Nov 26–29** and whether the field may stay up. Confirm demo room, projector input, slot, Wi-Fi.
- [ ] **Balls**: 3–4 identical light balls, ~18 cm, **≤ 150 g**, matte pink or orange. Weigh them.
- [ ] **Fence**: foam board or 1×4s, ≤ 12 cm high, ~7 m, **corners cut at 45°**. Required: the sim assumes walls.
- [ ] **Goals**: widen the mouths to 1.0 m (already in `config/field.json`).
- [ ] Keep people off the field during runs; use a fixed camera feed (the phone feed froze once).
- [ ] Cold setup drill at the venue ≤ 8 min (own router, camera boom, corner tags).
