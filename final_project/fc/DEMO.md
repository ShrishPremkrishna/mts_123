# G. Demo plan: Pupper FC, Dec 3 (written Nov 23)

Sources: `fc/` SPEC, README, RUNBOOK, lane notes, old `DEMO.md`. Nothing here is measured on a robot yet; thresholds are targets.

## What wins the room

A robotics engineer asks three things in two minutes: *Is it really deciding by itself? Does it survive being messed with? Does this person know its limits?* So the demo is built on four proofs:

1. **Nobody is driving.** Shrish's hands are empty. Nobody touches the laptop after one button.
2. **Glass box.** The screen shows what the robot believes (outline), what it wants (target cross, path), and why (role + phase in plain words). The audience can predict the next move.
3. **It adapts.** A judge puts the ball anywhere. Roles flip on screen.
4. **It fails safe, and we count our touches.** Blind it and it stops in 0.5 s. A "referee touches" counter stays on screen.

Toy signals: dead air while a slow robot walks, gimmicks talking over the presenter, fumbling after a fall.

**Two hard facts shape everything.** (a) The robot walks 0.3 m/s, so every spoken line must be said *while* it walks. (b) In sim a 1v1 match gives about one goal per minute in total, and the goalie is good (0.36 m body in a 0.61 m mouth). A cold kickoff has roughly a 1-in-4 chance of no goal in the slot. So the first goal is staged, and the wow is the role flip, not the score.

## 1. Run of show (Variant A, two robots, 120 s)

People: **Shrish** talks, hands free. **Referee** has the ball, a stick, and the gamepad (R3 = e-stop). **Operator** sits at the laptop (space = stop, fallback keys). Robots are homed, standing, bridge up and health green *before* the slot. Ball staged 0.7 m in front of red's goal, blue 0.5 m behind it, red out of position at midfield.

| Time | Robots | Screen | Shrish says |
|---|---|---|---|
| 0:00–0:08 | Operator presses **Play**. Blue walks at the ball. | Overhead view, outlines, target cross, chips "BLUE attack / getting behind ball", "RED defend". Timer starts. | "Two robot dogs, one laptop, nobody driving. Hands are empty." |
| 0:08–0:28 | Blue pushes, red runs back to its line. | Path line, phase "pushing". Latency chip "camera to legs 0.15 s". | "A camera overhead finds robots and ball 30 times a second. Each robot picks a job, attack or goalie, and a spot to walk to: that cross. Commands expire in a quarter second, so if anything dies, the dog stops." |
| 0:28–0:35 | **Goal** (or save). Robots pause 2 s. | GOAL flash, 1–0, short crowd sound. | "One–nil." (If saved: "Red read that.") |
| 0:35–1:05 | **Wow 1.** Referee hands the ball to a judge: "anywhere you like." Play resumes by itself when the ball rests. Roles flip. | Chips swap colour. Ticker: "red is 1.2 s closer, red attacks". | "Nobody assigns roles. Each robot estimates who reaches the ball first. Watch the labels flip." |
| 1:05–1:20 | **Wow 2.** Referee holds a card over the attacker's tag for 2 s, then removes it. | Red banner "BLUE: can't see itself, stopped". Clears. | "If it can't see, it stops. Half a second, measured. Then it carries on." |
| 1:20–1:45 | Play continues to goal 2 or a wall ball. Referee taps a stuck ball with the stick. | "Referee touches: 1". | "On the floor this week: N of M goals, median T seconds. The sim, loaded with our measured lag and heading error, predicted P. It can't dig a ball out of a corner yet, so the referee does, and we count it." |
| 1:45–2:00 | Operator hits space. Robots stop. | End card: 3 measured numbers, name, repo QR. | "Same brain runs the sim and the dogs. The brain is about 550 lines of Python on a stock walking policy. I'm Shrish. I built the control stack. Thanks." |

Rules: no goal by 0:40, go to Wow 1 anyway. Behind time, cut Wow 2 first. Stop at 1:55 whatever is happening.

## 2. Variants

**B. One robot (most likely). "Robot vs humans."**
- 0:00–0:35 solo goal from the staged spot, same lines.
- 0:35–1:10 judge places the ball on the wrong side; robot orbits and scores. This is Wow 1.
- 1:10–1:20 tag-cover stop.
- 1:20–1:45 operator presses **Goalie** (say so: "one button, I'm switching its job"). Referee rolls three slow shots. Saves are the fastest, most visual thing the robot does.
- 1:45–2:00 numbers and close.

If robot 2 arrives but robot 1's gyro is still faulty, the healthy robot is the star of B.

**C. Robots unreliable.**
- 0:00–0:35 live: the single most reliable primitive (goalie saves, or click-to-point plus tag-cover stop).
- 0:35–1:35 the film's hero cut: real speed, uncut, overlay beside it.
- 1:35–2:00 the honest story: "The gyro on this robot has a fault on one axis. We traced it to the chip with the motors off. Our monitor catches it and stops the robot."

**Decision rule (Dec 2, 8 pm, from Dec 1–2 counts; never upgrade on the day).**

| Run | Needs all of |
|---|---|
| A | Each robot: solo push ≥ 8/10 in ≤ 30 s. 1v1: a goal within 90 s in ≥ 4/5 matches; roles flip on a moved ball ≥ 8/10; 0 falls in 10 match-minutes. Both robots: 3 sessions of 10 min in a row with no `imu_bad`. 5 timed run-throughs: ≥ 4 clean. |
| B | Push ≥ 7/10 in ≤ 30 s. Moved-ball goal ≥ 6/10 in ≤ 40 s. Goalie ≥ 7/10. Tag-cover stop and resume 10/10. 3 sessions of 5 min with no `imu_bad`. Run-throughs ≥ 4/5 clean. |
| C | Anything less. |

"Clean" = no fall, no unscripted laptop touch, ends by 2:00. A degrades to B live (referee lifts the dead robot out), B to C (operator plays the film). Both must be one key.

## 3. Failures on stage

| Event | Action | Line |
|---|---|---|
| Fall | Referee: R3, lift it out. No restart (homing needs the stand and 100 s). Other robot plays on, or film. | "No get-up yet. That's the next thing I'd build. Blue plays on." |
| `imu_bad` stop | App has already zeroed it. Referee: R3. Continue as B or film. | "That's the health monitor. This gyro has a known fault. It stopped rather than walk on bad data." |
| Tracking lost | Wait 3 s, hands off. Then referee checks the tag for shadow or tilt. | "It lost sight of itself, so it stopped." |
| Ball stuck, robots tangled | Referee stick, counter +1. Announced as a rule, like a throw-in. | "Referee touch, that's one." |
| App crash | Bridge stops robots in 0.25 s. One relaunch (must be < 10 s, scripted). Else film. | "Laptop goes quiet, robots stop. You just saw it." |

Never debug on stage. Never press Play twice to hide a fault.

## 4. Must be true by Dec 1, and the daily plan

Besides the table above:
- **Venue.** Where is it, is our field there, setup minutes, projector input, slot order, Wi-Fi. Assume the worst: own router, own camera boom, corner tags to recalibrate. Cold setup drill ≤ 8 min.
- **IMU.** Verdict from the 600 s tests, and whether heat triggers it. Robot start to slot ≤ 4 min. Replacement fitted by Nov 28 or not at all.
- **R3 recovery.** Does X stand it back up after R3? Unknown; test Nov 25.
- **Liveliness.** One A/B: preset `fast` and a smaller marked field (`field.json`) against goals per minute and falls. Try the field in `sim_eval` first.

**Record every session from today**: fixed wide camera plus screen capture. The robot may die any day; footage insures C and the video.

| Day | Work |
|---|---|
| Nov 24 | Runbook 0–6: IMU verdict, five stops, step calibration, goto. Ask staff: board, robot 2, venue, lab access over Thanksgiving. |
| Nov 25 | Push ×10, goalie ×10, R3 drill. Fix the top failure. First goal on film. |
| Nov 26 | No floor. Features 1–5 below. Sim retuned with measured lag and bias. |
| Nov 27 | Variant B end to end: moved ball ×10, tag cover ×10, first timed run-through. `fast` and small-field A/B. |
| Nov 28 | Robot 2: stage, runbook 0–6, first 1v1. **Not playing under the brain tonight means A is dead.** Hardware freeze. |
| Nov 29 | Film day 1. Feature freeze. |
| Nov 30 | Pickups, counted trial sets, rough cut. |
| Dec 1 | Final counts, 5 timed run-throughs, edit. Param freeze. |
| Dec 2 | Submit video by noon. Pack-and-setup drill. 5 rehearsals with the script aloud. Decide variant 8 pm. Charge. No code. |
| Dec 3 | Early setup, calibrate, light check, two warm-up goals at most. |

**Video shots (≤ 2:50, real speed, sim labelled):**
1. Uncut 25 s autonomous goal, wide, hands visible.
2. The same clip beside the Play screen.
3. Ball moved mid-approach, robot re-plans.
4. Role flip (two robots, else labelled sim).
5. Three goalie saves.
6. Safety: tag cover, laptop killed, stop in 0.5 s.
7. Sim and real side by side, same brain.
8. Results card with attempts as well as goals.
9. One failure clip and the limit it shows.

## 5. Software still needed, ranked

| # | Feature | Verdict |
|---|---|---|
| 1 | Play screen + one-button Ready (in progress): big role chips, target cross, path, plain-words phase | Must. It is the glass box. |
| 2 | Goal flow without a reset: flash, score, 2 s pause, auto-resume when the ball rests in the field | Must. Removes every laptop touch and all dead time. |
| 3 | Plain-words stop banner from existing health data | Must. Turns failures into the safety story. |
| 4 | Decision ticker from real brain transitions ("red 1.2 s closer, attacks") | Do. This is the true commentary. |
| 5 | Match timer, referee-touch counter, drop-a-robot key, play-film key, end card | Do. Each is minutes. |
| 6 | Staged-kickoff preset (start poses via goto) | Do if #2 lands early. |
| 7 | Log replay onto the Play screen | Skip. Screen-record every run instead. |
| 8 | Robot goal celebration (spin) | Skip. Costs seconds, adds fall risk. Screen flash is enough. |
| 9 | Gemini live commentator voice | **Don't.** It talks over Shrish, lags events, can call things wrong, needs venue internet, and reads as garnish to control engineers. #4 is the truthful version. |
| 10 | Onboard camera feeds | Skip unless free. Not in the control loop; adds Pi heat. |
| 11 | Headbutt, self-righting, corner extraction | No. 25% in sim; unproven; the referee rule covers it. |
