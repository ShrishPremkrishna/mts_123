# Pupper FC status

**As of Fri Nov 27, 2026, late (floor session Nov 27 + sim work after it).** Update this page when a capability changes.
Plan: [DEMO.md](DEMO.md). Next floor session: [RUNBOOK.md](RUNBOOK.md). Contract: [SPEC.md](SPEC.md). App usage:
[README.md](README.md). Pi side: [robot/README.md](robot/README.md). Design notes and sim tables: [notes/](notes/).

Labels: **robot** = observed on the physical robot; **device-free** = automated tests, replays of logged runs or a
Pi/Docker run with no motion; **sim** = our kinematic sim (`fc.sim_eval`) or MuJoCo. Run IDs are `fc/runs/2026112xT<id>`.

## Floor history — robot (`mts`, purple ball)

| Day | Result |
|---|---|
| **Nov 27** (fenced field 3.36 × 2.70 m) | After service the IMU read a false ~10° tilt and the bridge lay the robot down (imu_tilt). Cause: accel **+3.05 m/s² bias on chip X**, a sensor fault, not the mount ([RCA](notes/rca-20261127-tilt.md)). Fixed with driver **v4** + `imu-level`: **0.26°** level on the stand. Ball colour and blue board (`robotcal blue`) recalibrated for the fenced field. On a corner ball the robot walked into the fence and **tipped** (fallen cutoff, 137°) — before the fence fix below. Playing vs a human: **defence weak, play not fast enough** (mean ~0.3 m/s) |
| Nov 25 | `ballcal` hue 150–162, `rollcal` `ball_decel` 0.149, field from camera 3.03–3.06 × 2.46–2.54 m, mouths 1.22 m. Push brains 6/6 goals (10.7–16.1 s). **Dribble v3** (now the attack in `play`): 3/3 goals (4.3–9.3 s), then 6/6 solo from anywhere (8.6–20.3 s). vs a human 0–3 (role flipping, own-tag/ball occlusion). An I2C hang froze the IMU mid-walk; touchscreen overlays disabled ([robot/README.md](robot/README.md#i2c-bus-change-on-mts-2026-11-25-operator-approved)) |
| Nov 24 | Homing 24–27 s, link E-STOP, dead-man 0.32 s, goto ×3 within 7 cm, `latency_s` ≤ 0.14, fast gait ~8° right of slow, 0 falls in 8 pushes at cap 0.8. LAN UDP ~8 ms; campus Wi-Fi RTT p95 **~200–250 ms** idle → own router recommended |

Older preset names in those runs were removed Nov 27 (git history, [notes/brain.md](notes/brain.md#simplification-2026-11-27)).
Dribble analysis (device-free, Nov 25 logs): stop-and-go came from stale-pose stops (tag blur in the fast gait).

## What works

| Capability | Verified by |
|---|---|
| Homing, stand, gentle lie-down, link E-STOP, dead-man, goto; fast gait at cap 0.8; push and dribble-v3 goals | **robot** Nov 24–25; fast gait also **sim** (MuJoCo 0.73 m/s, 0/8 falls) |
| Signed UDP link, 50 Hz bridge, 0.25 s dead-man, fallen cutoff 45° (fired Nov 27), E-STOP latched until Ready | **robot** + **device-free** |
| IMU health hold; bridge auto lie-down + latch on a sustained bad IMU | **robot** (Nov 25 I2C hang, Nov 27 false tilt) + **device-free** |
| **IMU driver v4** on mts (filter + v3 stall guard + accel bias from `~/fc-v1/run/imu_cal.json`); `fcctl.sh imu-level [check\|show]` | **robot** Nov 27 (0.26°); chip still degraded (gyro freezes during homing, RCA) |
| `ready` refuses at-rest tilt > 6° (exit 5, robot left standing); `stage.sh` keeps `imu_params.json` | **device-free** (`test_imu_level.py`); fcctl Linux tests and the ready check not yet seen on the robot |
| Field from corner tags, `ctl ballcal`, `ctl rollcal --apply`, `ctl robotcal`, robust ball velocity | **robot** Nov 25/27 + **device-free** |
| Play screen, `/ops` tests, run logs, presets with `_stack`, `fc.ctl`, merged UI (`/`, `/play`, `/play-old`) | **device-free** + **sim**, used on the **robot** (setup button not yet watched on mts) |
| CAMERA PAUSED on a frozen feed; goalie save counting; predicted ball path overlay | **device-free** + sim |

## Built since the Nov 27 floor session — sim only, not on the floor yet

| Change | Verified by (fenced 3.36 × 2.70 field unless noted) |
|---|---|
| **Two presets** (c68af93): `play` (max_vx 0.75, walker cap 1.5 via `_stack`) and `slow` (same brain, 0.15 m/s). Show settings are plain params in `config/params.json`. Higher speeds: `ctl set max_vx=…` | **sim** parity, 200 seeds: single-robot rows identical to the old vs-human preset; 1v1 4.8 vs 4.5 goals/3 min ([tables](notes/brain.md#simplification-2026-11-27)) + **device-free** |
| **Fence margin + bank shots** (`wall_margin_m` 0.28; wall balls banked off the fence) | **sim**: would-tip fence events **228 → 2**, runs with a tip 90 → 2 of 100 ([Fences](notes/brain.md#fences-2026-11-27-sim-only)) |
| **Keeper redesign**: shuffles mid-mouth, shifts toward the ball from 2.2 m out | **sim**: vs-human shots saved 58 → 72 % (dribbler) |
| **Long shot**: release at speed 0.4–1.0 m out, half turn, play on | **sim**: 54 of 98 solo goals are long shots |
| **1v1 Coach** in the app loop (two robots tracked: possession, one on the ball, the other shadows / keeps goal, anti-ram) | **sim**: bumps **6.1 → 4.1/min** at half the closing speed (0.51 → 0.22 m/s); 4.0 goals/3 min; **59 %** of 90 s games reach 2 goals ([Coach](notes/brain.md#1v1-coach-2026-11-27-sim-only)) + **device-free** (app wiring) |
| **Show flow**: warm-up **W**, kickoff **K** with a 3-2-1, 90 s match, scorer spin celebration, both back to the 25 / 75 % spots | **sim** + **device-free** |
| Speed caps up to 1.5 (`--speed-cap` 0.30–1.50, bridge `--max-vx`, `link.CEIL`) | **sim** (cmd 1.0/1.2/1.4/1.5 → 0.86/0.99/1.11/1.14 m/s, 0/24 falls; [fast-gait §6](notes/fast-gait.md)) + **device-free**; floor only at 0.8 |
| Board blob tracking (tag blurred → blue board near the last fix) | **device-free** replay: fresh pose in fast-gait frames 3–6 % → 92–96 %; board calibrated on the floor Nov 27, effect on stale stops not measured |
| Gyro heading between tags (bridge acks carry `gz`) | **device-free** (synthetic: ~1–2° over a 1 s gap vs ~21° from commands) |
| Autoplay (`ctl test autoplay`, key A) | **device-free** + **sim** (3 min, 4 goals) |
| Robot POV feed on Play (160×120 @ 5 fps, ~1.3 % Pi CPU) | **device-free** (cost measured on mts); not watched on Play |

## Unverified

- Everything in the table above on the live camera and robot; speed above 0.8; corner/wall balls after the fence fix.
- Contact ladder; goalie reaction time and save counting vs a hand count.
- Robot files since 1b2b879 (speed caps 1.5) are **not staged** on mts.
- Robot 2: not in hand; plan in [DEMO.md](DEMO.md#robot-2-critical-path). Its own card gets mts's OS/driver changes from `provision.sh --apply-os`. This is tested on a fake Pi only (2026-11-27) and has not run on a real card.

## Blockers (owner)

Own travel router (campus Wi-Fi RTT); robot 2; replacement IMU/control board for mts; red rigid tag ≥ 15 cm; more
light balls matching the purple one; Coda access.
