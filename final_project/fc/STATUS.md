# Pupper FC status

**As of Wed Nov 25, 2026 (S2 floor session + software since, `e70fa1c`).** Update this page when a capability changes.
Plan and gates: [DEMO.md](DEMO.md). Floor steps: [RUNBOOK.md](RUNBOOK.md). Contract: [SPEC.md](SPEC.md). App usage:
[README.md](README.md). Pi side: [robot/README.md](robot/README.md). Design notes: [notes/](notes/).

Labels: **robot** = observed on the physical robot; **device-free** = automated tests, replays of logged runs or a
Pi/Docker run with no motion; **sim** = our kinematic sim (`fc.sim_eval`) or MuJoCo. Run IDs are `fc/runs/2026112xT<id>`.

## Nov 25 floor session (robot `mts`, purple ball) — robot

| Result | Value |
|---|---|
| Calibration | `ballcal` purple ball HSV hue 150–162 (`config/ball_color.json`); `rollcal` `ball_decel` **0.149** from 6 rolls (in params.json); field from camera 3.03–3.06 × 2.46–2.54 m, goal mouths 1.22 m |
| `athletic` | 3/3 goals (10.7–16.1 s); one IMU hang (below) |
| `fast-light` | 3/3 goals (12.6–14.8 s) |
| Rejected | `control` preset: timeout, 12 wz reversals. `fast-side` (6 cm off-centre push): slow, 26–31 s |
| **`dribble` (v3)** | 3/3 goals in 4.3, 7.4, 9.3 s, then **6/6 solo goals from anywhere** (8.6–20.3 s) |
| vs a human | 0–3: role flipping and own-tag/ball occlusion |
| IMU / I2C | I2C bus stuck (SDA low) froze the IMU mid-walk; bridge lay down. Touchscreen overlays disabled on mts ([robot/README.md](robot/README.md#i2c-bus-change-on-mts-2026-11-25-operator-approved)); healthy since. IMU driver v3 installed (symlink), **not yet run with the stack** |
| Link | campus Wi-Fi RTT p95 ~200–250 ms with nothing streaming → own travel router recommended |

Dribble analysis (Fable, device-free on the logs): the remaining stop-and-go comes from stale-pose stops, caused by
the robot's tag blurring in the fast gait; contact at ~0.23 m/s; best finishes released the ball 0.9–1.0 m out.

**Nov 24 (S1, stock profile, robot):** homing 24–27 s, link E-STOP, dead-man 0.32 s, goto ×3 within 7 cm,
`latency_s` ≤ 0.14, slow heading offset 180 + 7°, fast gait ~8° right of slow, 0 falls in 8 fast pushes; slow 5/13,
`fast-light` 3/7 goals. LAN UDP ~8 ms (`robots.json` `udp_host` 10.5.6.112, DHCP may change). Heat: idle ~57 °C after
operator-approved OS changes (undo in [robot/README.md](robot/README.md)). IMU background: [RCA](notes/rca-20261124-shaking.md), [filter](notes/imu-filter.md).

## What works

| Capability | Verified by |
|---|---|
| Homing, stand, lie-down, link E-STOP, dead-man, goto, fast gait at cap 0.8; push goals with `fast-light`, `athletic`, `dribble` (v3) | **robot** Nov 24–25 (above); fast gait also **sim** (MuJoCo 0.73 m/s, 0/8 falls) |
| Signed UDP link, 50 Hz bridge, 0.25 s dead-man, fallen cutoff 45°, E-STOP latched until Ready | **robot** (stops) + **device-free** |
| IMU health hold; bridge auto lie-down + latch on a sustained bad IMU | **robot** (fired on the Nov 25 I2C hang) + **device-free** |
| IMU accel calibration (rca-20261127-tilt): driver v4 `FC_IMU_CAL` bias + `fcctl.sh imu-level` + ready at-rest check | **device-free** + measured on mts (accel X +3.05 m/s², false 10°); v4 built, NOT installed; fcctl tests unrun |
| IMU filter (`FC_IMU_FILTER`, default on in `stock.sh`); driver v3 stall guard (marker + re-init) + bridge short-stall hold (≤ 1 s stand and resume, longer lie down); `fcctl.sh i2c-recover` | v2 **robot** (stand 2.0–2.8°); v3 **device-free**, installed on mts, not yet run with the stack; i2c-recover untested on a hung bus |
| Field from corner-tag calibration, `ctl ballcal`, `ctl rollcal --apply`, robust ball velocity (`fc/ballvel.py`) | **robot** Nov 25 (calibrations above) + **device-free** |
| Play screen, `/ops` tests, match flow, run logs, presets with `_stack`, `fc.ctl` | **device-free** + **sim**, used on the **robot** |
| CAMERA PAUSED (frozen feed → robots stale and stop); camera goalie save counting; predicted ball path and plan overlay | **device-free** + sim |
| Goalie, interception | **sim** only (goalie: 1 robot run Nov 24, 0 conceded) |

## New since the floor session (not run on the robot yet)

| Change | Verified by |
|---|---|
| **Board blob tracking**: tag not decoded (fast-gait blur) → the blue tag board near the last fix gives position (`RobotObs.src` "blob"). Colours in `config/robot_colors.json` (`ctl robotcal <robot>`; red has none → tag only). Offline replay of Nov 25 runs: fresh pose in fast-gait frames **3–6 % → 92–96 %**, position vs live tag median 1.4 cm (p90 2.8), 0 distractor blobs, +0.5 ms/frame | **device-free** (replay) |
| **Gyro heading between tags**: bridge acks carry yaw rate `gz` (`drive_bridge.YawRate`); the eye integrates it across tag gaps (commands as fallback). Synthetic chain test: ~1–2.4° median over a 1 s gap vs ~21° from commands | **device-free** (synthetic); **not staged on mts** |
| **`vs-human` preset**: dribble v3 + contested-ball smoothing, hard ball-only role rules (2.5 s defend hold), dead reckoning through own-tag (1.5 s; app `stale_stop_s` 1.6, approved by owner) and ball occlusion, wide-mouth goalie. Sim vs a modelled dominant human: goals against 17.0 → 15.8 /3 min, role switches −16 %, stale stops 17 → 5 s; solo unchanged ([notes/brain.md](notes/brain.md)) | **sim** only |
| **Autoplay** (`ctl test autoplay minutes=N`, key A on Play): attack, score, dribble the ball back to centre, repeat; ball in a net → waits for a person to roll it out | **device-free** + **sim** (3 min, 4 goals in 4 cycles) |
| **Robot POV feed**: 160×120 @ 5 fps (~1.3 % Pi CPU, 0.13 Mbit/s, measured on mts), shown on Play for real robots; starts with Ready (`pov=false` / `ctl ready --pov off` to skip) | **device-free**; not watched on Play with the robot |
| **Merged UI**: operation registry (`/api/ops`, `fc.ctl ops`), new home `/` and `/play`, setup next-step button, `/play-old` fallback, safety fixes (fail-safe stop/e-stop, fresh-phase setup, confirmed mode switch) | **device-free** + **sim**; setup button not yet watched on mts (use `ctl ready --home` / `/play-old` until then) |

## Unverified

- Everything in the table above on the live camera and robot; IMU v3 under the stack.
- Contact ladder 0.4 / 0.6 / 0.75; goalie reaction time and save counting vs a hand count; interception.
- Bridge short-stall hold on a real fault. Robot 2: not in hand ([PROVISION.md](robot/provision/PROVISION.md)).

## Next session

Robot files from `e70fa1c` are **not staged** (Pi was offline at commit time).

1. `bash fc/robot/stage.sh mts` then `bash fc/robot/sha_check.sh mts`.
2. Re-home on the stand (first run of IMU driver v3): check `robot.log` for `FC_IMU_FILTER` and stall handling.
3. `ctl robotcal blue` on the live camera.
4. Dribble pushes with blob tracking + gyro heading: expect far fewer stale stops.
5. Autoplay, watched from afar.
6. vs-human round 2 (A/B `hard_roles`).
7. POV feed on Play; link RTT during play.

## Blockers (owner)

Own travel router (campus Wi-Fi RTT); fan; robot 2; replacement IMU/control board; field fence (45° corners);
red rigid tag ≥ 15 cm; more light balls matching the purple one; Coda access Nov 26–29.
