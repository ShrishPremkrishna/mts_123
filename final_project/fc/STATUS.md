# Pupper FC status

**As of Tue Nov 24, 2026, evening (S1 floor session + software since).** Update this page when a capability changes.
Plan and gates: [DEMO.md](DEMO.md). Floor steps: [RUNBOOK.md](RUNBOOK.md). Contract: [SPEC.md](SPEC.md). App usage:
[README.md](README.md). Pi side: [robot/README.md](robot/README.md). Design notes: [notes/](notes/).

Labels: **robot** = observed on the physical robot; **device-free** = automated tests, replays of logged runs or a
Pi/Docker run with no motion; **sim** = our kinematic sim (`fc.sim_eval`) or MuJoCo. Run IDs are `fc/runs/20261124T<id>`.

## Nov 24 floor session (robot `mts`, stock profile) — robot

| Result | Value |
|---|---|
| Homing on the stand / software stand / gentle lie-down (`down`) | 24–27 s / works / smooth, no drop (repeatedly) |
| Link E-STOP (Esc) on the stand; dead-man | confirmed; 0.32 s, 3.4 cm (`043546Z`) |
| goto ×3; step gains (slow) | 5.6–6.9 cm; forward 0.91–1.01, lateral 0.79, turn 1.0, `latency_s` ≤ 0.14 |
| Heading | slow gait offset 180 + 7°; fast gait walks ~8° right of slow (`073515Z`) |
| Fast gait (speed cap 0.8) | stable, gain 0.88 at 0.75; 0 falls in 8 fast pushes |
| Goals | slow 5/13 (26–44 s; 2/2 after push tuning); `fast-light` 3/7 (15.9, 21.7, 16.4 s), 4 timeouts |
| Goalie | 1 run, 0 conceded (saves were not counted then) |
| Eye / link | 31–35 fps live; LAN UDP ~8 ms vs Tailscale p95 1.1 s → `robots.json` `udp_host` 10.5.6.112 (DHCP may change) |

**Top failure: seeing the robot's own tag.** The old paper tag was seen in 18 % of far-half frames (61 % near);
scoring runs saw it 95–100 %, timeouts 46–89 %. **IMU:** the BNO085 shows intermittent gyro-X pulses and false
tilt; it seized once while standing ([RCA](notes/rca-20261124-shaking.md), [filter](notes/imu-filter.md)).
**Heat:** idle ~57 °C after operator-approved OS changes (undo in [robot/README.md](robot/README.md)).

## What works

| Capability | Verified by |
|---|---|
| Homing, stand, lie-down, link E-STOP, dead-man, goto, slow and `fast-light` push goals, fast gait at cap 0.8 | **robot** Nov 24 (rates above); fast gait also **sim** (MuJoCo 0.73 m/s, 0/8 falls) |
| Signed UDP link, 50 Hz bridge, 0.25 s dead-man, fallen cutoff 45°, E-STOP latched until Ready | **robot** (stops) + **device-free** |
| IMU health hold (caught a 14° lean); lean stack, heat in `status` | **robot** |
| IMU filter v2 (`FC_IMU_FILTER`, accel-corrected tilt, gyro-X de-spike) installed on mts; **default on** in `stock.sh` (`51c16f4`) | **robot** (5-min stand 2.0–2.8°, 3 fast runs clean). Default-on needs a restage |
| Bridge auto lie-down + latch on a sustained bad IMU | **device-free** + ROS in Docker; fired on mts Nov 25 (I2C bus hang, `frozen`) |
| IMU driver v3 stall guard (marker + re-init) and bridge short-stall hold (≤ 1 s: stand, resume; longer: lie down); `fcctl.sh i2c-recover` for a hung bus ([v3](notes/imu-filter.md#v3-bno-stall-guard-2026-11-25-built-not-installed)) | **device-free** (unit tests; .so `a64606d8…` built on mts, not installed; i2c-recover untested on a hung bus) |
| Play screen, `/ops` tests, match flow, run logs, presets with `_stack`, `fc.ctl` | **device-free** + **sim**, used on the **robot** |
| New UI (Nov 24 night, not on the robot or projector yet): home `/` (Play / Simulation / Test bench, mode switch restarts in place), `/play` on the [ui/kit](ui/kit/) primitives, robot setup as one next-step button, robot camera slot (sim view; real feed needs `cam_url`), one operation registry (`/api/ops`, `/api/op`, `fc.ctl ops` / `op`) | **device-free** + **sim** + synthetic camera, visual QA in a browser. Merged into fc-v1 via `fc-ui-merge` (Nov 25) with safety fixes: fail-safe stop/e-stop args, setup button re-checks status (fresh phase, `home=true` to home), mode switch asks and keeps the preset, robot ops off in plain sim, inline Space/Esc, `/play-old` fallback. Setup button not yet watched on mts: use `ctl ready --home` / `ctl down` or `/play-old` until then |
| Goalie, solo role switching vs a human, interception | **sim** only |

## New since the floor session (none run on the robot yet)

| Change | Verified by |
|---|---|
| Field length/width from the corner-tag calibration (`field_from_camera`, default on), one live Field; goal mouths **1.22 m** (field.json) | **device-free** |
| Ball colour calibration `ctl ballcal` / Shift-click on `/ops`, saved to `config/ball_color.json` | **device-free** + synthetic camera |
| Robust ball velocity `fc/ballvel.py`: replay of 14 Nov 24 runs gives 0 m/s at rest in every frame (old p95 0.03, max 0.19); synthetic rolls reach 90 % of true speed in 0.06 s | **device-free** |
| `ctl rollcal [--apply]` fits `ball_decel` (sim 0.438 for 0.45). `ball_decel` **0.18** is one global value in params.json, from real hand rolls of the old ball (0.12–0.29, median 0.15). `--apply` is live only | **device-free** + sim; not yet fitted for the new ball |
| CAMERA PAUSED: byte-identical frames are dropped, robots go stale and stop; banner, `ctl status`, run-log event | **device-free** (synthetic frozen feed) |
| Camera goalie save counting (`ShotCounter`: conceded / saved / missed / unclear); a ball lying in the goal scores once | **device-free** + sim (12/12 scripted shots) |
| Predicted ball path and the brain's committed plan drawn on Play and `/ops` | **device-free** + sim |
| **`athletic` preset**: fast-light + planned committed arcs (Dubins to a stage behind the predicted intercept, pure pursuit, replan on real change, `plan_flip_s`), prediction-led intercept, coast on a stale pose (`coast_s` 0.3), `fast_yaw_bias_deg` −8. `--floor`, 100 seeds vs fast-light: wz reversals −23 %, fast reversals −45 %, moving-ball hits 72 → 84, solo median 14.0 → 14.5 s (not faster), 1v1 goals 3.7 → 4.5 ([notes/brain.md](notes/brain.md)) | **sim** only |
| **`vs-human` preset** (dribble v3 + contested-ball model, hard ball-only role rules with a 2.5 s defend hold, dead reckoning through own-tag (1.5 s, app `stale_stop_s` 1.6) and ball (2 s, then hold goal centre) occlusion, wide-mouth goalie). Sim vs a modelled human: against a dominant human 15.8 vs 17.0 goals against /3 min, role switches −16 %, stale stops 17 → 5 s; against a human the robot can contest it scores less than dribble v3 (3.7 vs 6.2). Solo unchanged (99 %, 7.6 s). Needs an app restart ([notes/brain.md](notes/brain.md)) | **sim** only |
| New 7 in purple ball (radius 0.089 m); new rigid tag mount on blue (`eye_tag_height` 0.267 m, `eye_tag_offset_x` −0.02 m) | config from measurements; not run live |
| **Autoplay** (`ctl test autoplay minutes=N`, op `autoplay.start`, key A on Play): hands-off solo loop — attack the ball wherever it is, score, dribble the ball back to the centre with the brain's goto at 0.15 m/s, back off, attack the other goal; "please roll the ball out" banner (no time-out) for a ball in a net; per-cycle goals / time per goal / reset time / interventions in the summary. Brain unchanged | **device-free** + **sim** (Nov 25: 3-min run, 4 goals in 4 cycles, ball netted each time and rolled out by the scripted person) |
| Eye keeps the robot-tag crop search 1.5 s after a miss (`c52e180`): offline replay 32 → 56 %, 43 → 63 % tag-seen | **device-free** (replay) |

## Unverified

- Everything in the table above on the live camera and robot; the new tag mount's tag-seen fraction.
- Contact ladder 0.4 / 0.6 / 0.75 (only `contact_vx` 0.3 run); speed-dependent heading offset on the floor.
- Goalie reaction time and save counting vs a hand count; stage 3 (solo switching vs a human); interception.
- Bridge IMU lie-down on a real fault. Robot 2: not in hand ([PROVISION.md](robot/provision/PROVISION.md)).

## Next session

1. Restage mts (`robot/stage.sh mts`); confirm `FC_IMU_FILTER on` in `robot.log`; 5-min stand check.
2. `ctl ballcal` and `ctl rollcal --apply` with the purple ball; copy `ball_decel` into params.json.
3. Pushes: `fast-light` ×5 vs `athletic` ×5, logging goals, time, tag-seen fraction and `wz_reversals`.
4. Goalie ×5, saves counted by the app and by hand.
5. Solo role switching vs a human (stage 3).

## Blockers (owner)

Robot 2 and a replacement BNO085/board (staff reachable until Wed 25 evening); a second rigid tag ≥ 15 cm for red;
more light balls (≤ 150 g) matching the purple one; fence with 45° corners; Coda access Nov 26–29.
