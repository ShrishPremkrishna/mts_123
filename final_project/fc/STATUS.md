# Pupper FC status

**As of Tue Nov 24, 2026 (after the S1 floor session).** Update this page when a capability changes. Plan and
gates: [DEMO.md](DEMO.md). Floor steps: [RUNBOOK.md](RUNBOOK.md). Contract: [SPEC.md](SPEC.md). App usage:
[README.md](README.md). Pi side: [robot/README.md](robot/README.md).

Labels: **robot** = observed on the physical robot; **device-free** = automated tests or a Pi/Docker run with no
motion; **sim** = our kinematic sim (`fc.sim_eval`) or MuJoCo. Run IDs are `fc/runs/20261124T<id>`.

## Nov 24 floor session (robot `mts`, stock profile)

| Result | Value (robot) |
|---|---|
| Homing on the stand / software stand from the floor | 24–27 s / works |
| Gentle lie-down (`down`) | verified repeatedly: smooth, no drop |
| Link E-STOP (Esc) on the stand | confirmed |
| Dead-man stop | 0.32 s, 3.4 cm (`043546Z`) |
| goto ×3 | 5.6–6.9 cm (`043608Z`–`043624Z`); one later goto with a heading target missed by 14 cm (`071403Z`) |
| Step gains, slow | forward 0.91–1.01, lateral 0.79, turn 1.0; onset 0.3–0.8 s; `latency_s` ≤ 0.14 |
| Heading offset | 180 + 7° (slow gait). Fast gait walks ~8° right of slow (`073515Z` bias −7.9°): offset may need to depend on speed |
| Fast gait (speed cap 0.8) | stable; 0.47 m in the 0.75 burst, gain 0.88 (`073515Z`); 0 falls in 8 fast pushes |
| Slow goals | 5 of 13 attempts, 26–44 s. With today's push tuning (`push_max_heading_deg` 40, `push_lat_tol` 0.12, `push_turn_scale` 0.5, `k_turn` 1.0, `contact_vx` 0.2) 2/2: 27.4 s (5.5 m path, 3 turn reversals) and 42.6 s |
| Fast goals (`fast-light`, light ball) | 3 of 7: 15.9, 21.7, 16.4 s; 4 timeouts. Before the preset, one real own goal (`073530Z`: ball clipped while circling, `clear_margin` 0.06) → `clear_margin` 0.12, `contact_slow_m` 0.5 |
| Goalie | 1 run, 0 conceded (`071425Z`); saves are not counted in camera mode |
| Overhead eye | 11 → 31–35 fps live (absent-robot full-frame search off the hot path, `59fbd9a`) |
| Link | LAN UDP ~8 ms vs Tailscale p95 up to 1.1 s (`073726Z`). `robots.json` `udp_host` 10.5.6.112 (DHCP may change) |

**Limiting factor now: seeing the robot's own tag.** The small paper tag (~8 cm, ~30 px at 1920) is seen in 18 % of
frames in the far half of the field vs 61 % near. Scoring fast runs saw the tag in 95–100 % of frames; every timeout
saw it 46–89 %. Fix so far: `c52e180` keeps the crop search 1.5 s after a miss (offline replay 32 → 56 %, 43 → 63 %;
not yet run live). Other losses: the phone feed froze once (`074158Z`), a person walked on the field once (`074019Z`).

**IMU.** The BNO085 intermittently shows gyro-X pulses/bias and false tilt; the walker then leans or shakes. It
seized once while standing ([RCA](notes/rca-20261124-shaking.md)). The dyncal hypothesis is refuted; the battery
monitor shares `/dev/i2c-1` but is judged unlikely ([note](notes/imu-filter.md)). Guards and crutch are below.

**Heat.** The Pi idled at 68–70 °C because of the Hailo-8 and the desktop GUI. After operator-approved OS changes
(`pciex1` off, GUI off, earlier fan curve; undo in [robot/README.md](robot/README.md)) idle is ~57 °C (robot,
operator reading). Lean stack load 0.8 vs 3–5.

## What works

| Capability | Verified by |
|---|---|
| Homing, software stand, gentle lie-down, link E-STOP, dead-man, goto, laptop-driven motion | **robot** Nov 24 |
| Slow push goals (26–44 s) and fast `fast-light` goals (16–22 s) | **robot** Nov 24 (3/7 fast, see above) |
| Fast gait at speed cap 0.8, no falls | **robot** Nov 24; **sim** (MuJoCo 0.73 m/s, 0/8 falls) |
| IMU filter driver (`FC_IMU_FILTER=1`: accel-corrected tilt, gyro-X de-spike), v2 .so sha `19d60fad…` on mts | **robot**: 5-min stand check tilt 2.0–2.8°, 3 fast floor runs with no IMU incident. v1 mis-flagged normal near-zero gyro repeats as freezes. Stock profile now sets it by default (`51c16f4`, needs restage) |
| Bridge auto lie-down + latch on a sustained bad IMU (tilt / stale / frozen) | **device-free** + ROS end-to-end in Docker. Not yet triggered on the robot |
| IMU health hold stops motion on a gyro fault | **robot** (caught a 14° lean) |
| Lean stack (no `camera_node` / `foxglove_bridge`), heat in `status` | **robot** Nov 24 |
| Overhead eye (tags, ball, calibration) at 31–35 fps | **robot** Nov 24 |
| Signed UDP link, 50 Hz bridge, 0.25 s dead-man, fallen cutoff 45°, E-STOP latched until Ready | **robot** (stops) + **device-free** |
| Laptop app: Play screen, `/ops` tests, match flow, run logs, presets with `_stack` | **device-free** + **sim**, used on the **robot** |
| Brain: goalie, solo role switching vs a human, interception | **sim** only |
| Field length/width from the corner-tag calibration (`field_from_camera`, default on) into one live Field for brain, referee, match flow, UI; logged in each run's params.json, shown in `ctl status`. Goal mouth 1.22 m stays in field.json | **device-free** (unit test). Not yet on the robot |
| Ball colour calibration: `python -m fc.ctl ballcal [X Y]` / `--show`, Shift-click on the `/ops` camera; applied live, saved to `config/ball_color.json` (loaded at eye start). Ball radius: field.json only (measure the new ball) | **device-free** + synthetic camera. Not yet on the real camera |

## What is unverified

- Eye fix `c52e180` (1.5 s crop search) on the live feed.
- Fast pushes at a repeatable rate; contact ladder 0.4 / 0.6 / 0.75 (only `contact_vx` 0.3 run).
- Speed-dependent heading offset for the fast gait.
- Goalie: reaction time and save counting (not implemented in camera mode).
- Stage 3 (solo switching vs a human) and interception on the floor.
- Bridge IMU lie-down on a real fault; the filter's accel stream during a fault.
- Robot 2: not in hand. Plan: [robot/provision/PROVISION.md](robot/provision/PROVISION.md).

## Next session

1. Bigger rigid robot tag (≥ 15 cm, flat); measure its height and offset into `eye_tag_height` / `eye_tag_offset_x`.
2. Restage the Pi (`robot/stage.sh mts`); confirm `FC_IMU_FILTER on` in `robot.log` at start.
3. Fast pushes ×5 with `fast-light`, logging tag-seen fraction per run.
4. Goalie ×5 (count saves by hand until the app does).
5. Solo role switching vs a human (stage 3).

## Blockers (owner)

- **Robot 2**: still not available. Ask staff (reachable until Wed 25 evening).
- **IMU / control board**: ask staff for a replacement BNO085 or board.
- **Rigid tags ≥ 15 cm** ×2, **light balls** (≤ 150 g), **fence** with 45° corners, goals at 1.0 m.
- **Coda access Nov 26–29** and whether the field may stay up.
