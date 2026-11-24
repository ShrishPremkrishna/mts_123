# Pupper FC status

**As of Tue Nov 24, 2026.** Update this page when a capability changes. Plan and gates: [DEMO.md](DEMO.md).
Floor steps: [RUNBOOK.md](RUNBOOK.md). Contract: [SPEC.md](SPEC.md). App usage: [README.md](README.md). Pi side:
[robot/README.md](robot/README.md).

Labels: **robot** = observed on the physical robot; **device-free** = automated tests or a Pi/Docker run with no
motion; **sim** = our kinematic sim (`fc.sim_eval`) or MuJoCo.

## Nov 24 floor session (S1 first half, robot `mts`, stock profile, slow preset)

| Result | Value |
|---|---|
| Homing on the stand (`fcctl ready --confirm`) | 27 s |
| Software stand from the floor | works |
| Link E-STOP (Esc) on the stand | confirmed |
| First laptop-driven motion | works |
| Heading offset (tag yaw to robot forward) | 180 + 7° |
| Step gains | forward 0.91–1.01, lateral 0.79, turn 1.0 |
| Motion onset | 0.45–0.8 s |
| Dead-man stop | 0.32 s, 3.4 cm |
| goto ×3 | all within 7 cm |
| Goals at slow speed | 3 goals, 26–34 s each; still circling the ball. A/B the square-up switch `push_max_heading_deg` |
| Gentle lie-down (`down`) | verified twice: smooth, no drop |
| Overhead eye | 11 → 31 fps live. The stall was the full-frame search for the absent red robot (`59fbd9a`) |
| BNO085 gyro fault | came back at ~75 °C Pi temperature (after ~1 h with the stock stack up; clean at 41–72 °C). Raw-X pulses corrupted orientation, the walker leaned the robot a real 14°; the IMU health hold caught it |

**Pi heat (measured Nov 24, stack off, read-only).** With the stack stopped for 10 min, CPU ~95 % idle and the fan
at 7,900 rpm (state 3/4), the Pi still sat at 68–70 °C, falling ~0.2 °C/min. So the stack only adds a few degrees;
the baseline is already hot. Governor `ondemand`, mostly at 2.4 GHz. Running besides the stack: desktop GUI
(`pupper-gui` = pupper-rs, `labwc`, GPU rendering, ~6 % CPU), `hailort` with a Hailo-8 on PCIe (unused by fc).

## What works

| Capability | Verified by |
|---|---|
| Homing, software stand, link E-STOP, laptop-driven motion, goto, slow goals | **robot**, Nov 24 (above) |
| Gentle lie-down on `down`/`stop` (stock): legs lowered to the belly before motors go off | **robot** ×2 Nov 24; **sim** 14/14 |
| Lie-down abortable: the bridge stays up parked during the lowering; R3's list includes it via the overlay | **device-free** |
| IMU clean for 10 min standing, and for 10 min with motors off (at < 75 °C) | **robot**, Nov 23 (`imu-test`) |
| IMU health hold stops motion on a gyro fault | **robot**, Nov 24 (caught the 14° lean) |
| IMU health tells apart `stale` (no message 0.5 s) and `frozen` (messages arriving, quaternion+gyro+accel bit-identical 0.5 s; the RCA 2026-11-24 precursor); `gap_max_ms` would show a starved callback | **device-free** |
| IMU lie-down: bad IMU 0.75 s with the walker active → e-stop-style latch, STRICT switch to the lie-down controller, legs off after blend + hold; R3 at once without it ([RCA](notes/rca-20261124-shaking.md)) | **device-free** + ROS end-to-end in Docker (fake controller manager). Not yet run on the robot |
| Lean stack (default for `stock`): overlay launch skips `foxglove_bridge` and the onboard `camera_node`; `--camera on` keeps the camera, `--lean off` = stock node list. `fcctl status` / `fc.ctl pi` show lean, CPU governor/MHz, fan and the top 5 processes by CPU | **device-free** (overlay checked against the Pi's installed launch.py; tests incl. fcctl in Docker). Not yet started on the robot |
| ROS status probe: settling graph probe (`robot/rosq.py`) | **robot** bug, fix **device-free** |
| E-STOP over the link, latched until Ready; fallen-robot cutoff at 45° | **robot** (link E-STOP on stand) + **device-free** |
| Signed UDP link, bridge 50 Hz, 0.25 s dead-man, mux priority for the gamepad | **robot** (dead-man 0.32 s) + **device-free** |
| Laptop app: Play screen, `/ops` tests, match flow, run logs, presets with walker options (`_stack`) | **device-free** + **sim** |
| Brain (attack by running through the ball, goalie, auto roles, goto) | **sim**; slow attack and goto on the **robot** |
| Fast gait 0.75 m/s (walker cap raised to 0.8 by `fast-play-robot`) | **sim** (MuJoCo 0.73 m/s, 0/8 falls) |
| Overhead eye (tags, ball, calibration) at 31 fps on the live feed | **robot** Nov 24 |

## What is unverified

- Lean stack on the robot: its temperature effect and that nothing on the walking path changed (expect identical
  controller logs; check `status.heat.top`).
- Fast gait and ball contact at 0.4 / 0.6 / 0.75 on the floor; push with `push_max_heading_deg` on vs off.
- Goalie reaction time; real goalie runs do not yet count saves.
- Stage 3 (solo switching) and interception: sim only.
- Robot 2: not in hand. Second-robot plan: [robot/provision/PROVISION.md](robot/provision/PROVISION.md).

## Next

1. Cooling: start with the lean stack (default now), watch `fc.ctl pi blue`. Operator to decide on the OS-level
   changes (GUI off, Hailo off, fan curve, governor) and check the fan/heatsink seating; keep the Pi < 72 °C.
2. Push A/B: `push_max_heading_deg` 0 vs on, slow, ×5 each.
3. Fast mode (`fast-play-robot`), RUNBOOK 6/7 fast half, contact ladder.

## Blockers (owner)

- **Robot 2**: ask staff by Tue 24 noon; collect Wed 25 (last day staff are reachable).
- **Light balls** (≤ 150 g, ~18 cm), **fence** with 45° corners, **rigid 16 cm tags** ×2, goals at 1.0 m.
- **Coda access Nov 26–29** and whether the field may stay up.
