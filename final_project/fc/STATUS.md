# Pupper FC status

**As of Sat Nov 28, 2026 (floor session Nov 27 evening, robot 2 bring-up Nov 28).** Update this page when a capability changes.
Plan: [DEMO.md](DEMO.md). Next floor session: [RUNBOOK.md](RUNBOOK.md). Contract: [SPEC.md](SPEC.md). App usage:
[README.md](README.md). Pi side: [robot/README.md](robot/README.md). Robot 2: [PROVISION.md](robot/provision/PROVISION.md).
Design notes and sim tables: [notes/](notes/).

Labels: **robot** = observed on the physical robot; **device-free** = automated tests, replays of logged runs or a
Pi/Docker run with no motion; **sim** = our kinematic sim (`fc.sim_eval`) or MuJoCo. Run IDs are `fc/runs/<id>` (UTC, so
Nov 27 evening runs are named `20261128T…`).

## Floor history — robot (`mts`, purple ball, fenced field ~3.38 × 2.62 m from camera)

| Day | Result |
|---|---|
| **Nov 27 evening** (preset `play`, walker cap 1.5 staged) | **Speed ladder** (step fwd, `wall_brake` 0.5): `max_vx` 1.0 → gain 0.77 (~0.77 m/s), 1.2 → gain 0.84 (~1.0 m/s), both clean; **1.4 fell** (`065952Z-step`: heading 6° → 50° in 0.2 s as the command passed ~1.2 m/s on the accel ramp; the fallen cutoff turned the legs off). Steady veer ~8° right. **Solo dribble** at 1.0: 3/3 open balls scored (11.7, 8.6, 6.8 s); fence/corner balls deferred (bent fence). **Goalie** vs Shrish: 6/9 on-target saves, robot visible 55 % (`071036Z`); after the keeper lateral fix 0eafdc5 **14/18 (78 %)**, visible 99 %, moved 2× more (`072833Z`). **Show flow** (warm-up, walk to spot, 3-2-1, 90 s vs Shrish): **blue 5–1**, six goals each with celebration + reset, visible 93 % (`071744Z-match`); Shrish: it "seemed smarter", giving space and blocking. `down` gentle. Link RTT p95 ~75–80 ms (CS.Test.PSK Wi-Fi). Calibration dropped twice (`floor_reference_drift`, corner tag on a flexing fence) and the phone camera once: each needed an app restart |
| Nov 27 day | False ~10° IMU tilt = accel **+3.05 m/s² bias on chip X**, a sensor fault ([RCA](notes/rca-20261127-tilt.md)); fixed with driver **v4** + `imu-level` (0.26° on the stand). Ball colour and blue board recalibrated. Corner ball: walked into the fence and **tipped** (before the fence margin). vs a human: defence weak, slow (~0.3 m/s) |
| Nov 25 | `ballcal`, `rollcal` (`ball_decel` 0.149), field from camera, mouths 1.22 m. **Dribble v3**: 6/6 solo from anywhere (8.6–20.3 s). vs a human 0–3. An I2C hang froze the IMU; touchscreen overlays disabled ([robot/README.md](robot/README.md#i2c-bus-change-on-mts-2026-11-25-operator-approved)) |
| Nov 24 | Homing 24–27 s, link E-STOP, dead-man 0.32 s, goto ×3 within 7 cm, `latency_s` ≤ 0.14, 0 falls in 8 pushes at cap 0.8. Campus Wi-Fi RTT p95 ~200–250 ms idle |

## What works

| Capability | Verified by |
|---|---|
| Homing, stand, gentle lie-down, link E-STOP, dead-man, goto | **robot** Nov 24–27 |
| Speed: clean up to `max_vx` 1.2 (~1.0 m/s); **play at 1.0–1.1** (1.4 fell on the ramp). Walker cap 1.5 via `_stack` | **robot** Nov 27; sim 0/24 falls at 1.4 did not predict the fall ([fast-gait §6](notes/fast-gait.md)) |
| Dribble v3 on open balls at `max_vx` 1.0 + `wall_brake` 0.5 | **robot** Nov 25, Nov 27 (3/3) |
| Keeper (shuffle mid-mouth, shift toward the ball, lateral response 0eafdc5) | **robot** Nov 27: 78 % on-target saves vs Shrish |
| Show flow (W warm-up, walk to spot, K 3-2-1, 90 s, spin celebration, reset), one robot vs a human | **robot** Nov 27 (5–1) + sim |
| Signed UDP link, 50 Hz bridge, 0.25 s dead-man, fallen cutoff 45° (fired Nov 27 ×2), E-STOP latched until Ready | **robot** + **device-free** |
| IMU health hold; bridge auto lie-down on a sustained bad IMU; IMU driver v4 + `fcctl.sh imu-level` on mts | **robot** Nov 25/27; mts chip still degraded ([RCA](notes/rca-20261127-tilt.md)) |
| Field from corner tags, `ctl ballcal`, `rollcal`, `robotcal`; Play screen, `/ops`, run logs, presets, `fc.ctl` | **robot** + **device-free** |

## Built since the Nov 27 evening session — not on the floor yet

| Change | Verified by |
|---|---|
| **Self-healing calibration** (0571987, 4fa7ab5): saved `config/eye_cal.json`, per-tag monitor (1–2 moved tags ignored, ≥ 3 → pose-only re-solve), `CAL SUSPECT` / CALIBRATION UNVERIFIED banner, op `eye.recalibrate` (idle only) | **device-free**: replays of `071036Z-goalie`, `042513Z-match`, `071744Z-match` keep a pose on 100 % of frames; tests |
| **Robustness fixes** (dd1e6f8, 701a414, cb5058e): halted/link-dead robots out of the Coach; kickoff doesn't wait for absent robots; re-arm and Space pause a match (score kept); no phantom celebration; field size frozen during runs; ssh keepalives; app starts with preset `play`; `ready --home` refuses while the stack runs; link re-resolves a robot's address | **device-free** (tests) + **sim** |
| Keeper pre-shot hedge toward its slow side (10e77b6, `gk_asym_m` 0.15). Look-ahead study: prediction is not the limit (crossing error 1–2 cm with < 0.5 s to go); the walker's backward cap 0.3 m/s is | **sim** (gallery saves 90.7 → 92.3 %) + device-free log study ([brain notes](notes/brain.md)) |
| Fence margin + bank shots (`wall_margin_m` 0.28) | **sim** (would-tip 228 → 2); wall/corner balls not floor-tested |
| 1v1 Coach (possession, shadow/keep goal, anti-ram) | **sim** + **device-free**; needs two robots |
| Long shot; board blob tracking; gyro heading between tags; autoplay; robot POV feed | **sim** / **device-free**; not checked on the floor |

## Robot 2 (`red`, tag 11)

Cloned from mts on Nov 28 (live file copy onto a USB drive + offline identity reset, [PROVISION.md](robot/provision/PROVISION.md#path-a-clone-mts-used-nov-28)).
**device-free**: boots (Pi 5), `ssh pupper-red`, LAN 10.5.6.37, Tailscale node `pupper-red`; `provision.sh --check`: 20 pass,
1 fail (no paired DualSense); all Pi OS items ok incl. `imu_v4`; I2C shows IMU 0x4B + ADC 0x48. Not yet: DualSense,
`imu-level`, tag board + `robotcal red`, step tests, any motion.

## Open threads

- **Fence handling**: wall/corner balls untested on the floor; the fence in this room is bent.
- **Camera reconnect without an app restart** (the phone camera dropped once; hypothesis: capture in a child process).
- **Keeper speed**: the walker's backward cap 0.3 m/s limits saves; the fix is robot-side.
- **Per-robot eye offsets**: `eye_yaw_offset_deg` and tag height are global ([PROVISION.md](robot/provision/PROVISION.md#per-robot-params-how-the-app-should-load-them-not-wired-yet)).
- **Two-robot floor test**: anti-ram at `max_vx` ≤ 1.0, then the 1v1.
- **Reusable clone script** in `fc/robot/provision/` (the Nov 28 one is not in the repo).

## Blockers (owner)

Robot 2 bring-up on the floor; replacement IMU/control board for mts; red rigid tag ≥ 15 cm; more balls matching the
purple one; a straight fence.
