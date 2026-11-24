# Pupper FC status

**As of Mon Nov 23, 2026 (evening).** Update this page when a capability changes. Plan and gates: [DEMO.md](DEMO.md).
Floor steps: [RUNBOOK.md](RUNBOOK.md). Contract: [SPEC.md](SPEC.md). App usage: [README.md](README.md). Pi side:
[robot/README.md](robot/README.md).

Labels: **robot** = observed on the physical robot; **device-free** = automated tests or a Pi/Docker run with no
motion; **sim** = our kinematic sim (`fc.sim_eval`) or MuJoCo. Nothing below is laptop-driven motion on a real robot.

## What works

| Capability | Verified by |
|---|---|
| Homing on the stand: 17 s (`fcctl ready --confirm`, stock profile, robot `mts`) | **robot**, Nov 23–24 first live run |
| Software stand: 5 s | **robot**, same run |
| IMU clean for 10 min standing, and for 10 min with motors off | **robot**, same run (`imu-test`) |
| ROS status probe (`fcctl status` / `ready` verify): the fixed 1.5 s spin missed the mux; now a settling graph probe (`robot/rosq.py`) | **robot** bug, fix **device-free** (12/12 in a loaded repro on mts) |
| Gentle lie-down on `down`/`stop` (stock): legs lowered to the belly before motors go off. Replaces the drop seen on the robot | **sim** only (MuJoCo, 14/14 cases, 0.0 cm drop after motors off) |
| Lie-down abortable: the bridge stays up parked during the lowering (link E-STOP / fallen cutoff deactivate the lie-down controller), R3's list includes it via the overlay (Nov 23 pre-flight fix) | **device-free** (fcctl in Docker, bridge park + e-stop with real rclpy in Docker, overlay on the mts config copy) |
| E-STOP over the link (Esc), latched until Ready; fallen-robot cutoff at 45° | **device-free** (bridge tests; ROS in Docker: e-stop 1.2 ms, controller switch 7.3 ms) |
| Signed UDP link, bridge 50 Hz, 0.25 s dead-man, mux priority for the gamepad | **device-free** (tests; loopback on mts, LAN RTT 9.7 ms, no motion) |
| Laptop app: Play screen, `/ops` tests, match flow, run logs, presets with walker options (`_stack`) | **device-free** + **sim** (`pytest fc/tests fc/robot/tests`: 314 pass, 33 skipped on the Mac, Nov 23) |
| Brain (attack by running through the ball, goalie, auto roles, goto) | **sim**: fast-play preset, errors on, 1.0 m goals, 0.3 s lag: solo goals ≥ 98 % (median ~10 s), goalie saves 88 %, 1v1 ~4 goals / 3 min |
| Solo role switching vs a human (defend while the ball threatens our goal, else attack), attacker interception, faster return to goal (`bc8e68a`) | **sim** (scripted human, `return`/`rolling_ball` scenarios in `sim_eval`) |
| Fast gait 0.75 m/s (walker cap raised to 0.8 by `fast-play-robot`) | **sim** (MuJoCo 0.73 m/s, 0/8 falls nominal and harsh). Old stack: 1.0 m/s for 3 s on Nov 16, not via fc |
| Overhead eye (tags, ball, calibration) | **device-free** on rendered frames (0.1 cm median error). Not yet run by fc on the real phone feed |

## What is unverified

- **Any laptop-driven motion.** Latency, step gains and heading bias (G0) are unmeasured; the sim assumes 0.3 s total lag.
- E-STOP, fallen cutoff and gentle lie-down on the real robot (the old `down` **dropped** the robot on Nov 23–24).
- Fast gait and ball contact at 0.4 / 0.6 / 0.75 on the floor. The current heavy ball (0.43 kg) stalled and tipped the robot in MuJoCo; 0.15 kg was clean.
- Goalie reaction time ("ball moves → robot moves"); real goalie runs do not yet count saves.
- Stage 3 (solo switching) and interception: sim only; no floor test yet.
- Robot 2: not in hand. Second-robot plan: [robot/provision/PROVISION.md](robot/provision/PROVISION.md).

## Next floor session: S1, Tue Nov 24

Preset **slow**, block A ≤ 10 min powered, then cool down, then block B. Record wide phone + screen for every block.

1. RUNBOOK 1–4: Ready on stand and floor, drive 1 min, five stops, re-arm, deadman. Soft mat for the drops.
2. RUNBOOK 5: step fwd 0.1 / 0.2 / 0.3, turn ±0.5. **Write down `latency_s`.**
3. Not yet in RUNBOOK: **goto ×3** (centre, two far corners; pass < 8 cm, tag read at the far side).
4. RUNBOOK 7 slow half: push ×5. Film the first goal.
5. Block B, `fast-play-robot`: RUNBOOK 6 (step 0.4 and 0.75 ×3; add turn ±1.5 and stop distance), RUNBOOK 7 fast
   half, then a contact ladder `contact_vx` 0.4 then 0.6, ×3 each.
6. Not yet in RUNBOOK: **goalie ×5**, slow hand-rolled shots; read reaction time from the log.
7. RUNBOOK 8: `down`. First real check of the gentle lie-down; hands at its sides; Esc or R3 aborts.

The batch [sessions/S1.batch](sessions/S1.batch) follows the Nov 23 pre-flight review: link E-STOP first tested on
the stand, `sleep 10` before `check`, gotos at 0.3 / 1.0 in block B, a 0.8 s open-loop 0.75 step before closed-loop
fast motion, slow push ×2 (25 s timeout), fast push only with the light ball, `k_goalie` set by hand from the
measured latency, a goalie start pose, and `pi blue` (Pi temperature) before each pause. Not restaged yet: mts was
offline (run `stage.sh mts` and the SHA check first).

After: copy measured lag, gains and bias into the `sim_*` params and rerun `sim_eval`. Pass/fail per gate: [DEMO.md](DEMO.md#stages-and-gates).

## Blockers (owner)

- **Robot 2**: ask staff by Tue 24 noon; collect Wed 25 (last day staff are reachable).
- **Light balls** (≤ 150 g, ~18 cm), **fence** with 45° corners, **rigid 16 cm tags** ×2, goals at 1.0 m.
- **Coda access Nov 26–29** and whether the field may stay up.
