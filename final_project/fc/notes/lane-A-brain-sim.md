# Lane A: brain + sim (2026-11-23)

## Built

- **`brain.py`** (stdlib only):
  - **Attack:** a curved approach behind the ball. On the shot line it aims past the ball and keeps walking through it. From the wrong side it orbits the ball. Next to a wall it rotates the push direction until the stage behind the ball is reachable.
  - **Headbutt:** line up, charge at a locked point on the line, back off. Frozen: don't tune it on the floor.
  - **Goalie:** intercept y clamped to the goal mouth. `sideways` or `facing` style. Clears slow balls in front of goal.
  - **Auto roles:** filtered travel-time comparison with hysteresis and a hold time. A defender attacks when it is not slower, so both robots never defend at once.
  - **Goto** (`role_mode: "goto"`, per-call `goto_x`, `goto_y`, optional `goto_yaw` in rad): uses the same `_goto`, prediction and shaping as play, and works without a ball. `dbg["done"]` is true once the robot is within `goto_tol_m` and `goto_yaw_tol_deg`.
  - **Robustness:** own-pose prediction through latency; stale pose → immediate stop; ball lost → `hold`/`search`; opponent repulsion; stuck → brief reverse; clamps and rate limits.
- **Public helpers for the app:**
  - `brain.shape(cmd, prev, dt, params, accel_vx=None) -> Cmd`: the one output shaper (clamp, then rate-limit from `prev`). An all-zero command passes through at once.
  - `Brain.sent(cmd)`: tells the brain what was actually sent when the app overrode it (stale stop, teleop), so its ramp and pose prediction restart from there.
- **`sim.py`:** deterministic 100 Hz kinematics. All measured errors are `sim_*` knobs (`sim_errors: 0` turns them off). Walls bounce except at the goal mouths. Goals pause, then reset to kickoff. A "trapped" referee nudges a wall ball that stays within 10 cm for `sim_trap_s` (was 5 cm, so a ball rattling in a corner deadlocked).
- **`python -m fc.sim_eval [--off] [--set k=v ...]`** runs in about 15 s. `--set` tries values without editing `params.json`.

## Senior-review fixes (2026-11-23)

- **`yaw_learn` off by default.** Rate is 0.02 when enabled. It learns only from clean 0.6 s windows: forward, `|wz| < 0.25`, `|vy| < 0.03`, no ball or robot nearby, not pushing/charging/unsticking, and it moved at least half the commanded distance. The total correction is clamped to ±`yaw_learn_max_deg` (20). `reset(forget_yaw=True)` clears it. Measured over 6 × 3 min 1v1:
  - No tag bias: max residual 1.3° (the old learner injected up to 19–44°).
  - Learning is slow: a constant 15° bias only reaches a median 9.9° residual after 3 min, and the sim's facing-dependent bias is not helped (p90 14.6° vs 11.0° off).
  - **A constant bias is better fixed by calibrating the eye's yaw offset from the `step` test.**
- **Orbit side** is held for `orbit_hold_s` (2 s). Waypoints up to `orbit_slack` (0.10 m) outside the margin are clamped in instead of rejected.
- **Calmer steering:** `k_turn` 1.5, `accel_wz` 2.0.

## sim_eval (40 seeds, 8×3 min; `--set accel_wz=2.0 yaw_learn=0` until params.json changes)

| | before, ON | after, ON | after, OFF |
|---|---|---|---|
| solo push ≤40 s | 95%, median 15 s | **100%**, median 17 s | 100%, 12 s |
| solo headbutt | 50% | 25% | 58% |
| goalie, sideways / facing | 98% / 98% | **98% / 98%** | 100% |
| 1v1 goals/3 min, blue–red | 2.0–2.8 | **0.8–1.8** | 3.1–1.9 |
| deadlocks / traps | 0 / 8 | **0** / 15 | 0 / 8 |
| wz reversals/min | 20.0 | 13.7 | 12.1 |
| fast wz reversals (<1 s apart)/min | 3.6 | **0.9** | 1.5 |
| orbit side flips/min | 10.9 | **2.5** | 3.6 |

Why 1v1 goals and headbutt fell (A/B, 16 matches): the old fast learner was hiding the sim's ±10° tag bias.
- With `yaw_learn` off in both brains, the new brain scores about the same as the old one and has fewer stalls (2 vs 8 before the trap fix).
- `k_turn` 1.5 costs about 25% of 1v1 goals and most of headbutt's line-ups. With the old gains, the new brain's headbutt gets 36/40 with errors OFF.

Rerun after the floor `step` test gives real numbers for bias and lag.

## Weaknesses

- **Corner balls** can't be extracted by the brain (both robots push into the corner); the sim's referee nudges them.
- **Goalie is easy:** a 0.36 m body in a 0.61 m mouth.
- **Dropout bursts** cause brief `pose_stale` stops.
- **Contact is idealised:** circles, no spin.

## Floor-tuning priorities

1. Camera heading calibration (`step` test). Then `yaw_learn` only if the remaining bias is > 5°.
2. `max_vx`, `push_vx`, `k_turn`, `push_turn_scale` (speed versus wobble).
3. `act_lag_s` (match the measured lag).
4. `k_curve`, `push_lat_tol`, `behind_cone_deg`.
5. `role_hold_s`, `role_hyst_s`, `role_tau_s`.
6. `goalie_dist`, `clear_depth`.
7. `pose_stale_s`, `field_margin`.
