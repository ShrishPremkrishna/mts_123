# Lane A: brain + sim (2026-11-23)

## Built

- **`brain.py`** (stdlib only):
  - **Attack:** a curved approach behind the ball. On the shot line it aims past the ball and keeps walking through it. From the wrong side it orbits the ball. Next to a wall it rotates the push direction until the stage behind the ball is reachable.
  - **Headbutt:** line up, charge at a locked point on the line, back off.
  - **Goalie:** intercept y clamped to the goal mouth. `sideways` or `facing` style. Clears slow balls in front of goal.
  - **Auto roles:** filtered travel-time comparison with hysteresis and a hold time. A defender attacks when it is not slower, so both robots never defend at once.
  - **Robustness:** own-pose prediction through latency; stale pose → immediate stop; ball lost → `hold`/`search`; opponent repulsion; stuck → brief reverse; clamps and rate limits; `yaw_learn` learns the tag-heading bias from straight walking.
- **`sim.py`:** deterministic 100 Hz kinematics. All measured errors are `sim_*` knobs (`sim_errors: 0` turns them off). Walls bounce except at the goal mouths. Goals pause, then reset to kickoff. A "trapped" referee nudges wall-pinned balls. Works with `app.py` unchanged.
- **`python -m fc.sim_eval [--off]`** runs in about 13 s. 22 tests pass in 0.04 s.

## sim_eval (40 seeds, 8×3 min)

| | errors ON | OFF |
|---|---|---|
| solo push ≤40 s | **95%**, median 15 s | 100%, 11 s |
| solo headbutt | 50%, 27 s | 82%, 20 s |
| goalie, sideways / facing (empty net: 40/40 score) | **98% / 98%** | 100% |
| 1v1 goals/3 min, blue–red | **2.0–2.8** | 3.5–2.5 |
| role flips/min | **4.6** | 5.0 |
| deadlocks (ball still 10 s) | **0** | 0 |
| wz reversals/min | 20 | 16 |

**Constant 15° heading-bias stress:** push 38/40 with `yaw_learn` on, 30/40 with it off. Without it, 1v1 goes lopsided because the bias is mirrored between the teams.

## Weaknesses

- **Headbutt** at 0.3 m/s is a slow push. It only helps if the gait can lunge.
- **Corner balls** can't be extracted by the brain; the trapped/out-of-bounds rule is still open.
- **Goalie is easy:** a 0.36 m body in a 0.61 m mouth.
- **Dropout bursts** cause brief `pose_stale` stops.
- **Contact is idealised:** circles, no spin.

## Floor-tuning priorities

1. `yaw_learn` / camera heading calibration (the largest error).
2. `max_vx`, `push_vx`, `k_turn`, `push_turn_scale` (speed versus wobble).
3. `act_lag_s` (match the measured lag).
4. `k_curve`, `push_lat_tol`, `behind_cone_deg`.
5. `role_hold_s`, `role_hyst_s`, `role_tau_s`.
6. `goalie_dist`, `clear_depth`.
7. `pose_stale_s`, `field_margin`.

`params.json` gained eight keys. The rest come from `DEFAULTS`/`SIM_DEFAULTS`, which the app already merges. No contract changes.
