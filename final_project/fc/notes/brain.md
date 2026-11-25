# Brain and sim design notes (from 2026-11-23)

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
- **`python -m fc.sim_eval [--off] [--preset base NAME ...] [--set k=v ...] [--field k=v ...]`** runs in about 15 s per column. `--preset` prints one column per preset (`base` = params.json); `--set` tries values without editing `params.json`; `--field` tries geometry (e.g. a wider goal).

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

## Fast play (2026-11-23)

**Sim walker model** (`sim_*`, from [fast-gait.md](fast-gait.md)): vx gain 0.92 up to cmd 0.3, trot saturating to 0.36 m/s at 0.49, fast gait from cmd 0.5 (0.62 / 0.73 / 0.86 / 1.14 at 0.6 / 0.75 / 1.0 / 1.5), backward capped at 0.3, wz clipped at 2.0. Fast gait adds ±0.5 rad/s yaw wobble; each gait switch adds a short random yaw kick. The existing 0.15 s lag already gives the measured stop (0.10–0.11 m from 0.73 m/s). `sim_speed_curve: 0` restores plain gains. The sim also logs first touches of the ball (`sim.contacts`, robot speed) and robot–robot touches (`sim.bumps`).

**Brain knobs** (defaults leave today's 0.3 m/s play unchanged):
- `gait_band`, `gait_lo` 0.45, `gait_hi` 0.6, `gait_hold_s`: forward commands never sit in (0.45, 0.6); the output steps across the band.
- `contact_vx`, `contact_slow_m`: closing speed toward the ball ramps from `max_vx` down to `contact_vx` inside the slow zone.
- `decel_vx`, `decel_wz`: separate rates toward zero (quick braking; damped turn reversals).
- `goalie_vmax` (guard), `return_vx` (walk back to goal).

**Preset `config/presets/fast-play.json`** (40 seeds, 16 × 3 min, errors ON):

| | base | fast-play |
|---|---|---|
| solo push, median | 100%, 16.8 s | 100%, **11.9 s** |
| goalie saves (sideways / facing) | 98% / 98% | 100% / 100% |
| 1v1 goals/3 min, total | 2.4 | 2.3 |
| deadlocks | 0 | 0 |
| fast wz reversals/min, side flips/min | 1.0, 2.3 | 1.8, 3.7 |
| contact speed mean / p90, share > 0.45 | 0.17 / 0.27, 0% | **0.29 / 0.40, 6%** |
| robot bumps/3 min, closing mean / max | 14, 0.18 / 0.48 | 25, 0.36 / 0.84 |
| 1v1 mean robot speed | 0.16 | 0.23 |
| gait switches/min, time in band | 0, 0% | 15, 0% |

Missed targets and why:
- **Solo ≤ 10 s:** fast-play reaches 11.9 s (9.0 s with errors OFF). Dribbling is capped by contact speed (about 0.32 m/s, continuous contact), and approach time is limited by turning. `accel_wz` 4 without `decel_wz` gives about 10.5 s but about 2.3 fast wz reversals/min, which breaks the jitter limit.
- **1v1 ≥ 4 goals:** goals do not rise with speed. Faster defenders get back to goal sooner while the dribble stays at contact speed (`max_vx` 0.3 with the other fast knobs scored 3.4). A guard in position leaves 0.125 m on each side of a 0.18 m ball. Weakening the goalie or a wider goal (`--field goal_width=0.8`) are the levers, not brain speed.
- **Tried with no gain (20–24 matches each):** role hysteresis 0.3–0.7; a symmetric role hand-off (fewer robots attacking at once also meant fewer goals: scrambles create the chances); aiming at the side away from the goalie; slowing near the opponent; changing `k_curve`, `push_ahead`, `clear_depth`, `goalie_dist`.

**Not modelled:** the robot slowing or falling while pushing a heavy ball (the study saw 0.75 → 0.25 m/s and falls of 1/3 at 0.6), and robot–robot impacts at a 0.84 m/s closing speed. Floor-test both before using the preset in a match.

## Solo roles, interception, return to goal (2026-11-23)

For the "Switch" stage: one robot against a human. All knobs are in `brain.DEFAULTS`; `params.json` is unchanged. Turning `solo_roles`, `intercept_max_s`, `return_detour`, `return_block` and `return_back_vx` all to 0 reproduces the previous brain exactly (core sim_eval rows checked against HEAD).

- **Solo roles** (`solo_roles` 1, `solo_hold_s` 1.0): in auto mode with no opponent seen, the robot defends while the ball threatens its goal. A threat is either:
  - moving toward goal faster than `solo_def_vmin` (0.15 m/s), reaching the goal line within `solo_def_horizon_s` (3 s) and crossing within `solo_def_margin` (0.3 m) of the mouth; or
  - moving (not away from goal) inside our `solo_third`.

  Otherwise it attacks. To stay defending, a slower ball (half of `solo_def_vmin`) is enough. `solo_zone` (off) makes a defender leave its goal only for a ball in its home zone.
- **Ball model** `ball_decel` (0.45 m/s², same as the sim): straight line with constant deceleration. `_arrival` gives the time and y where the ball crosses a line; `_ball_at` gives its position at a time. Measure the real ball's value on the floor.
- **Interception** (`intercept_max_s` 2, `intercept_vmin` 0.15, `intercept_near_m` 0.4): for a moving ball, the attacker aims where the ball will be when it arrives (fixed point of `_cost`). It is skipped while pushing or within 0.4 m of the ball: chasing a dribbled ball's predicted position broke the push (moving-ball goals at 0.7 s lag went 29 → 15 of 40 before this gate).
- **Return to goal:**
  - Not slowed to contact speed.
  - `return_detour` steps around a ball that is not coming at goal. It is skipped for incoming balls because touching those is a save; with the detour applied to them, goalie saves fell 88 → 82%.
  - `return_back_vx` 0.25: while turning round, the robot backs toward its goal. Time until it moves toward goal: 1.57 → 0.90 s.
  - `return_block`: if the guard line cannot be reached before the ball, step into the ball's path. It rarely fires (about 1.5 s per 12 min), because human shots pass the robot within about 0.5 s.
- **Sim:** `kick`, `shoot`, `aim`, `schedule` (scripted human) and scenarios `return` and `rolling_ball` (also usable from the UI scenario box).
- **sim_eval:**
  - new row groups `human`, `return`, `moving`;
  - `--scen` picks row groups;
  - `--lag L ...` adds a column per total lag (camera L/2 + walker L/2, `act_lag_s` L/2);
  - `human_solo` honours `role_mode` (defend = keeper reference).

Results, fast-play, errors ON. Columns: before / after at 0.3 s total lag, then before / after at 0.7 s. Runs: 60 seeds; human = 10 × 3 min.

| | 0.3 before | 0.3 after | 0.7 before | 0.7 after |
|---|---|---|---|---|
| human: robot goals / conceded per 3 min | 1.5 / 11.9 | 1.9 / 10.9 | 0.9 / 10.9 | 0.8 / 10.6 |
| human: saves of shots | 28% | 34% | 34% | 36% |
| human: right role, shots / rolls | 0 / 100% | 98 / 98% | 0 / 100% | 96 / 98% |
| human: shot → defend / → moving back (s) | – / 1.57 | 0.23 / 0.90 | – / 1.90 | 0.43 / 1.40 |
| human: role switches /min, fast wz rev /min | 0, 1.3 | 11.8, 2.2 | 0, 1.6 | 11.2, 2.3 |
| return: on line (s after shot), ball's flight (s) | 5.8, 2.2 | 5.4, 2.2 | 5.7, 2.2 | 5.5, 2.2 |
| return (auto): saves | 66% | 66% | 57% | 57% |
| moving: hit / toward goal / goal (of 60) | 60/58/59 | 60/59/58 | 58/56/40 | 59/53/46 |
| goalie saves; 1v1 fast wz rev /min | 87%; 1.9 | 87%; 1.7 | 72%; 2.6 | 72%; 2.1 |

**What this means for the Switch stage:**
- The rule picks the right role and reacts in time.
- It saves only about 35% of human shots. A pure keeper (`role_mode` defend) saves 77% and concedes 3.9 against 10.9 goals per 3 min.
- The reason: from upfield, getting back takes 5.4 s, but a shot reaches goal in 2.2 s. No speed knob fixes that. `return_vx` 0.75 reaches the line in 4.4 s but adds no saves (28–35%), so keep the preset's 0.45.
- Also tried: `solo_hold_s` 1.5 / 2.5 give 34 / 35% saves, 10.9 / 10.7 switches per min, and at 0.7 s lag robot goals of 0.9 → 0.3. Keep 1.0: the human's actions drive the switches, not the hold.
- `solo_zone` 0.6: 37% saves, 1.2 robot goals.

The scripted human is harsh (an action every 4–9 s from 1.5–2.3 m out). Interception is neutral in 1v1: 3.4 vs 3.7 goals per 3 min over 24 matches, within noise, with gait switches 12.6 → 14.9 per min.

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

## Athletic: planned approach, coast, fast-gait heading bias (2026-11-24)

Preset `config/presets/athletic.json` = fast-light + `plan` 1, `coast_s` 0.3, `fast_yaw_bias_deg` -8, `ball_decel` 0.18.
Each new knob at 0 gives fast-light's behaviour exactly (brain DEFAULTS document them).

- **Plan** (`plan`): picks the fastest path to a stage `plan_stage_m` behind the predicted intercept point on the
  ball→goal line: Dubins arcs at `plan_radius` or 0.6×, optionally after a spin on the spot, or turn–straight–turn,
  directly or via a tangent point beside the ball. Each planned left/right switch costs `plan_flip_s`. It is followed
  by pure pursuit (lookahead grows with speed, `plan_look_s`). Replans happen only when:
  - the intercept moves more than `plan_replan_m` (shrinks to 30% over the last metre);
  - the push direction turns more than `plan_replan_deg`;
  - the robot is more than `plan_dev_m` off the path;
  - the path is used up.

  Behind the ball on the line, a pure-pursuit line follower takes over and slides off lateral error. If no path fits,
  it falls back to the old approach. Planning costs ≤ 2 ms. The committed path (≤ 20 points) is in `dbg["plan"]` for
  the UI.
- **Coast** (`coast_s`): when the own pose is older than this but younger than `pose_stale_s`, the robot keeps the
  last command and slows toward `coast_vx`. It does not steer on a stale pose.
- **`fast_yaw_bias_deg`**: added to the heading while the executing command is fast. It should equal the step test's
  `heading_bias_deg` at speed (travel − tag). In the sim, −8 beats 0 and +8.
- **sim / sim_eval**: `sim_burst_s_max`, `sim_fast_dir_deg`; `--floor` = Nov 24 floor errors (0.1 s latency,
  0.3–1.5 s dropouts, fast gait ×0.83 walking 8° right, walker lag 0.2, ball decel 0.18). New rows: solo jitter
  (wz reversals, RMS, replans/min, coast share), athletic (mean speed, time to ball), and 1v1 wz RMS / replans.

`--floor`, 100 seeds, 6 × 3 min, errors ON (`python -m fc.sim_eval --seeds 100 --matches 6 --floor --preset fast-light athletic`):

| | fast-light | athletic |
|---|---|---|
| solo goals / median / p90 | 97% / 14.0 / 22.4 s | 98% / 14.5 / 28.0 s |
| solo wz rev / fast rev /min | 12.7 / 1.1 | **9.8 / 0.6** (−23% / −45%) |
| solo replans /min, time to ball | –, 5.1 s | 8.5, 4.9 s |
| moving ball: hit ≤ 6 s / toward / goal | 72 / 88 / 59 | **84 / 94** / 58 |
| human: right role on shots, saves | 43%, 34% | 91%, 39% (ball_decel 0.18) |
| 1v1 goals/3 min; wz rev / fast rev; replans | 3.7; 14.6 / 1.4; – | 4.5; 15.9 / 2.2; 20.6 |

Targets: goal rate held and jitter dropped. Wz reversals fell 23% at 100 seeds, against the ≥ 30% target; at 40 seeds
the drop was 31%. The solo median did **not** drop: it rose 0.5 s, and the tail is worse. Time to ball improved, so
the time is lost after first touch:
- dribbling is capped by `contact_vx` 0.3 (47% of solo time);
- corner and wall balls cost time.

Against the old 0.45 ball model, athletic was 14.0 vs 14.3 s with −33% reversals. In 1v1 the opponent keeps moving
the ball, which causes ~20 replans/min and more reversals.

**`ball_decel` matters:** with the floor ball (0.18), fast-light (0.45) predicts shots stop short and defends only 43%
of them. Set `ball_decel` from `ctl rollcal` in every preset.

**Floor A/B to run:** `fast-light` vs `athletic`, ×5 each. Log goals, time, and wz reversals from the run logs.

**Things that did not work:**
- A square-up latch (slower).
- Turning the line follower by the paced speed (35/40 goals).
- `plan_near_lat` 0.25/0.3.
- A shorter-stage push direction (own goals).
- `plan_wz` 2.1, `plan_radius` 0.4, `plan_look_s` 0.3/0.8.

## Dribble mode (2026-11-25, sim only)

`dribble: 1` (preset `config/presets/dribble.json`; fast-light unchanged). Built from the Nov 25 floor notes: predicting the meeting point and sprinting there worked; clunky turning near the ball and slow recovery after a sideways deflection did not. Second round of floor feedback: **no lateral offset** (6 cm and 3 cm both made the ball drift sideways) and **hit hard and straight** (a soft touch lets the slightly non-spherical ball curve away on the carpet).

- **Meeting point:** fixed point of "ball after our ETA" (ETA = onset scaled by how much we are already moving that way + distance at `dribble_gait_gain * max_vx` + half the turn), horizon `dribble_horizon_s` (3 s). Solved every tick from the ball's filtered velocity, also next to the ball. A velocity change > `dribble_cone_deg` or 0.3 m/s is a deflection: filters snap and the meeting point is re-solved.
- **Steering by geometry:** pure pursuit on the ball line (dead centre; `push_side_m` > 0 is still available); lookahead `dribble_look_m + dribble_look_s * v`; `wz = v * kappa`, v capped at `sqrt(dribble_alat / |kappa|)`. Far off the line the stage sits further back (1.2 x lateral offset) so the robot joins the line at a shallow angle. The orbit (`_route`) is kept for a robot in front of the ball.
- **Speed:** closing speed relative to the ball: `dribble_close` (0.55) at the touch, `dribble_margin` (0.65) 0.15 m out, braking-limited beyond; firm hits, then it keeps driving through. Jerk-limited ramp (`dribble_jerk`) that steps across the gait band.
- **Sim ball curve:** `sim_ball_curve` (rad/s at 1 m/s, redrawn at every touch/kick): heading rate = k / speed, so slow balls curve and fast ones go straight. `--floor25` sets 0.08 along with the Nov 25 ball decel 0.149, gait 0.88 and cap 0.8.
- Full knob list and superseded knobs: brain.py module doc. `python -m fc.sim_eval --floor25 --scen solo moving` prints touches/possession, gap in possession, wz near the ball and deflection re-touch.

100 seeds, `--floor25` (with ball curve), errors ON:

| | fast-light | dribble |
|---|---|---|
| solo goals ≤40 s / median / p90 | 98%, 13.1 s, 25.6 s | 93%, **8.5 s**, 23.2 s |
| mean speed, s to first touch | 0.25 m/s, 4.6 s | **0.35 m/s**, 4.2 s |
| touches/possession, mean gap in possession | 4.0, 0.04 m | 2.7, 0.10 m |
| near ball (<0.6 m): wz rms, reversals/min | **0.71, 12.8** | 1.05, 22.1 |
| deflections: re-touch ≤8 s, median | 86%, 2.2 s (136) | 85%, 3.6 s (68) |
| moving ball: hit ≤6 s, goal ≤15 s, median | 72, 54 of 100, 11.8 s | **96, 70 of 100, 7.5 s** |

Soft touches (close 0.15) under the same curve model: median 11.8 s, moving median 10.8 s. Speed sweep (first round, max_vx 0.75/0.9/1.0 at cap 0.8 vs 1.0) moved goal times < 1 s: no dribble-fast preset. Open: near-ball turning is busier than fast-light (lookahead 0.5 m + 0.5 s; longer is calmer but slower); 7/100 solo misses are mostly wall balls the sim referee had to free.
