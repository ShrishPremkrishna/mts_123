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

## Dribble v3 (2026-11-25, sim only)

`config/presets/dribble.json` is v3; `dribble-v2.json` keeps v2 for A/B (v3 misses the alignment target and recovers from deflections slower). New knobs default to 0 (v2 unchanged; tests check it); details in the brain.py module doc.

- **Where v2's near-ball reversals came from** (per-reversal context): 2/3 happened within 0.1 m of the ball, most > 0.2 m off the meeting-point line. The meeting point 1–3 s ahead swung the line under a carried ball, and pure pursuit then turned into the ball and shouldered it. Fixes:
  - `dribble_near_m` 0.15: at the ball, play the ball itself.
  - Line mode (`dribble_line_*`, `dribble_k_head`, `dribble_db_deg` 7, `dribble_k_vy` 3, `max_vy` 0.3): heading follows the push direction with a deadband, and vy slides off the lateral error. Off centre at the ball it keeps pace without closing in.
  - `dribble_commit_s` 0.3: a turn the other way waits 0.3 s unless it is big (> 0.8 rad/s).
- **Shot** (`dribble_shot_m` 1.0): latched when centred (4 cm) and lined up within 1 m of the goal. In the sim carrying already runs near top speed, so the gain is small (shot 0.50 → 0.58 m/s). The first version froze steering during the shot; walking drift then made the hit off-centre and sent the ball wide. Steering stays on.
- **`fast_yaw_bias_deg` −8** (STATUS: the fast gait walks ~8° right). This is worth ~1.5 s of median: v3 without it scores 97% at a 9.7 s median with 15° alignment. v2 with it reaches 8.1 s. Set it from the step test.
- **Goalie:** `goalie_vmax` 0.75, `k_goalie` 6, `return_vx` 0.75. `solo_reach` 1 switches to attack when the robot can stand behind a threatening ball ≥ 0.4 s before it arrives, so it meets the shot head-on and counters.
- **Tried, no gain (dropped):** a wz low-pass (τ 0.15–0.3 s: slower, same reversals), fading the line pull near the ball, a different stage distance (0.6–1.6 × lateral), a wider centring tolerance (worse), max_vx 0.8, dribble_close 0.7, dribble_alat 1.3.

`python -m fc.sim_eval --seeds 100 --matches 10 --floor25 --scen solo moving human goalie --preset fast-light dribble-v2 dribble`, errors ON:

| | fast-light | dribble v2 | dribble v3 |
|---|---|---|---|
| solo goals / median / p90 | 98%, 13.1, 25.6 s | 93%, 8.5, 23.2 s | **98%, 7.7, 17.9 s** |
| mean speed, s to first touch | 0.25, 4.6 | 0.35, 4.2 | 0.38, 4.1 |
| near ball: wz rms, reversals/min | 0.71, 12.8 | 1.05, 22.1 | 0.94, **12.1** |
| heading error at arrival, median / p90 | 26 / 118° | 19 / 66° | 19 / 77° (target 15: missed) |
| near ball: vy in use, shot speed at goal | 35%, 0.22 | 28%, 0.50 | 49%, 0.58 m/s |
| deflections: re-touch ≤ 8 s, median | 86%, 2.2 s | 85%, 3.6 s | 71%, 4.6 s (41 vs 68 deflections) |
| moving ball: hit / goal of 100, median | 72 / 54, 11.8 s | 96 / 70, 7.5 s | 96 / **83**, 7.3 s |
| goalie saves (defend) | 89% | 89% | **94%** |
| human: robot goals / conceded per 3 min, saves | 1.3 / 7.4, 55% | 2.6 / 8.4, 48% | **4.5 / 7.3, 55%** |
| human: defend → attack switches/min | 6.5 | 6.5 | 6.3 |
| 1v1: goals/3 min, traps | 5.3, 10 | 7.4, 16 | 8.5, 18 |

Alignment is the heading at the first touch of each possession against the direction from the ball to the goal centre. One in ten arrivals is a touch from the wrong side (~145°), and chase arrivals are 12–18° off. Medians at 100 seeds move about ±0.4 s between nearby settings. At 200 seeds the ranking holds: v3 98%, 8.2 s, 12.2 reversals/min; v2 94%, 8.7 s, 22.2.

## Contested play vs a human (2026-11-25, sim only)

Floor game vs a human, dribble v3: 0–3 in 3 min, 30.7 m walked, 66 wz reversals (22 fast), own tag lost 33 times (human
body over the robot). Preset `config/presets/vs-human.json` = dribble v3 + the knobs below (all default 0 = off; knob
docs in the brain.py module doc). Without opponent pressure (no foreign touch for `press_s` 8 s) solo play is v3.

- **Ball model** (`contest`): a ball whose velocity changed over ~0.2 s by more than `contest_dv` 0.3 m/s (or 0.25 m off
  the rolling prediction) on two frames, not a wall bounce (change along the wall normal) and not our push (change
  pointing away from us within 0.4 s of being at the ball), was touched by someone else: **contested** for 1 s. A ball
  back from > 0.3 s hidden far from the model's prediction also counts. Contested: alpha-beta smoothed ball, 0.5 s
  prediction horizon, no deflection snaps, and the target is committed while we are > 0.4 m from the ball. The goalie
  always tracks the raw ball (smoothing lag cost ~1 goal/3 min). Detection: about 2/3 of the human touches made while the ball is visible;
  1 false touch in 60 solo runs.
- **Occlusion:** own tag hidden → dead reckoning on the commands (×0.9 gain) at 0.6× speed up to 1.5 s, then stop (under
  pressure from 0.3 s; otherwise after `pose_stale_s`, so solo is v3). The preset sets the app's `stale_stop_s` 1.6, or
  the app stops the robot first (**a robot can walk blind for 1.6 s at reduced speed: watch this on the floor**). Ball
  hidden → frozen (contested) or rolled on (free) for up to 2 s; a defender holds the goal centre; beyond 2 s the robot
  guards the goal centre instead of standing still. `dbg.occluded`, `dbg.ball_state`.
- **Wide-mouth goalie** (`guard_line_m` 0.2; the real mouth is 1.22 m): on the line it slides instead of turning to walk
  to a guard point that moves along the mouth; 15° heading deadband with a latched sideways heading; returns aim at the
  mouth centre first; clear ↔ guard holds 0.8 s; defender turns use the 0.3 s turn commitment. These removed the extra
  fast reversals that defending added (guard/return/clear flip-flops were 2/3 of them).
- `gk_out` (narrow the angle by coming out on the ball–goal line): no gain (10.2 vs 10.4 against). Off.

**Hard role rules** (`hard_roles`, solo, under pressure; ball-only, no human tag; `dbg.rule`):

| state | condition (checked in order) | → |
|---|---|---|
| defending | less than `hr_hold_s` 2.5 s since the switch | defend (`def_hold`) |
| defending | ball in the opponent half (+0.1 m), no shot coming | attack (`opp_half`) |
| defending | ball free, < `hr_exit_v` 0.3 m/s, not coming at us, our ETA ≤ `hr_exit_eta` 2 s | attack (`free_reach`) |
| defending | otherwise | defend (`def_stay`) |
| attacking | shot predicted at the mouth (+`solo_def_margin`) within 3 s | defend (`shot`), or attack if free and we can stand behind it first (`meet_shot`, solo_reach) |
| attacking | ball within `hr_zone` 0.5 × length of our goal line and (contested or moving at us > `hr_vin` 0.1) | attack if within `hr_stick_m` 0.35 m of it and goal-side (`stick`), or free and reachable first / met head-on from goal-side (`reach`), or < `solo_hold_s` since the last switch (`att_hold`); else defend (`zone`) |
| attacking | otherwise | attack |

Defending clears a contested ball near goal only from goal-side (walking round it opened the goal).

**Results** (`--floor25`, errors ON, 80 × 3 min per column, ± = standard error; `sim_eval --scen vshuman` prints both
profiles; the human model is `sim_eval.HUMAN`, ball-only, with its body hiding the robot/ball for 0.5–2 s while close):

| | dribble v3 | vs-human | vs-human, hard_roles 0 |
|---|---|---|---|
| *dominant human* (not put off by robot touches): goals for / against | 0.9 / 17.0 ±0.3 | 0.7 / **15.8** ±0.2 | 0.8 / 17.9 |
| role switches /min | 21.8 | **18.3** | 23.0 |
| wz rev / fast rev /min | 16.0 / 2.4 | 19.0 / 3.9 | 16.6 / 2.6 |
| goal open s/3 min; stalled while hidden; app stale-stop s/3 min | 34; 37%; 17 | **30; 24%; 5** | 36; 34%; 6 |
| *dribbler human* (robot can win the ball): goals for / against | **6.2** ±0.4 / **9.2** ±0.2 | 3.7 / 10.4 | 5.4 / 9.8 |
| role switches /min; fast rev /min | 15.3; 2.4 | 15.1; 3.2 | 16.7; 2.6 |
| solo push ≤40 s / median / p90 (100 seeds) | 98%, 7.7, 17.9 s | 99%, 7.6, 20.7 s | |
| moving ball hit / goal; goalie saves sideways | 96 / 83; 94% | 96 / 81; 100% | |

What this says:
- **The commitment pays only against a human who wins the ball anyway** (like the floor game): −1.2 goals against
  (−7%), −16% role switches, less goal-open time. Against a human the robot can contest, pressing beats keeping:
  the rules cost 1.7–2.5 goals for and concede ~1 more. In the sim the human dribbles in and scores from close range
  past a keeper on a 1.22 m mouth (saving more needs a narrower goal, not a role rule). Floor A/B: vs-human vs
  vs-human with `hard_roles: 0`.
- **Item 1 (retargeting jank) is not shown by the sim:** the sim gives dribble v3 16–17 reversals/min, 2.4 fast;
  the floor game had 22 / 7. The contested model did not lower reversals in the sim (the sim human's ball rolls freely
  between touches, so predictions stay good). The remaining fast reversals sit in the defending phases, which the
  robot uses much more under the rules. Check on the floor with the run logs (`dbg.ball_state`, `dbg.rule`).
- **Occlusion:** no stalls from a lost ball (v3: ~8 s per match of `ball_lost_hold`), stale stops 17 → 5 s per 3 min,
  recovery median 0.17 → 0 s.

## Fences (2026-11-27, sim only)

Floor Nov 27: a corner ball, the robot routed round it to get behind, walked into the fence and tipped (fallen
cutoff 137 deg). Cause in code: targets were kept only `field_margin` 0.12 m inside the edge, less than the 0.18 m
body radius, and a wall ball's push direction was rotated just enough to put the stage point 0.12 m off the fence.

Knobs (brain.py doc, presets dribble + vs-human): `wall_margin_m` 0.28 clamps every target / stage / plan waypoint /
carrot into the field shrunk by 0.28 (goal mouth: to 5 cm short of the line) and guards the final command in every
role (speed toward a fence capped at what brakes at 0.8 m/s^2 before 0.18 m after act_lag_s; inside the margin a
push away, up to 0.2 m/s). `wall_ball_m` 0.32: a ball on a side fence is banked (push aimed at the goal mirrored
through the fence, steepened until the contact point keeps the margin; the ball comes off the fence toward goal).

Geometry limit (not a bug): with the robot centre >= 0.28 m off both fences and contact distance 0.27 m, a ball
pressed into a corner cannot be pushed out at all (every contact pushes it into both fences). The robot stays
clear; the 6 s referee nudge (sim) or a person frees it. That is why traps go up.

Sim: `wall_balls` scenario (ball against a fence 75% / in a corner 25%, robot in the open), fence contact = body
(0.16 m) touching a side fence or an end fence outside the mouth, would-tip = touching it > 0.15 m/s or 4 cm deep.
`--floor25 --seeds 100 --preset dribble --scen wall solo --field length=3.36 width=2.70`:

| | v3 (fence off) | margin only | margin + bank (preset) |
|---|---|---|---|
| wall balls: goals <=40 s, median s | 63/73, 17.0 | 70/73, 18.7 | 67/73, 19.6 |
| corner balls: goals, median s | 25/27, 13.7 | 27/27, 16.4 | 25/27, 16.2 |
| fence contacts / would-tip / referee traps | 252 / 228 / 60 | 14 / 7 / 95 | 6 / 2 / 87 |
| runs with a tip | 90/100 | 6/100 | 2/100 |
| open solo (attack_solo): goals, median s | 96/100, 9.3 | 98/100, 10.2 | 98/100, 10.0 |

Goals from wall balls lean on referee nudges in every column. Open-field solo is within noise on goals, ~0.7 s
slower median (attack_solo balls start up to 0.25 m from a fence, so some are wall balls). Not floor-tested.
