# 1v1 exhibition design (Nov 27, design only, nothing built)

A small `Coach` decides once per tick who owns the ball and who may go for it. One robot goes for the ball; the other shadows goal-side and dashes only when it gets there first.

**Why.** Today each brain picks its own role (`_role`: "a defender attacks when not slower"), so both chase whenever the travel-time gap is under 1 s. One decision cannot disagree with itself. No coach input = today's brain.

## 1. State model

Signals per robot *i* (raw World poses): gap `g` = centre distance to ball − 0.27 m (0 = touching); bearing `β` = ball angle off the robot's nose; ball velocity; last touch (`g` < 0.03).

**Possession** ∈ {blue, red, loose}:
- loose → *i*: `g` < 0.12 and `β` < 50° for 0.3 s, and the other robot is not that close too.
- *i* → loose, any of: `g` > 0.35 for 0.2 s (ignored if *i* touched < 0.6 s ago and the ball rolls within 40° of its heading: a dribbled ball runs ahead between touches); `β` > 90° for 0.3 s; other robot also within 0.12 for 0.2 s (contested); ball receding > 0.7 m/s (shot, knock); pose stale.
- Never *i* → *j* directly: always through loose, ≥ 0.3 s.

**Tasks.** `first` = the robot allowed to go for the ball. `J` = shadow point: on the line ball → own goal centre, 0.75 m from the ball, never behind the guard line.

| Task | When | Motion (reused) |
|---|---|---|
| ATTACK | has the ball, or first on a loose ball | `_attack_dribble` as is |
| CHALLENGE | not first, opening (§2) | `_attack_dribble`, push turned 30° away from the opponent: ball knocked aside, not squeezed |
| JOCKEY | not first, within 0.35 m of `J` | hold `J`, face the ball, mirror sideways with vy, sway |
| GOALIE | `J` clamped to the guard line (ball ≤ 1.0 m out = `dribble_shot_m`) | `_defend` guard |
| RECOVER | not first, > 0.35 m from `J` | `_defend` "return" at fast gait, target `J` |

JOCKEY → GOALIE is one formula hitting a clamp: no jump.

## 2. Transitions and openings

**First on a loose ball:** lower ETA (`_dr_eta`, which counts start-up, turning and walking round the ball). It changes hands only if the other is ≥ 0.4 s better and ≥ 1.5 s passed; at once if first is stale or unsticking. Tie (< 0.2 s, e.g. kickoff): the robot that did not last have the ball.

**Opening = one number.** `adv` = opponent's ETA to the ball − my ETA to where the ball will be (`_ball_at`). Dash when all hold:
- `adv` ≥ 0.5 s for 0.2 s (lag ~0.3 s plus margin);
- the ball is > 0.20 m clear of the opponent's body;
- I am goal-side of the ball;
- the meeting point is inside the fence margin (no dashes into corners).

A separated ball, an attacker turning, circling or stopped, a ball rolling toward me: each raises `adv`. These are the on-screen reason, not separate rules.

**Commitment:** CHALLENGE holds ≥ 1.0 s. It ends on a touch (possession takes over), a 2.5 s timeout, or an abort (opponent back within 0.10 m while I am > 0.40 m away; reads as a feint). Then 1.5 s before the next dash.

`clear` near goal (today: walk into any slow ball, even one the opponent carries) follows the same rule. More dashes: shorter `jockey_m` or lower `open_adv_s`.

## 3. Spacing, motion, smoothness

- **Opponent guard** (new, `_fence_guard` maths, final command, every task): speed toward the other robot is capped at what brakes at 0.8 m/s² before 0.45 m centre distance. Bodies touch at 0.36 m; with the ball between them robots are 0.54 m apart, so ball duels work and body rams do not.
- **Keep-out 0.75 m, no both-chase:** only `first` and a committed challenger approach the ball. Keep-out applies to JOCKEY/RECOVER/GOALIE targets and detours (`_detour` takes the opponent as a second obstacle). ATTACK and CHALLENGE keep today's `_repel`.
- **Alive:** JOCKEY sways ±0.10 m across the ball–goal line, period 2.5 s, and backpedals as the dribbler advances. Shadow target low-passed 0.3 s (ball jitter is not mirrored).
- **Gait as body language:** JOCKEY stays in the trot (≤ 0.45); RECOVER and CHALLENGE use the fast gait.
- **Smooth:** `_goto` already curves; detour side latched 2 s; speed ≤ √(`dribble_alat`/curvature) (radius ≥ 0.44 m at 0.66 m/s); `_turn_commit` 0.3 s; 10° heading deadband in JOCKEY.

## 4. Code mapping and validation

Parts:
1. `Coach` (new, ~80 lines, brain.py): `update(world) → {robot: task, why, adv}`, once per tick in the app loop and `sim_eval._loop`, passed as per-call params like `goto_x`.
2. `_role`: with a task, attack/challenge → "attack", others → "defend"; skip the cost logic.
3. `_defend`: guard point `G = J` where `gk_out` computes it today (same shape); heading from the `facing` style. Return, `_block`, back-up-while-turning reused.
4. `_opp_guard` (~15 lines).
5. `_dr_eta`: starting speed as an argument (for the opponent). `_dr_dir`: knock angle.
6. Preset `duel.json` = dribble + `duel` 1, `poss_in_m` 0.12, `poss_out_m` 0.35, `poss_dwell_s` 0.3, `jockey_m` 0.75, `jockey_sway_m` 0.10, `jockey_sway_s` 2.5, `open_adv_s` 0.5, `chal_hold_s` 1.0, `chal_cool_s` 1.5, `first_hyst_s` 0.4, `first_hold_s` 1.5, `keep_out_m` 0.75, `opp_stop_m` 0.45, `duel_knock_deg` 30.

Build order: Coach + JOCKEY + opponent guard; the dash; the knock angle only if head-on bumps remain.

Validate: `sim_eval.match()`, `--floor25 --field length=3.36 width=2.70`, 20 × 3 min, `dribble` vs `duel`; counters from sim truth (`sim.contacts`, `sim.bumps`, poses). Baselines in brackets are from notes/brain.md (older presets and field): rerun `dribble` first.

| Metric | Target |
|---|---|
| Bumps /min; hardest closing | ≤ 1; ≤ 0.3 m/s (was ~8; 0.84) |
| Both within 0.5 m of the ball | ≤ 10% of play |
| Second robot goal-side during opponent possession | ≥ 70%; ≤ 4 s after a turnover |
| Possession changes /min | 4–8 |
| Goals /3 min, both sides | 3–6 (was 8.5) |
| Mean speed; stopped (< 0.05 m/s) | ≥ 0.30 m/s (was 0.23); ≤ 5% |
| wz reversals; fast ones /min | ≤ 16; ≤ 2.5 (was 15.9; 2.2) |
| Task changes /robot-min; deadlocks | ≤ 8; 0 |

Solo rows must not change.

## 5. Play screen

Per robot chip: role word plus intent with its number (`dbg.task`, `why`, `adv`; text via the `ACTION`/`TICKER` maps):
- ON THE BALL: "dribbling at goal" / "shooting"
- SHADOW: "goal-side, waiting for an opening"
- CHALLENGE: "opening: gets there 0.6 s first (ball got away from Red)"
- KEEPER: "Red in shooting range: on the line"
- RECOVER: "beaten: sprinting back goal-side"

Draw a ring on the ball in the owner's colour (grey = loose) and the shadow line ball → goal with the `J` cross. Optional: an `adv` bar filling toward 0.5 s: the audience sees the dash coming.

## Risks

- Goals will drop (notes: scrambles create the chances). Under 3, shorten `jockey_m`; do not bring back double chasing.
- The sim does not model robot impacts or falls: build the opponent guard first, floor-check it before any two-robot run.
- Robot 2 is not in hand; feature freeze is Nov 29. Sim-only until then.
