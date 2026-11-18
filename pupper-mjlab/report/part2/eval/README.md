# Part 2 extended evaluation: definitions and observations

Code: `src/mjlab/scripts/part2_eval.py` (`suite`, `push`, `compare`, `annotate`, `batch`); tests: `tests/test_part2_eval.py`.
Ranking rules: `report/decisions/2026-11-18-w2-eval-metrics.md`. Ranked: falls (and categories), `F_runmax_med_BW`,
`R:push_stand@0.65` and `R:push_move@0.5`. Columns marked `(i)` or `(flag)` are informational only.
Commands and run times: `scripts/opt_lab_2/PART2_TOOLING.md`. Simulation only.

## Suite (384 robot-runs)

16 scripted command schedules × 24 envs, 20 s each (1000 steps at 50 Hz), seed 1234, play config (startup physics
randomization and observation noise kept), standard physics flags (deployment target clamp, driver limits, COM
randomization, torso contact). Heading hold is **off** (the robot gets exactly the scripted command; open-loop drift
is measured). Envs auto-reset after a fall and continue their schedule; the command term is patched so a reset keeps the
scheduled command.

| Metric | Definition |
|---|---|
| falls | terminations that are not timeouts, over all 384 robot-runs |
| falls by category | context of the step the fall happened in: `reversal` = ≤2 s after a vx sign flip with \|vx\|≥1; `stop` = ≤2 s after a step from \|vx\|≥1 to 0; `turn` = yaw≠0 or vy≠0 (turns and sidesteps); else `steady` (stand, ramps, constant speed) |
| falls<2s_reset | falls less than 2 s after a (re)set (episode age < 2 s); also counted inside their category. Every fall is also logged in `falls_detail` with schedule, time, episode age, time since the last command change and category |
| err±1.5 | mean \|v_xy − cmd_xy\| (body frame) in steady windows (≥2 s after the last command change, no reset) at +1.5 / −1.5 m/s |
| jitter | mean \|joint velocity\| (rad/s, 12 joints) in steady zero-command windows |
| sat | fraction of joint-torque samples with \|τ\| ≥ 0.99·3 Nm (`qfrc_actuator`; the actuator force limit is exactly 3 Nm, so τ never exceeds it), all alive samples and steady ±1.5 windows |
| F (all) | per-foot contact-force norm divided by the env's **actual** weight m·g (per-world `body_mass` summed over robot bodies; masses 3.06–3.84 kg vs 3.17 nominal); in-contact alive samples, excluding the first 1 s after every reset (spawn drop; unfiltered values kept as `*_incl_reset_drop`); sampled at 50 Hz (1 in 5 physics steps, so impacts alias low, equally for every policy) |
| **F_runmax_med_BW** (ranked) | median over the 384 robot-runs of each run's maximum per-foot force |
| F_p999_BW / F_p99_BW | 99.9th / 99th percentile over all in-contact samples |
| F_peak_BW(flag) | single-sample maximum, with `foot_force_peak_where`; a flag only, never ranked (it varies 4.9–11.7 BW across identical reruns) |
| drift± | \|Δ unwrapped heading\| per contiguous no-reset steady ±1.5 window with zero yaw command, scaled to deg/10 s; mean and max (windows ending in a fall are dropped and counted) |
| splay± | mean outward hind abduction (`leg_back_*_2` relative to the default pose; + = feet away from the midline, sign from FK: outward = (+q_R − q_L)/2) at ±1.5 |
| slip | mean in-contact foot xy speed in steady moving windows (also at ±1.5) |
| top± | max (min) over envs of the 1-s moving-average body vx, windows without a reset |
| tau2 | mean τ² over joints and alive samples |

Every JSON carries provenance (checkpoint sha256, reference dir + file hashes, git rev, seed, schedule hash) and a
`training` block (train num_envs, seed, iteration, parent, iterations past the parent). `compare` refuses to build a
table across differing train num_envs or iterations past the parent unless `--allow-mismatch` (then the table is
marked UNMATCHED), per `report/decisions/2026-11-18-infra-num-envs.md`.

## Push test

Protocol per `report/decisions/2026-11-18-w2-eval-metrics.md` (b): 128 envs in each of 4 conditions (stand; vx
+0.5; +1.5; −1.5; 512 envs) for 30 s. From 4 s, every 3 ± 0.25 s, an instantaneous lateral velocity impulse
(perpendicular to the heading, random sign) of **0.5, 0.65 or 0.8 m/s** (random per push) is added to the base via
`write_root_link_velocity_to_sim` (the mechanism of mjlab's `push_by_setting_velocity`). A push fails if the robot falls
within 2 s. Pushes within 2 s of a reset are not counted. About 4070 counted pushes per policy, about 340 per
condition × magnitude cell. A self-check records the lateral velocity gain one control step after each push
(0.46 / 0.60 / 0.74 m/s for 0.5 / 0.65 / 0.8).

| Column | Definition |
|---|---|
| **R:push_stand@0.65** (ranked) | standing falls/push at the pre-registered standing rung `PUSH_STANDING_RUNG` = 0.65 m/s (frozen by the pilot below) |
| **R:push_move@0.5** (ranked) | falls/push at 0.5 m/s pooled over the three moving conditions |
| vx_<cond>, push@0.5_<cond> | each moving condition's achieved mean body vx (alive samples ≥ 4 s, ≥ 2 s after a reset, outside push windows) next to its 0.5 m/s falls/push. Moving rates are confounded by achieved speed |
| stand@m(i), push(i), push_stand(i), push_move(i), push<m>(i) | informational only (pooled or unranked cells) |

**Pilot (2026-11-18, frozen rung).** Both baselines, two push runs each. Standing falls/push at 0.65 m/s:
0.060 / 0.069 (wandb-2999), 0.491 / 0.515 (cont4500), mean 0.284. At 0.8 m/s: 0.576 / 0.581 and 0.969 / 0.975,
mean 0.775, which is outside 5–60 %. So 0.65 is frozen (the rung with a mean in 5–60 % closest to 25 %). Repeat spread
between the two pilot runs: at most 0.061 in any cell (cont4500 walk +0.5 @0.65); ranked cells at most 0.024 (standing @0.65)
and 0.007 (moving @0.5). The final runs (a third repeat) fall inside the same ranges: standing @0.65 0.072 / 0.506,
moving @0.5 0.095 / 0.136. A 512-env push run takes 113–143 s, so the 96-envs fallback was not needed.

## Observations on the two valid baselines (2026-11-18)

- **Run-to-run spread at a fixed seed:** the GPU rollout is not bit-reproducible. Six identical suite runs of
  `baseline-s42-cont4500` gave 72, 62, 78, 67, 76 and 69 falls (stop falls 55–64). Treat fall gaps under ~10–15 as
  noise. The single-sample peak foot force varies 4.9–8.6 BW (wandb) and 6.3–11.7 BW (cont4500) across identical runs.
  The ranked per-run-max median moves by at most 0.03 BW over two runs: 1.919 / 1.945 (wandb) and 2.489 / 2.476
  (cont4500). p99.9 (1.70 / 2.66) and p99 (1.30 / 1.67) are equally stable.
- **Falls soon after a reset:** 0 of 4 (wandb) and 0 of 76 (cont4500) category falls happen < 2 s after a reset, so the
  categories are not polluted by post-reset falls (the decision's 20 % trigger is not reached).
- **Push (decision protocol, final runs):** standing falls/push at 0.5 / 0.65 / 0.8 m/s: 0.003 / 0.072 / 0.564 (wandb)
  and 0.030 / 0.506 / 0.955 (cont4500); the earlier 1.0 m/s rung knocked over 100 % of standing robots. Moving @0.5 pooled:
  0.095 vs 0.136. Per condition @0.5 with achieved vx: walk +0.5 0.048 (0.41 m/s) vs 0.024 (0.42); run +1.5 0.090
  (0.25 m/s) vs 0.100 (1.48 m/s); run −1.5 0.144 (−0.21) vs 0.276 (−0.15). The recovered old numbers (falls/push 5.4 % for
  the parent, 6.7–10.4 % with stand-still rewards) used an unknown, lost push protocol (`commands_push.json`), so compare
  them in direction only.
- **Heading hold:** with `--heading-hold`, drift falls from ~30 to ~7–8 deg/10 s, so the metric responds to it; the
  baselines trained with heading hold active in their command term.
- **Baseline behaviour:** `baseline-s42-wandb` (2999) plays the stock trot in the fast slot and reaches only ~0.25 m/s
  at a ±1.5 command. `baseline-s42-cont4500` reaches +1.47 m/s forward (slip 0.64 m/s; it shuffles at ~6 Hz, i.e. it
  does not step on the 1.33 Hz reference clock) but only −0.15 m/s backward; most of its falls are stops from +1.5.
