# How many PPO iterations should the Part 3 Bumpy + Lab 4 DR fine-tune run?
Status: grader-2 opinion (input to consolidation) · 2026-11-18 · Requested by: W4 (Part 3)
Criticality: Medium. It sets the cost of the one Part 3 training run. Nothing is matched against it.
Reversible?: Yes. Checkpoints are saved every 50 iterations, so a shorter answer is free afterwards (export an earlier checkpoint). A longer one costs ≈ 10–12 min of GPU per +500 iterations plus one eval.

## Conclusion
Use a fixed budget of **1500 iterations** (parent N → N+1500). Run it in one process with the default 50-iteration checkpoints. **Evaluate at +500, +1000 and +1500** against the same parent evaluated at +0 (flat_dr + bumpy_dr suite at the intermediate checkpoints, the full suite + seam + push at +1500). The deliverable is the +1500 checkpoint unless the fallback rule below picks +1000. Stop early only on the hard failures listed below. Extend in +500 legs, up to +3000 at most, only on the rule below. Set `FULL_ITERATIONS = 1500`.

## Evidence used
- **Recovered precedent** (`configs/optional2/README.md` table, `foundation/results/optional2-checked-bumpy-review.json`). It ran the same task family (MixedGaits-Bumpy, checked 0.10 m grid, warm start from a ~5500-iteration flat MixedGaits checkpoint) for **1500** iterations (model_6996). Failures over 3 seeds went 36/29/36 → 25/19/37 (101 → 81), mean xy error 0.093 → 0.085, and high-speed error 0.134 → 0.117. But standing torque, slip and yaw regressed, and individual commands failed more often, so it was not promoted. This is the only measured Bumpy fine-tune budget: 1500 gives a real but modest gain, with side costs.
- **Smoke** (`rp-p3-parent-bumpyft-smoke`, 50 iterations on the Part 1 parent). Training error_vel_xy stayed at ≈ 0.23–0.25 and ep_len at ≈ 470–480 after it 3020, with no trend. Its eval is still being written (no summary.md or push yet). The completed conditions are worse than the parent: flat_dr falls 40→63, bumpy_dr 81→91, flat_nodr 100→114, bumpy_nodr 107→128. The bumpy_dr seam verdict went PASS→FAIL (control max tilt 45.9°, and seam n 475→761, so the harness differs). So 50 iterations is still the post-switch transient, and any budget needs intermediate evals.
- **Speed.** In the same 18:34–18:36 window the smoke ran 4.6 s/iter, against 3.75–4.3 s/iter for the concurrent Flat Part 2 legs. Bumpy therefore costs ≈ 1.1–1.2× Flat, or ≈ 0.9 s/iter solo. At the ≈ 1.25 it/s aggregate (§4.3), 1500 iterations ≈ 23–25 min, and a full eval ≈ 10 min (the parent's suite + seam + push took 564 s). Total ≈ 0.75 GPU-h.
- **Notebook.** `ITERATIONS = 3000` is the default for a from-scratch run. The Bumpy task comment ("a converged flat checkpoint warm-starts the bumpy run") implies a fine-tune, which needs fewer iterations. The Bumpy task has no terrain curriculum, and the iteration-keyed curricula are already past at warm start.
- **Part 2 queue** (`w3/status-all.json`): ≈ 7000 iterations still pending (≈ 1.6 h aggregate). Part 3 cannot start until the Part 2 final is chosen, so it will mostly run with the GPU to itself.

## Thought process
Priorities: (1) enough PPO updates to get past the warm-start transient and actually adapt to terrain and DR. (2) A budget with a known outcome, so the result can be sanity-checked (§4.6). (3) Limit forgetting on flat and standing (the precedent's failure mode). (4) Cost, which is minor for every option (≤ 1.5 GPU-h). 1500 is the only budget with a measured Bumpy outcome in this task family, and it matches the Part 2 +1500 horizon. Evals at +500 and +1000 get the 500/1000 information almost for free (checkpoints already exist), so there is no reason to commit to a shorter run blind. Fixed budget (§3): the run always goes to +1500, and every checkpoint is reported with its iteration count. No arm is matched against it, so reading the intermediate checkpoints breaks no comparison.

Rules (fixed before launch). One seed, so gaps under 10 falls are noise (§3).
- **Stop early** (and re-decide) on: any nonzero physics-overflow bit; NaN or divergence; training ep_len < 400 or fell_over trending up for > 100 iterations after +100; or, at +500, flat_dr falls ≥ 2× the +0 value **and** bumpy_dr no better than +0.
- **Fallback:** export +1000 instead of +1500 if +1500 is worse than +1000 by > 10 falls on bumpy_dr or flat_dr, or the flat_dr seam verdict flips to FAIL. Report both.
- **Extend** by +500 (up to +3000) only if bumpy_dr falls drop by > 10 from +1000 to +1500 while flat_dr stays within 10 falls of +0. Stop at the first leg that gains < 10.

## Alternatives considered
| Option | Why not chosen |
|---|---|
| 500 | The smoke shows the early phase still regressing, and nothing shows 500 is past it. It stays available as the +500 checkpoint. |
| 1000 (current placeholder) | Reasonable, but there is no precedent. The extra 500 iterations cost ≈ 8 min, and +1000 stays available through the fallback rule. |
| 3000 | From-scratch default, not a fine-tune budget. Doubles the time spent on forgetting flat and standing without evidence it's needed. Reachable via the extension rule. |
| Shorter run + 8192 envs | Barred by the infra decision. The smoke shows no sample starvation. |

## Severe flagged issues
- **No control arm (§3).** A +1500 flat continuation of the same Part 2 final (≈ 0.4 GPU-h) is needed to attribute any robustness gain to Bumpy + DR rather than to more training. Recommend it if GPU time allows. Otherwise state the confound in the report.
- **The smoke is a confounded proxy:** a different parent (Part 1), a weight switch (splay → splay_stand_still), only 50 iterations, an incomplete eval and a changed seam harness (n 475 vs 761). Do not read its regressions as the expected Part 3 outcome, but do expect a transient.
- **One seed, and checkpoint selection on the same eval seeds** is optimistic. Report the fallback choice as a selection.
- **The precedent's side costs** (standing torque, slip, yaw, per-command failures) must be checked directly. Do not rank on aggregate falls or reward alone.

## What would flip this decision
- At the +500 eval, bumpy_dr already at its best and flat_dr regressing → ship +500 (or +1000).
- +1000 → +1500 still gaining > 10 falls on bumpy_dr → extend (rule above).
- Training visibly sample-starved (no learning-curve progress, high value loss) → separate num-envs decision (infra flip condition).
