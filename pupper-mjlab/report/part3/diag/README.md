# Part 3 final run: diagnostics (NOT selection inputs)

Requested by the coordinator after leg 2. These are kept apart from `report/part3/eval/` so the runbook never
reads them as selection inputs.

- `diag_seed4321.sh`: seed-4321 flat_dr/bumpy_dr suites + nominal-PD on +500 (model_4995) and +1000 (model_5494),
  both arms, launched through `gpu_slot_run.py`. Outputs: `p3-final-*-seed4321/`, `p3-final-*/ablate/`.
- `training_log_diag.py` → `training_log_diag.json`: per-leg training-log windows (reward terms, episode
  length, terminations, action std, entropy, value loss), learning rate from TensorBoard, and the realized
  kp/kd per leg.

| arm | point | flat_dr s1234 / s4321 | bumpy_dr s1234 / s4321 | nominal-PD s1234 / s4321 |
|---|---|---|---|---|
| +0 (W3 final) | – | 14 / 16 | 35 / – | 39 / 37 |
| Bumpy (WIDEN) | +500 | 14 / 19 | 31 / 36 | 25 / 31 |
| Bumpy (WIDEN) | +1000 | 34 / 39 | 46 / 59 | 42 / 41 |
| flat control (WIDEN) | +500 | 16 / 10 | 38 / 41 | 28 / 23 |
| flat control (WIDEN) | +1000 | 18 / 18 | 47 / 42 | 34 / 25 |

Finding:
- The Bumpy arm's +500 → +1000 regression replicates on both eval seeds. The added falls are in stop/reversal/mixed
  transients: flat_dr stop+reversal goes 7+7 → 20+14 and 9+10 → 22+17, while flat steady-state falls stay at 0.
- The training logs show no matching drift between legs 1 and 2:
  - reward 161.7 vs 163.6; episode length 477 vs 479; fell_over 0.74 vs 0.66;
  - err_xy 0.225 vs 0.223; action std 0.43 vs 0.43;
  - median learning rate 1.7e-4 in both legs; identical realized gains.
- The one shift is in leg 1: action std goes 0.37 → 0.43 in the Bumpy arm while the control goes 0.36 → 0.34.
- Each leg restarts the adaptive learning rate at 1e-5. It recovers within ~10 iterations.
