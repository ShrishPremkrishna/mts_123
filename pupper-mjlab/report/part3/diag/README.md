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

## Checkpoint-to-checkpoint variance (eval seed 1234; `neighbours.sh`, outputs in `neighbours/`)

Bumpy (WIDEN) arm, flat_dr suite falls / nominal-PD falls. Neighbours are the 50-iteration saves; leg boundaries
restart the learning rate and episode buffers.

| around | −100 | −50 | point | +50 | +100 |
|---|---|---|---|---|---|
| +1000 (model_5494) | 5400: 48 / 60 | 5450: 34 / 41 | **34 / 42** | 5550: 31 / 27 | 5600: 32 / 53 |
| +1500 (model_5993) | 5900: 21 / 28 | 5950: 15 / 34 | **11 / 14** | 6050: 16 / 17 | 6100: 13 / 17 |

model_6000 (+7 iterations after +1500, inside leg 4's restart) gives 4 / 9.

Reading:
- The +1000 dip is a real stretch of training, not one bad checkpoint: flat_dr 31–48 across the neighbourhood.
- The +1500 neighbourhood is genuinely better, with flat_dr 4–21 (median 15) and nominal-PD 9–34 (median 17).
- +1500's own 11 / 14 sits at the good end of its neighbourhood, so read its single-checkpoint numbers with about
  ±7–10 falls of checkpoint variance on top of eval noise.

## Push recovery (part2_eval protocol; ranked `R:push_stand@0.65` and `R:push_move@0.5`; `push.sh` → `push/`)

Standing falls per push at 0.65 m/s, flat_dr (Bumpy-condition values in parentheses where measured). Moving falls
per push at 0.5 m/s are 0–0.023 everywhere.

| checkpoint | seed 1234 | seed 4321 | neighbours (flat_dr, seed 1234) |
|---|---|---|---|
| P (W3 final) | 0.096 (Bumpy 0.159) | 0.122 | – |
| Bumpy +500 (4995) | 0.798 (Bumpy 0.727) | – | – |
| Bumpy +1000 (5494) | 0.754 (Bumpy 0.662) | – | – |
| Bumpy +1500 (5993) | 0.533 (Bumpy 0.509) | – | 5950: 0.856, 6050: 0.710 |
| Bumpy +2000 (6492) | 0.489 (Bumpy 0.486) | 0.497 | 6400: 0.563, 6450: 0.721 |
| flat control +1500 | 0.024 (Bumpy 0.129) | – | – |
| flat control +2000 | 0.091 (Bumpy 0.163) | – | – |

The Bumpy (WIDEN) arm loses standing push recovery from +500 on, at every measured checkpoint and on both eval seeds
(0.49–0.86 vs P's 0.10–0.12). The matched flat control (same PD ranges) keeps it, so terrain, not the PD range, is
the likely cause. Standing push is a ranked W2 metric that neither the Part 3 selection guards nor the HOLD rule include.
