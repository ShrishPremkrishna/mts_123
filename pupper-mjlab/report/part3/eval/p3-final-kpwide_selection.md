# Part 3 checkpoint selection: p3-final-kpwide

Matched selection per the [budget decision](../../decisions/2026-11-18-w4-finetune-budget.md) as amended by its [devil's-advocate review](../../decisions/2026-11-18-w4-finetune-budget/devils-advocate.md): a fallback must replicate on eval seeds 1234 and 4321; the control is compared at the same iteration.

Selected: **+2000** `/home/ubuntu/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`; matched control `/home/ubuntu/runs/opt-lab-2/rp-p3-final-flatctrl-kpwide-leg4/model_6492.pt`.

- ship +2000 (no regression vs +1500 on seed 1234)

| point | arm | flat_dr | bumpy_dr | nompd | flat_dr seam |
|---|---|---|---|---|---|
| +0 | parent | 14 | 35 | 39 | PASS |
| +500 | bumpy | 14 | 31 | 25 | PASS |
| +500 | flat control | 16 | 38 | 28 | PASS |
| +1000 | bumpy | 34 | 46 | 42 | PASS |
| +1000 | flat control | 18 | 47 | 34 | PASS |
| +1500 | bumpy | 11 | 23 | 14 | PASS |
| +1500 | flat control | 23 | 52 | 51 | PASS |
| +2000 | bumpy | 10 | 18 | 7 | PASS |
| +2000 | flat control | 12 | 29 | 20 | PASS |
