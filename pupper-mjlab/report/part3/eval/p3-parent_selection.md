# Part 3 checkpoint selection: p3-parent

Guards from [the budget decision](../../decisions/2026-11-18-w4-finetune-budget.md), applied mechanically.

Verdict: **OK**. Selected: `/home/ubuntu/runs/opt-lab-2/rp-p3-parent-bumpyft-smoke/model_3048.pt` (+49).

- ship +49 (no guard fired)

| point | flat_dr falls / seam | bumpy_dr falls / seam | flat_nodr falls / seam | bumpy_nodr falls / seam |
|---|---|---|---|---|
| +0 (warm-start parent) | 40 / PASS | 81 / PASS | 100 / FAIL | 107 / FAIL |
| +49 | 63 / PASS | 91 / FAIL | 114 / FAIL | 128 / FAIL |
| flat control +49 (matched) | 63 / PASS | 82 / FAIL | 108 / FAIL | 133 / FAIL |
