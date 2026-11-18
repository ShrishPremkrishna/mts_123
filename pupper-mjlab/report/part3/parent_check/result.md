# Parent-pick check at nominal PD gains: result

Rule (pre-registered in `scripts/opt_lab_2/part3_parent_check.py` and `README.md` here): reopen iff excess(fast_trot) <= 15 AND nompd(fast_trot) <= nompd(gallop_c) - 20.

| arm | train seed | eval seed | Lab 4 DR falls (flat_dr) | nominal-PD falls |
|---|---|---|---|---|
| fast_trot | s42 | 1234 | 50 | 91 |
| fast_trot | s42 | 4321 | 41 | 83 |
| fast_trot | s43 | 1234 | 69 | 93 |
| fast_trot | s43 | 4321 | 52 | 88 |
| gallop_c | s42 | 1234 | 40 | 99 |
| gallop_c | s42 | 4321 | 41 | 79 |
| gallop_c | s43 | 1234 | 38 | 67 |
| gallop_c | s43 | 4321 | 38 | 61 |

| arm | mean Lab 4 | mean nominal-PD | excess |
|---|---|---|---|
| fast_trot | 53.0 | 88.8 | +35.8 |
| gallop_c | 39.2 | 76.5 | +37.2 |

(a) excess(fast_trot) +35.8 <= 15: **False**; (b) nompd(fast_trot) 88.8 <= nompd(gallop_c) 76.5 - 20: **False**.

**Verdict: the parent pick STANDS.**

Simulation only; 384 robot-runs per suite; GPU rollouts are not bit-reproducible (gaps < 10-15 falls are ties).
