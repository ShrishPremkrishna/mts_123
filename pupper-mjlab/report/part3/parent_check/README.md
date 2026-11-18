# Parent-pick check at nominal PD gains: pre-registration

Committed **before** any of the measurements below existed (requested by the coordinator, 2026-11-18).

**Question.** Does the Part 3 nominal-PD finding (`report/part3_notebook.md` §4) reopen the Part 1 parent decision
([gallop_c over fast_trot](../../decisions/2026-11-18-p1-parent-gait.md))?

**Measurements.** Each one is a Part 2 extended suite (384 robot-runs) on flat ground. Every arm is measured with its
own reference dir, on training seeds s42 and s43 and eval seeds 1234 and 4321.
- **Lab 4 DR falls:** `part3_eval run --conditions flat_dr`.
- **Nominal-PD falls:** `part3_eval ablate --events pd_gains`, i.e. the PD-gain DR event at nominal and every other DR event on.
- gallop_c s42 is the Part 3 parent. Its four numbers are already measured with the same tools
  (`report/part3/eval/p3-parent*`).

**Rule (fixed now).** Let excess = mean nominal-PD falls − mean Lab 4 falls, averaged over training seeds and eval seeds.
fast_trot **reopens** the parent decision only if **both** of these hold:
- (a) excess(fast_trot) ≤ 15;
- (b) mean nominal-PD falls(fast_trot) ≤ mean nominal-PD falls(gallop_c) − 20.

Otherwise the pick stands. A reopen changes nothing here; it goes to the coordinator.

Code: `scripts/opt_lab_2/part3_parent_check.py`. Results: `result.md` / `result.json` (written by the script).
