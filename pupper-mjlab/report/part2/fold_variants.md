# Fold variants on the smoke capture (facts only)

Capture: `baseline-s42-cont4500/model_4498.pt` (valid; trained on the shipped default references, whose fast slot
is the stock trot at a 1× clock), 64 envs × 15 s per command at +1.5 and −1.5 m/s, 3 s warmup excluded, heading hold
off, standard physics flags, seed 42. 1 fall at +1.5, 17 at −1.5. Achieved vx: +1.498 / −0.149 m/s.
At +1.5 the policy's feet land every ~8 control steps (~6 Hz) instead of once per 37.5-step reference cycle; at −1.5
they land every ~38 steps (on the clock).

Code: `src/mjlab/scripts/part2_reference.py` (module docstring defines the variants); tests:
`tests/test_part2_reference.py` (synthetic traces with a known lag/jitter). Folded dirs:
`configs/opt-lab-2/bootstrap/smoke_cont4500_<variant>/` (`fold_summary.json`, `diff_report.md`, `diff.png`,
manifest provenance). All four use median reduction, 6 harmonics, and the default pin (`leg_back_r_2`, `leg_back_l_2`
held at the parent's values); all four pass `verify` through the real loader.

| variant | dir | cycles | dispersion (rad) | wrap mismatch before repair (rad) | lag (steps) | lag-fit residual / zero-shift RMS (rad) | RMS diff vs parent (rad) | peak joint vel (rad/s) |
|---|---|---|---|---|---|---|---|---|
| naive | fwd / back | 891 / 886 | 0.068 / 0.045 | 0.002 / 0.001 | – | – | 0.241 / 0.195 | 18.7 / 17.4 |
| lag (auto) | fwd / back | 956 / 949 | 0.068 / 0.045 | 0.004 / 0.003 | 2.18 / 1.94 | 0.134 vs 0.233 / 0.064 vs 0.176 | 0.148 / 0.105 | 18.7 / 17.4 |
| phase (front_r touchdowns) | fwd / back | 891 / 880 | 0.068 / 0.071 | 0.002 / 0.001 | – | – | 0.241 / 0.195 | 18.7 / 16.4 |
| sym (lag + mirror) | fwd / back | 956 / 949 | 0.068 / 0.045 | 0.004 / 0.003 | 2.18 / 1.94 | as lag | 0.147 / 0.103 | 18.1 / 17.3 |

Notes:
- `phase` forward fell back to the clock (`--touchdown-fallback`) because the policy does not step on the reference
  clock at +1.5; without the flag it refuses. Backward used touchdown alignment (anchor phase 0.36) and its cycle
  dispersion rose from 0.045 to 0.071 rad.
- `sym` measured a left/right pair offset of 0.5 cycles (trot-like) with 0.041 / 0.042 rad residual asymmetry before
  averaging.
- The gait_tracking reward compares joint positions with the reference rewound by `phase_lead_steps` = 1.8 control
  steps; the measured lags (2.18 / 1.94) are close to it. A naive fold therefore stores the trajectory ≈2 steps late
  relative to the reference the policy was trained to follow; every naive round would add that delay again.
- The diff reports flag front abduction (`leg_front_*_2`, parent range 0.147 rad) as a near-constant channel moved by
  ~0.15–0.16 rad in every variant (it is not pinned by default; only back abduction is).
- Recovered history (`report/recovery/A_code_recovery.md` §capture variants, `REPRO_PLAN.md` §2.3): the old round 1
  compared naive vs lag; all final old rounds used a variant named `lag_m2` (semantics unknown); the back-abduction pin
  was the recovered fix for the round-1 crouching collapse.
- This capture is from a smoke parent, not the Part 1 gait the real Part 2 rounds will use (W1's pick, pending).
