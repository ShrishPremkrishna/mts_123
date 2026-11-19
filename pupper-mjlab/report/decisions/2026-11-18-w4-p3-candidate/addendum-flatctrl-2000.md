# Addendum: option (ii) evidence, flat control +2000 (conclusion of the parent decision unchanged)
Added 2026-11-19 by W4 at the coordinator's request. Evals and export only, no training. Simulation only.
Checkpoint `~/runs/opt-lab-2/rp-p3-final-flatctrl-kpwide-leg4/model_6492.pt` (W&B `13m4vcia`).
Script: `report/part3/diag/flatctrl2000.sh`; outputs in `report/part3/eval/p3-final-flatctrl-kpwide-leg4-it6492{,-seed4321}/`.

| metric | seed 1234 | seed 4321 | two-seed mean | Bumpy +2000 (same seeds) | P |
|---|---|---|---|---|---|
| flat_dr falls | 12 | 13 | 12.5 | 10 / 11 | 14 / 16 |
| bumpy_dr falls | 29 | 34 | 31.5 | 18 / 23 | 35 / – |
| nominal-PD falls | 20 | 18 | 19.0 | 7 / 12 | 39 / 37 |
| push_stand@0.65 (flat_dr) | 0.091 | 0.089 | 0.090 | 0.489 / 0.497 | 0.096 / 0.122 |
| push_stand@0.65 (bumpy_dr) | 0.163 | 0.151 | 0.157 | 0.486 / – | 0.159 / – |
| push_move@0.5 (flat / bumpy) | 0 / 0.023 | 0 / 0.027 | – | 0 / 0.018 | 0 / 0.022 |
| seam verdict (flat_dr, bumpy_dr) | PASS, PASS | PASS, PASS | – | PASS, PASS | PASS, PASS |

**Two-seed HOLD readout:**
- (a) nominal-PD − flat_dr = 19.0 − 12.5 = **+6.5** (limit 15): clear.
- (b) nominal-PD − nominal-PD(P) = 19.0 − 38.0 = **−19.0** (limit 10): clear.
- WIDEN control check: flat_dr − P = 12 − 14 = −2 (limit 10): clear.
- HOLD(c): push_stand two-seed mean **0.090** ≤ P + 0.10 = 0.209: clear.

**Flat control +2000 is CLEAR on all four.**

**Export audit:** PASS.
- Actions: JSON vs torch 1.07e-5, run auto-export 1.07e-5, ONNX 3.4e-6.
- Golden residual 1.1e-5.
- Every limit check passes (kp 5.5, kd 0.25, 3 Nm effort, ±1.5 clip, reviewed files unchanged).
- Deploy validator and C++ gait-reference test pass.
- W&B `policy.json` is 404, the known runner bug; nothing was uploaded.

**The 62 BW F_peak flag (bumpy_dr, seed 1234) is a single-sample fall-impact spike, not a gait property:**
- It is one sample: `stops_fwd`, env 188, t = 3.88 s, foot 1.
- The per-fall log shows that same env terminating at t = 4.20 s (a steady-state fall at 1.5 m/s), 0.32 s later.
- The distribution is normal: p99.9 is 2.58 BW and the run-max median 2.84 BW, the same as P (2.77 / 3.09) and Bumpy +2000
  (2.42 / 2.68).
- Seed 4321 peaks at 7.7 BW.
- Per the W2 metric decision, the single-sample peak is a flag only.

**Caveats for the human:**
- The flat control is not the spec's Bumpy pass and was not a pre-registered option.
- Its bumpy_dr falls (29 / 34) are not better than P's (35) beyond noise.
- Its gains over P are in nominal-PD (−19) and the WIDEN PD ranges. Push at nominal PD is still unmeasured for every policy.
