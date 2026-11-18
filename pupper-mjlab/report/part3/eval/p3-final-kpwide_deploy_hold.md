# Deployment hold check: p3-final-kpwide

Decision [2026-11-18-w4-kp-coverage](../../decisions/2026-11-18-w4-kp-coverage.md). Selected `/home/ubuntu/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt`; matched control `/home/ubuntu/runs/opt-lab-2/rp-p3-final-flatctrl-kpwide-leg4/model_6492.pt`. Verdict: **CLEAR**.

- (a) nompd - flat_dr = 9.5 - 10.5 = -1.0 (hold if > 15)
- (b) nompd(selected) - nompd(P) = 9.5 - 38.0 = -28.5 (hold if > 10)
- (WIDEN) matched control flat_dr - P flat_dr = 12 - 14 = -2 (hold if > 10)
- branch wide: clear
