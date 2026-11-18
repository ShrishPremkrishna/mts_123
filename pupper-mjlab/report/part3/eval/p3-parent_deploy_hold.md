# Deployment hold check: p3-parent

Decision [2026-11-18-w4-kp-coverage](../../decisions/2026-11-18-w4-kp-coverage.md). Selected `/home/ubuntu/runs/opt-lab-2/rp-p3-parent-bumpyft-smoke/model_3048.pt`. Verdict: **HOLD**.

- (a) nompd - flat_dr = 106.0 - 62.5 = +43.5 (hold if > 15)
- (b) nompd(selected) - nompd(P) = 106.0 - 89.0 = +17.0 (hold if > 10)
- branch lab4: HOLD -> launch the pre-authorized WIDEN pair
