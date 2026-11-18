# WIDEN smoke checks: p3-final -- **PASS**

Decision [2026-11-18-w4-kp-coverage](../../decisions/2026-11-18-w4-kp-coverage.md), smoke items 1-5.

| check | ok | detail |
|---|---|---|
| 1. Lab 4 record byte-unchanged vs origin/main | yes | lab4_runs/shrish_stablegait/training-settings.json |
| 1. bumpy arm: run config records the WIDEN override and tag | yes | override {'kp_range': [0.6, 1.3], 'kd_range': [0.7, 1.5]}, tags ['p3', 'bumpy', 'p3dr-widekp', 'smoke'] |
| 2. bumpy arm: realized kp/5.5 spans ~0.6-1.3, kd/0.25 ~0.7-1.5, 17-23 % envs mean kp >= 1.0 | yes | kp [0.600, 1.300], kd [0.700, 1.500], frac(mean kp >= 1.0) 0.191 |
| 3. bumpy arm: training exited 0, no NaN | yes | /home/ubuntu/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-smoke.log |
| 3. bumpy arm: final episode length >= 450 and err_xy <= 0.30 | yes | ep length 464.08, err_xy 0.2218 |
| 1. control arm: run config records the WIDEN override and tag | yes | override {'kp_range': [0.6, 1.3], 'kd_range': [0.7, 1.5]}, tags ['p3', 'flat-control', 'p3dr-widekp', 'smoke'] |
| 2. control arm: realized kp/5.5 spans ~0.6-1.3, kd/0.25 ~0.7-1.5, 17-23 % envs mean kp >= 1.0 | yes | kp [0.600, 1.300], kd [0.700, 1.500], frac(mean kp >= 1.0) 0.201 |
| 3. control arm: training exited 0, no NaN | yes | /home/ubuntu/runs/opt-lab-2/rp-p3-final-flatctrl-kpwide-smoke.log |
| 3. control arm: final episode length >= 450 and err_xy <= 0.30 | yes | ep length 492.06, err_xy 0.203 |
| 3. bumpy arm: physics overflow 0 | yes | /home/ubuntu/runs/opt-lab-2/rp-p3-final-bumpyft-kpwide-smoke/physics_overflow_audit.json |
| 4. export PASS with policy.json kp 5.5 / kd 0.25 | yes | PASS, kp 5.5, kd 0.25 |
| 5. nompd ran on the smoke checkpoint (pipeline check, not a result) | yes | falls 45 |
