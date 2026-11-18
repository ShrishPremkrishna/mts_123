# Nominal-PD check on the +1500 candidates (simulation)

Same measurement as decision `2026-11-18-w4-kp-coverage` step 0 (`part3_eval run --conditions flat_dr --modes suite` and `part3_eval ablate --events pd_gains`), on eval seeds 1234 and 4321, 384 robot-runs each. nompd = Lab 4 DR with the PD gains at nominal (kp 5.5, kd 0.25 on every joint). The s42 parent was measured by W4: flat_dr 40 / 41, nompd 99 / 79 (`report/part3/eval/p3-parent_kp_coverage.md`). Raw JSONs: `report/part2/eval/w3-nompd/<name>[ -seed4321]/` and `<name>/ablate/`.

| checkpoint | flat_dr falls 1234 / 4321 | nompd falls 1234 / 4321 | nompd − flat_dr (mean) | nompd mean | nompd err±1.5 (1234) | nompd / flat_dr sat@1.5 (1234) | nompd F_runmax (1234) | nompd vx ±1.5 (1234) |
|---|---|---|---|---|---|---|---|---|
| p1-gallop_c-s43 | 38 / 41 | 69 / 67 | +31 / +26 (+28.5) | 68.0 | 0.168 | 0.231 / 0.176 | 3.60 | +1.64 / -1.62 |
| p2-s42-r3 | 21 / 24 | 52 / 41 | +31 / +17 (+24.0) | 46.5 | 0.147 | 0.203 / 0.159 | 3.18 | +1.63 / -1.62 |
| p2-s43-r3 | 22 / 27 | 30 / 37 | +8 / +10 (+9.0) | 33.5 | 0.141 | 0.196 / 0.151 | 3.12 | +1.57 / -1.65 |
| p2-s42-r2-cont-1500 | 14 / 17 | 39 / 37 | +25 / +20 (+22.5) | 38.0 | 0.149 | 0.209 / 0.160 | 3.32 | +1.61 / -1.64 |
| p2-s43-r2-cont-1500 | 23 / 16 | 44 / 30 | +21 / +14 (+17.5) | 37.0 | 0.154 | 0.205 / 0.157 | 3.46 | +1.59 / -1.67 |
| p2-s42-r1-cont-1500 | 27 / 25 | 49 / 39 | +22 / +14 (+18.0) | 44.0 | 0.148 | 0.222 / 0.174 | 3.41 | +1.60 / -1.62 |
| p2-s43-r1-cont-1500 | 26 / 29 | 55 / 43 | +29 / +14 (+21.5) | 49.0 | 0.146 | 0.219 / 0.171 | 3.49 | +1.59 / -1.64 |
| p2-s42-ctrl-1500 | 30 / 36 | 68 / 57 | +38 / +21 (+29.5) | 62.5 | 0.139 | 0.233 / 0.185 | 3.33 | +1.58 / -1.55 |
| p2-s43-ctrl-1500 | 18 / 20 | 31 / 31 | +13 / +11 (+12.0) | 31.0 | 0.143 | 0.228 / 0.180 | 3.35 | +1.59 / -1.62 |
