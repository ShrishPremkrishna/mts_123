| Arm (WandB) | err_xy ±1.5 (mean / +1.5 / −1.5) | achieved vx +1.5 / −1.5 | err_yaw ±1.5 | mean τ² ±1.5 [Nm²] | slip ±1.5 [m/s] | 3 Nm sat ±1.5 | foot force, median robot max ±1.5 [BW] |
|---|---|---|---|---|---|---|---|
| old baseline (recovered ckpt, bare profile) (`old run baseline-s42-wandb`) | **1.272** / 1.240 / 1.304 | +0.26 / -0.20 | 0.189 | 0.76 | 0.175 | 1.7 % | 1.69 |
| baseline (`7rvis31p`) | **1.303** / 1.288 / 1.317 | +0.21 / -0.19 | 0.195 | 0.74 | 0.163 | 1.4 % | 1.58 |
| fast_trot (`ozyh16k2`) | **0.173** / 0.171 / 0.175 | +1.36 / -1.46 | 0.374 | 2.06 | 0.640 | 12.6 % | 3.48 |
| gallop_a (`dhz6j34z`) | **1.036** / 0.278 / 1.795 | +1.26 / +0.29 | 0.292 | 1.79 | 0.493 | 9.0 % | 2.96 |
| gallop_b (`biewl0aa`) | **1.070** / 0.231 / 1.910 | +1.32 / +0.41 | 0.346 | 1.89 | 0.462 | 9.8 % | 3.23 |
| gallop_c (`1c5r0n8c`) | **0.133** / 0.134 / 0.132 | +1.55 / -1.48 | 0.357 | 2.81 | 0.661 | 17.9 % | 3.23 |

| Arm | +0.30 | +0.75 | +1.00 | +1.50 | -0.30 | -0.75 | -1.00 | -1.50 |
|---|---|---|---|---|---|---|---|---|
| old baseline (recovered ckpt, bare profile) | trot (1.4 Hz, vx +0.23, fl 0%) | other* (3.1 Hz, vx +0.61, fl 0%) | other* (3.1 Hz, vx +0.59, fl 0%) | trot (1.4 Hz, vx +0.26, fl 0%) | trot* (2.1 Hz, vx -0.29, fl 0%) | trot* (3.3 Hz, vx -0.39, fl 0%) | trot (2.2 Hz, vx -0.34, fl 0%) | other (1.6 Hz, vx -0.20, fl 0%) |
| baseline | trot (1.4 Hz, vx +0.24, fl 0%) | other* (2.9 Hz, vx +0.46, fl 0%) | trot* (2.4 Hz, vx +0.42, fl 0%) | trot (1.4 Hz, vx +0.21, fl 0%) | trot* (3.1 Hz, vx -0.29, fl 0%) | trot* (2.9 Hz, vx -0.33, fl 0%) | trot (1.6 Hz, vx -0.29, fl 0%) | other (1.6 Hz, vx -0.19, fl 0%) |
| fast_trot | trot (1.4 Hz, vx +0.24, fl 0%) | trot (2.6 Hz, vx +0.74, fl 0%) | trot (2.6 Hz, vx +0.97, fl 2%) | trot (2.6 Hz, vx +1.36, fl 6%) | trot (1.4 Hz, vx -0.23, fl 0%) | trot (2.6 Hz, vx -0.74, fl 0%) | trot (2.6 Hz, vx -0.96, fl 0%) | trot (3.3 Hz, vx -1.46, fl 6%) |
| gallop_a | trot (1.4 Hz, vx +0.22, fl 0%) | bound (2.6 Hz, vx +0.77, fl 2%) | bound (2.6 Hz, vx +0.96, fl 6%) | bound* (2.6 Hz, vx +1.26, fl 24%) | trot (1.4 Hz, vx -0.22, fl 0%) | other (4.5 Hz, vx +0.33, fl 16%) | other (4.2 Hz, vx +0.33, fl 15%) | other (4.2 Hz, vx +0.29, fl 15%) |
| gallop_b | trot (1.4 Hz, vx +0.22, fl 0%) | rotary LH>RH>RF>LF (2.6 Hz, vx +0.73, fl 1%) | rotary LH>RH>RF>LF (2.6 Hz, vx +0.96, fl 1%) | rotary LH>RH>RF>LF (2.6 Hz, vx +1.32, fl 7%) | trot (1.4 Hz, vx -0.24, fl 0%) | other (2.8 Hz, vx +0.46, fl 10%) | other (2.8 Hz, vx +0.45, fl 10%) | other (2.6 Hz, vx +0.41, fl 10%) |
| gallop_c | trot (1.4 Hz, vx +0.23, fl 0%) | rotary LH>RH>RF>LF (3.1 Hz, vx +0.77, fl 0%) | rotary LH>RH>RF>LF* (4.2 Hz, vx +1.01, fl 0%) | other* (4.2 Hz, vx +1.55, fl 5%) | trot (1.6 Hz, vx -0.23, fl 0%) | rotary RH>LH>LF>RF (2.9 Hz, vx -0.76, fl 0%) | rotary RH>LH>LF>RF* (3.1 Hz, vx -1.00, fl 0%) | transverse RH>LH>RF>LF* (4.5 Hz, vx -1.48, fl 1%) |

| name | falls | rev | stop | steady | turn | err+1.5 | err-1.5 | sat@1.5 | F_runmax_med_BW | F_p99_BW | drift+ | drift- | splay+ | splay- | slip | top+ | top- | tau2 | R:push_stand@0.65 | R:push_move@0.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| p1-baseline-s42 | 1 | 0 | 0 | 1 | 0 | 1.3 | 1.31 | 0.0128 | 1.85 | 1.23 | 21.8 | 20.3 | -0.0466 | -0.137 | 0.185 | 0.589 | -0.463 | 0.634 | 0.0359 | 0.0942 |
| p1-fast_trot-s42 | 53 | 8 | 44 | 1 | 0 | 0.174 | 0.176 | 0.126 | 3.17 | 1.86 | 45 | 65.9 | -0.134 | 0.00108 | 0.353 | 1.48 | -1.86 | 1.02 | 0.681 | 0.00594 |
| p1-gallop_a-s42 | 69 | 2 | 53 | 14 | 0 | 0.275 | 1.76 | 0.0883 | 2.83 | 1.63 | 64.9 | 64.1 | -0.145 | -0.161 | 0.325 | 1.56 | -0.422 | 1 | 0.85 | 0.00494 |
| p1-gallop_b-s42 | 43 | 2 | 41 | 0 | 0 | 0.221 | 1.89 | 0.0977 | 2.96 | 1.68 | 63.7 | 73.7 | -0.1 | -0.114 | 0.311 | 1.54 | -0.422 | 1.05 | 0.54 | 0.00691 |
| p1-gallop_c-s42 | 44 | 2 | 41 | 1 | 0 | 0.13 | 0.146 | 0.179 | 3.03 | 1.88 | 67.6 | 65.7 | 0.0352 | 0.115 | 0.382 | 1.75 | -1.76 | 1.35 | 0.453 | 0.00197 |
| p1-gallop_c-s43 | 41 | 6 | 35 | 0 | 0 | 0.123 | 0.129 | 0.176 | 3.09 | 1.93 | 53.8 | 51.3 | 0.0469 | 0.106 | 0.382 | 1.69 | -1.72 | 1.37 | 0.00901 | 0 |
