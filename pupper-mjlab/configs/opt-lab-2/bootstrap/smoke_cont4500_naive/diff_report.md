# Reference diff: `smoke_cont4500_naive` vs parent

## reach_fwd

| channel | rms diff | max abs diff | mean offset | parent range | new range | note |
|---|---|---|---|---|---|---|
| leg_front_r_1 | 0.213 | 0.472 | -0.007 | 1.244 | 1.205 |  |
| leg_front_r_2 | 0.170 | 0.222 | -0.160 | 0.147 | 0.109 | **near-constant channel shifted** |
| leg_front_r_3 | 0.351 | 0.742 | -0.051 | 1.478 | 1.421 |  |
| leg_front_l_1 | 0.208 | 0.421 | +0.016 | 1.248 | 1.126 |  |
| leg_front_l_2 | 0.159 | 0.219 | +0.148 | 0.147 | 0.122 | **near-constant channel shifted** |
| leg_front_l_3 | 0.341 | 0.688 | +0.041 | 1.478 | 1.470 |  |
| leg_back_r_1 | 0.220 | 0.468 | -0.018 | 1.327 | 1.117 |  |
| leg_back_r_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_r_3 | 0.325 | 0.728 | -0.037 | 1.732 | 1.444 |  |
| leg_back_l_1 | 0.206 | 0.393 | +0.012 | 1.325 | 1.106 |  |
| leg_back_l_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_l_3 | 0.344 | 0.792 | +0.012 | 1.732 | 1.387 |  |

## reach_back

| channel | rms diff | max abs diff | mean offset | parent range | new range | note |
|---|---|---|---|---|---|---|
| leg_front_r_1 | 0.181 | 0.471 | -0.041 | 1.244 | 0.986 |  |
| leg_front_r_2 | 0.233 | 0.308 | -0.220 | 0.147 | 0.198 | **near-constant channel shifted** |
| leg_front_r_3 | 0.242 | 0.492 | -0.070 | 1.478 | 1.138 |  |
| leg_front_l_1 | 0.178 | 0.461 | +0.020 | 1.248 | 1.050 |  |
| leg_front_l_2 | 0.171 | 0.244 | +0.164 | 0.147 | 0.129 | **near-constant channel shifted** |
| leg_front_l_3 | 0.226 | 0.487 | +0.041 | 1.478 | 1.186 |  |
| leg_back_r_1 | 0.187 | 0.417 | -0.024 | 1.327 | 1.112 |  |
| leg_back_r_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_r_3 | 0.246 | 0.693 | +0.026 | 1.732 | 1.365 |  |
| leg_back_l_1 | 0.189 | 0.444 | +0.027 | 1.325 | 1.085 |  |
| leg_back_l_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_l_3 | 0.260 | 0.699 | -0.004 | 1.732 | 1.354 |  |
