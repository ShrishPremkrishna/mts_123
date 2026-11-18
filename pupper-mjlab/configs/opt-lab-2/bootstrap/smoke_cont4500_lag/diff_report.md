# Reference diff: `smoke_cont4500_lag` vs parent

## reach_fwd

| channel | rms diff | max abs diff | mean offset | parent range | new range | note |
|---|---|---|---|---|---|---|
| leg_front_r_1 | 0.123 | 0.271 | -0.006 | 1.244 | 1.202 |  |
| leg_front_r_2 | 0.170 | 0.222 | -0.160 | 0.147 | 0.109 | **near-constant channel shifted** |
| leg_front_r_3 | 0.209 | 0.422 | -0.052 | 1.478 | 1.423 |  |
| leg_front_l_1 | 0.143 | 0.363 | +0.014 | 1.248 | 1.124 |  |
| leg_front_l_2 | 0.158 | 0.217 | +0.147 | 0.147 | 0.121 | **near-constant channel shifted** |
| leg_front_l_3 | 0.200 | 0.448 | +0.040 | 1.478 | 1.468 |  |
| leg_back_r_1 | 0.121 | 0.295 | -0.019 | 1.327 | 1.115 |  |
| leg_back_r_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_r_3 | 0.167 | 0.399 | -0.039 | 1.732 | 1.444 |  |
| leg_back_l_1 | 0.134 | 0.317 | +0.010 | 1.325 | 1.105 |  |
| leg_back_l_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_l_3 | 0.174 | 0.359 | +0.014 | 1.732 | 1.386 |  |

## reach_back

| channel | rms diff | max abs diff | mean offset | parent range | new range | note |
|---|---|---|---|---|---|---|
| leg_front_r_1 | 0.078 | 0.223 | -0.040 | 1.244 | 0.987 |  |
| leg_front_r_2 | 0.236 | 0.309 | -0.220 | 0.147 | 0.199 | **near-constant channel shifted** |
| leg_front_r_3 | 0.113 | 0.293 | -0.072 | 1.478 | 1.137 |  |
| leg_front_l_1 | 0.053 | 0.160 | +0.021 | 1.248 | 1.050 |  |
| leg_front_l_2 | 0.173 | 0.244 | +0.164 | 0.147 | 0.129 | **near-constant channel shifted** |
| leg_front_l_3 | 0.094 | 0.278 | +0.041 | 1.478 | 1.187 |  |
| leg_back_r_1 | 0.048 | 0.138 | -0.024 | 1.327 | 1.111 |  |
| leg_back_r_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_r_3 | 0.078 | 0.227 | +0.025 | 1.732 | 1.364 |  |
| leg_back_l_1 | 0.050 | 0.152 | +0.026 | 1.325 | 1.086 |  |
| leg_back_l_2 | 0.000 | 0.000 | +0.000 | 0.006 | 0.006 | pinned |
| leg_back_l_3 | 0.078 | 0.248 | -0.003 | 1.732 | 1.353 |  |
