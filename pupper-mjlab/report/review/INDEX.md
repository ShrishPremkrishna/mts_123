# Things to look at when you have time

Nothing here blocks the work. Each item says what a machine or subagent concluded, so you can check the ones you care about. Visual verdicts are an agent's judgment from sampled frames. Human verdicts are from driving the policy interactively in the viewer.

## Needs you eventually

<!-- RECOVERY GAP: line 7 of the 3219d63 version is not in any W&B diff -->
<!-- RECOVERY GAP: line 8 of the 3219d63 version is not in any W&B diff -->
| Push access: `gh auth login`, then `gh auth setup-git` | Commits on `opt-lab-2` stay on this machine until then | Open |
| Second training seed (seed 43) of baseline and `fast_long` | Checks the Part 1 result isn't a single-seed accident | Done: result reproduces (`../tables/two_training_seeds.md`) |

## Human review of `fast_long` (2026-11-09)

Driven by hand in the browser viewer, `fast_long-s42` at iteration 3000, same physics settings as training.

| Observation | Human verdict | Measurement |
|---|---|---|
| Overall | Fairly strong policy; it really runs. The issues below are kinks to work out, not fundamental problems, but they should be expected to show up in sim-to-real transfer | See Part 1 notebook |
| Holding still | Feet and body jitter a lot; unclear whether real or a simulation artifact | Real policy output, not rendering: see "Measured follow-ups" below |
| 2 to 2.5 m/s (beyond the ±1.5 training range) | Running looks desperate; left and right sides look misaligned; probably dangerous for the real robot | See "Measured follow-ups" below |
| Instant +1.5 to -1.5 m/s reversal | A robot ends up on one leg and is about to fall, so the environment resets | See "Measured follow-ups" below |
| Stopping from speed vs reversing | Stopping is much less of a problem than instantly reversing along the same axis | See "Measured follow-ups" below |

## To review

| Question | Machine or subagent view | Files | Your note |
|---|---|---|---|
| Is `fast_long` really running, or skating? | Looks fine both ways: plant and lift, level body, no falls (agent). Slip 0.53 m/s at +1.5 m/s is still high (metrics) | `../media/policy_fast_long/` | Really runs; fairly strong |
| Does `fast_trot` look worth its falls? | Most regular gait to the eye, but 36 falls at +1.5 m/s, mostly within 1 s of a reset. The short clips showed none | `../media/policy_fast_trot/` | |
| `fast_wide` backward | Odd: a front leg thrown high and outward, body rocking (agent); worst backward tracking of the fast gaits | `../media/policy_fast_wide/seg01_vx-1.50_vy+0.00_wz+0.00.mp4` | |
| Baseline continued to 4500 iterations, +1.5 m/s | Looks tidy, but short steps for 1.49 m/s; may be skating (agent). Slip 0.645 m/s (metrics) | `../media/policy_baseline4500/` | |
| Reference tables before training | Nothing broken; `fast_trot` and `fast_stagger` swing clearance looked modest | `references/verdict.md` | |
| Gallop references (`fast_gallop_a`, `b`, `c`) before training | Footfall correct in all three: forward LB, BR, FR, FL; backward is the exact time reversal (checked from the npz). Played open loop on the ground, `a` forward tips over and every forward run drags the left hind foot; `c` needs the most joint speed (agent and probe) | `part1-gallop/reference_verdict.md` | |
| Will any of this transfer? | Fast gaits put about 11% of torque samples at the 3 Nm limit, as does the stock trot once it runs that fast | `../part1_notebook.md` | Kinks will show up in transfer: standing jitter, reversal, over-speed |

Full visual notes: [policies/verdict.md](policies/verdict.md), [references/verdict.md](references/verdict.md).

## Measured follow-ups

Filled in from rollouts in `../data/evals/` (names starting `rev-` and `over-`).

<!-- RECOVERY NOTE: in the 3219d63+uncommitted state the file ended here (hunk reached EOF). Later committed versions (blob 0984844, from dd213ab on) differ somewhere not covered by any W&B diff; only the gallop row above is known to have been added after. -->
