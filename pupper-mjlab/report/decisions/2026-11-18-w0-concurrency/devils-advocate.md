# Devil's-advocate review: max 4 concurrent 4096-env trainers
Reviewer: red-team agent, 2026-11-18 (retroactive). No GPU jobs run; CPU analysis of `~/runs/opt-lab-2` logs, checkpoint mtimes and queue logs only.

**Verdict: STANDS WITH NEW CAVEAT.** The cap is enforced by a racy check-then-launch with no shared lock, and it was breached once (5 trainers, 18:32:53–18:36:45). Separately, the 1.25 it/s budget figure is ~12–20 % optimistic for real runs.

## Strongest counter-case (tested, mostly refuted)
- **Claim tested:** concurrent per-run logs (P1 ≈ 2.6 s/iter at N=3, P2 ≈ 3.4 s/iter at N=4 → 1.15–1.17 it/s) are below the 0.78 s/iter (1.28 it/s) solo benchmark. That would make concurrency net-negative, so serial would be strictly better: same work, earlier first result, more memory headroom.
- **Why it fails:** the comparison is apples-to-oranges. The benchmark ran 40 iterations in the *pretrain-curriculum* phase. The production solo run `rp-p1-fast_trot-s43` (16:25–17:13, the only N=1 production run, no co-running trainer) ran 0.79 s/iter for iterations 0–499, then **0.87 s/iter** once `Curriculum/pretrain_rewards/active` drops to 0 at iteration 500. P1 arms show the same phase effect. Matched like for like:

| N trainers (production) | per-run "Iteration time" median | per-run **wall** s/iter (ckpt mtimes) | aggregate wall it/s, steady 10-min bins |
|---|---|---|---|
| 1 (fast_trot-s43) | 0.86 | 0.95 | 1.03–1.06 |
| 3 (P1 waves 11:12–13:40, 13:40–16:10) | 2.60–2.61 | 2.76–2.95 | 1.09–1.11 |
| 4 (P2, e.g. r1 wave 17:33–18:03) | 3.38–3.51 | 3.5–3.8 | 1.10–1.12 |
| 2 + 2 eval workers (20:20–20:43 tail) | 2.83 | 3.05 | 0.66 (training only) |

  The "Iteration time" (collection+learn) metric gives 1.16 / 1.15 / 1.17 it/s for N = 1/3/4, which is flat. On wall clock, concurrency is slightly *positive* (+4–7 %), because the untimed per-iteration overhead (W&B/logging/checkpointing, ≈ 0.08 s/iter solo) overlaps across processes. Serial would *lose* ~5 % throughput.
- **Latency argument, honestly scored:** serial gives the first of 4 results in ~9 min instead of ~30, but a matched comparison needs all 4 (§3 equal iterations + control), and serial delivers the 4th at ~34 min against ~30 min concurrent (4 × [0.95 × 500 + 40 s startup]). Serial wins only when a *single* run gates the next step (a lineage leg). The cap allows this ("ceiling, not target"), but nothing implements it: time-slicing ignores priority, so a critical leg co-running with 3 others gets 1/4 of the GPU. In practice the s42 chain r1→r2→r3→cont took 17:32→20:13 (2.7 h), as the grader predicted (2.5–3 h).
- **Memory:** not binding. 4 × 3.7 GB + evals ≈ 17 GB of 40 GB, and even the 5-trainer breach was ≈ 20 GB. 0.9 GB is in use now. The argument for serial "headroom" has no force.

## What the grader missed
1. **Enforcement is fragmented and racy.** No script reads `~/recovery/CONCURRENCY`. Instead "4" is a duplicated CLI default: `part2_queue.py --max-gpu 4` and `part3_run.py --max-gpu-procs 4`, and `part1_queue.py --max-concurrent` counted only its *own* jobs (run with 3). part2/part3 count `nvidia-smi` PIDs and then launch, but a new process registers a CUDA context only ~40 s later, and `part3_run.py` does not take W3's `launch.lock`. **Observed breach:** P2 launched `s43-r1-naive` at 18:32:42 and P3 launched `bumpyft-smoke` at 18:32:53. Together with 3 running P2 trainers that made 5 trainers until 18:36:45 (smoke ran 4.5 s/iter). It was harmless this time, but this exact path defeats the cap.
2. **Doc/code mismatch:** the decision says evals are "outside the cap", but both enforcing scripts count every compute PID, evals included. The effective policy is "4 GPU contexts total", which is stricter and arguably better given the 27 % per-context eval tax.
3. **Budget constant:** "total iterations / 1.25" understates real time. Steady production is 1.05–1.12 it/s wall, and the whole P1+P2 span averaged 0.94 it/s including idle gaps (16:10–16:24, 17:13–17:32). Part 1 was ~21 k iterations in 6.0 h, against 4.7 h at 1.25. Plan with **~1.1 it/s**.
4. The benchmark's 40-iteration window sits entirely in the cheaper pretrain phase. The `a100_throughput.md` solo number should carry that caveat.

## Flip conditions status
- Aggregate changes > ~10 % with N: **not met** (+4–7 % in favour of N ≥ 3 on wall clock, flat on timed).
- Repeated co-scheduled faults: **not met** (32/32 runs `exit code 0`, 0 resumes in the P1 status files).
- 8192 envs adopted: **not met** (all 32 launch.json have `--num-envs 4096`).
- Fewer Part 2 lanes needed: **not met** (both seeds ran, 25/25 queue jobs done 20:44:27).

## Cost of reopening
Re-deciding the number buys nothing, since no data argues for 3, 5 or 1. The real fix is mechanical and does not need a decision cycle: make part2_queue/part3_run (and any W4 fine-tune launcher) read `~/recovery/CONCURRENCY` and hold one shared `flock` across check + launch + ~60 s grace (or count pending launches). Changing the budget constant to 1.1 it/s is a one-line edit in REPRO_PLAN §4.3 / a100_throughput.md. Both cost ~30 min of agent time and no GPU.

## Bottom line
"Max 4" is the right number. The new production data confirm flat throughput and refute the claim that concurrency is net-negative (that claim came from a phase-mismatched baseline). The decision's weak point is not the number but its enforcement, which is per-script, check-then-launch and unlocked, and has already let 5 trainers run at once.
