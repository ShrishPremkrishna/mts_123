# A100 training throughput: concurrency and perf options

Measured 2026-11-18 on 1× A100-PCIe-40GB (driver 570.195.03, 250 W, max SM clock 1410 MHz), 30 CPUs. Stack: torch 2.9.0+cu128,
warp 1.14.0, mujoco-warp 3.10.0.2. Workstream W0. Sim-only infrastructure evidence.

## Setup

- Command, identical for every process (only `--out` and `--seed` vary):
  `python -m mjlab.scripts.pupper_locomotion train --profile baseline --num-envs 4096 --iterations 40
  --logger tensorboard --deployment-target-clamp --compose-com-randomization --locomotion-torso-contact
  --driver-limits ../foundation/configs/class-driver-joint-limits.json`. Task `Mjlab-MixedGaits-Flat-Pupper-v3`.
- N independent processes are started at the same moment. They are plain separate CUDA contexts, **no MPS**.
- One iteration is 4096 envs × 24 steps = 98,304 env steps.
- s/iter is `Perf/collection_time + Perf/learning_time` from each run's tfevents. Only iterations ≥ 3 that fall inside
  the window where all N processes were iterating are counted (steady state; startup and compile excluded). The table
  reports the median over all processes. Aggregate = N × 98,304 / median s/iter.
- GPU memory is the peak per-process `nvidia-smi --query-compute-apps` value, polled every 2 s.
- **Contamination:** other agents' small `evaluate` processes (≈570 MiB) ran on the GPU at times. Iterations
  that coincided with a foreign PID are excluded from the "clean" column. Only N=2 was seriously affected (14 clean samples).
- Scripts and raw logs: scratch only (not committed). They were `run_bench.sh` and `analyze.py` in the W0 session scratchpad.

## Results (4096 envs per process)

| N procs | s/iter per process (median) | aggregate it/s | aggregate env-steps/s | collection / learning s | GPU util (nvidia-smi) | GPU mem / proc |
|---|---|---|---|---|---|---|
| 1 | **0.78** | 1.29 | **127 k** | 0.59 / 0.19 | ~45 % | 3.7 GB |
| 2 | 1.72 (clean, n=14); 2.30 with a foreign eval present | 1.16 | 114 k | — | ~92 % | 3.7 GB |
| 3 | 2.46 | 1.22 | 120 k | 2.20 / 0.26 | ~95 % | 3.7 GB |
| 4 | 3.25 | 1.23 | 121 k | 2.95 / 0.30 | ~94 % | 3.7 GB |
| 6 | 4.76 | 1.26 | 124 k | 4.40 / 0.37 | ~94 % | 3.7 GB |

Startup (process launch → first iteration) is 34–42 s, with the warp kernel cache warm.

**Main finding: aggregate throughput is flat at ≈ 1.2–1.3 iterations/s (≈ 120–127 k env-steps/s) for any N
from 1 to 6.** Concurrency on this GPU only divides that fixed rate between runs. A single process leaves the GPU about
55 % idle (host/launch-bound). Extra processes fill the utilization counter up to ~94 %, but the CUDA time-slicing between contexts
uses up the gain. Overlapping kernels from different processes would need MPS, which we have ruled out (fault isolation).
Memory is not a constraint: about 10 processes would fit in 40 GB.

Side effect worth knowing: a **small co-scheduled eval process costs training disproportionately**. With one
~570 MiB `evaluate` process present, a solo 4096-env training ran at a median 1.07 s/iter instead of 0.78 (−27 %).
In the 2-process run, each process fell from 1.72 to 2.30 s/iter. Time-slicing is roughly per context, not per amount of work.

## Hardware-specific options tried

| Option | Status in code | Measured | Verdict |
|---|---|---|---|
| TF32 matmul/cuDNN | **Already on.** `pupper_locomotion.train` calls `mjlab.utils.torch.configure_torch_backends()` (default `allow_tf32=True` → `torch.backends.cuda.matmul.fp32_precision="tf32"` on torch ≥ 2.9) | Forced off (IEEE): 0.80 s/iter (clean samples) vs 0.78 on. The difference is ≤ 3 % | Nothing to change. PPO's 256-128-128-128 MLP is too small for TF32 to matter |
| mujoco-warp CUDA graph capture | **Already on.** `Simulation._should_use_cuda_graph()` (`src/mjlab/sim/sim.py:560`) needs warp mempool and driver ≥ 12.4. Driver 12.8 qualifies, and no "CUDA Graphs disabled" warning appears in any log. `step`/`forward`/`reset`/`sense` are captured | n/a | Nothing to change |
| NVIDIA MPS | Ruled out (REPRO_PLAN §4.2: a fault in one run killed all co-scheduled runs) | not tried | Rejected by policy |
| `--num-envs 8192` (one process) | Plan marks this as a human decision; **not adopted**, since it changes the PPO batch and comparability with recovered runs | **0.90 s/iter → 8192 × 24 / 0.90 ≈ 218 k env-steps/s, 1.7× the 4096-env rate** (n=17 iterations before a foreign process arrived), 6.7 GB | Reported only. This is the only large lever found. See below |

**No code change was kept.** The two "cheap" options are already enabled. The diff would be zero.

The 8192 result shows the bottleneck is per-step host overhead, not GPU compute: doubling the envs costs +15 % time.
A larger batch per process is therefore the real throughput lever, but it changes PPO numerics (batch 98 k → 197 k
samples per update, minibatch size doubles). It is **not** comparable with the recovered 4096-env runs and needs a [H] decision
(REPRO_PLAN §4.4). If someone wants it, the comparable test is: baseline at 8192 envs for 1500 iterations vs the
recovered baseline at 3000, at matched env-steps.

## Recommendation

- Throughput is about **1.27 it/s for the whole GPU** at 4096 envs. Budget compute as `total iterations / 1.27 s`.
  For example, 6 × 3000-iteration runs ≈ 3.9 h whether they run one at a time or all at once.
- Concurrency does not buy throughput. It trades time-to-first-result against arms finishing together and against
  fault/startup isolation. Recommended maximum concurrency: see `~/recovery/CONCURRENCY` and the decision doc
  `report/decisions/2026-11-18-w0-concurrency.md`.
- Run evals sparingly while training is running, or batch them, since each extra CUDA context takes a time slice.
