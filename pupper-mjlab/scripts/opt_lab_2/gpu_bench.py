"""GPU bottleneck benchmark for the Pupper PPO training loop (report/infra/gpu_bottleneck.md).

Builds the exact training env + runner that ``pupper_locomotion train`` builds (Part 1 production
flags: fast_trot reference, splay weights, deployment clamp, driver limits, COM composition, torso
hull), then runs a copy of ``rsl_rl.OnPolicyRunner.learn`` with per-phase wall-clock timers. The
timers do not synchronize, so they show where the *host* spends its time; the loop's own sync
points (``check_nan``, ``nonzero`` in env.step, ``.item()`` in PPO.update) are left untouched.

Modes (combinable):
  default          N timed iterations, per-phase host time + process CPU time
  --profile-iters  extra K iterations under torch.profiler: kernel count, GPU busy time, launch and
                   sync API time
  --physics-bench  sim.step() alone in a synchronized loop (physics GPU ceiling)
  --mini-batches   override PPO num_mini_batches (learning-phase sensitivity only)

Writes one JSON with everything. Never trains a real run, writes no checkpoints.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics as st
import sys
import time
from dataclasses import asdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
PKG = REPO / "pupper-mjlab"


def build(a):
  os.environ["PUPPER_REFERENCE_DIR"] = str(a.reference_dir)
  import numpy as np  # noqa: F401
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import RslRlVecEnvWrapper
  from mjlab.scripts import pupper_locomotion as pl
  from mjlab.tasks.registry import load_runner_cls
  from mjlab.utils.torch import configure_torch_backends

  ns = argparse.Namespace(
    task=a.task, num_envs=a.num_envs, seed=a.seed, checkpoint=None, command="train",
    encoder_observation_mode=None, allow_encoder_observation_mode_change=False,
    deployment_target_clamp=True, driver_limits=REPO / "foundation/configs/class-driver-joint-limits.json",
    target_slew_rad_s=None, compose_com_randomization=True, locomotion_torso_contact=True,
    torso_contact_audit=False, nominal_mechanics=False, device="cuda:0")
  cfg, agent = pl.settings(ns, False)
  weights = json.loads(Path(a.weights).read_text())
  for r in cfg.rewards.values():
    r.weight = 0.
  for k, v in weights.items():
    cfg.rewards[k].weight = float(v)
  agent.max_iterations = a.iterations
  agent.logger = "tensorboard"
  agent.save_interval = 10**9
  if a.mini_batches:
    agent.algorithm.num_mini_batches = a.mini_batches
  configure_torch_backends()
  env = ManagerBasedRlEnv(cfg, device="cuda:0")
  wrapped = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
  a.out.mkdir(parents=True, exist_ok=True)
  runner = load_runner_cls(a.task)(wrapped, asdict(agent), str(a.out), "cuda:0")
  return env, wrapped, runner, agent


def timed_iterations(runner, n, rec=None):
  """Copy of OnPolicyRunner.learn's body with host-side phase timers (no added syncs)."""
  import torch
  from rsl_rl.utils import check_nan
  env, alg = runner.env, runner.alg
  steps = runner.cfg["num_steps_per_env"]
  obs = env.get_observations().to(runner.device)
  alg.train_mode()
  rows = []
  for _ in range(n):
    ph = dict(act=0., env_step=0., nan_check=0., process=0., returns=0., update=0.)
    c0, t0 = time.process_time(), time.perf_counter()
    with torch.inference_mode():
      for _ in range(steps):
        t = time.perf_counter(); actions = alg.act(obs)
        u = time.perf_counter(); ph["act"] += u - t
        obs, rewards, dones, extras = env.step(actions.to(env.device))
        t = time.perf_counter(); ph["env_step"] += t - u
        check_nan(obs, rewards, dones)
        u = time.perf_counter(); ph["nan_check"] += u - t
        obs, rewards, dones = obs.to(runner.device), rewards.to(runner.device), dones.to(runner.device)
        alg.process_env_step(obs, rewards, dones, extras)
        runner.logger.process_env_step(rewards, dones, extras, None)
        t = time.perf_counter(); ph["process"] += t - u
      collect = time.perf_counter() - t0
      t = time.perf_counter(); alg.compute_returns(obs); ph["returns"] = time.perf_counter() - t
    t = time.perf_counter(); alg.update(); ph["update"] = time.perf_counter() - t
    torch.cuda.synchronize()
    ph["iter"] = time.perf_counter() - t0
    ph["collect"] = collect
    ph["learn"] = ph["iter"] - collect
    ph["cpu"] = time.process_time() - c0
    rows.append(ph)
    if rec: rec.step()
  return rows


def physics_bench(env, n=200):
  import torch
  sim = env.sim
  for _ in range(10): sim.step()
  torch.cuda.synchronize(); t = time.perf_counter()
  for _ in range(n): sim.step()
  torch.cuda.synchronize(); dt = (time.perf_counter() - t) / n
  # one full env-step worth of the decimation loop incl. torch-side manager ops, synchronized
  ua = env.unwrapped
  def substeps():
    for _ in range(ua.cfg.decimation):
      ua.action_manager.apply_action(); ua.scene.write_data_to_sim(); ua.sim.step()
      ua.scene.update(dt=ua.physics_dt); ua.metrics_manager.compute_substep()
  for _ in range(3): substeps()
  torch.cuda.synchronize(); t = time.perf_counter()
  for _ in range(n // 5): substeps()
  torch.cuda.synchronize(); dl = (time.perf_counter() - t) / (n // 5)
  # forward + sense alone
  torch.cuda.synchronize(); t = time.perf_counter()
  for _ in range(n): sim.forward(); sim.sense()
  torch.cuda.synchronize(); df = (time.perf_counter() - t) / n
  return dict(sim_step_s=dt, decimation_loop_s=dl, forward_sense_s=df,
              use_cuda_graph=bool(getattr(sim, "use_cuda_graph", None) or getattr(sim, "_use_cuda_graph", None)))


def profile(runner, k):
  import torch
  from torch.profiler import ProfilerActivity, profile as tprof
  timed_iterations(runner, 1)
  with tprof(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA]) as prof:
    rows = timed_iterations(runner, k)
    torch.cuda.synchronize()
  wall = sum(r["iter"] for r in rows)
  ev = prof.events()
  dev_ints, launches, syncs, kernels_by_name = [], {}, {}, {}
  for e in ev:
    dt = getattr(e, "device_type", None)
    if dt is not None and dt.name == "CUDA" and e.device_time_total is not None:
      s = e.time_range.start; t = e.time_range.end
      if t > s:
        dev_ints.append((s, t))
        kernels_by_name.setdefault(e.name[:90], [0, 0.])
        kernels_by_name[e.name[:90]][0] += 1; kernels_by_name[e.name[:90]][1] += (t - s)
    else:
      n = e.name
      if any(x in n for x in ("LaunchKernel", "GraphLaunch", "cuLaunch", "LaunchCooperative")):
        launches.setdefault(n, [0, 0.]); launches[n][0] += 1; launches[n][1] += e.cpu_time_total
      if any(x in n for x in ("Synchronize", "Memcpy", "cudaStreamWait", "aten::item", "aten::_local_scalar_dense", "aten::nonzero")):
        syncs.setdefault(n, [0, 0.]); syncs[n][0] += 1; syncs[n][1] += e.cpu_time_total
  dev_ints.sort(); busy = 0.; cur_s = cur_e = None
  for s, t in dev_ints:
    if cur_e is None or s > cur_e:
      if cur_e is not None: busy += cur_e - cur_s
      cur_s, cur_e = s, t
    else: cur_e = max(cur_e, t)
  if cur_e is not None: busy += cur_e - cur_s
  top = sorted(kernels_by_name.items(), key=lambda kv: -kv[1][1])[:25]
  return dict(iters=k, wall_s=wall, gpu_busy_s=busy / 1e6, kernel_count=len(dev_ints),
              kernel_time_sum_s=sum(t - s for s, t in dev_ints) / 1e6,
              launch_api={k2: dict(n=v[0], cpu_s=v[1] / 1e6) for k2, v in launches.items()},
              sync_api={k2: dict(n=v[0], cpu_s=v[1] / 1e6) for k2, v in syncs.items()},
              top_kernels=[dict(name=k2, n=v[0], s=v[1] / 1e6) for k2, v in top],
              timed=rows)


def sync_sites(runner, k=1):
  """Count host<-device sync call sites (Tensor.item/__bool__/tolist/cpu/nonzero) for k iterations."""
  import collections
  import traceback
  import torch
  counts = collections.Counter()
  T = torch.Tensor
  orig = {n: getattr(T, n) for n in ("item", "__bool__", "tolist", "cpu", "nonzero")}
  def wrap(name, fn):
    def w(self, *a, **kw):
      if self.is_cuda:
        site = "?"
        for fr in reversed(traceback.extract_stack(limit=12)[:-1]):
          if "/torch/" not in fr.filename:
            site = f"{fr.filename.split('site-packages/')[-1].split('pupper-mjlab/')[-1]}:{fr.lineno} {fr.name}"
            break
        counts[f"{name} @ {site}"] += 1
      return fn(self, *a, **kw)
    return w
  for n, f in orig.items(): setattr(T, n, wrap(n, f))
  try:
    timed_iterations(runner, k)
  finally:
    for n, f in orig.items(): setattr(T, n, f)
  return [dict(site=s, per_iter=c / k) for s, c in counts.most_common(40)]


def cprofile_breakdown(runner, k=3):
  """cProfile k iterations; cumulative host time of the env-step pieces and the PPO update."""
  import cProfile
  import pstats
  pr = cProfile.Profile(); pr.enable(); rows = timed_iterations(runner, k); pr.disable()
  stt = pstats.Stats(pr); out = {}
  want = ("step", "compute", "_reset_idx", "apply", "apply_action", "write_data_to_sim", "sense", "forward",
          "update", "compute_substep", "process_action", "act", "process_env_step", "check_nan", "evaluate",
          "compute_returns", "_update_command", "launch", "capture_launch")
  for (fn, ln, name), (cc, nc, tt, ct, callers) in stt.stats.items():
    if name in want and ct / k > 0.004:
      short = fn.split("site-packages/")[-1].split("pupper-mjlab/")[-1]
      out[f"{short}:{ln} {name}"] = dict(calls=nc / k, cum_s=ct / k, self_s=tt / k)
  top_self = sorted(stt.stats.items(), key=lambda kv: -kv[1][2])[:30]
  return dict(iters=k, wall_per_iter=sum(r["iter"] for r in rows) / k,
              funcs=dict(sorted(out.items(), key=lambda kv: -kv[1]["cum_s"])),
              top_self=[dict(f=f"{fn.split('site-packages/')[-1].split('pupper-mjlab/')[-1]}:{ln} {nm}", calls=v[1] / k, self_s=v[2] / k)
                        for (fn, ln, nm), v in top_self])


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--num-envs", type=int, default=4096)
  p.add_argument("--iterations", type=int, default=30)
  p.add_argument("--warmup", type=int, default=5)
  p.add_argument("--mini-batches", type=int)
  p.add_argument("--profile-iters", type=int, default=0)
  p.add_argument("--physics-bench", action="store_true")
  p.add_argument("--sync-sites", action="store_true")
  p.add_argument("--cprofile", type=int, default=0)
  p.add_argument("--task", default="Mjlab-MixedGaits-Flat-Pupper-v3")
  p.add_argument("--reference-dir", type=Path, default=PKG / "configs/opt-lab-2/fast_trot")
  p.add_argument("--weights", type=Path, default=PKG / "configs/opt-lab-2/weights/splay.json")
  p.add_argument("--seed", type=int, default=42)
  p.add_argument("--out", type=Path, required=True)
  p.add_argument("--json", type=Path, required=True)
  a = p.parse_args()
  t_start = time.time()
  import torch
  env, wrapped, runner, agent = build(a)
  runner.logger.init_logging_writer()
  wrapped.episode_length_buf = torch.randint_like(wrapped.episode_length_buf, high=int(wrapped.max_episode_length))
  t_ready = time.time()
  res = dict(num_envs=a.num_envs, mini_batches=agent.algorithm.num_mini_batches,
             epochs=agent.algorithm.num_learning_epochs, steps_per_env=agent.num_steps_per_env,
             startup_s=t_ready - t_start, obs_dims={k: list(v.shape) for k, v in wrapped.get_observations().items()})
  timed_iterations(runner, a.warmup)
  res["timed_window_epoch"] = [time.time()]
  rows = timed_iterations(runner, a.iterations)
  res["timed_window_epoch"].append(time.time())
  res["rows"] = rows
  med = {k: st.median(r[k] for r in rows) for k in rows[0]}
  res["median"] = med
  res["env_steps_per_s"] = a.num_envs * agent.num_steps_per_env / med["iter"]
  res["max_mem_alloc_gb"] = torch.cuda.max_memory_allocated() / 1e9
  res["max_mem_reserved_gb"] = torch.cuda.max_memory_reserved() / 1e9
  print(json.dumps(dict(num_envs=a.num_envs, median=med, env_steps_per_s=res["env_steps_per_s"]), indent=1), flush=True)
  if a.physics_bench:
    res["physics"] = physics_bench(env)
    print(json.dumps(res["physics"]), flush=True)
  if a.profile_iters:
    res["profile_window_epoch"] = [time.time()]
    res["profile"] = profile(runner, a.profile_iters)
    pr = res["profile"]
    print(json.dumps({k: pr[k] for k in ("iters", "wall_s", "gpu_busy_s", "kernel_count", "kernel_time_sum_s")}), flush=True)
  if a.cprofile:
    res["cprofile"] = cprofile_breakdown(runner, a.cprofile)
  if a.sync_sites:
    res["sync_sites"] = sync_sites(runner)
    for r in res["sync_sites"][:20]: print(r, flush=True)
  a.json.parent.mkdir(parents=True, exist_ok=True)
  a.json.write_text(json.dumps(res, indent=1, default=str) + "\n")
  env.close()


if __name__ == "__main__":
  sys.exit(main())
