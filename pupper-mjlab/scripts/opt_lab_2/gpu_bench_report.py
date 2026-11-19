"""Summarize a gpu_bench_sweep root into tables + a plot (report/infra/gpu_bottleneck.md)."""
from __future__ import annotations

import json
import re
import statistics as st
import sys
from pathlib import Path


def dmon_window(path, t0, t1):
  """Median sm%, mem%, power W, pclk, rx/tx MB/s over dmon lines whose epoch is inside [t0, t1]."""
  cols = None; rows = []
  for ln in Path(path).read_text().splitlines():
    parts = ln.split()
    if len(parts) < 3: continue
    if parts[1] == "#" and parts[2] == "gpu":
      cols = parts[2:]; continue
    if parts[1].startswith("#") or cols is None: continue
    try: ts = float(parts[0]); vals = parts[1:]
    except ValueError: continue
    if t0 <= ts <= t1 and len(vals) == len(cols):
      rows.append(dict(zip(cols, vals)))
  def med(k):
    v = [float(r[k]) for r in rows if r.get(k, "-") not in ("-", "")]
    return st.median(v) if v else float("nan")
  return dict(n=len(rows), sm=med("sm"), mem=med("mem"), pwr=med("pwr"), pclk=med("pclk"),
              rx=med("rxpci"), tx=med("txpci"))


def pidstat_main(path):
  """Median CPU% of the busiest thread and of the whole process (lines with '-' TID are process totals)."""
  proc, threads = [], {}
  for ln in Path(path).read_text().splitlines():
    f = ln.split()
    # time UID TGID TID %usr %system %guest %wait %CPU CPU Command
    if len(f) < 11 or not f[1].isdigit(): continue
    tgid, tid = f[2], f[3]
    try: pct = float(f[8])
    except ValueError: continue
    if tid == "-": proc.append(pct)
    else: threads.setdefault(tid, []).append(pct)
  if not proc: return {}
  busiest = max(threads.values(), key=lambda v: st.median(v)) if threads else []
  ranked = sorted(((st.median(v), t) for t, v in threads.items()), reverse=True)[:4]
  # steady-state = upper half of samples (drop startup/compile)
  return dict(proc_med=st.median(sorted(proc)[len(proc) // 2:]), busiest_thread_med=st.median(busiest) if busiest else None,
              top_threads=[round(x, 1) for x, _ in ranked])


def main(root):
  root = Path(root)
  out = []
  for j in sorted(root.glob("*.json"), key=lambda q: (len(q.stem), q.stem)):
    if j.name == "summary.json": continue
    d = json.loads(j.read_text()); tag = j.stem
    m = d["median"]
    rec = dict(tag=tag, num_envs=d["num_envs"], mb=d["mini_batches"], iter=m["iter"], collect=m["collect"],
               learn=m["learn"], act=m["act"], env_step=m["env_step"], nan=m["nan_check"], process=m["process"],
               update=m["update"], cpu=m["cpu"], sps=d["env_steps_per_s"], mem_gb=d["max_mem_reserved_gb"],
               startup=d["startup_s"])
    base = tag.rsplit("-", 1)[0] if re.search(r"-\d+$", tag) else tag
    if (root / f"{base}.dmon").exists():
      rec["dmon"] = dmon_window(root / f"{base}.dmon", *d["timed_window_epoch"])
    if (root / f"{tag}.pidstat").exists():
      rec["pidstat"] = pidstat_main(root / f"{tag}.pidstat")
    if (root / f"{base}.apps").exists():
      t0, t1 = d["timed_window_epoch"]; seen = set()
      for ln in (root / f"{base}.apps").read_text().splitlines():
        ts, rest = ln.split(" ", 1)
        if t0 <= float(ts) <= t1: seen.add(rest)
      rec["apps_seen"] = sorted(seen)
    if "physics" in d: rec["physics"] = d["physics"]
    if "profile" in d:
      p = d["profile"]; k = p["iters"]
      rec["prof"] = dict(wall=p["wall_s"] / k, gpu_busy=p["gpu_busy_s"] / k, kernels=p["kernel_count"] / k,
                         launches={a: v["n"] / k for a, v in p["launch_api"].items()},
                         launch_cpu_s={a: v["cpu_s"] / k for a, v in p["launch_api"].items()},
                         sync={a: v["n"] / k for a, v in p["sync_api"].items()},
                         sync_cpu_s={a: v["cpu_s"] / k for a, v in p["sync_api"].items()},
                         top=p["top_kernels"][:8])
    out.append(rec)
  (root / "summary.json").write_text(json.dumps(out, indent=1) + "\n")
  for r in out:
    dm = r.get("dmon", {})
    print(f"{r['tag']:>12} N={r['num_envs']:>6} mb={r['mb']} iter={r['iter']:.3f} coll={r['collect']:.3f} learn={r['learn']:.3f} "
          f"cpu={r['cpu']:.3f} sps={r['sps']/1e3:.0f}k mem={r['mem_gb']:.1f} sm={dm.get('sm')} memutil={dm.get('mem')} "
          f"pwr={dm.get('pwr')} pid={r.get('pidstat',{})} busy={r.get('prof',{}).get('gpu_busy')} ")
  return out


if __name__ == "__main__":
  main(sys.argv[1])


def plot(root, png):
  """Two panels, one y-axis each: (a) env-steps/s vs num_envs, (b) s/iter split vs num_envs."""
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  recs = [r for r in json.loads((Path(root) / "summary.json").read_text()) if re.fullmatch(r"e\d+", r["tag"])]
  recs.sort(key=lambda r: r["num_envs"])
  N = [r["num_envs"] for r in recs]
  steps = 24
  wall = [r["iter"] for r in recs]
  busy = [r["prof"]["gpu_busy"] for r in recs]
  phys = [r["physics"]["sim_step_s"] * 5 * steps for r in recs]
  host_floor = 0.66
  ink, ink2, grid, surf = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
  c1, c2, c3 = "#2a78d6", "#eb6834", "#1baf7a"
  fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.2), facecolor=surf)
  for ax in (a, b):
    ax.set_facecolor(surf); ax.set_xscale("log", base=2); ax.set_xticks(N); ax.set_xticklabels([f"{n//1024}k" for n in N])
    ax.grid(True, color=grid, lw=0.8); ax.set_axisbelow(True)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    for s in ("left", "bottom"): ax.spines[s].set_color(ink2)
    ax.tick_params(colors=ink2); ax.set_xlabel("num_envs (one process)", color=ink2)
  sps = [n * steps / w / 1e3 for n, w in zip(N, wall)]
  ceil = [n * steps / g / 1e3 for n, g in zip(N, busy)]
  hostc = [n * steps / host_floor / 1e3 for n in N]
  a.plot(N, sps, "-o", color=c1, lw=2, ms=8, mec=surf, mew=2, label="measured")
  a.plot(N, ceil, "--o", color=c2, lw=2, ms=8, mec=surf, mew=2, label="if GPU were never idle (N·24 / GPU-busy s)")
  a.plot(N, hostc, ":", color=ink2, lw=1.5, label="if GPU were free (N·24 / 0.66 s host floor)")
  for n, v in zip(N, sps):
    a.annotate(f"{v:.0f}k", (n, v), textcoords="offset points", xytext=(6, -14), ha="left", color=ink, fontsize=9)
  a.axvline(4096, color=ink2, lw=0.8); a.text(4096, 560, " production (4096)", color=ink2, fontsize=8, va="top")
  a.set_ylim(0, 700); a.set_ylabel("env-steps/s (thousands)", color=ink2)
  a.set_title("(a) Throughput vs num_envs", color=ink, loc="left", fontsize=11)
  a.legend(frameon=False, fontsize=8, loc="upper left", labelcolor=ink)
  b.plot(N, wall, "-o", color=c1, lw=2, ms=8, mec=surf, mew=2, label="wall s/iter (measured)")
  b.plot(N, busy, "--o", color=c2, lw=2, ms=8, mec=surf, mew=2, label="GPU busy s/iter (profiler, kernel union)")
  b.plot(N, phys, "-o", color=c3, lw=2, ms=8, mec=surf, mew=2, label="physics alone s/iter (120 graph replays)")
  b.axhline(host_floor, color=ink2, lw=1, ls=":"); b.text(1024, host_floor - 0.08, "host floor ≈ 0.66 s", color=ink2, fontsize=8, ha="left")
  for n, w in zip(N, wall):
    b.annotate(f"{w:.2f}", (n, w), textcoords="offset points", xytext=(0, 9), ha="center", color=ink, fontsize=9)
  b.set_ylim(0, 2.1); b.set_ylabel("seconds per PPO iteration", color=ink2)
  b.set_title("(b) Where an iteration's time goes", color=ink, loc="left", fontsize=11)
  b.legend(frameon=False, fontsize=8, loc="upper left", labelcolor=ink)
  fig.tight_layout(); fig.savefig(png, dpi=130, facecolor=surf)


if __name__ == "__main__" and len(sys.argv) > 2:
  plot(sys.argv[1], sys.argv[2])
