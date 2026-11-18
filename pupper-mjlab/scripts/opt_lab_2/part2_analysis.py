"""Opt Lab 2 Part 2 analysis (simulation, CPU only): matched tables, paired deltas, reward terms.

Reads the W2 eval JSONs (`<eval-dir>/<arm>.json` + `<arm>_push.json`, written by
src/mjlab/scripts/part2_eval.py) and the training run dirs (`<runs>/rp-<arm>[-rK]/`: provenance.json,
model_<iter>.pt, events*). Arms without an eval JSON are skipped with a note; rerun as legs finish.

  uv run --no-sync python scripts/opt_lab_2/part2_analysis.py [all|matched|rewards] [--out-dir DIR]
  uv run --no-sync python scripts/opt_lab_2/part2_analysis.py readouts EVAL.json [...]

matched  Walk each evaluated arm's warm-start chain (provenance `checkpoint`, merging `-rK` crash-resume
         segments into one leg) back to the Part 1 root; cumulative PPO iterations past the root = sum over
         legs of (end_iter - parent_iter + 1). HARD ASSERTIONS (exit 2 after writing chain_check.json):
         every leg complete (= its --iterations), num_envs 4096 and seed s on every leg and the root, root =
         rp-p1-gallop_c-s<s>/model_2999.pt, cumulative = the arm's level, eval `training` block consistent
         with the chain; within a (seed, level) row identical num_envs/cumulative/seed/root; across seeds at
         a level identical num_envs/cumulative. Then part2_eval.compare per (seed, level)
         -> matched_s<s>_plus<L>.md (+ matched_parents.md), and matched_summary.{md,json}: per level, both
         seeds' values, arm - same-seed control deltas, two-seed means, sign agreement.
rewards  tfevents along each arm's chain (root's last 50 iterations onward), x = iterations past root:
         reward_terms_last50.csv (mean of the last 50 iterations of each arm's own final leg) and
         reward_terms_s<s>.png (one panel per key term, one line per arm over its own leg, 25-iteration
         trailing rolling mean; the first 25 iterations of every leg/resume segment are dropped because the
         episode statistics refill after the all-env reset at launch). Works on in-progress runs.
readouts Print the readouts extracted from arbitrary eval JSONs (no chain check).

Readout -> JSON key (suite `metrics.`, push `metrics.`), extracted via part2_eval.SUITE_COLUMNS /
_push_columns so they match `compare` exactly: see READOUT_KEYS below.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

from mjlab.scripts import part2_eval as pe

REPO = Path(__file__).resolve().parents[2]
RUNS = Path.home() / "runs/opt-lab-2"
SEEDS = (42, 43)
NUM_ENVS = 4096
ROOT_TEMPLATE = "rp-p1-gallop_c-s{s}/model_2999.pt"
LEVELS = {
  0: ["p1-gallop_c-s{s}"],
  500: ["p2-s{s}-r1-naive", "p2-s{s}-r1-lag", "p2-s{s}-r1-backpin", "p2-s{s}-ctrl-500"],
  1000: ["p2-s{s}-r2", "p2-s{s}-r1-cont-1000", "p2-s{s}-ctrl-1000"],
  1500: ["p2-s{s}-r3", "p2-s{s}-r2-cont-1500", "p2-s{s}-r1-cont-1500", "p2-s{s}-ctrl-1500"],
}
RANKED = {"falls", "rev", "stop", "steady", "turn", "F_runmax_med_BW"}
FALL_LABELS = ("falls", "rev", "stop", "steady", "turn", "falls<2s_reset")
SUITE_ORDER = ("falls", "rev", "stop", "steady", "turn", "falls<2s_reset", "err±1.5", "err+1.5", "err-1.5",
               "vx+1.5", "vx-1.5", "top+", "top-", "F_runmax_med_BW", "F_p99_BW", "F_p999_BW",
               "F_peak_BW(flag)", "sat@1.5", "sat", "drift+", "drift-", "splay+", "splay-", "jitter",
               "slip", "tau2")
PUSH_ORDER = tuple(c for c, _ in pe._push_columns() if not c.endswith("(i)") and c != "n_push")
READOUTS = SUITE_ORDER + PUSH_ORDER
assert all(c in dict(pe.SUITE_COLUMNS) for c in SUITE_ORDER if c != "err±1.5"), "part2_eval columns changed"
_RUNG, _MR = pe.PUSH_STANDING_RUNG, str(pe.PUSH_MOVING_RUNG)
READOUT_KEYS = {
  "falls": "falls_total", "rev": "falls_by_category.reversal", "stop": "falls_by_category.stop",
  "steady": "falls_by_category.steady", "turn": "falls_by_category.turn",
  "falls<2s_reset": "falls_soon_after_reset", "err±1.5": "mean(track_err_fwd, track_err_back)",
  "err+1.5": "track_err_fwd", "err-1.5": "track_err_back", "vx+1.5": "mean_vx_fwd", "vx-1.5": "mean_vx_back",
  "top+": "top_speed_fwd", "top-": "top_speed_back", "F_runmax_med_BW": "foot_force_runmax_median_bw",
  "F_p99_BW": "foot_force_p99_bw", "F_p999_BW": "foot_force_p999_bw", "F_peak_BW(flag)": "foot_force_peak_bw",
  "sat@1.5": "torque_sat_frac_fast", "sat": "torque_sat_frac", "drift+": "heading_drift_deg_per_10s_fwd.mean",
  "drift-": "heading_drift_deg_per_10s_back.mean", "splay+": "hind_splay_rad_fwd",
  "splay-": "hind_splay_rad_back", "jitter": "stand_jitter_rad_s", "slip": "slip_m_s", "tau2": "torque_sq_mean",
  f"R:push_stand@{_RUNG}": f"push: rank_push(metrics).standing_at_rung.rate = by_condition.stand.{_RUNG}.rate",
  f"R:push_move@{_MR}": f"push: rank_push(metrics).moving_at_rung.rate (pooled by_condition.<moving>.{_MR})",
  **{f"vx_{n}": f"push: by_condition.{n}.achieved_vx_m_s" for n in pe._MOVING},
  **{f"push@{_MR}_{n}": f"push: by_condition.{n}.{_MR}.rate" for n in pe._MOVING},
}
KEY_TERMS = ("gait_tracking", "track_linear_velocity", "track_yaw_velocity", "stand_still_pose",
             "stand_still_joint_velocity", "action_rate_l2", "abduction_angle", "roll_asymmetry",
             "error_vel_xy", "fell_over")
METRIC_TAGS = {"error_vel_xy": "Metrics/twist/error_vel_xy", "error_vel_yaw": "Metrics/twist/error_vel_yaw",
               "fell_over": "Episode_Termination/fell_over", "mean_reward": "Train/mean_reward"}
SMOOTH = 25
MASK_START = 25  # iterations dropped at each segment start: episode stats refill after the all-env reset
NOTES = [
  "GPU rollouts are not bit-reproducible: single-eval fall gaps under ~10-15 are ties (six identical suite "
  "reruns gave 62-78 falls); fall deltas with |delta| < 15 in both seeds are marked `tie`.",
  "sat@1.5 (3 Nm, steady +-1.5) is a hard gate: a round that raises it beyond the noise band vs the matched "
  "control is not acceptable as final. It depends on achieved speed (compare vx+-1.5); the noise band is "
  "estimated from repeat evals (report/part2/analysis/repeat_spread).",
  "F_peak_BW(flag) is a single-sample maximum (4.9-11.7 BW across identical reruns): flag only, never ranked.",
  "Push moving rates are confounded by achieved speed (vx_<cond>). (R) = ranked column.",
]


def arm_names(s, level):
  return [a.format(s=s) for a in LEVELS[level]]


def base(arm, s):
  return arm.replace(f"p2-s{s}-", "").replace(f"-s{s}", "")


def is_ctrl(b):
  return b.startswith("ctrl-")


def ckpt_iter(p):
  m = re.search(r"model_(\d+)\.pt$", str(p))
  return int(m.group(1)) if m else None


def read_prov(d):
  f = Path(d) / "provenance.json"
  return json.loads(f.read_text()) if f.is_file() else None


def load_status(paths):
  jobs, seg2job = {}, {}
  for p in paths:
    if Path(p).is_file():
      for j in json.loads(Path(p).read_text()):
        jobs[j["name"]] = j
        for sg in j.get("segments", []):
          if j.get("kind") == "train" and sg.get("name"):
            seg2job[sg["name"]] = j["name"]
  return jobs, seg2job


def same_job(child, parent, seg2job):
  """True when run dir `child` is a crash-resume continuation of run dir `parent`."""
  a, b = seg2job.get(child.name), seg2job.get(parent.name)
  if a or b:
    return a is not None and a == b
  if re.fullmatch(re.escape(parent.name) + r"-r\d+", child.name):
    return True
  strip = lambda n: re.sub(r"-r\d+$", "", n)  # noqa: E731
  return (child.name != parent.name and strip(child.name) == strip(parent.name)
          and (child.parent / strip(child.name)).is_dir())


def arm_segments(arm, runs, jobs):
  j = jobs.get(arm)
  if j and j.get("segments"):
    return [runs / sg["name"] for sg in j["segments"] if (runs / sg["name"]).is_dir()]
  pat = re.compile(rf"rp-{re.escape(arm)}(?:-r(\d+))?")
  ds = [(d, pat.fullmatch(d.name)) for d in runs.glob(f"rp-{arm}*") if d.is_dir()]
  return [d for d, m in sorted(((d, m) for d, m in ds if m), key=lambda t: int(t[1].group(1) or 1))]


def latest_model(d):
  ms = [(ckpt_iter(f), f) for f in Path(d).glob("model_*.pt") if ckpt_iter(f) is not None]
  return max(ms)[1] if ms else None


def walk(final, seg2job, root_prefix="rp-p1-"):
  """Chain from `final` back to the Part 1 root: legs (oldest first), each merging its resume segments."""
  cur, legs, problems, root, root_prov = Path(final), [], [], None, None
  for _ in range(50):
    d, prov = cur.parent, read_prov(cur.parent)
    if prov is None:
      problems.append(f"no provenance.json in {d}")
      break
    if not prov.get("checkpoint") or d.name.startswith(root_prefix):
      root, root_prov = cur, prov
      break
    segs, seg_dir, seg_prov, seg_end = [], d, prov, ckpt_iter(cur)
    while True:
      parent = Path(seg_prov["checkpoint"])
      segs.insert(0, dict(dir=str(seg_dir), parent=str(parent), start=ckpt_iter(parent), end=seg_end,
                          num_envs=seg_prov.get("num_envs"), seed=seg_prov.get("seed"),
                          iterations_arg=seg_prov.get("iterations"), reference_dir=seg_prov.get("reference_dir")))
      if same_job(seg_dir, parent.parent, seg2job):
        seg_dir, seg_end, seg_prov = parent.parent, ckpt_iter(parent), read_prov(parent.parent)
        if seg_prov is None:
          problems.append(f"no provenance.json in resume predecessor {seg_dir}")
          break
        continue
      break
    start, end = segs[0]["start"], segs[-1]["end"]
    legs.insert(0, dict(job=seg2job.get(Path(segs[0]["dir"]).name, Path(segs[0]["dir"]).name[3:]),
                        segments=segs, parent=segs[0]["parent"], start=start, end=end, iters=end - start + 1,
                        planned_iters=segs[0]["iterations_arg"], resumed=len(segs) > 1,
                        lineage_updates=sum(sg["end"] - sg["start"] + 1 for sg in segs)))
    cur = Path(segs[0]["parent"])
  else:
    problems.append("chain longer than 50 legs (cycle?)")
  return dict(final=str(final), final_iter=ckpt_iter(final), final_exists=Path(final).is_file(),
              root=str(root) if root else None, root_num_envs=(root_prov or {}).get("num_envs"),
              root_seed=(root_prov or {}).get("seed"), root_reference_dir=(root_prov or {}).get("reference_dir"),
              legs=legs, cumulative_iters=sum(lg["iters"] for lg in legs), problems=problems)


def arm_final(arm, runs, jobs, eval_dir):
  """(final checkpoint, source, suite, push) for an arm; suite None when not evaluated yet."""
  f, fp = eval_dir / f"{arm}.json", eval_dir / f"{arm}_push.json"
  suite = json.loads(f.read_text()) if f.is_file() else None
  push = json.loads(fp.read_text()) if fp.is_file() else None
  if suite:
    return Path(suite["training"]["checkpoint"]), "eval training.checkpoint", suite, push
  if jobs.get(arm, {}).get("final_checkpoint"):
    return Path(jobs[arm]["final_checkpoint"]), "status final_checkpoint", None, push
  segs = arm_segments(arm, runs, jobs)
  ck = latest_model(segs[-1]) if segs else None
  return ck, ("latest model in run dir" if ck else None), None, push


def check_arm(arm, s, level, chain, suite, push, root_expected):
  v, t = [], suite["training"]
  same = lambda a, b: a is not None and b is not None and Path(a).resolve() == Path(b).resolve()  # noqa: E731
  v += chain["problems"]
  if not chain["final_exists"]:
    v.append(f"final checkpoint missing: {chain['final']}")
  if not same(chain["root"], root_expected):
    v.append(f"root {chain['root']} != {root_expected}")
  if chain["cumulative_iters"] != level:
    v.append(f"cumulative iterations past root {chain['cumulative_iters']} != level {level}")
  if chain["root_num_envs"] != NUM_ENVS or chain["root_seed"] != s:
    v.append(f"root num_envs/seed {chain['root_num_envs']}/{chain['root_seed']} != {NUM_ENVS}/{s}")
  for lg in chain["legs"]:
    if lg["iters"] != lg["planned_iters"]:
      v.append(f"leg {lg['job']} incomplete: {lg['iters']} iterations != --iterations {lg['planned_iters']}")
    for sg in lg["segments"]:
      if sg["num_envs"] != NUM_ENVS or sg["seed"] != s:
        v.append(f"{Path(sg['dir']).name}: num_envs/seed {sg['num_envs']}/{sg['seed']} != {NUM_ENVS}/{s}")
  if suite.get("name") != arm:
    v.append(f"eval name {suite.get('name')} != {arm}")
  last = chain["legs"][-1]["segments"][-1] if chain["legs"] else None
  exp = dict(iter=chain["final_iter"], train_num_envs=NUM_ENVS, train_seed=s,
             iters_past_parent=(last["end"] - last["start"] + 1) if last else chain["final_iter"] + 1)
  for k, want in exp.items():
    if t.get(k) != want:
      v.append(f"eval training.{k}={t.get(k)} != chain {want}")
  if last and not same(t.get("parent"), last["parent"]):
    v.append(f"eval training.parent {t.get('parent')} != chain {last['parent']}")
  if not last and t.get("parent"):
    v.append(f"eval training.parent {t.get('parent')} on a root arm")
  ref = last["reference_dir"] if last else chain["root_reference_dir"]
  if Path(t.get("train_reference_dir") or "").name != Path(ref or "").name:
    v.append(f"eval train_reference_dir {t.get('train_reference_dir')} != provenance {ref}")
  if not str(t.get("reference_check", "")).startswith("ok"):
    v.append(f"eval reference_check: {t.get('reference_check')}")
  if push is not None and not same(push.get("training", {}).get("checkpoint"), t["checkpoint"]):
    v.append("push JSON evaluated a different checkpoint than the suite JSON")
  return v


def readouts(name, suite, push):
  header, rows = pe.table_rows([(name, suite, push)])
  r = dict(zip(header, rows[0]))
  out = {c: r.get(c) for c in READOUTS}
  e1, e2 = out["err+1.5"], out["err-1.5"]
  out["err±1.5"] = (e1 + e2) / 2 if e1 is not None and e2 is not None else None
  return {k: (float(x) if isinstance(x, (int, float, np.number)) and not isinstance(x, bool) else x)
          for k, x in out.items()}


def fmt(x, signed=False):
  if x is None:
    return "-"
  return ("+" if signed and x > 0 else "") + pe._fmt(x)


def label(c):
  return c + (" (R)" if c in RANKED or c.startswith("R:") else "") + (" (gate)" if c == "sat@1.5" else "")


# ---------------------------------------------------------------------------------------------- matched
def run_matched(a):
  runs, ev, out = a.runs, a.eval_dir, a.out_dir
  jobs, seg2job = load_status(a.status)
  out.mkdir(parents=True, exist_ok=True)
  arms, skipped, violations = {}, [], []
  for s in a.seeds:
    root = runs / ROOT_TEMPLATE.format(s=s)
    for level in LEVELS:
      for arm in arm_names(s, level):
        final, src, suite, push = arm_final(arm, runs, jobs, ev)
        chain = walk(final, seg2job) if final else None
        rec = dict(seed=s, level=level, base=base(arm, s), final_source=src, eval=suite is not None,
                   push_eval=push is not None, chain=chain)
        if suite is None:
          skipped.append(f"{arm}: no eval JSON in {ev} (" + (f"chain from {src}, not asserted)" if chain
                                                             else "no run dir yet)"))
          rec["violations"] = None
        else:
          rec["violations"] = check_arm(arm, s, level, chain, suite, push, root)
          rec["eval_training"] = suite["training"]
          rec["readouts"] = readouts(arm, suite, push)
          violations += [f"{arm}: {x}" for x in rec["violations"]]
        arms[arm] = rec
  rows = {}
  for level in LEVELS:
    for s in a.seeds:
      ok = [arms[x]["chain"] for x in arm_names(s, level) if arms[x]["eval"]]
      rows[f"s{s}+{level}"] = {k: sorted({str(c[f]) for c in ok}) for k, f in
                                (("num_envs", "root_num_envs"), ("cumulative", "cumulative_iters"),
                                 ("seed", "root_seed"), ("root", "root"))}
      rows[f"s{s}+{level}"]["num_envs"] = sorted({str(sg["num_envs"]) for c in ok for lg in c["legs"]
                                                  for sg in lg["segments"]} | {str(c["root_num_envs"]) for c in ok})
      for k, vals in rows[f"s{s}+{level}"].items():
        if len(vals) > 1:
          violations.append(f"row s{s} +{level}: {k} differs {vals}")
    both = [rows[f"s{s}+{level}"] for s in a.seeds]
    for k in ("num_envs", "cumulative"):
      vals = sorted({x for r in both for x in r[k]})
      if len(vals) > 1:
        violations.append(f"level +{level} across seeds: {k} differs {vals}")
  (out / "chain_check.json").write_text(json.dumps(dict(
    assertions=dict(num_envs=NUM_ENVS, root=ROOT_TEMPLATE, cumulative="= level", seeds=a.seeds),
    violations=violations, rows=rows, skipped=skipped,
    arms={k: {kk: vv for kk, vv in v.items() if kk != "readouts"} for k, v in arms.items()}), indent=1) + "\n")
  if violations:
    print("CHAIN CHECK FAILED (evidence in chain_check.json):\n  " + "\n  ".join(violations), file=sys.stderr)
    sys.exit(2)
  print(f"chain check passed for {sum(r['eval'] for r in arms.values())} evaluated arms; {len(skipped)} skipped")
  # part2_eval.compare per (seed, level)
  for s in a.seeds:
    for level in LEVELS:
      paths = [ev / f"{x}.json" for x in arm_names(s, level) if arms[x]["eval"]]
      if level == 0 or not paths:
        continue
      dst = out / f"matched_s{s}_plus{level}.md"
      try:
        pe.compare(paths, dst, push_dir=ev)
      except pe.MismatchError as e:
        resumed = [x for x in arm_names(s, level) if arms[x]["eval"] and arms[x]["chain"]["legs"][-1]["resumed"]]
        if not resumed:
          raise
        pe.compare(paths, dst, push_dir=ev, allow_mismatch=True)
        dst.write_text(f"Note: compare's per-leg check fails only because {resumed} resumed after a crash "
                       f"(eval parent = resume checkpoint); the chain check (chain_check.json) confirms "
                       f"{level} cumulative iterations for every row. ({e})\n\n" + dst.read_text())
      print("wrote", dst)
  parents = [ev / f"{arm_names(s, 0)[0]}.json" for s in a.seeds if arms[arm_names(s, 0)[0]]["eval"]]
  if parents:
    pe.compare(parents, out / "matched_parents.md", push_dir=ev)
  write_summary(a, arms, skipped)


def write_summary(a, arms, skipped):
  val = lambda arm, c: (arms.get(arm) or {}).get("readouts", {}).get(c)  # noqa: E731
  js = dict(notes=NOTES, readout_json_keys=READOUT_KEYS, skipped=skipped, levels={})
  md = ["# Part 2 matched summary (simulation)", "", "Deltas are arm - same-seed control at the same "
        "cumulative iterations past the root; `mean` = two-seed mean; `agree` = both seeds' deltas have the "
        "same sign. Chain evidence: chain_check.json.", ""] + [f"- {n}" for n in NOTES] + [""]
  par = {s: arm_names(s, 0)[0] for s in a.seeds}
  pair = lambda f: " / ".join(fmt(f(s)) for s in a.seeds)  # noqa: E731
  md += ["## +0 (parents)", "", "| readout | " + " / ".join(f"s{s}" for s in a.seeds) + " | mean |", "|---|---|---|"]
  js["levels"]["0"] = {"parent": {s: arms[par[s]].get("readouts") for s in a.seeds}}
  for c in READOUTS:
    xs = [val(par[s], c) for s in a.seeds if val(par[s], c) is not None]
    md.append(f"| {label(c)} | {pair(lambda s: val(par[s], c))} | {fmt(np.mean(xs) if xs else None)} |")
  for level in [lv for lv in LEVELS if lv]:
    bases = [base(x, a.seeds[0]) for x in arm_names(a.seeds[0], level)]
    ctrl = next(b for b in bases if is_ctrl(b))
    name = {(b, s): next(x for x in arm_names(s, level) if base(x, s) == b) for b in bases for s in a.seeds}
    lj = js["levels"][str(level)] = dict(control=ctrl, arms={})
    md += ["", f"## +{level} iterations past root (control: {ctrl})", ""]
    have = [b for b in bases if not is_ctrl(b) and any(arms[name[b, s]]["eval"] for s in a.seeds)]
    if not have:
      md.append("No evaluated arms yet.")
      continue
    md += ["| readout | arm | parent s42/s43 | ctrl s42/s43 (mean) | arm s42/s43 (mean) | delta s42/s43 | mean delta | agree |",
           "|---|---|---|---|---|---|---|---|"]
    for b in have:
      aj = lj["arms"][b] = dict(values={}, control_values={}, delta={}, mean_level={}, control_mean_level={},
                                mean_delta={}, sign_agree={})
      for s in a.seeds:
        aj["values"][s] = arms[name[b, s]].get("readouts")
        aj["control_values"][s] = arms[name[ctrl, s]].get("readouts")
    for c in READOUTS:
      for b in have:
        aj = lj["arms"][b]
        av = {s: val(name[b, s], c) for s in a.seeds}
        cv = {s: val(name[ctrl, s], c) for s in a.seeds}
        d = {s: (av[s] - cv[s]) if isinstance(av[s], float) and isinstance(cv[s], float) else None for s in a.seeds}
        mean = lambda m: (float(np.mean([x for x in m.values() if x is not None]))  # noqa: E731
                          if any(x is not None for x in m.values()) else None)
        nd = [x for x in d.values() if x is not None]
        agree = (None if len(nd) < 2 else bool(all(x > 0 for x in nd) or all(x < 0 for x in nd)))
        aj["delta"][c] = d; aj["mean_delta"][c] = mean(d); aj["sign_agree"][c] = agree
        aj["mean_level"][c] = mean(av); aj["control_mean_level"][c] = mean(cv)
        tag = "-" if not nd else (f"n={len(nd)}" if agree is None else ("yes" if agree else "NO"))
        if nd and any(x == 0 for x in nd):
          tag = "0"
        if c in FALL_LABELS and nd and all(abs(x) < 15 for x in nd):
          tag += ", tie"
        md.append(f"| {label(c)} | {b} | {pair(lambda s: val(par[s], c))} | {pair(lambda s: cv[s])} "
                  f"({fmt(mean(cv))}) | {pair(lambda s: av[s])} ({fmt(mean(av))}) | "
                  f"{' / '.join(fmt(d[s], True) for s in a.seeds)} | {fmt(mean(d), True)} | {tag} |")
  md += ["", "Skipped (no eval JSON yet):", ""] + [f"- {x}" for x in skipped] + ["", pe.LEGEND]
  (a.out_dir / "matched_summary.md").write_text("\n".join(md) + "\n")
  (a.out_dir / "matched_summary.json").write_text(json.dumps(js, indent=1, default=str) + "\n")
  print("wrote", a.out_dir / "matched_summary.md")


# ---------------------------------------------------------------------------------------------- rewards
_EVENTS = {}


def events(d):
  """{tag: {step: value}} from every events* file of a run dir (cached)."""
  if d not in _EVENTS:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    out = {}
    for f in sorted(Path(d).glob("events*")):
      acc = EventAccumulator(str(f), size_guidance={"scalars": 0})
      acc.Reload()
      for tag in acc.Tags()["scalars"]:
        out.setdefault(tag, {}).update({e.step: e.value for e in acc.Scalars(tag)})
    _EVENTS[d] = out
  return _EVENTS[d]


def chain_series(chain):
  """{tag: {x: value}}, x = iterations past root (root's last iterations at x <= 0)."""
  ser, root_it = {}, ckpt_iter(chain["root"])
  for tag, sv in events(str(Path(chain["root"]).parent)).items():
    ser.setdefault(tag, {}).update({st - root_it: v for st, v in sv.items() if root_it - 49 - SMOOTH < st <= root_it})
  c = 0
  for i, lg in enumerate(chain["legs"]):
    for j, sg in enumerate(lg["segments"]):
      open_end = i == len(chain["legs"]) - 1 and j == len(lg["segments"]) - 1  # in-progress: all logged steps
      for tag, sv in events(sg["dir"]).items():
        ser.setdefault(tag, {}).update({c + st - lg["start"] + 1: v for st, v in sv.items()
                                        if st >= sg["start"] + MASK_START and (open_end or st <= sg["end"])})
    c += lg["iters"]
  return ser, c - (chain["legs"][-1]["iters"] if chain["legs"] else 0)


def short(tag):
  return next((k for k, t in METRIC_TAGS.items() if t == tag), tag.split("/")[-1])


def run_rewards(a):
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  jobs, seg2job = load_status(a.status)
  a.out_dir.mkdir(parents=True, exist_ok=True)
  data = {}
  for s in a.seeds:
    for level in LEVELS:
      for arm in arm_names(s, level):
        final, src, _, _ = arm_final(arm, a.runs, jobs, a.eval_dir)
        if final is None:
          print(f"rewards: skip {arm} (no run dir / checkpoint yet)")
          continue
        chain = walk(final, seg2job)
        if chain["problems"] or chain["root"] is None:
          print(f"rewards: skip {arm}: {chain['problems']}")
          continue
        ser, own_start = chain_series(chain)
        data[arm] = dict(seed=s, level=level, ser=ser, own_start=own_start, chain=chain, src=src)
  # reward terms non-zero in any Part 2 run
  p2 = {sg["dir"] for d in data.values() for lg in d["chain"]["legs"] for sg in lg["segments"]}
  terms = sorted({t for dd in p2 for t, sv in events(dd).items()
                  if t.startswith("Episode_Reward/") and any(v != 0 for v in sv.values())})
  cols = [short(t) for t in terms] + list(METRIC_TAGS)
  tags = dict(zip(cols, terms + list(METRIC_TAGS.values())))
  rows = []
  for arm, d in data.items():
    lo = d["own_start"] if d["level"] else -50
    xs = [x for x in d["ser"].get("Train/mean_reward", {}) if x > lo]
    xe = max(xs) if xs else None
    row = dict(arm=arm, seed=d["seed"], level=d["level"], final=Path(d["chain"]["final"]).parent.name + "/"
               + Path(d["chain"]["final"]).name, final_source=d["src"], iters_past_root_end=xe,
               complete=xe == d["level"], resumed=any(lg["resumed"] for lg in d["chain"]["legs"]))
    for c in cols:
      sv = d["ser"].get(tags[c], {})
      w = [sv[x] for x in sv if xe is not None and xe - 50 < x <= xe and x > lo]
      row[c] = float(np.mean(w)) if len(w) >= 25 else None
    rows.append(row)
  with open(a.out_dir / "reward_terms_last50.csv", "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["arm"])
    wr.writeheader()
    wr.writerows(rows)
  print("wrote", a.out_dir / "reward_terms_last50.csv", f"({len(rows)} arms, {len(terms)} non-zero reward terms)")
  palette = {"p1-gallop_c": "0.45", "ctrl": "k", "r1-naive": "tab:orange", "r1-lag": "tab:blue",
             "r1-backpin": "tab:purple", "r2": "tab:green", "r3": "tab:red", "r1-cont": "tab:cyan",
             "r2-cont": "tab:olive"}
  for s in a.seeds:
    fig, axes = plt.subplots(2, 5, figsize=(22, 8), sharex=True)
    for ax, term in zip(axes.flat, KEY_TERMS):
      tag = METRIC_TAGS.get(term, f"Episode_Reward/{term}")
      for arm, d in data.items():
        if d["seed"] != s or tag not in d["ser"]:
          continue
        x = np.array(sorted(d["ser"][tag]))
        y = np.array([d["ser"][tag][k] for k in x])
        cs = np.cumsum(np.insert(y, 0, 0.0))
        lo = np.searchsorted(x, x - SMOOTH, side="right")  # trailing window over iterations (x - 25, x]
        sm = (cs[1:] - cs[lo]) / (np.arange(1, len(y) + 1) - lo)
        keep = (x > d["own_start"]) if d["level"] else (x > -50)
        b = base(arm, s)
        fam = "ctrl" if is_ctrl(b) else re.sub(r"-\d+$", "", b)
        ax.plot(x[keep], sm[keep], color=palette.get(fam, None), lw=1.2,
                ls={0: "-", 500: "-", 1000: "--", 1500: ":"}[d["level"]], label=b)
      for v in (0, 500, 1000, 1500):
        ax.axvline(v, color="0.8", lw=0.6, zorder=0)
      ax.set_title(term, fontsize=10)
      ax.tick_params(labelsize=8)
    for ax in axes[-1]:
      ax.set_xlabel("PPO iterations past root", fontsize=9)
    hs = {}
    for ax in axes.flat:
      for h, lab in zip(*ax.get_legend_handles_labels()):
        hs.setdefault(lab, h)
    fig.legend(list(hs.values()), list(hs), loc="lower center", ncol=min(len(hs), 7) or 1, fontsize=9)
    fig.suptitle(f"Part 2 training-log terms, seed {s} (root {ROOT_TEMPLATE.format(s=s)}; "
                 f"{SMOOTH}-iteration trailing rolling mean; each arm drawn over its own leg;\n"
                 f"first {MASK_START} iterations of every leg/segment masked: episode statistics refill after "
                 f"the all-env reset at launch; simulation)",
                 fontsize=11)
    fig.tight_layout(rect=(0, 0.06, 1, 0.96))
    fig.savefig(a.out_dir / f"reward_terms_s{s}.png", dpi=90)
    plt.close(fig)
    print("wrote", a.out_dir / f"reward_terms_s{s}.png")


def run_readouts(a):
  for p in a.paths:
    p = Path(p)
    suite = json.loads(p.read_text())
    pp = p.with_name(p.stem + "_push.json")
    push = json.loads(pp.read_text()) if pp.is_file() else None
    r = readouts(suite.get("name", p.stem), suite, push)
    print(f"== {p.name} (push: {pp.name if push else 'none'})")
    for c in READOUTS:
      print(f"  {label(c):28s} {fmt(r[c]):>10s}   <- {READOUT_KEYS[c]}")


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("cmd", nargs="?", default="all", choices=("all", "matched", "rewards", "readouts"))
  p.add_argument("paths", nargs="*", help="eval suite JSONs (readouts only)")
  p.add_argument("--eval-dir", type=Path, default=REPO / "report/part2/eval/w3")
  p.add_argument("--out-dir", type=Path, default=REPO / "report/part2/analysis")
  p.add_argument("--runs", type=Path, default=RUNS)
  p.add_argument("--status", type=Path, nargs="*", default=[RUNS / "w3/status-s42.json", RUNS / "w3/status-s43.json"])
  p.add_argument("--seeds", type=int, nargs="*", default=list(SEEDS))
  a = p.parse_args()
  a.seeds = tuple(a.seeds)
  if a.cmd == "readouts":
    return run_readouts(a)
  if a.cmd in ("all", "matched"):
    run_matched(a)
  if a.cmd in ("all", "rewards"):
    run_rewards(a)


if __name__ == "__main__":
  main()
