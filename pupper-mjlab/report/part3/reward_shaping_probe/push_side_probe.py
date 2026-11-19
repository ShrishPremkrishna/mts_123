"""Standing push, split by push side (robot's left vs right), flat_dr.

One push per env at t = 5 s, lateral (heading-perpendicular) velocity change of +v (left) or -v (right),
same mechanics as part2_eval.run_push. Fall = termination within 2 s of the push.
Also records each env's pre-push hind/front motor-2 pair sums (q_R_2 + q_L_2, rad) to test whether
the sustained standing lean predicts which side falls.

  python push_side_probe.py --out OUT.json [--envs 2048] [--mags 0.55,0.65]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shape_probe import POLICIES, REF  # noqa: E402

PUSH_T, AFTER_S, DT = 5.0, 2.0, 0.02


def probe(name, ckpt, mags, envs, seed):
  import torch

  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
  from mjlab.scripts import part2_eval as pe
  from mjlab.scripts.part3_common import CONDITIONS, rollout_conditions

  task, dr = CONDITIONS["flat_dr"]
  p = argparse.ArgumentParser()
  pe._add_rollout_args(p)
  p.add_argument("--envs", type=int)
  a = p.parse_args(["--checkpoint", str(ckpt), "--reference-dir", REF, "--name", f"{name}-side",
                    "--seed", str(seed), "--task", task, "--out-dir", "/tmp/unused"])
  pe._prepare(a, lambda m: sys.exit(m))
  a.dr, a.condition = dr, "flat_dr"
  T = int(round((PUSH_T + AFTER_S + 0.5) / DT))
  kpush = int(round(PUSH_T / DT))
  M = len(mags)
  # env e: magnitude mags[e % M], sign + for first half of each magnitude block, - for second
  mag = np.array([mags[e % M] for e in range(envs)])
  sign = np.where((np.arange(envs) // M) % 2 == 0, 1.0, -1.0)
  with rollout_conditions(a, dr=dr):
    roll = pe._Rollout(a, envs, T)
    robot = roll.robot
    jid = [robot.joint_names.index(j) for j in JOINT_NAMES]
    dev = a.device
    cmd_tn = torch.zeros(T, envs, 3, device=dev)
    dv_all = torch.tensor(mag * sign, dtype=torch.float32, device=dev)
    pre = {}

    def record(k):
      if k == kpush:
        q = robot.data.joint_pos[:, jid]
        pre["front"] = (q[:, 1] + q[:, 4]).cpu().numpy()
        pre["hind"] = (q[:, 7] + q[:, 10]).cpu().numpy()

    def pushes(k):
      if k != kpush:
        return
      vel = robot.data.root_link_vel_w.clone()
      h = robot.data.heading_w
      vel[:, 0] += -torch.sin(h) * dv_all
      vel[:, 1] += torch.cos(h) * dv_all
      robot.write_root_link_velocity_to_sim(vel)

    try:
      term, _ = roll.run_loop(cmd_tn, record=record, pushes=pushes)
    finally:
      roll.close()
  before = term[:kpush].any(0)
  fell = term[kpush:kpush + int(AFTER_S / DT) + 1].any(0)
  ok = ~before
  out = {}
  for m in mags:
    for s, lab in ((1.0, "left"), (-1.0, "right")):
      sel = ok & (mag == m) & (sign == s)
      out[f"{m}_{lab}"] = dict(pushes=int(sel.sum()), falls=int(fell[sel].sum()),
                               rate=float(fell[sel].mean()) if sel.any() else None)
  hind = pre["hind"][ok]
  out["pre_push_pair_sum_mean"] = dict(front=float(pre["front"][ok].mean()), hind=float(hind.mean()),
                                       hind_abs=float(np.abs(hind).mean()))
  # does lean sign predict fall side? split by sign of hind pair sum
  for m in mags:
    for s, lab in ((1.0, "left"), (-1.0, "right")):
      sel = ok & (mag == m) & (sign == s)
      for hs, hl in ((1, "hindsum>0"), (-1, "hindsum<0")):
        ss = sel & (np.sign(pre["hind"]) == hs)
        out.setdefault("by_lean", {})[f"{m}_{lab}_{hl}"] = [int(ss.sum()), float(fell[ss].mean()) if ss.any() else None]
  return out


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--out", required=True)
  ap.add_argument("--envs", type=int, default=2048)
  ap.add_argument("--seed", type=int, default=1234)
  ap.add_argument("--mags", default="0.55,0.65")
  ap.add_argument("--policies", default=",".join(POLICIES))
  cli = ap.parse_args()
  mags = [float(x) for x in cli.mags.split(",")]
  res = {}
  for pn in cli.policies.split(","):
    print(f"[side] {pn}", flush=True)
    res[pn] = probe(pn, POLICIES[pn], mags, cli.envs, cli.seed)
    Path(cli.out).write_text(json.dumps(res, indent=1) + "\n")
  print("done", flush=True)
