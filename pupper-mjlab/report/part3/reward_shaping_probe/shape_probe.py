"""Reward-shaping probe: action rate, standing posture/jitter, left-right symmetry.

Read-only eval (no training). Reuses part2_eval._Rollout under part3_common.rollout_conditions,
same as the push sweep driver. Commands are held fixed per env group for 20 s; the first 3 s
are discarded.

  python shape_probe.py --out OUT.json [--envs-per 96] [--seed 1234] [--conditions flat_dr,flat_nodr]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

REF = "configs/opt-lab-2/p2/s42/r2-lag"
RUNS = Path.home() / "runs" / "opt-lab-2"
POLICIES = {
  "bumpy_p2000": RUNS / "rp-p3-final-bumpyft-kpwide-leg4" / "model_6492.pt",
  "flatctrl_p2000": RUNS / "rp-p3-final-flatctrl-kpwide-leg4" / "model_6492.pt",
  "part2_final": RUNS / "rp-p2-s42-r2-cont-1500" / "model_4496.pt",
}
GROUPS = [  # name, (vx, vy, wz)
  ("stand", (0.0, 0.0, 0.0)),
  ("fwd0.5", (0.5, 0.0, 0.0)),
  ("fwd1.5", (1.5, 0.0, 0.0)),
  ("back0.5", (-0.5, 0.0, 0.0)),
  ("back1.5", (-1.5, 0.0, 0.0)),
  ("yaw+1", (0.0, 0.0, 1.0)),
  ("yaw-1", (0.0, 0.0, -1.0)),
  ("side+0.3", (0.0, 0.3, 0.0)),
  ("side-0.3", (0.0, -0.3, 0.0)),
]
SECONDS, DT, SKIP_S = 20.0, 0.02, 3.0


def probe(name, ckpt, cond, envs_per, seed):
  import torch

  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import FOOT_SITE_NAMES, JOINT_NAMES
  from mjlab.scripts import part2_eval as pe
  from mjlab.scripts.part3_common import CONDITIONS, rollout_conditions
  from mjlab.utils.lab_api.math import quat_apply_inverse

  task, dr = CONDITIONS[cond]
  p = argparse.ArgumentParser()
  pe._add_rollout_args(p)
  p.add_argument("--envs", type=int)
  a = p.parse_args(["--checkpoint", str(ckpt), "--reference-dir", REF, "--name", f"{name}-{cond}",
                    "--seed", str(seed), "--task", task, "--out-dir", "/tmp/unused"])
  pe._prepare(a, lambda m: sys.exit(m))
  a.dr, a.condition = dr, cond
  G = len(GROUPS)
  n = G * envs_per
  T = int(round(SECONDS / DT))
  with rollout_conditions(a, dr=dr):
    roll = pe._Rollout(a, n, T)
    assert abs(roll.dt - DT) < 1e-9
    robot = roll.robot
    jid = [robot.joint_names.index(j) for j in JOINT_NAMES]
    sid, _ = robot.find_sites(FOOT_SITE_NAMES, preserve_order=True)
    contact = roll.env.scene["feet_ground_contact"]
    am = roll.env.action_manager
    cmd = torch.tensor([g[1] for g in GROUPS], dtype=torch.float32, device=a.device)
    cmd = cmd.repeat_interleave(envs_per, 0)
    cmd_tn = cmd.unsqueeze(0).expand(T, n, 3).contiguous()
    dev = a.device
    B = dict(
      act=torch.zeros(T, n, 12, device=dev),
      q=torch.zeros(T, n, 12, device=dev),
      qd=torch.zeros(T, n, 12, device=dev),
      foot_b=torch.zeros(T, n, 4, 3, device=dev),
      force=torch.zeros(T, n, 4, device=dev),
      contact=torch.zeros(T, n, 4, dtype=torch.bool, device=dev),
      grav=torch.zeros(T, n, 3, device=dev),
      vel=torch.zeros(T, n, 3, device=dev),
      height=torch.zeros(T, n, device=dev),
    )
    data = robot.data

    def record(k):
      B["act"][k] = am.action
      B["q"][k] = data.joint_pos[:, jid] - data.default_joint_pos[:, jid]
      B["qd"][k] = data.joint_vel[:, jid]
      rel = data.site_pos_w[:, sid, :] - data.root_link_pos_w[:, None, :]
      quat = data.root_link_quat_w[:, None, :].expand(-1, 4, -1)
      B["foot_b"][k] = quat_apply_inverse(quat.reshape(-1, 4), rel.reshape(-1, 3)).reshape(n, 4, 3)
      B["force"][k] = torch.linalg.vector_norm(contact.data.force, dim=-1)
      B["contact"][k] = contact.data.found > 0
      B["grav"][k] = data.projected_gravity_b
      B["vel"][k] = torch.cat([data.root_link_lin_vel_b[:, :2], data.root_link_ang_vel_b[:, 2:3]], 1)
      B["height"][k] = data.root_link_pos_w[:, 2]

    try:
      term, tout = roll.run_loop(cmd_tn, record=record)
      X = {k: v.cpu().numpy() for k, v in B.items()}
    finally:
      roll.close()

  skip = int(SKIP_S / DT)
  ever_fell = term.any(0)
  out = {}
  for gi, (gname, c) in enumerate(GROUPS):
    sl = slice(gi * envs_per, (gi + 1) * envs_per)
    keep = ~ever_fell[sl]  # whole-run survivors only, so resets do not pollute rates
    if keep.sum() == 0:
      out[gname] = dict(survivors=0)
      continue
    g = {k: v[skip:, sl][:, keep] for k, v in X.items()}
    act = g["act"]
    da = np.diff(act, axis=0)
    rate = (da ** 2).sum(-1)  # same quantity as the action_rate_l2 term
    # spectral share of action power above 6 Hz (gait fundamental 1.33-3 Hz)
    a0 = act - act.mean(0, keepdims=True)
    P = np.abs(np.fft.rfft(a0, axis=0)) ** 2
    f = np.fft.rfftfreq(act.shape[0], DT)
    hf = float(P[f > 6.0].sum() / max(P[f > 0.2].sum(), 1e-12))
    q = g["q"]
    # mirror: default left = -right for every motor, so q_R + q_L = 0 for a mirrored pose
    pair = {"front": (0, 3), "hind": (6, 9)}
    mirror = {}
    for pn, (r, l) in pair.items():
      s = q[..., r:r + 3] + q[..., l:l + 3]  # (T, E, 3) per motor 1,2,3
      mirror[pn] = dict(
        mean_abs_env=[float(x) for x in np.abs(s.mean(0)).mean(0)],  # sustained (per-env time mean)
        signed_mean=[float(x) for x in s.mean((0, 1))],
      )
    fb = g["foot_b"]  # FR, FL, BR, BL
    width_front = float((fb[..., 1, 1] - fb[..., 0, 1]).mean())
    width_hind = float((fb[..., 3, 1] - fb[..., 2, 1]).mean())
    # mirrored foot-position mismatch (cm): (x, -y, z) left vs (x, y, z) right, env time-mean
    mir_pos = []
    for r, l in ((0, 1), (2, 3)):
      dl = fb[..., l, :] * np.array([1, -1, 1])
      mir_pos.append(float(np.linalg.norm((dl - fb[..., r, :]).mean(0), axis=-1).mean() * 100))
    force = g["force"]
    tot = force.sum(-1) + 1e-9
    left_share = float(((force[..., 1] + force[..., 3]) / tot).mean())
    left_share_env_abs = float(np.abs(((force[..., 1] + force[..., 3]) / tot).mean(0) - 0.5).mean())
    contact = g["contact"]
    duty = contact.mean((0, 1))  # FR FL BR BL
    roll_deg = np.degrees(np.arcsin(np.clip(g["grav"][..., 1], -1, 1)))
    out[gname] = dict(
      survivors=int(keep.sum()), envs=int(envs_per), fell=int(ever_fell[sl].sum()),
      action_rate_l2=float(rate.mean()), action_rate_l2_p95=float(np.percentile(rate.mean(0), 95)),
      action_abs_step=float(np.abs(da).mean()), action_hf_share_gt6hz=hf,
      joint_speed_abs=float(np.abs(g["qd"]).mean()),
      mirror_joint=mirror, mirror_foot_cm=dict(front=mir_pos[0], hind=mir_pos[1]),
      width_front_m=width_front, width_hind_m=width_hind,
      left_load_share=left_share, left_load_share_env_absdev=left_share_env_abs,
      duty_FR_FL_BR_BL=[float(x) for x in duty],
      feet_down_mean=float(contact.sum(-1).mean()),
      feet_down_le2_frac=float((contact.sum(-1) <= 2).mean()),
      roll_deg_signed=float(roll_deg.mean()), roll_deg_env_abs=float(np.abs(roll_deg.mean(0)).mean()),
      height_m=float(g["height"].mean()),
      vy_signed=float(g["vel"][..., 1].mean()), wz_signed=float(g["vel"][..., 2].mean()),
      vx=float(g["vel"][..., 0].mean()),
      q_mean_dev=[float(x) for x in q.mean((0, 1))],
    )
  return out


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--out", required=True)
  ap.add_argument("--envs-per", type=int, default=96)
  ap.add_argument("--seed", type=int, default=1234)
  ap.add_argument("--conditions", default="flat_dr,flat_nodr")
  ap.add_argument("--policies", default=",".join(POLICIES))
  cli = ap.parse_args()
  res = {}
  for cond in cli.conditions.split(","):
    for pn in cli.policies.split(","):
      print(f"[probe] {pn} {cond}", flush=True)
      res[f"{pn}|{cond}"] = probe(pn, POLICIES[pn], cond, cli.envs_per, cli.seed)
      Path(cli.out).write_text(json.dumps(res, indent=1) + "\n")
  print("done", flush=True)
