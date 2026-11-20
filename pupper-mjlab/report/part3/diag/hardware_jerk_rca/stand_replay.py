"""Test-stand replay: does the policy limit-cycle with feet unloaded?

Closed loop in the training plant (mjlab, play config, DR on), one env per
(condition x replica). Conditions: support {ground, stand} x action lag
{4, 9, 14 physics steps = 16/36/56 ms} x command {zero, trot 0.3, turn 1.0,
fast 1.0}. "stand" pins the base upright and still at STAND_H every control
step (same mechanism as the training airborne warmup, mdp.warmup), so the feet
hang. Reports action-rate / jerk / joint-velocity / high-frequency share.

  python stand_replay.py --checkpoint CKPT --reference-dir DIR --name NAME \
      --task Mjlab-MixedGaits-Flat-Pupper-v3 --heading-hold --out OUT.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

STAND_H = 0.40
LAGS = (4, 9, 14)
CMDS = {"zero": (0.0, 0.0, 0.0), "trot0.3": (0.3, 0.0, 0.0), "turn1.0": (0.0, 0.0, 1.0),
        "fast1.0": (1.0, 0.0, 0.0)}
SUPPORTS = ("ground", "stand")
# --cmdset seam: per-tick command noise (stick quantization/thumb tremor) at the branch seams.
SEAM_CMDS = {"gallop0.6": (0.6, 0.0, 0.0), "seam0.5": (0.5, 0.0, 0.0), "trot0.45": (0.45, 0.0, 0.0),
             "seam0.5_noisy": (0.5, 0.0, 0.0), "zero_noisy": (0.0, 0.0, 0.0)}
SEAM_NOISE = {"seam0.5_noisy": (0.02, 0.0, 0.0), "zero_noisy": (0.03, 0.0, 0.03)}
_PIN = {}


def pin_stand(env, env_ids, height):
  del env_ids
  mask = _PIN.get("mask")
  if mask is None or not bool(mask.any()):
    return
  robot = env.scene["robot"]
  ids = mask.nonzero(as_tuple=False).squeeze(-1)
  rs = robot.data.default_root_state[ids].clone()
  rs[:, 0:2] += env.scene.env_origins[ids, 0:2]
  rs[:, 2] = height
  rs[:, 3] = 1.0
  rs[:, 4:7] = 0.0
  rs[:, 7:] = 0.0
  robot.write_root_state_to_sim(rs, env_ids=ids)


def main():
  global CMDS, LAGS
  sys.path.insert(0, "/home/ubuntu/mts_123/pupper-mjlab/src")
  from mjlab.scripts import part2_eval
  p = argparse.ArgumentParser()
  part2_eval._add_rollout_args(p)
  p.add_argument("--replicas", type=int, default=4)
  p.add_argument("--seconds", type=float, default=10.0)
  p.add_argument("--out", type=Path, required=True)
  p.add_argument("--max-lag", type=int, default=max(LAGS))
  p.add_argument("--cmdset", choices=("base", "seam"), default="base")
  p.add_argument("--no-noise", action="store_true", help="actor obs noise off (hardware sensors are quieter)")
  p.add_argument("--policy-json", type=Path, help="check env gait tables against this deploy JSON")
  args = p.parse_args()
  part2_eval._prepare(args, p.error)

  import dataclasses
  import torch
  import mjlab.envs as envs
  from mjlab.managers.event_manager import EventTermCfg
  from mjlab.scripts import pupper_locomotion as cli
  from mjlab.scripts.part3_common import rollout_conditions

  noise = {}
  if args.cmdset == "seam":
    CMDS, LAGS, noise = SEAM_CMDS, (4, 9), SEAM_NOISE
  conds = [(s, l, c) for s in SUPPORTS for l in LAGS for c in CMDS]
  n = len(conds) * args.replicas
  cond_of = [conds[i // args.replicas] for i in range(n)]

  with rollout_conditions(args, dr=True):
    inner = cli.settings

    def settings(s_args, play):
      cfg, agent = inner(s_args, play)
      ent = cfg.scene.entities["robot"]
      ent.articulation = dataclasses.replace(
        ent.articulation,
        actuators=tuple(dataclasses.replace(a, delay_min_lag=args.max_lag, delay_max_lag=args.max_lag)
                        for a in ent.articulation.actuators))
      cfg.events["pin_stand"] = EventTermCfg(mode="step", func=pin_stand, params={"height": STAND_H})
      cfg.terminations.pop("fell_over", None)  # keep every env in its condition
      if args.no_noise:
        cfg.observations["actor"].terms["policy"].params["add_noise"] = False
      return cfg, agent

    cli.settings = settings
    steps = int(round(args.seconds / 0.02))
    roll = part2_eval._Rollout(args, n, steps)
    dev = roll.device
    _PIN["mask"] = torch.tensor([c[0] == "stand" for c in cond_of], device=dev)
    lags = torch.tensor([c[1] for c in cond_of], device=dev, dtype=torch.long)
    bufs = {id(a._delay_buffer): a._delay_buffer for a in roll.robot._actuators if a._delay_buffer is not None}
    for b in bufs.values():
      b.min_lag = 0
      b._sample_lags = (lambda mask, _l=lags: _l)
    ref_check = None
    if args.policy_json:
      from mjlab.tasks.pupper_gait.export import gait_reference_metadata
      mine, theirs = gait_reference_metadata(roll.env), json.loads(args.policy_json.read_text())["gait_reference"]
      ref_check = {k: float(np.abs(np.asarray(mine[k]) - np.asarray(theirs[k])).max())
                   for k in ("trot_table", "lift_table") if k in mine and k in theirs}
      ref_check.update({k: (mine.get(k), theirs.get(k)) for k in ("frequency", "gallop_speed", "blend_speed")})
      print("[stand] reference check vs policy.json:", ref_check, flush=True)
    print(f"[stand] {n} envs, {len(bufs)} delay buffer(s), dt {roll.dt}", flush=True)
    cmd = torch.tensor([CMDS[c[2]] for c in cond_of], device=dev, dtype=torch.float32)
    nscale = torch.tensor([noise.get(c[2], (0.0, 0.0, 0.0)) for c in cond_of], device=dev, dtype=torch.float32)
    gen = torch.Generator(device=dev).manual_seed(0)
    def cmd_now():
      return cmd + nscale * (2 * torch.rand(cmd.shape, device=dev, generator=gen) - 1)
    roll.set_command(cmd)
    obs, _ = roll.wrapped.reset()
    roll.set_command(cmd)
    jid = roll.robot.indexing.joint_ids if hasattr(roll.robot.indexing, "joint_ids") else slice(None)
    A, Q, QD, TAU, G = [], [], [], [], []
    with torch.inference_mode():
      for k in range(steps):
        a = roll.policy(obs)
        A.append(a.cpu().numpy())
        Q.append(roll.robot.data.joint_pos.cpu().numpy())
        QD.append(roll.robot.data.joint_vel.cpu().numpy())
        TAU.append(roll.robot.data.actuator_force.cpu().numpy() if hasattr(roll.robot.data, "actuator_force") else np.zeros_like(Q[-1]))
        G.append(roll.robot.data.projected_gravity_b.cpu().numpy())
        roll.set_command(cmd_now())
        obs, _, _, _ = roll.wrapped.step(a)
    roll.env.close()

  A, Q, QD, TAU, G = map(np.stack, (A, Q, QD, TAU, G))  # (T, N, 12)
  s0 = 50  # skip 1 s
  A, Q, QD, TAU, G = A[s0:], Q[s0:], QD[s0:], TAU[s0:], G[s0:]
  dA = np.diff(A, axis=0)
  ddA = np.diff(A, 2, axis=0)
  # High-frequency share of the action signal: power above 8 Hz / total (detrended), 50 Hz sampling.
  F = np.fft.rfft(A - A.mean(0), axis=0)
  f = np.fft.rfftfreq(A.shape[0], d=0.02)
  P = (np.abs(F) ** 2).sum(-1)  # (F, N)
  hf = P[f > 8.0].sum(0) / np.maximum(P[f > 0.2].sum(0), 1e-12)
  peak = f[1:][np.argmax(P[1:], axis=0)]
  rows = []
  for i in range(n):
    s, l, c = cond_of[i]
    rows.append(dict(support=s, lag=l, cmd=c,
                     act_rate=float(np.abs(dA[:, i]).mean()), act_acc=float(np.abs(ddA[:, i]).mean()),
                     act_std=float(A[:, i].std(0).mean()), qd_rms=float(np.sqrt((QD[:, i] ** 2).mean())),
                     tau_sat=float((np.abs(TAU[:, i]) >= 2.95).mean()), hf_share=float(hf[i]),
                     peak_hz=float(peak[i]), tilt_max=float(np.degrees(np.arccos(np.clip(-G[:, i, 2], -1, 1))).max())))
  agg = {}
  for r in rows:
    key = f"{r['support']}|lag{r['lag']}|{r['cmd']}"
    agg.setdefault(key, []).append(r)
  summary = {k: {m: float(np.mean([r[m] for r in v])) for m in v[0] if m not in ("support", "lag", "cmd")}
             for k, v in agg.items()}
  args.out.parent.mkdir(parents=True, exist_ok=True)
  args.out.write_text(json.dumps(dict(name=args.name, checkpoint=str(args.checkpoint), stand_h=STAND_H,
                                      replicas=args.replicas, seconds=args.seconds, summary=summary, rows=rows, no_noise=args.no_noise, ref_check=ref_check), indent=1))
  hdr = f"{'condition':28s} {'act_rate':>8s} {'act_acc':>8s} {'act_std':>8s} {'qd_rms':>7s} {'tau_sat':>7s} {'hf>8Hz':>7s} {'peakHz':>6s} {'tilt':>5s}"
  print(hdr)
  for k, v in summary.items():
    print(f"{k:28s} {v['act_rate']:8.3f} {v['act_acc']:8.3f} {v['act_std']:8.3f} {v['qd_rms']:7.2f} "
          f"{v['tau_sat']:7.3f} {v['hf_share']:7.3f} {v['peak_hz']:6.2f} {v['tilt_max']:5.0f}")
  np.savez_compressed(args.out.with_suffix(".npz"), A=A, Q=Q, QD=QD, cond=np.array([f"{c[0]}|{c[1]}|{c[2]}" for c in cond_of]))


if __name__ == "__main__":
  main()
