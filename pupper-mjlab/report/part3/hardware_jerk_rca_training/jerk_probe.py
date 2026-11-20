"""Hardware-jerk RCA probe: action smoothness of one policy on a simulated test stand and on the ground.

Read-only eval (no training). One policy per process, because the gait reference directory is
read once at import. Reuses part2_eval._Rollout under part3_common.rollout_conditions, like
reward_shaping_probe/shape_probe.py, so every policy runs on the same plant (our current
deployment-contract physics: target clamp, driver limits, torso contact, composed CoM DR).

Conditions
  stand_nodr / stand_dr : base pinned upright and motionless 0.30 m above flat ground every
                          physics-step (the training `hold_base_airborne` event, never released),
                          so the feet never touch: a simulated test stand. nodr = nominal motor
                          gains (kp 5.5, kd 0.25), dr = Lab 4 randomization.
  flat_nodr / flat_dr / bumpy_dr : on the ground.

Reference handling
  Every policy observes a 12-dim gait-reference offset. Our policies are run with their own
  reference dir. fasxqy4z (course commit 9cfcb14, Colab) is run with its own exported tables:
  the four tables from its policy.json are written into the reference-table cache before the env
  is built, and its clock (1.5 Hz, gallop_speed 1000 = never gallops) is set on the obs term.
  For our policies the tables built from the reference dir are compared with their policy.json
  tables and the max difference is recorded (a check of the mapping).

  python jerk_probe.py --policy bumpy --out OUT.json [--envs-per 64] [--conditions ...]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

RUNS = Path.home() / "runs" / "opt-lab-2"
REPO = Path(__file__).resolve().parents[3]
R2 = REPO / "configs/opt-lab-2/p2/s42/r2-lag"
POLICIES = {  # name: (checkpoint, policy.json, reference dir, override tables from policy.json?)
  "fasxqy4z": (RUNS / "ext-fasxqy4z/model_2999.pt", RUNS / "ext-fasxqy4z/policy.json", R2, True),
  "bumpy": (RUNS / "rp-p3-final-bumpyft-kpwide-leg4/model_6492.pt",
            RUNS / "rp-p3-final-bumpyft-kpwide-leg4/policy.json", R2, False),
  "flatctrl": (RUNS / "rp-p3-final-flatctrl-kpwide-leg4/model_6492.pt",
               RUNS / "rp-p3-final-flatctrl-kpwide-leg4/policy.json", R2, False),
  "p1": (RUNS / "rp-p1-gallop_c-s42/model_2999.pt", RUNS / "rp-p1-gallop_c-s42/policy.json",
         REPO / "configs/opt-lab-2/gallop_c", False),
}
GROUPS = [("cmd0", (0.0, 0.0, 0.0)), ("fwd0.1", (0.1, 0.0, 0.0)), ("fwd0.3", (0.3, 0.0, 0.0)),
          ("fwd0.5", (0.5, 0.0, 0.0))]
CONDS = {  # name: (part3 condition, test stand?, fixed kp multiplier or None, actor-obs lag in steps)
  "stand_nodr": ("flat_nodr", True, None, 0), "stand_dr": ("flat_dr", True, None, 0),
  "flat_nodr": ("flat_nodr", False, None, 0), "flat_dr": ("flat_dr", False, None, 0),
  "bumpy_dr": ("bumpy_dr", False, None, 0),
  # Stress tests (all other mechanics nominal; kd nominal): fixed kp multiplier, and +1 step
  # (20 ms) of observation latency on the whole actor observation.
  "stand_kp0.6": ("flat_dr", True, 0.6, 0), "stand_kp1.0": ("flat_dr", True, 1.0, 0),
  "stand_kp1.3": ("flat_dr", True, 1.3, 0), "stand_kp1.6": ("flat_dr", True, 1.6, 0),
  "stand_kp1.0_lag1": ("flat_dr", True, 1.0, 1), "flat_kp1.0_lag1": ("flat_dr", False, 1.0, 1),
}
SECONDS, DT, SKIP_S, HF_HZ = 15.0, 0.02, 3.0, 5.0
TABLE_KEYS = {"trot": "trot_table", "reach": "gallop_table", "reach_back": "gallop_back_table",
              "lift": "lift_table"}
STAND_HEIGHT = 0.30


def probe(pname, cond, envs_per, seed):
  import torch

  from mjlab.scripts import part2_eval as pe
  from mjlab.scripts.part3_common import CONDITIONS

  ckpt, pjson, ref, override = POLICIES[pname]
  pol = json.loads(Path(pjson).read_text())
  gr = pol["gait_reference"]
  p3cond, stand, kp_fixed, lag = CONDS[cond]
  task, dr = CONDITIONS[p3cond]
  p = argparse.ArgumentParser()
  pe._add_rollout_args(p)
  a = p.parse_args(["--checkpoint", str(ckpt), "--reference-dir", str(ref), "--name", f"{pname}-{cond}",
                    "--seed", str(seed), "--task", task, "--out-dir", "/tmp/unused",
                    "--allow-reference-mismatch"])
  pe._prepare(a, lambda m: sys.exit(m))
  a.dr, a.condition = dr, cond
  # Task imports only after _prepare has exported PUPPER_REFERENCE_DIR (read once, at import).
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
  from mjlab.managers.event_manager import EventTermCfg
  from mjlab.scripts import pupper_locomotion as cli
  from mjlab.scripts.part3_common import rollout_conditions
  from mjlab.tasks.pupper_gait.mdp import gait as gmod
  from mjlab.tasks.pupper_gait.mdp import gait_reference as gref
  from mjlab.tasks.pupper_gait.mdp.warmup import hold_base_airborne

  def settings(s_args, play):
    cfg, agent = base_settings(s_args, play)  # = rollout_conditions' wrapper (checks Lab 4 DR first)
    terms = [cfg.observations["actor"].terms["policy"], cfg.rewards["gait_tracking"]]
    if "gait_reference" in cfg.observations["critic"].terms:
      terms.append(cfg.observations["critic"].terms["gait_reference"])
    for t in terms:  # each policy runs with the clock it was trained and exported with
      t.params["frequency"] = float(gr["frequency"])
      t.params["gallop_speed"] = float(gr["gallop_speed"])
    if kp_fixed is not None:
      for e in ("foot_friction", "encoder_bias", "base_com", "inertial"):
        cfg.events.pop(e, None)
      cfg.events["pd_gains"].params["kp_range"] = (kp_fixed, kp_fixed)
      cfg.events["pd_gains"].params["kd_range"] = (1.0, 1.0)
    if lag:
      pt = cfg.observations["actor"].terms["policy"]
      pt.delay_min_lag = pt.delay_max_lag = int(lag)
    if stand:
      cfg.events["test_stand"] = EventTermCfg(
        func=hold_base_airborne, mode="step", params={"height": STAND_HEIGHT, "until_step": 10**12})
    return cfg, agent

  gref._GAIT_FREQ_MULT["reach"] = float(gr["gallop_freq_mult"])
  dev = a.device
  if override:
    for g, key in TABLE_KEYS.items():
      gmod._TABLE_CACHE[(int(gr["n_samples"]), g, str(dev))] = torch.tensor(
        gr[key], dtype=torch.float32, device=dev)
  G = len(GROUPS)
  n = G * envs_per
  T = int(round(SECONDS / DT))
  try:
    with rollout_conditions(a, dr=dr):
      base_settings = cli.settings
      cli.settings = settings
      try:
        roll = pe._Rollout(a, n, T)
      finally:
        cli.settings = base_settings
      assert abs(roll.dt - DT) < 1e-9
      robot, env = roll.robot, roll.env
      table_check = {}
      for g, key in TABLE_KEYS.items():
        built = gmod._reference_table(env, int(gr["n_samples"]), g).cpu().numpy()
        table_check[g] = float(np.abs(built - np.asarray(gr[key])).max())
      jid = [robot.joint_names.index(j) for j in JOINT_NAMES]
      contact = env.scene["feet_ground_contact"]
      am = env.action_manager
      term = am.get_term("joint_pos")
      assert list(term.target_names) == list(JOINT_NAMES), term.target_names
      clip = term._clip[0].cpu().numpy() if getattr(term, "_clip", None) is not None else None
      lo = np.asarray(pol["joint_lower_limits"]); hi = np.asarray(pol["joint_upper_limits"])
      cmd = torch.tensor([g[1] for g in GROUPS], dtype=torch.float32, device=dev).repeat_interleave(envs_per, 0)
      B = {k: torch.zeros(T, n, 12, device=dev) for k in ("act", "tgt", "q", "qd", "tau")}
      B["contact"] = torch.zeros(T, n, 4, dtype=torch.bool, device=dev)
      data = robot.data
      terminated = torch.zeros(T, n, dtype=torch.bool, device=dev)
      jac = {}
      roll.set_command(cmd)
      obs, _ = roll.wrapped.reset()
      jac_step = int(8.0 / DT)
      with torch.inference_mode():
        for k in range(T):
          roll.set_command(cmd)
          actions = roll.policy(obs)
          if k == jac_step and not lag:
            jac = jacobians(roll.policy, obs, actions, am.action, envs_per, G)
          obs, _, _, _ = roll.wrapped.step(actions)
          B["act"][k] = am.action
          B["tgt"][k] = term._processed_actions
          B["q"][k] = data.joint_pos[:, jid]
          B["qd"][k] = data.joint_vel[:, jid]
          B["tau"][k] = data.qfrc_actuator[:, jid]
          B["contact"][k] = contact.data.found > 0
          terminated[k] = env.reset_terminated
      X = {k: v.cpu().numpy() for k, v in B.items()}
      term_np = terminated.cpu().numpy()
      roll.close()
  finally:
    pass

  skip = int(SKIP_S / DT)
  fell = term_np.any(0)
  out = {"_meta": dict(policy=pname, checkpoint=str(ckpt), condition=cond, stand=stand, task=task, dr=dr,
                       kp_fixed=kp_fixed, obs_lag_steps=lag, reference_dir=str(ref), tables_from_policy_json=override, table_maxdiff=table_check,
                       frequency=gr["frequency"], gallop_speed=gr["gallop_speed"],
                       gallop_freq_mult=gr["gallop_freq_mult"], seed=seed, seconds=SECONDS, skip_s=SKIP_S,
                       clip=None if clip is None else clip.tolist())}
  for gi, (gname, c) in enumerate(GROUPS):
    sl = slice(gi * envs_per, (gi + 1) * envs_per)
    keep = ~fell[sl]
    rec = dict(cmd=c, envs=envs_per, fell=int(fell[sl].sum()), survivors=int(keep.sum()),
               jacobian=jac.get(gname))
    if keep.sum() == 0:
      out[gname] = rec
      continue
    g = {k: v[skip:, sl][:, keep] for k, v in X.items()}
    act, tgt, q, qd, tau = g["act"], g["tgt"], g["q"], g["qd"], g["tau"]
    da = np.diff(act, axis=0)
    a0 = act - act.mean(0, keepdims=True)
    P = (np.abs(np.fft.rfft(a0, axis=0)) ** 2).sum((1, 2))  # summed over envs and joints
    f = np.fft.rfftfreq(act.shape[0], DT)
    band = f > 0.2
    qd0 = qd - qd.mean(0, keepdims=True)
    Pq = (np.abs(np.fft.rfft(qd0, axis=0)) ** 2).sum((1, 2))
    tq = (np.abs(np.fft.rfft(tgt - tgt.mean(0, keepdims=True), axis=0)) ** 2).sum((1, 2))
    at_clip = np.zeros_like(tgt, dtype=bool) if clip is None else (
      (tgt <= clip[:, 0] + 1e-4) | (tgt >= clip[:, 1] - 1e-4))
    near_lim = (q <= lo + 0.05) | (q >= hi - 0.05)
    # What the on-robot controller would send: neural_controller.cpp clamps default + scale * a to
    # the policy.json joint limits (wider than the sim action clip for the abduction joints).
    hw = np.clip(np.asarray(pol["default_joint_pos"]) + float(pol["action_scale"]) * act, lo, hi)
    hw_at = (hw <= lo + 1e-4) | (hw >= hi - 1e-4)
    rec.update(
      action_abs_step=float(np.abs(da).mean()),                 # mean |a_t - a_{t-1}| per joint per step
      target_abs_step_rad=float(np.abs(np.diff(tgt, axis=0)).mean()),
      hw_target_abs_step_rad=float(np.abs(np.diff(hw, axis=0)).mean()),
      hw_target_at_limit_frac=float(hw_at.mean()),
      sim_vs_hw_target_mean_abs_diff=float(np.abs(hw - tgt).mean()),
      action_rate_l2=float((da ** 2).sum(-1).mean()),            # the action_rate_l2 reward quantity
      action_rate_l2_p95_env=float(np.percentile((da ** 2).sum(-1).mean(0), 95)),
      action_hf_share=float(P[f > HF_HZ].sum() / max(P[band].sum(), 1e-12)),
      target_hf_share=float(tq[f > HF_HZ].sum() / max(tq[band].sum(), 1e-12)),
      jointvel_hf_share=float(Pq[f > HF_HZ].sum() / max(Pq[band].sum(), 1e-12)),
      action_peak_hz=float(f[band][np.argmax(P[band])]),
      action_std_time=float(act.std(0).mean()),                 # per-env per-joint temporal std of a
      action_abs_mean=float(np.abs(act).mean()),
      action_abs_p99=float(np.percentile(np.abs(act), 99)),
      joint_speed_abs=float(np.abs(qd).mean()),
      joint_speed_p99=float(np.percentile(np.abs(qd), 99)),
      torque_abs_mean=float(np.abs(tau).mean()),
      torque_abs_p99=float(np.percentile(np.abs(tau), 99)),
      torque_abs_max=float(np.abs(tau).max()),
      torque_sat_frac=float((np.abs(tau) >= 0.99 * 3.0).mean()),
      target_at_clip_frac=float(at_clip.mean()),
      q_near_limit_frac=float(near_lim.mean()),
      feet_contact_frac=float(g["contact"].mean()),
      per_joint=dict(
        action_abs_step=np.abs(da).mean((0, 1)).round(4).tolist(),
        action_mean=act.mean((0, 1)).round(3).tolist(),
        target_at_clip_frac=at_clip.mean((0, 1)).round(3).tolist(),
        target_mean=tgt.mean((0, 1)).round(3).tolist(),
        target_abs_step=np.abs(np.diff(tgt, axis=0)).mean((0, 1)).round(4).tolist(),
        hw_target_abs_step=np.abs(np.diff(hw, axis=0)).mean((0, 1)).round(4).tolist(),
        hw_target_mean=hw.mean((0, 1)).round(3).tolist(),
        hw_target_at_limit_frac=hw_at.mean((0, 1)).round(3).tolist(),
      ),
      # spectrum, 1 Hz bins, for plots
      action_power_1hz=[float(P[(f >= b) & (f < b + 1)].sum() / max(P[band].sum(), 1e-12)) for b in range(26)],
    )
    out[gname] = rec
  return out


def jacobians(policy, obs, act, last_action, envs_per, G, eps=0.02):
  """Finite-difference sensitivity of the action to the newest-frame last_action and joint_pos_rel.

  Locates the newest frame by matching the last_action slice against the action manager's
  current action. Returns per command group: spectral radius / norms of
    J_a = d a_t / d last_action   (pure policy-internal feedback, identical in sim and on the robot)
    J_q = d a_t / d joint_pos_rel (feedback through the plant)
  and the eigenvalue of J_a + 0.75 J_q with the most negative real part (period-2 risk if <= -1
  under the crude assumption q follows the target within one step).
  """
  import torch

  from tensordict import TensorDict

  x = obs["actor"]
  N = x.shape[0]
  o = x.reshape(N, 20, 48)

  def run(oo):
    td = obs.clone()
    td["actor"] = oo.reshape(N, -1)
    return policy(td)

  la = last_action
  err = [(o[:, k, 24:36] - la).abs().max().item() for k in range(20)]
  k_new = int(np.argmin(err))
  res = {"newest_frame_index": k_new, "newest_frame_match_err": err[k_new]}
  Ja = torch.zeros(N, 12, 12, device=x.device)
  Jq = torch.zeros(N, 12, 12, device=x.device)
  for j in range(12):
    for off, J in ((24, Ja), (12, Jq)):
      op = o.clone(); om = o.clone()
      op[:, k_new, off + j] += eps; om[:, k_new, off + j] -= eps
      J[:, :, j] = (run(op) - run(om)) / (2 * eps)
  out = {}
  for gi in range(G):
    sl = slice(gi * envs_per, (gi + 1) * envs_per)
    ja = Ja[sl].double().cpu().numpy(); jq = Jq[sl].double().cpu().numpy()
    M = ja + 0.75 * jq
    ev_a = np.linalg.eigvals(ja); ev_m = np.linalg.eigvals(M)
    out_g = dict(
      Ja_spec_radius=float(np.abs(ev_a).max(-1).mean()),
      Ja_fro=float(np.linalg.norm(ja, axis=(1, 2)).mean()),
      Ja_diag_mean=float(np.diagonal(ja, axis1=1, axis2=2).mean()),
      Jq_fro=float(np.linalg.norm(jq, axis=(1, 2)).mean()),
      Jq_diag_mean=float(np.diagonal(jq, axis1=1, axis2=2).mean()),
      Jq_2norm=float(np.linalg.norm(jq, ord=2, axis=(1, 2)).mean()),
      M_spec_radius=float(np.abs(ev_m).max(-1).mean()),
      M_min_real=float(ev_m.real.min(-1).mean()),
    )
    out_g.update(res)
    out[["cmd0", "fwd0.1", "fwd0.3", "fwd0.5"][gi]] = out_g
  return out


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--policy", required=True, choices=list(POLICIES))
  ap.add_argument("--out", required=True)
  ap.add_argument("--envs-per", type=int, default=64)
  ap.add_argument("--seed", type=int, default=1234)
  ap.add_argument("--conditions", default=",".join(CONDS))
  cli_a = ap.parse_args()
  res = {}
  for cond in cli_a.conditions.split(","):
    t0 = time.time()
    print(f"[jerk] {cli_a.policy} {cond}", flush=True)
    res[cond] = probe(cli_a.policy, cond, cli_a.envs_per, cli_a.seed)
    print(f"[jerk] {cond} done in {time.time() - t0:.0f}s", flush=True)
    Path(cli_a.out).write_text(json.dumps(res, indent=1) + "\n")
  print("done", flush=True)
