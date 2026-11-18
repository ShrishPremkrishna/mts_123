"""Opt Lab 2 Part 3: deploy export, round-trip check and limit audit (simulation only).

The spec's deploy path: "The controller already reproduces every reference table
from policy.json with its own phase clock ... The trained +-1.5 m/s command clip
ships inside the export." ``pupper_gait_deploy/deploy.sh <run>`` downloads the
run's W&B ``policy.json``, which ``PupperOnPolicyRunner`` re-exports at every
checkpoint save. This tool does NOT deploy anything. It:

1. **Exports** ``policy.json`` (+ ``gait_golden.json``, the robot-side C++ parity
   fixture) from the checkpoint in a play env built exactly like every other
   run here (standard physics flags, the checkpoint's reference dir, heading
   hold on so its contract is stamped), with the same exporter functions the
   runner and ``export-pupper-policy`` use.
2. **Round-trips** it: rolls the torch policy out over the seam sweeps plus
   turn/sidestep commands (flat, Lab 4 DR on), records the actor observations,
   and compares the actions of (a) the torch actor, (b) the exported JSON
   through ``json_forward`` (the RTNeural re-implementation, robot newest-first
   order), (c) the run dir's auto-exported ``policy.json`` (what the robot would
   download) and (d) the run's ONNX, all on the same recorded observations.
3. **Audits limits** against the repo's reviewed values: joint limits / kp / kd /
   action scale vs the shipped ``pupper_gait_deploy/policies/default_policy.json``;
   kp/kd vs the driver's ``kp_max`` / ``kd_max`` and the 3 Nm ``effort_max`` in the
   robot description; class driver limits vs that description (and its recorded
   sha256); the +-1.5 command clip; reviewed files unchanged vs ``origin/main``.
4. Runs the deploy repo's own checks offline: ``download_policy.py
   --validate-only`` and the host build of ``test/test_gait_reference.cpp``
   against the new golden fixture and policy.

  uv run --no-sync python -m mjlab.scripts.part3_export --checkpoint CKPT \
      --reference-dir REF --name NAME [--run-dir RUN_DIR] [--wandb-run ENTITY/PROJECT/ID]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np

from mjlab.scripts.part3_common import REPO_ROOT

MONO = REPO_ROOT.parent
DEPLOY_REPO = MONO / "pupper_gait_deploy"
SHIPPED_POLICY = DEPLOY_REPO / "policies" / "default_policy.json"
XACRO = MONO / "pupperv3-monorepo/ros2_ws/src/pupper_v3_description/description/components.xacro"
DRIVER_LIMITS = MONO / "foundation/configs/class-driver-joint-limits.json"
JSON_HPP_DIR = MONO / "pupperv3-monorepo/ros2_ws/src/neural_controller/modules/RTNeural/modules/json"
REVIEWED_FILES = (SHIPPED_POLICY, XACRO, DRIVER_LIMITS,
                  REPO_ROOT / "src/mjlab/asset_zoo/robots/pupper_v3/pupper_constants.py")
DEFAULT_OUT = Path.home() / "runs" / "opt-lab-2" / "p3-export"
DEFAULT_REPORT = REPO_ROOT / "report" / "part3" / "export"
PARITY_TOL = 1e-3  # export-pupper-policy's parity_tol and the runner's _PARITY_TOL
ONNX_TOL = 1e-4
SPEC_VX_CLIP = (-1.5, 1.5)

##
# Limit audit (pure; unit tested).
##


def parse_xacro_limits(text: str) -> dict[str, dict[str, float]]:
  """{joint: {position_min, position_max, effort_max, kp_max, kd_max}} from components.xacro."""
  out: dict[str, dict[str, float]] = {}
  for m in re.finditer(r'<joint name="([^"]+)">(.*?)</joint>', text, re.S):
    params = dict(re.findall(r'<param name="([a-z_]+)">([-0-9.eE+]+)</param>', m.group(2)))
    keys = ("position_min", "position_max", "effort_max", "kp_max", "kd_max")
    if all(k in params for k in keys):
      out[m.group(1)] = {k: float(params[k]) for k in keys}
  return out


def audit_limits(policy: dict, shipped: dict, xacro: dict, driver: dict, sim_effort_limit: float,
                 xacro_sha256: str | None = None) -> dict[str, Any]:
  """Return {checks: [{name, ok, detail}], ok}."""
  checks = []

  def check(name, ok, detail):
    checks.append(dict(name=name, ok=bool(ok), detail=detail))

  names = policy.get("gait_reference", {}).get("joint_names") or list(driver["joint_limits"])
  for key in ("joint_lower_limits", "joint_upper_limits"):
    d = float(np.max(np.abs(np.asarray(policy[key]) - np.asarray(shipped[key]))))
    check(f"{key} == shipped default_policy.json", d < 1e-6, f"max |diff| {d:.2e} rad")
  for key in ("kp", "kd", "action_scale"):
    check(f"{key} == shipped default_policy.json", math.isclose(policy[key], shipped[key], abs_tol=1e-9),
          f"{policy[key]} vs {shipped[key]}")
  for key in ("default_joint_pos",):
    d = float(np.max(np.abs(np.asarray(policy[key]) - np.asarray(shipped[key]))))
    check(f"{key} == shipped default_policy.json", d < 1e-6, f"max |diff| {d:.2e} rad")
  xj = [xacro[n] for n in names]
  check("kp <= driver kp_max (robot description)", all(policy["kp"] <= j["kp_max"] for j in xj),
        f"kp {policy['kp']} vs kp_max {sorted({j['kp_max'] for j in xj})}")
  check("kd <= driver kd_max (robot description)", all(policy["kd"] <= j["kd_max"] for j in xj),
        f"kd {policy['kd']} vs kd_max {sorted({j['kd_max'] for j in xj})}")
  effort = sorted({j["effort_max"] for j in xj})
  check("effort_max (current/torque limit) == 3.0 Nm == sim PUPPER_EFFORT_LIMIT",
        effort == [3.0] and math.isclose(sim_effort_limit, 3.0), f"xacro {effort}, sim {sim_effort_limit}")
  dl = driver["joint_limits"]
  bad = [n for n in names if not (math.isclose(dl[n][0], xacro[n]["position_min"]) and
                                  math.isclose(dl[n][1], xacro[n]["position_max"]))]
  check("class driver limits JSON == robot description position_min/max", not bad, f"mismatched: {bad}")
  lo, hi = np.asarray(policy["joint_lower_limits"]), np.asarray(policy["joint_upper_limits"])
  dlo = np.asarray([dl[n][0] for n in names])
  dhi = np.asarray([dl[n][1] for n in names])
  check("exported joint limits contain the driver limits (driver clamps tighter on hardware)",
        bool(np.all(lo <= dlo + 1e-6) and np.all(hi >= dhi - 1e-6)),
        f"model-minus-driver margin min {float(min((dlo - lo).min(), (hi - dhi).min())):.3f} rad")
  if xacro_sha256 is not None:
    rec = [s.get("sha256") for s in driver.get("sources", []) if str(s.get("path", "")).endswith("components.xacro")]
    check("robot description sha256 == the one the driver limits were reviewed against",
          xacro_sha256 in rec, f"{xacro_sha256[:12]} vs {[r[:12] for r in rec if r]}")
  clip = policy.get("command_clip") or {}
  check("command_clip vx == +-1.5 m/s (spec: the trained clip ships in the export)",
        clip.get("vx") is not None and tuple(clip["vx"]) == SPEC_VX_CLIP, f"{clip}")
  g = policy.get("gait_reference") or {}
  check("gait_reference present with fast-forward, fast-backward and lift tables",
        all(k in g for k in ("trot_table", "gallop_table", "gallop_back_table", "lift_table")),
        f"keys {sorted(k for k in g if k.endswith('_table'))}")
  if g:
    check("gait_reference switch constants: gallop_speed 0.5, dir_threshold 0.05",
          math.isclose(g["gallop_speed"], 0.5) and math.isclose(g["dir_threshold"], 0.05),
          f"gallop_speed {g['gallop_speed']}, gallop_freq_mult {g.get('gallop_freq_mult')}")
    check("gait_reference clamp == exported joint limits",
          np.allclose(g["clamp_lower"], lo) and np.allclose(g["clamp_upper"], hi), "")
  check("heading_hold contract stamped", "heading_hold" in policy, f"{policy.get('heading_hold')}")
  return dict(ok=all(c["ok"] for c in checks), checks=checks)


def compare_json(a: dict, b: dict) -> dict[str, Any]:
  """Field-by-field diff of two deploy JSONs (layers compared numerically)."""
  diffs = {}
  for k in sorted(set(a) | set(b)):
    if k not in a or k not in b:
      diffs[k] = "missing in " + ("first" if k not in a else "second")
    elif k == "layers":
      d = max(float(np.max(np.abs(np.asarray(la["weights"][i]) - np.asarray(lb["weights"][i]))))
              for la, lb in zip(a[k], b[k], strict=True) for i in (0, 1))
      if d > 0:
        diffs[k] = f"max |weight diff| {d:.2e}"
    elif a[k] != b[k]:
      diffs[k] = "differs"
  return diffs


def action_err(x, y):
  e = np.abs(np.asarray(x, np.float64) - np.asarray(y, np.float64))
  per = e.max(axis=1)
  return dict(max=float(per.max()), p99=float(np.percentile(per, 99)), median=float(np.median(per)), n=int(len(per)))


def sha256(path: Path) -> str:
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


##
# GPU parts.
##


def _record_obs(roll, every, seconds, seed):
  """Roll out the torch policy over seam sweeps + turn/sidestep; record actor obs."""
  import torch

  from mjlab.scripts import part3_seam as ps

  n = roll.num_envs
  scheds = [ps.sweep(1, 0.5), ps.sweep(-1, 1.0), ps.dither(1), ps.dither(-1)]
  T = int(round(seconds / ps.DT))
  cmd = np.zeros((T, n, 3))
  rng = np.random.default_rng(seed)
  for e in range(n):
    kind = e % 6
    if kind < 4:
      vx = scheds[kind]
      L = min(T, len(vx))
      cmd[:L, e, 0] = vx[:L]
    elif kind == 4:
      cmd[50:, e, 2] = rng.choice([-1.5, 1.5])  # turn in place (lift slot)
    else:
      cmd[50:, e, 1] = rng.choice([-0.4, 0.4])  # sidestep
  cmd_t = torch.tensor(cmd, dtype=torch.float32, device=roll.device)
  obs_rec, cmd_rec, act_rec = [], [], []
  roll.set_command(cmd_t[0])
  obs, _ = roll.wrapped.reset()
  with torch.inference_mode():
    for k in range(T):
      x = obs["actor"] if isinstance(obs, dict) or hasattr(obs, "keys") else obs
      actions = roll.policy(obs)
      if k % every == 0:
        obs_rec.append(x.detach().clone())
        cmd_rec.append(cmd_t[k].clone())
        act_rec.append(actions.detach().clone())
      roll.set_command(cmd_t[min(k + 1, T - 1)])
      obs, _, _, _ = roll.wrapped.step(actions)
  return torch.cat(obs_rec), torch.cat(cmd_rec), torch.cat(act_rec)


def export_and_roundtrip(args) -> dict[str, Any]:
  import torch

  from mjlab.scripts import part2_eval
  from mjlab.scripts.part3_common import rollout_conditions

  t0 = time.time()
  out = Path(args.out_dir) / args.name
  out.mkdir(parents=True, exist_ok=True)
  args.task, args.heading_hold = "Mjlab-MixedGaits-Flat-Pupper-v3", True
  report: dict[str, Any] = dict(name=args.name, checkpoint=str(args.checkpoint),
                                checkpoint_sha256=sha256(args.checkpoint),
                                reference_dir=str(args.reference_dir) if args.reference_dir else None)
  with rollout_conditions(args, dr=True) as cond:
    roll = part2_eval._Rollout(args, args.num_envs, 10)
    try:
      from mjlab.scripts import export_pupper_policy as X
      from mjlab.tasks.pupper.export import (
        command_clip_from_env,
        export_pupper_policy_from_env,
        json_forward,
        to_robot_order,
      )
      from mjlab.tasks.pupper_gait.export import (
        GAIT_OBS_COMPONENTS,
        GAIT_SINGLE_OBS_DIM,
        gait_frame_kind,
        gait_reference_metadata,
        heading_hold_metadata,
      )
      from mjlab.utils.torch import configure_torch_backends

      configure_torch_backends(allow_tf32=False)  # parity needs IEEE fp32 on the torch side
      env, actor = roll.env, roll.policy
      if gait_frame_kind(env) != "mixed":
        raise ValueError(f"expected a mixed-gaits frame, got {gait_frame_kind(env)}")
      policy = export_pupper_policy_from_env(
        actor, env, single_obs_dim=GAIT_SINGLE_OBS_DIM, obs_components=GAIT_OBS_COMPONENTS,
        command_clip=command_clip_from_env(env))
      policy["gait_reference"] = gait_reference_metadata(env)
      hh = heading_hold_metadata(env)
      if hh is not None:
        policy["heading_hold"] = hh
      obs, cmds, loop_act = _record_obs(roll, args.every, args.seconds, args.seed)
      with torch.no_grad():
        torch_act = actor.mlp(actor.obs_normalizer(obs)).cpu().numpy()
      torch_act_policy = loop_act.cpu().numpy()  # what the rollout's inference call returned
      frame = policy["single_observation_size"]
      robot_obs = to_robot_order(obs, obs.shape[-1] // frame, frame)
      golden = X.make_gait_golden(env, policy["gait_reference"])
      obs_np = obs.cpu().numpy()
    finally:
      roll.close()
  report["condition"] = cond
  policy_path = out / "policy.json"
  policy_path.write_text(json.dumps(policy))
  golden_path = out / "gait_golden.json"
  golden_path.write_text(json.dumps(golden))
  np.savez_compressed(out / "roundtrip_obs.npz", obs=obs_np.astype(np.float32), cmd=cmds.cpu().numpy(),
                      torch_actions=torch_act.astype(np.float32))
  rt: dict[str, Any] = dict(
    observations=int(len(obs_np)), every_steps=args.every, seconds=args.seconds,
    command_coverage=dict(vx_min=float(cmds[:, 0].min()), vx_max=float(cmds[:, 0].max()),
                          wz_absmax=float(cmds[:, 2].abs().max()), vy_absmax=float(cmds[:, 1].abs().max()),
                          frac_fast_slot=float((cmds[:, 0].abs() >= 0.5).float().mean())),
    torch_inference_vs_mlp=action_err(torch_act_policy, torch_act),
    exported_json_vs_torch=action_err(json_forward(policy, robot_obs), torch_act),
    tolerance=PARITY_TOL,
    golden_reference_residual_vs_training=golden["max_residual_vs_training"],
  )
  if args.run_dir:
    run_dir = Path(args.run_dir)
    auto = run_dir / "policy.json"
    if auto.is_file():
      auto_p = json.loads(auto.read_text())
      rt["run_dir_policy_json"] = dict(path=str(auto), sha256=sha256(auto),
                                       vs_torch=action_err(json_forward(auto_p, robot_obs), torch_act),
                                       field_diffs_vs_fresh_export=compare_json(policy, auto_p))
    onnx_files = sorted(run_dir.glob("*.onnx"))
    if onnx_files:
      rt["onnx"] = _onnx_check(onnx_files[0], obs_np, torch_act, args.onnx_samples)
  if args.wandb_run:
    rt["wandb_policy_json"] = _wandb_check(args.wandb_run, out, policy, robot_obs, torch_act)
  report["roundtrip"] = rt
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import PUPPER_EFFORT_LIMIT as lim  # noqa: N811

  report["limits"] = audit_limits(policy, json.loads(SHIPPED_POLICY.read_text()),
                                  parse_xacro_limits(XACRO.read_text()), json.loads(DRIVER_LIMITS.read_text()),
                                  float(lim), sha256(XACRO))
  report["limits"]["reviewed_files_vs_origin_main"] = _git_unchanged(REVIEWED_FILES)
  report["deploy_validator"] = _run([sys.executable, str(DEPLOY_REPO / "download_policy.py"),
                                     "--validate-only", str(policy_path)])
  report["deploy_cpp_gait_reference_test"] = _cpp_test(golden_path, policy_path)
  ok = (rt["exported_json_vs_torch"]["max"] < PARITY_TOL
        and rt["golden_reference_residual_vs_training"] < 1e-4
        and report["limits"]["ok"] and report["limits"]["reviewed_files_vs_origin_main"]["ok"]
        and report["deploy_validator"]["rc"] == 0 and report["deploy_cpp_gait_reference_test"]["rc"] == 0)
  if "run_dir_policy_json" in rt:
    ok = ok and rt["run_dir_policy_json"]["vs_torch"]["max"] < PARITY_TOL
  if "onnx" in rt:
    ok = ok and rt["onnx"]["vs_torch"]["max"] < ONNX_TOL
  report["artifacts"] = dict(policy_json=str(policy_path), policy_json_sha256=sha256(policy_path),
                             gait_golden=str(golden_path), obs=str(out / "roundtrip_obs.npz"))
  report["verdict"] = "PASS" if ok else "FAIL"
  report["hardware"] = "NOT deployed. Deployment needs a human operator and TA approval; see report/part3_notebook.md."
  if args.upload_wandb:
    report["wandb_upload"] = (_wandb_upload(args.wandb_run, policy_path, out) if ok else
                              dict(skipped="verdict FAIL: not uploading a policy that failed its checks"))
    ok = ok and report["wandb_upload"].get("verified", False)
    report["verdict"] = "PASS" if ok else "FAIL"
  report["wall_clock_s"] = time.time() - t0
  _write_report(report, Path(args.report_dir))
  print(json.dumps({k: report[k] for k in ("name", "verdict", "roundtrip")}, indent=1, default=str))
  for c in report["limits"]["checks"]:
    print(("OK  " if c["ok"] else "FAIL") + f" {c['name']}: {c['detail']}")
  return report


def _onnx_check(path, obs_np, torch_act, n):
  import onnxruntime as ort

  sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
  inp = sess.get_inputs()[0]
  idx = np.linspace(0, len(obs_np) - 1, min(n, len(obs_np))).astype(int)
  batch = inp.shape[0] if isinstance(inp.shape[0], int) else None
  if batch in (None, len(idx)):
    got = sess.run(None, {inp.name: obs_np[idx].astype(np.float32)})[0]
  else:
    got = np.concatenate([sess.run(None, {inp.name: obs_np[i : i + 1].astype(np.float32)})[0] for i in idx])
  return dict(path=str(path), sha256=sha256(path), samples=len(idx), vs_torch=action_err(got, torch_act[idx]),
              tolerance=ONNX_TOL)


def _wandb_check(run_path, out, policy, robot_obs, torch_act):
  from mjlab.tasks.pupper.export import json_forward

  try:
    import wandb

    f = wandb.Api().run(run_path).file("policy.json")
    dl = out / "wandb"
    dl.mkdir(exist_ok=True)
    f.download(root=str(dl), replace=True)
    p = json.loads((dl / "policy.json").read_text())
    return dict(run=run_path, updated_at=str(getattr(f, "updated_at", "")), sha256=sha256(dl / "policy.json"),
                vs_torch=action_err(json_forward(p, robot_obs), torch_act),
                field_diffs_vs_fresh_export=compare_json(policy, p))
  except Exception as exc:  # noqa: BLE001 - network/auth problems are reported, not fatal
    return dict(run=run_path, error=repr(exc))


def _wandb_upload(run_path, policy_path, out):
  """Upload the verified policy.json as the run's ``policy.json`` and read it back.

  The runner's per-save upload did not persist on finished runs here (the parent
  run 1c5r0n8c has no policy.json on W&B although its log reports uploads), so
  ``deploy.sh <run>`` would fail. Uploading after the run finished and
  re-downloading proves the file the robot will fetch is byte-identical.
  """
  import wandb

  try:
    run = wandb.Api().run(run_path)
    if run.state != "finished":
      return dict(run=run_path, error=f"run state {run.state!r}; upload only after the run finished")
    run.upload_file(str(policy_path), root=str(policy_path.parent))
    back = out / "wandb_readback"
    back.mkdir(exist_ok=True)
    wandb.Api().run(run_path).file("policy.json").download(root=str(back), replace=True)
    same = sha256(back / "policy.json") == sha256(policy_path)
    return dict(run=run_path, uploaded=str(policy_path), sha256=sha256(policy_path), verified=same)
  except Exception as exc:  # noqa: BLE001
    return dict(run=run_path, error=repr(exc), verified=False)


def _git_unchanged(paths):
  rows, ok = [], True
  for p in paths:
    p = Path(p)
    r = subprocess.run(["git", "-C", str(p.parent), "diff", "--quiet", "origin/main", "--", p.name],
                       capture_output=True)
    rows.append(dict(path=str(p.relative_to(MONO)), unchanged=r.returncode == 0, sha256=sha256(p)))
    ok &= r.returncode == 0
  return dict(ok=ok, files=rows)


def _run(cmd, cwd=None):
  r = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
  return dict(cmd=" ".join(map(str, cmd)), rc=r.returncode, tail=(r.stdout + r.stderr)[-2500:])


def _cpp_test(golden, policy):
  """Host build of the deploy repo's gait-reference parity test (no ROS)."""
  src = DEPLOY_REPO / "src"
  with tempfile.TemporaryDirectory() as tmp:
    inc = Path(tmp) / "include" / "neural_controller"
    inc.mkdir(parents=True)
    for h in ("gait_reference.hpp", "gait_reference_json.hpp", "command_filter.hpp", "obs_spec.hpp"):
      if (src / h).exists():
        (inc / h).write_bytes((src / h).read_bytes())
    exe = Path(tmp) / "test_gait_reference"
    b = _run(["g++", "-std=c++17", "-O2", "-I", str(Path(tmp) / "include"), "-I", str(JSON_HPP_DIR),
              f'-DGAIT_GOLDEN_JSON="{golden}"', "-o", str(exe), str(DEPLOY_REPO / "test/test_gait_reference.cpp")])
    if b["rc"]:
      return dict(stage="build", **b)
    return dict(stage="run", **_run([str(exe), str(golden), str(policy)]))


def _write_report(report, report_dir):
  report_dir.mkdir(parents=True, exist_ok=True)
  path = report_dir / f"{report['name']}.json"
  path.write_text(json.dumps(report, indent=2, default=str) + "\n")
  rt = report["roundtrip"]
  lines = [f"# Export round-trip: {report['name']} -- **{report['verdict']}**", "",
           f"Checkpoint `{report['checkpoint']}` (sha256 `{report['checkpoint_sha256'][:12]}`), reference "
           f"`{report['reference_dir']}`. Simulation/offline checks only; nothing was deployed.", "",
           "| comparison (same recorded observations) | n | max abs action err | p99 | tolerance |",
           "|---|---|---|---|---|"]

  def row(label, e, tol):
    lines.append(f"| {label} | {e['n']} | {e['max']:.2e} | {e['p99']:.2e} | {tol:g} |")

  row("exported policy.json (json_forward, robot order) vs torch actor", rt["exported_json_vs_torch"], PARITY_TOL)
  row("rollout inference actions vs actor.mlp(normalizer(x)) (obs capture check)", rt["torch_inference_vs_mlp"], PARITY_TOL)
  if "run_dir_policy_json" in rt:
    row("run dir auto-exported policy.json vs torch", rt["run_dir_policy_json"]["vs_torch"], PARITY_TOL)
  if "onnx" in rt:
    row(f"run ONNX ({Path(rt['onnx']['path']).name}) vs torch", rt["onnx"]["vs_torch"], ONNX_TOL)
  if "wandb_policy_json" in rt and "vs_torch" in rt["wandb_policy_json"]:
    row("W&B policy.json (what deploy.sh downloads) vs torch", rt["wandb_policy_json"]["vs_torch"], PARITY_TOL)
  lines += ["", f"Observations: {rt['observations']} (every {rt['every_steps']} steps over {rt['seconds']} s, "
            f"coverage {rt['command_coverage']}). Gait-reference golden residual vs the training term: "
            f"{rt['golden_reference_residual_vs_training']:.2e} (< 1e-4).", ""]
  for key in ("run_dir_policy_json", "wandb_policy_json"):
    if key in rt:
      lines.append(f"{key}: field diffs vs fresh export = {rt[key].get('field_diffs_vs_fresh_export', rt[key].get('error'))}")
  lines += ["", "## Limit audit (reviewed values)", "", "| check | ok | detail |", "|---|---|---|"]
  for c in report["limits"]["checks"]:
    lines.append(f"| {c['name']} | {'yes' if c['ok'] else '**NO**'} | {c['detail']} |")
  g = report["limits"]["reviewed_files_vs_origin_main"]
  lines.append(f"| reviewed files unchanged vs origin/main | {'yes' if g['ok'] else '**NO**'} | "
               f"{', '.join(f['path'] for f in g['files'])} |")
  for key, label in (("deploy_validator", "pupper_gait_deploy download_policy.py --validate-only"),
                     ("deploy_cpp_gait_reference_test", "pupper_gait_deploy test_gait_reference.cpp (host build)")):
    r = report[key]
    tail = r["tail"].strip().splitlines()[-1] if r["tail"].strip() else ""
    lines.append(f"| {label} | {'yes' if r['rc'] == 0 else '**NO**'} | rc {r['rc']}: {tail[:200]} |")
  lines += ["", report["hardware"], ""]
  (report_dir / f"{report['name']}.md").write_text("\n".join(lines))


def main():
  from mjlab.scripts import part2_eval

  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  part2_eval._add_rollout_args(p)
  p.set_defaults(out_dir=DEFAULT_OUT)
  p.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT)
  p.add_argument("--run-dir", type=Path, help="Training run dir (auto-exported policy.json, *.onnx)")
  p.add_argument("--wandb-run", help="entity/project/run_id: also check the W&B policy.json deploy.sh fetches")
  p.add_argument("--upload-wandb", action="store_true",
                 help="After all checks pass, upload policy.json to --wandb-run and verify the read-back")
  p.add_argument("--num-envs", type=int, default=48)
  p.add_argument("--seconds", type=float, default=16.0)
  p.add_argument("--every", type=int, default=4)
  p.add_argument("--onnx-samples", type=int, default=2000)
  args = p.parse_args()
  if args.upload_wandb and not args.wandb_run:
    p.error("--upload-wandb needs --wandb-run")
  part2_eval._prepare(args, p.error)
  rep = export_and_roundtrip(args)
  sys.exit(0 if rep["verdict"] == "PASS" else 1)


if __name__ == "__main__":
  main()
