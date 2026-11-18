"""Part 1 gait-design checks for an authored fast reference dir (npz + manifest).

Two subcommands, both run against exactly the reference dir that training uses:

  analyze REFDIR OUT_PREFIX
      Kinematic check from the joint table alone (FK of the stored angles):
      duty factor per leg, footfall order, feet-down statistics, flight fraction,
      implied no-slip body speed (stride x cadence), peak joint speed, the largest
      joint change per 20 ms control step and the kp * dq torque proxy the course
      uses for actuator feasibility (3 Nm limit), and margin to the class driver
      limits (the deployment joint-limit clamp). Writes OUT_PREFIX.json and
      OUT_PREFIX.png (foot x-z paths + footfall diagram).

  render REFDIR OUT_PREFIX [--seconds 4] [--no-gravity]
      Open-loop playback in MuJoCo with gravity ON (default): joints are driven
      to the reference at the fast-slot cadence, forward then backward (time
      reversed, as the task plays it). Records whether the reference carries the
      robot (achieved vx, pitch, tilt, falls) and writes OUT_PREFIX_{fwd,back}.gif
      plus OUT_PREFIX_render.json. An open-loop reference is not a policy; this
      only flags references that tip the body over by themselves.

Run from pupper-mjlab:  MUJOCO_GL=egl uv run --no-sync python scripts/opt_lab_2/part1_gait_check.py ...
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

CONTROL_DT = 0.02
KP = 5.5
KD = 0.25
EFFORT = 3.0
BASE_CYCLE_S = 0.75
LEGS = ("FR", "FL", "BR", "LB")
DRIVER_LIMITS = Path(__file__).resolve().parents[3] / "foundation/configs/class-driver-joint-limits.json"


def load_ref(refdir: Path):
  refdir = Path(refdir)
  manifest = json.loads((refdir / "reference_manifest.json").read_text())
  with np.load(refdir / "captured_reach_gaits.npz", allow_pickle=False) as f:
    fwd = np.asarray(f["reach_fwd"], dtype=np.float64)
  return fwd, manifest


def foot_paths(table):
  from mjlab.tasks.pupper_gait.mdp.gait_reference import _FK
  return np.stack([[_FK[leg](row[3 * leg:3 * leg + 3]) for leg in range(4)] for row in table])  # (N,4,3)


def analyze(refdir: Path, out_prefix: Path):
  table, manifest = load_ref(refdir)
  mult = float(manifest["frequency_mult"])
  cycle = BASE_CYCLE_S / mult
  n = len(table)
  feet = foot_paths(table)
  z = feet[:, :, 2]
  down = z < (z.min(axis=0, keepdims=True) + 0.001)  # stance plane, 1 mm tolerance
  duty = down.mean(axis=0)
  count = down.sum(axis=1)
  # Touchdown phase = first down sample after an up sample.
  td = []
  for leg in range(4):
    d = down[:, leg]
    edges = np.flatnonzero(d & ~np.roll(d, 1))
    td.append(float(edges[0] / n) if len(edges) else float("nan"))
  # Cyclic footfall order, starting from the left hind (LB) touchdown.
  rel = [((t - td[3]) % 1.0) for t in td]
  order = [LEGS[i] for i in np.argsort(rel)]
  # Implied body speed: stance foot slides backward relative to body.
  dt_sample = cycle / n
  vx = np.gradient(np.unwrap(feet[:, :, 0], axis=0), dt_sample, axis=0)
  stance_speed = [float(-np.median(vx[down[:, leg], leg])) for leg in range(4)]
  stride = [float(feet[down[:, leg], leg, 0].max() - feet[down[:, leg], leg, 0].min()) for leg in range(4)]
  # Joint change per 20 ms control step at the playback cadence (periodic interp).
  steps_per_sample = CONTROL_DT / dt_sample
  grid = np.arange(n * 10) / (n * 10)
  xp = np.arange(n + 1) / n
  fp = np.vstack([table, table[:1]])
  dense = np.stack([np.interp(grid, xp, fp[:, j]) for j in range(12)], axis=1)
  shift = int(round(steps_per_sample * 10))
  dq_step = np.abs(np.roll(dense, -shift, axis=0) - dense).max(axis=0)
  vel = np.abs(np.diff(np.vstack([table, table[:1]]), axis=0)).max(axis=0) / dt_sample
  limits = json.loads(DRIVER_LIMITS.read_text())["joint_limits"]
  names = manifest["joint_order"]
  lo = np.array([limits[k][0] for k in names]); hi = np.array([limits[k][1] for k in names])
  margin = np.minimum(table.min(axis=0) - lo, hi - table.max(axis=0))
  result = dict(
    refdir=str(refdir), frequency_mult=mult, cycle_s=cycle, stride_hz=1 / cycle,
    duty_factor=dict(zip(LEGS, duty.round(3).tolist())),
    touchdown_phase=dict(zip(LEGS, td)), footfall_order=order,
    feet_down_min=int(count.min()), feet_down_mean=float(count.mean()), flight_fraction=float((count == 0).mean()),
    feet_down_histogram={int(k): float((count == k).mean()) for k in range(5)},
    stance_stride_m=dict(zip(LEGS, np.round(stride, 4).tolist())),
    implied_speed_m_s=float(np.mean(stance_speed)),
    peak_joint_speed_rad_s=float(vel.max()), peak_joint_speed_by_joint=dict(zip(names, vel.round(2).tolist())),
    max_dq_per_control_step_rad=float(dq_step.max()), kp_dq_torque_proxy_nm=float(KP * dq_step.max()),
    kd_peak_speed_torque_nm=float(KD * vel.max()), effort_limit_nm=EFFORT,
    driver_limit_min_margin_rad=float(margin.min()), driver_limit_worst_joint=names[int(np.argmin(margin))],
    stance_height_m=float(-z.min()),
  )
  out_prefix = Path(out_prefix); out_prefix.parent.mkdir(parents=True, exist_ok=True)
  out_prefix.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
  _plot(feet, down, result, out_prefix.with_suffix(".png"), manifest)
  print(json.dumps({k: result[k] for k in ("duty_factor", "footfall_order", "feet_down_min", "flight_fraction",
                                           "implied_speed_m_s", "peak_joint_speed_rad_s", "kp_dq_torque_proxy_nm",
                                           "driver_limit_min_margin_rad")}, indent=1))
  return result


def _plot(feet, down, result, path, manifest):
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  colors = {"FR": "#1f77b4", "FL": "#ff7f0e", "BR": "#2ca02c", "LB": "#d62728"}
  fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw=dict(width_ratios=[1.1, 1]))
  ax = axes[0]
  hip = {"FR": (0.06, -0.09), "FL": (0.06, 0.09), "BR": (-0.11, -0.09), "LB": (-0.11, 0.09)}
  for leg, name in enumerate(LEGS):
    ax.plot(feet[:, leg, 0] - hip[name][0], feet[:, leg, 2], color=colors[name], label=name)
  ax.set_xlabel("foot x rel. hip [m] (+ forward)"); ax.set_ylabel("foot z [m]"); ax.set_aspect("equal")
  ax.legend(fontsize=8, ncol=4); ax.set_title("Foot paths (body frame, from FK of the table)", fontsize=9)
  ax = axes[1]
  n = len(down)
  phase = np.arange(n) / n
  order = ["LB", "BR", "FR", "FL"]
  for row, name in enumerate(order):
    leg = LEGS.index(name)
    d = down[:, leg]
    for i in np.flatnonzero(d):
      ax.barh(row, 1 / n, left=phase[i], color=colors[name], height=0.7, linewidth=0)
  ax.set_yticks(range(4)); ax.set_yticklabels(["LH (LB)", "RH (BR)", "RF (FR)", "LF (FL)"])
  ax.invert_yaxis(); ax.set_xlim(0, 1); ax.set_xlabel("phase (forward)")
  ax.set_title(f"Footfall (bars = stance). duty {np.mean(list(result['duty_factor'].values())):.2f}, "
               f"min feet down {result['feet_down_min']}, flight {result['flight_fraction']:.0%}", fontsize=9)
  p = manifest.get("provenance", {}).get("parameters", {})
  fig.suptitle(f"{Path(result['refdir']).name}: half-stride {p.get('half_stride')}, stance {p.get('stance_z')}, "
               f"lift {p.get('swing_lift')}, swing kf {p.get('swing_keyframes')}, {result['frequency_mult']}x clock "
               f"({result['cycle_s']:.3f} s); implied {result['implied_speed_m_s']:.2f} m/s; "
               f"peak {result['peak_joint_speed_rad_s']:.0f} rad/s; kp*dq {result['kp_dq_torque_proxy_nm']:.1f} Nm",
               fontsize=8.5)
  fig.tight_layout()
  fig.savefig(path, dpi=110)
  plt.close(fig)


def render(refdir: Path, out_prefix: Path, seconds: float, gravity: bool, gif: bool):
  os.environ["PUPPER_REFERENCE_DIR"] = str(Path(refdir).resolve())
  import torch
  import mjlab.tasks  # noqa: F401
  from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES, PUPPER_ACTION_SCALE
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.tasks.pupper_gait.mdp.gait_reference import _GAIT_FREQ_MULT, build_joint_reference_table
  from mjlab.tasks.pupper_gait.pupper_gait_env_cfg import GAIT_FREQUENCY
  from mjlab.tasks.registry import load_env_cfg

  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  mult = _GAIT_FREQ_MULT["reach"]
  freq = GAIT_FREQUENCY * mult
  table = torch.tensor(build_joint_reference_table(100, "reach"), device=device, dtype=torch.float32)
  results = {}
  for direction in ("fwd", "back"):
    cfg = load_env_cfg("Mjlab-VelocityFS-Flat-Pupper-v3", play=True)
    cfg.scene.num_envs = 1
    cfg.viewer.height, cfg.viewer.width = 360, 480
    cfg.viewer.distance = 0.9; cfg.viewer.elevation = -15.0; cfg.viewer.azimuth = 120.0
    if not gravity:
      cfg.sim.mujoco.gravity = (0.0, 0.0, 0.0)
    env = ManagerBasedRlEnv(cfg=cfg, device=device, render_mode="rgb_array" if gif else None)
    robot = env.scene["robot"]
    default = robot.data.default_joint_pos
    env.reset()
    frames, vx, pitch, tilt, height = [], [], [], [], []
    steps = int(seconds / env.step_dt)
    with torch.inference_mode():
      for k in range(steps):
        t = env.episode_length_buf.float() * env.step_dt
        phase = torch.frac(t * freq)
        if direction == "back":
          phase = torch.frac(1.0 - phase)
        x = phase * 100
        i0 = torch.floor(x).long() % 100; i1 = (i0 + 1) % 100
        frac = (x - torch.floor(x)).unsqueeze(1)
        ref = table[i0] * (1 - frac) + table[i1] * frac
        # Ramp in over the first 0.5 s so the start transient does not dominate.
        ramp = min(1.0, k * env.step_dt / 0.5)
        action = ramp * (ref - default) / PUPPER_ACTION_SCALE
        env.step(action)
        q = robot.data.root_link_quat_w[0].cpu().numpy()
        w, xq, yq, zq = q
        pitch.append(float(np.arcsin(np.clip(2 * (w * yq - zq * xq), -1, 1))))
        tilt.append(float(np.arccos(np.clip(1 - 2 * (xq ** 2 + yq ** 2), -1, 1))))
        vx.append(float(robot.data.root_link_lin_vel_b[0, 0]))
        height.append(float(robot.data.root_link_pos_w[0, 2]))
        if gif and k % 2 == 0:
          frames.append(env.render())
    env.close()
    settled = slice(int(1.0 / 0.02), None)
    results[direction] = dict(mean_vx_m_s=float(np.mean(vx[settled])), mean_pitch_deg=float(np.degrees(np.mean(pitch[settled]))),
                              max_tilt_deg=float(np.degrees(np.max(tilt))), fell_45deg=bool(np.max(tilt) > np.radians(45)),
                              mean_height_m=float(np.mean(height[settled])))
    if gif:
      import imageio.v2 as imageio
      path = Path(f"{out_prefix}_{direction}.gif")
      path.parent.mkdir(parents=True, exist_ok=True)
      imageio.mimsave(path, frames, duration=2 * 0.02 * 2, loop=0)  # half speed
  result = dict(refdir=str(refdir), gravity=gravity, seconds=seconds, frequency_mult=mult, open_loop=results,
                note="open-loop joint playback, not a policy; gifs play at half speed")
  Path(f"{out_prefix}_render.json").write_text(json.dumps(result, indent=2) + "\n")
  print(json.dumps(result, indent=1))


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="cmd", required=True)
  a = sub.add_parser("analyze"); a.add_argument("refdir", type=Path); a.add_argument("out", type=Path)
  r = sub.add_parser("render"); r.add_argument("refdir", type=Path); r.add_argument("out", type=Path)
  r.add_argument("--seconds", type=float, default=4.0); r.add_argument("--no-gravity", action="store_true")
  r.add_argument("--no-gif", action="store_true")
  args = p.parse_args()
  if args.cmd == "analyze":
    analyze(args.refdir, args.out)
  else:
    render(args.refdir, args.out, args.seconds, not args.no_gravity, not args.no_gif)


if __name__ == "__main__":
  main()
