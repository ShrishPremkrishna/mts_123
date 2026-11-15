"""Score a header-kick checkpoint in simulation with the deterministic policy.

Each env contributes exactly its first episode, so short episodes (falls) are
not over-counted. Prints outcome rates and kick quality, split by start
distance, and optionally writes JSON and an mp4 of env 0.

  python -m mjlab.tasks.pupper_kick.evaluate --checkpoint <run>/model_<N>.pt \
    --episodes 512 --level 1.0 --video kick.mp4
"""

from __future__ import annotations

import argparse
import io
import json
import math
import warnings
from contextlib import redirect_stdout
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch

TASK = "Mjlab-HeaderKick-Flat-Pupper-v3"
_REASONS = ("kick_done", "foul", "fell_over", "body_ground", "missed_ball", "time_out")
_DIST_BINS = ((0.45, 0.8), (0.8, 1.2), (1.2, 1.51))


def _summary(records: list[dict]) -> dict:
  def rate(rs, key):
    return float(np.mean([r[key] for r in rs])) if rs else float("nan")

  def stat(rs, key, fn=np.mean):
    vals = [r[key] for r in rs]
    return float(fn(vals)) if vals else float("nan")

  out = {"episodes": len(records)}
  for key in ("header", "success", "foul"):
    out[f"{key}_rate"] = rate(records, key)
  for reason in _REASONS:
    out[f"end_{reason}"] = float(np.mean([r["reason"] == reason for r in records]))
  heads = [r for r in records if r["header"]]
  scored = [r for r in heads if r["recorded"]]
  out["impact_speed_mean"] = stat(heads, "impact_speed")
  out["time_to_header_mean"] = stat(heads, "time_to_header")
  out["ball_speed_aim_mean"] = stat(scored, "ball_speed_aim")
  out["ball_speed_aim_p90"] = stat(scored, "ball_speed_aim", lambda v: np.percentile(v, 90))
  out["aim_error_deg_median"] = stat(scored, "aim_error_deg", np.median)
  out["by_distance"] = {}
  for lo, hi in _DIST_BINS:
    rs = [r for r in records if lo <= r["distance"] < hi]
    sc = [r for r in rs if r["header"] and r["recorded"]]
    out["by_distance"][f"{lo:.2f}-{hi:.2f}"] = {
      "episodes": len(rs),
      "header_rate": rate(rs, "header"),
      "success_rate": rate(rs, "success"),
      "fell_rate": float(np.mean([r["reason"] == "fell_over" for r in rs])) if rs else float("nan"),
      "ball_speed_aim_mean": stat(sc, "ball_speed_aim"),
      "impact_speed_mean": stat([r for r in rs if r["header"]], "impact_speed"),
    }
  return out


def _json_policy(path: Path):
  """The exported deploy JSON as a policy: same frame order and dense stack as the
  robot (the command clip is applied by the env's command term, as on the robot)."""
  from mjlab.tasks.pupper.export import json_forward, to_robot_order

  spec = json.loads(Path(path).read_text())
  history, frame = int(spec["observation_history"]), int(spec["single_observation_size"])

  def policy(obs):
    actor = obs["actor"] if not isinstance(obs, torch.Tensor) else obs
    action = json_forward(spec, to_robot_order(actor, history, frame))
    return torch.as_tensor(action, dtype=torch.float32)

  return policy


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  src = ap.add_mutually_exclusive_group(required=True)
  src.add_argument("--checkpoint", type=Path, help="training checkpoint (PyTorch actor)")
  src.add_argument(
    "--policy-json",
    type=Path,
    help="exported deploy JSON, run as neural_controller runs it (newest-first "
    "frames through the RTNeural dense stack)",
  )
  ap.add_argument("--episodes", type=int, default=512, help="one episode per env")
  ap.add_argument("--level", type=float, default=1.0, help="curriculum level 0..1")
  ap.add_argument("--perception", choices=("full", "perfect"), default="full")
  ap.add_argument("--seed", type=int, default=0)
  ap.add_argument("--json", type=Path, default=None)
  ap.add_argument("--video", type=Path, default=None, help="mp4 of env 0")
  ap.add_argument("--video-seconds", type=float, default=12.0)
  args = ap.parse_args()

  import mjlab.tasks  # noqa: F401  (registers tasks)
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.tasks.pupper_kick import mdp
  from mjlab.tasks.pupper_kick.kick_env_cfg import pupper_header_kick_env_cfg
  from mjlab.tasks.registry import load_rl_cfg, load_runner_cls

  torch.manual_seed(args.seed)
  cfg = pupper_header_kick_env_cfg(
    play=True,
    perception=mdp.perfect_perception() if args.perception == "perfect" else None,
  )
  cfg.commands["kick"].initial_level = args.level
  cfg.scene.num_envs = args.episodes
  cfg.seed = args.seed
  if args.video:
    # Env 0 only (all worlds share one origin on the plane), close chase camera.
    from mjlab.viewer.viewer_config import ViewerConfig

    v = cfg.viewer
    v.max_extra_envs = 0
    v.width, v.height = 640, 480
    v.distance, v.elevation, v.azimuth = 1.4, -20.0, 135.0
    v.origin_type = ViewerConfig.OriginType.ASSET_BODY
    v.entity_name, v.body_name = "robot", "base_link"
  agent_cfg = load_rl_cfg(TASK)

  with warnings.catch_warnings(), redirect_stdout(io.StringIO()):
    warnings.simplefilter("ignore")
    env = ManagerBasedRlEnv(
      cfg=cfg, device="cpu", render_mode="rgb_array" if args.video else None
    )
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    if args.checkpoint is not None:
      runner_cls = load_runner_cls(TASK) or MjlabOnPolicyRunner
      runner = runner_cls(wrapped, asdict(agent_cfg), device="cpu")
      runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location="cpu")
  if args.checkpoint is not None:
    policy = runner.get_inference_policy(device="cpu")
  else:
    policy = _json_policy(args.policy_json)

  cmd = env.command_manager.get_term("kick")
  done_once = torch.zeros(env.num_envs, dtype=torch.bool)
  records: list[dict] = []
  original = cmd._log_finished

  def record(env_ids: torch.Tensor) -> None:
    # Runs inside the env reset, before the command resamples: the buffers still
    # hold the finished episode, and the termination flags hold this step.
    first = env_ids[~done_once[env_ids]]
    reasons = {n: env.termination_manager.get_term(n) for n in _REASONS}
    success = cmd.success()
    for e in first.tolist():
      records.append(
        {
          "distance": float(cmd.init_distance[e]),
          "header": bool(cmd.kicked[e]),
          "success": bool(success[e]),
          "foul": bool(cmd.leg_touched[e] or cmd.torso_touched[e]),
          "recorded": bool(cmd.recorded[e]),
          "ball_speed_aim": float(cmd.ball_speed_aim[e]),
          "aim_error_deg": math.degrees(float(cmd.aim_error[e])),
          "impact_speed": float(cmd.impact_speed[e]),
          "time_to_header": float(cmd.time_to_header[e]),
          "reason": next((n for n in _REASONS if bool(reasons[n][e])), "other"),
        }
      )
    done_once[first] = True
    original(env_ids)

  cmd._log_finished = record  # type: ignore[method-assign]

  frames: list[np.ndarray] = []
  video_steps = int(args.video_seconds / env.step_dt) if args.video else 0
  obs = wrapped.get_observations()
  step = 0
  max_steps = int(cfg.episode_length_s / env.step_dt) + 5
  with torch.no_grad():
    while (not done_once.all() and step < max_steps) or step < video_steps:
      obs, _, _, _ = wrapped.step(policy(obs))
      if step < video_steps:
        frames.append(env.render())
      step += 1

  summary = _summary(records)
  summary.update(
    checkpoint=str(args.checkpoint or args.policy_json), level=args.level, perception=args.perception, seed=args.seed
  )
  print(json.dumps(summary, indent=2))
  if args.json:
    args.json.write_text(json.dumps({"summary": summary, "episodes": records}, indent=1))
  if args.video and frames:
    import imageio

    imageio.mimsave(args.video, frames, fps=int(round(1.0 / env.step_dt)))
    print(f"[INFO] wrote {args.video} ({len(frames)} frames)")
  env.close()


if __name__ == "__main__":
  main()
