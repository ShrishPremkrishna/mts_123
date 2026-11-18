"""Viser share viewer for an Opt Lab 2 checkpoint played with its own reference dir.

Prints a public share URL (viser relay) that anyone can open in a browser. The
policy observes its gait reference, so pass the reference dir the checkpoint was
trained on (omit it for checkpoints trained on the shipped default references).

  cd pupper-mjlab
  MUJOCO_GL=egl uv run --no-sync python scripts/opt_lab_2/viewer.py \
      --checkpoint ~/runs/opt-lab-2/rp-gallop_c-s42/model_2999.pt \
      --reference-dir configs/opt-lab-2/gallop_c [--command 1.5,0,0] [--url-file url.txt]

Other checkpoints in the same directory can be switched from the viewer's GUI.
The physics flags every Opt Lab 2 run used (deployment target clamp, driver
limits, COM randomization, torso contact) are applied unless
--no-standard-physics. Auto-pauses while no browser is connected.
To view a reference alone (no policy), use
``python -m mjlab.tasks.pupper_gait.visualize_reference --viser True --share True``
with ``PUPPER_REFERENCE_DIR=<dir>`` set.
"""

from __future__ import annotations

import argparse
import os
import threading
import time
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[2]  # pupper-mjlab/
DRIVER_LIMITS = REPO.parent / "foundation/configs/class-driver-joint-limits.json"


def main() -> None:
  p = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
  )
  p.add_argument("--checkpoint", type=Path, required=True)
  p.add_argument("--reference-dir", type=Path)
  p.add_argument("--task", default="Mjlab-MixedGaits-Flat-Pupper-v3")
  p.add_argument(
    "--command",
    help="freeze the command 'vx,vy,yaw' (default: the play config's resampled commands)",
  )
  p.add_argument("--num-envs", type=int, default=1)
  p.add_argument(
    "--no-share",
    action="store_true",
    help="local server only (port 8080), no public URL",
  )
  p.add_argument("--url-file", type=Path)
  p.add_argument("--no-standard-physics", action="store_true")
  p.add_argument("--device", default=None)
  args = p.parse_args()
  if args.reference_dir is not None:
    ref = args.reference_dir.resolve()
    if not (ref / "captured_reach_gaits.npz").exists():
      p.error(f"{ref} has no captured_reach_gaits.npz")
    # gait_reference reads this at import time: set it before any task import.
    os.environ["PUPPER_REFERENCE_DIR"] = str(ref)
  else:
    os.environ.pop("PUPPER_REFERENCE_DIR", None)

  import torch
  import viser

  import mjlab.tasks  # noqa: F401  (register tasks)
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.scripts.pupper_locomotion import settings
  from mjlab.tasks.registry import load_runner_cls
  from mjlab.viewer import ViserPlayViewer
  from mjlab.viewer.viser.viewer import CheckpointManager, format_time_ago

  server = viser.ViserServer(label="pupper-opt-lab-2")
  if not args.no_share:
    url = server.request_share_url()
    print("Share URL:", url, flush=True)
    if args.url_file is not None:
      args.url_file.write_text(f"{url}\n")
  device = args.device or ("cuda:0" if torch.cuda.is_available() else "cpu")
  std = not args.no_standard_physics
  ns = argparse.Namespace(
    task=args.task, num_envs=args.num_envs, seed=0, checkpoint=args.checkpoint, command="evaluate",
    deployment_target_clamp=std, driver_limits=DRIVER_LIMITS if std else None,
    compose_com_randomization=std, locomotion_torso_contact=std,
  )  # fmt: skip
  env_cfg, agent_cfg = settings(ns, True)
  if args.command:
    vx, vy, yaw = (float(v) for v in args.command.split(","))
    twist: Any = env_cfg.commands["twist"]
    twist.ranges.lin_vel_x, twist.ranges.lin_vel_y, twist.ranges.ang_vel_z = (
      (vx, vx),
      (vy, vy),
      (yaw, yaw),
    )
    for name in (
      "rel_standing_envs",
      "rel_heading_envs",
      "rel_world_envs",
      "rel_forward_envs",
      "rel_turn_envs",
    ):
      if hasattr(twist, name):
        setattr(twist, name, 0.0)
    twist.heading_command = False
  env = RslRlVecEnvWrapper(
    ManagerBasedRlEnv(cfg=env_cfg, device=device), clip_actions=agent_cfg.clip_actions
  )
  from dataclasses import asdict

  runner = (load_runner_cls(args.task) or MjlabOnPolicyRunner)(
    env, asdict(agent_cfg), device=device
  )
  ckpt_dir = args.checkpoint.resolve().parent

  def load(path):
    runner.load(str(path), load_cfg={"actor": True}, strict=True, map_location=device)
    return runner.get_inference_policy(device=device)

  def available():
    now = time.time()
    items = sorted(ckpt_dir.glob("model_*.pt"), key=lambda f: int(f.stem.split("_")[1]))
    return [(f.name, format_time_ago(int(now - f.stat().st_mtime))) for f in items]

  policy = load(args.checkpoint)
  manager = CheckpointManager(
    current_name=args.checkpoint.name,
    fetch_available=available,
    load_checkpoint=lambda n: load(ckpt_dir / n),
  )
  server.gui.add_markdown(
    f"**{ckpt_dir.name}/{args.checkpoint.name}**  \nreference: "
    f"`{args.reference_dir or 'shipped default'}`  \ncommand: `{args.command or 'resampled'}`"
  )
  viewer = ViserPlayViewer(env, policy, viser_server=server, checkpoint_manager=manager)

  def idle_monitor() -> None:
    auto = False
    while True:
      time.sleep(2.0)
      try:
        n = len(server.get_clients())
      except Exception:
        continue
      if n == 0 and not viewer._is_paused:
        viewer.request_toggle_pause()
        auto = True
      elif n > 0 and viewer._is_paused and auto:
        viewer.request_toggle_pause()
        auto = False

  threading.Thread(target=idle_monitor, daemon=True).start()
  viewer.run()


if __name__ == "__main__":
  main()
