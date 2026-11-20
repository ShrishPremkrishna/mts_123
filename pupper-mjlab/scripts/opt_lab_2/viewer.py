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

Generated (e.g. bumpy) terrains are streamed to the browser as heightfield tiles
near the robot only (--terrain-radius): mjviser otherwise ships all 200 bumpy
tiles to every new client (~225 MB of GLB, ~85 MB on the wire), which freezes the
browser, stalls the sim loop and overloads the share relay. Physics always keeps the full terrain. The share tunnel is kept up
across relay hiccups, and re-requested (new URL, rewritten --url-file) if it dies.
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

import numpy as np

REPO = Path(__file__).resolve().parents[2]  # pupper-mjlab/
DRIVER_LIMITS = REPO.parent / "foundation/configs/class-driver-joint-limits.json"


def _harden_share_tunnel() -> None:
  """Keep one relay slot's TLS error from killing the whole share tunnel.

  viser's per-slot proxy loop only swallows ConnectionError when closing a
  socket; an ``ssl.SSLError`` (e.g. APPLICATION_DATA_AFTER_CLOSE_NOTIFY after a
  large transfer) escapes, cancels every slot and leaves the URL returning 404.
  Restart the slot instead. Inherited by the forked tunnel process.
  """
  import asyncio

  from viser import _tunnel

  if getattr(_tunnel, "_opt_lab_hardened", False):
    return
  orig = _tunnel._simple_proxy

  async def robust(*a: Any, **kw: Any) -> None:
    close_event = a[4] if len(a) > 4 else kw["close_event"]
    while True:
      try:
        return await orig(*a, **kw)
      except Exception:
        if close_event.is_set():
          return
        await asyncio.sleep(0.5)

  _tunnel._simple_proxy = robust
  _tunnel._opt_lab_hardened = True


def _share_watchdog(server: Any, url_file: Path | None) -> None:
  """Re-request a share URL whenever the tunnel is gone (new URL -> url_file)."""

  def dead() -> bool:
    t = server._share_tunnel
    if t is None:
      return True
    try:
      if t._process is not None and not t._process.is_alive():
        return True
      return t.get_status() in ("failed", "closed")
    except Exception:
      return True

  backoff = 5.0
  while True:
    time.sleep(backoff)
    if not dead():
      backoff = 5.0
      continue
    print("(viewer) share tunnel down; requesting a new share URL", flush=True)
    old = server._share_tunnel
    server._share_tunnel = None
    if old is not None:
      try:
        old.close()
      except Exception:
        pass
    box: list[str | None] = []
    th = threading.Thread(
      target=lambda box=box: box.append(server.request_share_url()), daemon=True
    )
    th.start()
    th.join(timeout=60.0)
    url = box[0] if box else None
    if url:
      print("Share URL:", url, flush=True)
      if url_file is not None:
        url_file.write_text(f"{url}\n")
      backoff = 5.0
    else:
      backoff = min(backoff * 2.0, 120.0)


class TerrainTiles:
  """Stream fixed heightfield tiles near the robot instead of the whole terrain.

  Hides every fixed heightfield geom from mjviser (alpha 0 on the viewer's host
  MjModel, which physics never reads) and adds per-tile meshes under
  ``/fixed_bodies`` -- so they follow the camera-tracking offset -- for tiles
  within ``radius`` m of any robot, removing them once ``radius + 4`` m away.
  """

  def __init__(self, server: Any, mj_model: Any, radius: float, stride: int) -> None:
    import copy

    import mujoco

    self.server, self.radius, self.stride = server, radius, max(1, stride)
    self.vis = copy.deepcopy(mj_model)  # original colors for our own meshes
    data = mujoco.MjData(self.vis)
    mujoco.mj_kinematics(self.vis, data)
    hf = int(mujoco.mjtGeom.mjGEOM_HFIELD)
    self.tiles: dict[int, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for g in range(mj_model.ngeom):
      b = int(mj_model.geom_bodyid[g])
      fixed = (
        mj_model.body_weldid[b] == 0
        and mj_model.body_mocapid[mj_model.body_rootid[b]] < 0
      )
      if mj_model.geom_type[g] != hf or not fixed or mj_model.geom_rgba[g, 3] == 0:
        continue
      sx, sy = self.vis.hfield_size[int(mj_model.geom_dataid[g])][:2]
      xmat = data.geom_xmat[g].reshape(3, 3)
      half = np.abs(xmat[:2, :2]) @ np.array([sx, sy])
      self.tiles[g] = (data.geom_xpos[g].copy(), xmat.copy(), half)
      mj_model.geom_rgba[g, 3] = 0.0  # mjviser skips fully transparent geoms
    self.moving = mj_model.body_weldid != 0
    self.handles: dict[int, Any] = {}
    self.lock = threading.Lock()

  def _mesh(self, g: int) -> Any:
    import trimesh
    from mjviser.conversions import create_primitive_mesh

    mesh = create_primitive_mesh(self.vis, g)
    s = self.stride
    if s > 1:
      h = int(self.vis.geom_dataid[g])
      nr, nc = int(self.vis.hfield_nrow[h]), int(self.vis.hfield_ncol[h])
      ri = np.unique(np.r_[np.arange(0, nr, s), nr - 1])
      ci = np.unique(np.r_[np.arange(0, nc, s), nc - 1])
      idx = (ri[:, None] * nc + ci[None, :]).ravel()
      r, c = np.mgrid[: len(ri) - 1, : len(ci) - 1]
      i0 = (r * len(ci) + c).ravel()
      faces = np.column_stack(
        [i0, i0 + 1, i0 + len(ci) + 1, i0, i0 + len(ci) + 1, i0 + len(ci)]
      ).reshape(-1, 3)
      vis = mesh.visual
      out = trimesh.Trimesh(vertices=mesh.vertices[idx], faces=faces, process=False)
      if isinstance(vis, trimesh.visual.TextureVisuals) and vis.uv is not None:
        out.visual = trimesh.visual.TextureVisuals(
          uv=vis.uv[idx], material=vis.material
        )
      else:
        out.visual = trimesh.visual.ColorVisuals(
          mesh=out, vertex_colors=vis.vertex_colors[idx]
        )
      mesh = out
    pos, xmat, _ = self.tiles[g]
    tf = np.eye(4)
    tf[:3, :3], tf[:3, 3] = xmat, pos
    mesh.apply_transform(tf)
    return mesh

  def update(self, points_xy: np.ndarray) -> None:
    """Show tiles near any of ``points_xy`` (N, 2), drop far ones."""
    if len(points_xy) == 0:
      return
    with self.lock:
      for g, (pos, _, half) in self.tiles.items():
        gap = np.maximum(np.abs(points_xy - pos[:2]) - half, 0.0)
        d = float(np.min(np.linalg.norm(gap, axis=1)))
        if g not in self.handles and d <= self.radius:
          self.handles[g] = self.server.scene.add_mesh_trimesh(
            f"/fixed_bodies/terrain_tiles/g{g}",
            self._mesh(g),
            cast_shadow=False,
            receive_shadow=0.2,
          )
        elif g in self.handles and d > self.radius + 4.0:
          self.handles.pop(g).remove()

  def follow(self, get_xpos: Any, fallback_xy: np.ndarray) -> None:
    """Background loop: track the robots' moving bodies every 0.5 s."""
    self.update(fallback_xy)
    while True:
      time.sleep(0.5)
      try:
        xpos = get_xpos()
        if xpos is None:
          continue
        xpos = np.asarray(xpos)
        if xpos.ndim == 2:
          xpos = xpos[None]
        self.update(xpos[:, self.moving, :2].reshape(-1, 2))
      except Exception as e:  # never take the viewer down over the terrain
        print(f"(viewer) terrain tile update failed: {e!r}", flush=True)


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
  p.add_argument(
    "--terrain-radius",
    type=float,
    default=10.0,
    help="viewer only: stream heightfield tiles within this many m of the robot "
    "(physics keeps the full terrain); <= 0 sends the whole terrain as one mesh",
  )
  p.add_argument(
    "--terrain-stride",
    type=int,
    default=1,
    help="viewer only: draw every Nth heightfield sample (visual decimation)",
  )
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
    _harden_share_tunnel()
    url = server.request_share_url()
    print("Share URL:", url, flush=True)
    if args.url_file is not None:
      args.url_file.write_text(f"{url}\n")
    threading.Thread(
      target=_share_watchdog, args=(server, args.url_file), daemon=True
    ).start()
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
  tiles = None
  if args.terrain_radius > 0:
    sim = env.unwrapped.sim
    tiles = TerrainTiles(server, sim.mj_model, args.terrain_radius, args.terrain_stride)
    if tiles.tiles:
      print(
        f"(viewer) streaming {len(tiles.tiles)} heightfield tiles within "
        f"{args.terrain_radius:g} m of the robot (stride {tiles.stride})",
        flush=True,
      )
    else:
      tiles = None
  viewer = ViserPlayViewer(env, policy, viser_server=server, checkpoint_manager=manager)
  if tiles is not None:
    root_xy = env.unwrapped.scene["robot"].data.root_link_pos_w[:, :2].cpu().numpy()
    threading.Thread(
      target=tiles.follow,
      args=(
        lambda: getattr(getattr(viewer, "_scene", None), "_last_body_xpos", None),
        root_xy,
      ),
      daemon=True,
    ).start()

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
