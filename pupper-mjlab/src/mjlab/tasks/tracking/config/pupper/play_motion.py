"""Replay a Pupper reference motion in simulation and say whether it holds up.

Physics replay (the default) drives the joints straight to the reference angles,
open loop, starting from the motion's first frame (root pose included), with
gravity and contacts on. If the robot tips over or drifts far from the
reference, plain replay won't work on the real robot either, and the motion is
a candidate for training with Mjlab-Tracking-Flat-Pupper-v3.

    # headless: print a verdict (good for coding agents)
    uv run python -m mjlab.tasks.tracking.config.pupper.play_motion \\
        src/mjlab/tasks/tracking/config/pupper/tricks/sit.npz

    # watch it: the robot plus a translucent ghost of the reference
    uv run python -m mjlab.tasks.tracking.config.pupper.play_motion motion.npz --viewer viser
    uv run python -m mjlab.tasks.tracking.config.pupper.play_motion motion.npz --viewer viser --share True

    # the reference alone, no physics (the robot is placed on each frame)
    ... motion.npz --no-physics True --viewer viser

    # a heavier robot (a payload: every link's mass and inertia x1.3)
    ... motion.npz --mass-scale 1.3
    # the torso's center of mass 2 cm to the left (a battery or camera off-centre)
    ... motion.npz --com-y 0.02

    # a trained tracking policy instead of plain replay, scored the same way
    ... motion.npz --checkpoint logs/rsl_rl/pupper_tracking/<run>/model_3000.pt
"""

from __future__ import annotations

import math
import json
from pathlib import Path
from typing import Literal

import torch
import numpy as np
import tyro

import mjlab
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import PUPPER_ACTION_SCALE

# The robot's animation player plays tricks with these gains (animation_controller_py),
# stiffer than the walking policies' kp 5.5, so replay in sim uses them too.
REPLAY_KP = 7.5
REPLAY_KD = 0.25
# Verdict thresholds on the body's distance from the reference.
WOBBLE_POS_M, WOBBLE_TILT_DEG = 0.03, 10.0  # beyond this, replay wobbles
FALL_POS_M, FALL_TILT_DEG = 0.10, 45.0  # beyond this, the robot has fallen


def main(
  motion_file: tyro.conf.Positional[Path],
  viewer: Literal["none", "viser", "native"] = "none",
  share: bool = False,
  no_physics: bool = False,
  loops: int = 1,
  checkpoint: Path | None = None,
  mass_scale: float = 1.0,
  com_x: float = 0.0,
  com_y: float = 0.0,
  com_z: float = 0.0,
  seed: int = 42,
  randomize: bool = False,
  report_file: Path | None = None,
  video_file: Path | None = None,
  deployment_target_clamp: bool = False,
) -> None:
  """mass_scale scales every link's mass and inertia (1.3: 30% heavier, like a
  payload); com_x, com_y, com_z move the torso's center of mass (x forward,
  y left, z up, in m). Trick policies train on mass x0.9-1.3 and a center of mass up to
  1 cm off; plain replay was never adjusted to any of it. Everything else is the
  nominal robot, so a verdict is repeatable."""
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import RslRlVecEnvWrapper
  from mjlab.tasks.tracking.config.pupper.env_cfgs import pupper_tracking_env_cfg
  from mjlab.tasks.tracking.mdp import MotionCommand
  from mjlab.sensor import ContactMatch, ContactSensorCfg

  if loops < 1 or not math.isfinite(mass_scale) or mass_scale <= 0:
    raise ValueError("loops and mass_scale must be positive")
  if any(not math.isfinite(v) for v in (com_x, com_y, com_z)):
    raise ValueError("centre-of-mass offsets must be finite")
  if randomize and (mass_scale != 1.0 or any((com_x, com_y, com_z))):
    raise ValueError("choose training randomization OR fixed mass/COM offsets")
  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  cfg = pupper_tracking_env_cfg(motion_file=str(motion_file.resolve()), play=True)
  cfg.seed = seed
  cfg.scene.num_envs = 1
  if deployment_target_clamp:
    from mjlab.tasks.pupper.deployment_contract import apply_deployment_target_clamp
    apply_deployment_target_clamp(cfg)
  cfg.scene.sensors += (ContactSensorCfg(
    name="score_foot_ground", primary=ContactMatch(
      mode="geom", pattern=r"^leg_(front|back)_(r|l)_3_collision$", entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"), reduce="netforce", num_slots=1,
  ),)
  cfg.terminations = {}  # watch the whole motion even if it goes wrong
  # The nominal robot, not a random draw of the training randomization: same
  # verdict every run. Only the mass/inertia scale is applied (exactly).
  if not randomize:
    for name in ("encoder_bias", "foot_friction", "pd_gains"):
      cfg.events.pop(name, None)
    cfg.events["base_com"].params["ranges"] = {
      i: (d, d) for i, d in enumerate((com_x, com_y, com_z))
    }
    alpha = math.log(mass_scale) / 2  # pseudo_inertia scales mass by exp(2 * alpha)
    cfg.events["inertial"].params["alpha_range"] = (alpha, alpha)
  articulation = cfg.scene.entities["robot"].articulation
  assert articulation is not None
  actuator = articulation.actuators[0]
  assert isinstance(actuator, BuiltinPositionActuatorCfg)
  if checkpoint is None:  # a trained policy keeps the gains it was trained with
    actuator.stiffness, actuator.damping = REPLAY_KP, REPLAY_KD
  env = ManagerBasedRlEnv(cfg=cfg, device=device)
  robot = env.scene["robot"]
  cmd = env.command_manager.get_term("motion")
  assert isinstance(cmd, MotionCommand)
  default = robot.data.default_joint_pos
  n_frames = int(cmd.motion.time_step_total)
  dt = env.step_dt

  class ReplayPolicy:
    """Joint targets = the reference's next frame; optionally teleport to the reference."""

    def __call__(self, obs) -> torch.Tensor:
      del obs
      nxt = torch.clamp(cmd.time_steps + 1, max=n_frames - 1)
      if no_physics:
        root = robot.data.default_root_state.clone()
        root[:, 0:3] = cmd.anchor_pos_w
        root[:, 3:7] = cmd.anchor_quat_w
        root[:, 7:] = 0.0
        robot.write_root_state_to_sim(root)
        robot.write_joint_state_to_sim(cmd.joint_pos, torch.zeros_like(cmd.joint_pos))
      return (cmd.motion.joint_pos[nxt] - default) / PUPPER_ACTION_SCALE

  wrapped = RslRlVecEnvWrapper(env)
  policy = ReplayPolicy()
  if checkpoint is not None:
    from dataclasses import asdict

    from mjlab.tasks.registry import load_rl_cfg
    from mjlab.tasks.tracking.rl import MotionTrackingOnPolicyRunner

    rl_cfg = load_rl_cfg("Mjlab-Tracking-Flat-Pupper-v3")
    runner = MotionTrackingOnPolicyRunner(wrapped, asdict(rl_cfg), device=device)
    runner.load(str(checkpoint), map_location=device)
    policy = runner.get_inference_policy(device=device)

  if viewer != "none":
    from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer

    if viewer == "viser":
      import viser as viser_lib

      server = viser_lib.ViserServer(label="pupper-motion", share=share)
      if share:
        print("Open the viewer:", server.request_share_url())
      ViserPlayViewer(wrapped, policy, viser_server=server).run()
    else:
      NativeMujocoViewer(wrapped, policy).run()
    return

  wrapped.reset()
  # env.reset() computes commands with dt=0, but the motion command advances its
  # frame even then. Restore frame zero so a single-file score cannot include an
  # accidental final teleport that erases its last physical state.
  cmd.reset_to_frame(torch.arange(env.num_envs, device=device), 0)
  env.scene.write_data_to_sim()
  env.sim.forward()
  env.sim.sense()
  cmd.update_relative_body_poses()
  obs = wrapped.get_observations()
  worst_pos, worst_tilt, fell_at, wobble_at = 0.0, 0.0, None, None
  timeline = []
  samples = []
  video_poses = []
  reference_resets = 0
  previous_frame = int(cmd.time_steps[0])
  limits = robot.data.joint_pos_limits[0]
  for step in range(n_frames * loops - 1):
    with torch.inference_mode():
      action = policy(obs)
      requested_target = default + action * PUPPER_ACTION_SCALE
      violation = torch.clamp(limits[:, 0] - requested_target, min=0) + torch.clamp(requested_target - limits[:, 1], min=0)
      obs, *_ = wrapped.step(action)
    pos_err = float(torch.linalg.norm(cmd.robot_anchor_pos_w - cmd.anchor_pos_w))
    # angle between the robot's and the reference's body orientation
    dot = float(torch.abs((cmd.robot_anchor_quat_w * cmd.anchor_quat_w).sum()))
    if not math.isfinite(dot):
      env.close()
      raise RuntimeError("non-finite orientation in motion evaluation")
    tilt = math.degrees(2 * math.acos(min(1.0, dot)))
    if not math.isfinite(pos_err) or not math.isfinite(tilt):
      env.close()
      raise RuntimeError("non-finite state in motion evaluation")
    frame = int(cmd.time_steps[0])
    reference_resets += int(frame < previous_frame)
    previous_frame = frame
    joint_error = cmd.robot_joint_pos - cmd.joint_pos
    body_error = torch.linalg.vector_norm(cmd.robot_body_pos_w - cmd.body_pos_w, dim=-1)
    sample = {"time_s": (step + 1) * dt, "reference_frame": frame,
              "position_error_m": pos_err, "orientation_error_deg": tilt,
              "joint_error_rms_rad": float(torch.sqrt(torch.mean(joint_error ** 2))),
              "joint_error_max_rad": float(torch.abs(joint_error).max()),
              "tracked_body_error_max_m": float(body_error.max()),
              "joint_speed_max_rad_s": float(torch.abs(cmd.robot_joint_vel).max()),
              "motor_torque_max_nm": float(torch.abs(robot.data.qfrc_actuator).max()),
              "body_tilt_from_upright_deg": math.degrees(math.acos(float(torch.clamp(
                -robot.data.projected_gravity_b[0, 2], -1, 1)))),
              "foot_contact_count": int((env.scene["score_foot_ground"].data.found > 0).sum()),
              "root_height_m": float(robot.data.root_link_pos_w[0, 2])}
    sample.update(target_limit_violation_fraction=float((violation > 1e-6).float().mean()),
                  target_limit_violation_max_rad=float(violation.max()))
    if video_file is not None:
      video_poses.append((cmd.robot_anchor_pos_w[0].cpu().numpy().copy(),
                         cmd.robot_anchor_quat_w[0].cpu().numpy().copy(),
                         cmd.robot_joint_pos[0].cpu().numpy().copy(), frame))
    if not all(math.isfinite(value) for value in sample.values()):
      env.close()
      raise RuntimeError("non-finite joint/body metrics in motion evaluation")
    samples.append(sample)
    worst_pos, worst_tilt = max(worst_pos, pos_err), max(worst_tilt, tilt)
    if (step + 1) % int(round(0.5 / dt)) == 0:
      timeline.append(f"{(step + 1) * dt:4.1f}s {pos_err * 100:5.1f}cm {tilt:5.1f}deg")
    if fell_at is None and (pos_err > FALL_POS_M or tilt > FALL_TILT_DEG):
      fell_at = (step + 1) * dt
    if wobble_at is None and (pos_err > WOBBLE_POS_M or tilt > WOBBLE_TILT_DEG):
      wobble_at = (step + 1) * dt
  mode = (
    f"trained policy {checkpoint.name}"
    if checkpoint
    else "kinematic (no physics)"
    if no_physics
    else "physics, open-loop joint replay"
  )
  print(f"motion        {motion_file.name}: {n_frames} frames, {n_frames * dt:.1f} s")
  print(f"mode          {mode}")
  print("error over time (every 0.5 s):")
  for line in timeline:
    print("   ", line)
  print(f"worst body position error   {worst_pos * 100:.1f} cm")
  print(f"worst body orientation error {worst_tilt:.1f} deg")
  if reference_resets:
    print(f"reference resets {reference_resets}: whole-file wraps teleport the robot; this is not continuous endurance")
  train = (
    "uv run train Mjlab-Tracking-Flat-Pupper-v3 "
    f"--env.commands.motion.motion-file {motion_file}"
  )
  if fell_at is not None:
    print(
      f"verdict       FALLS at t = {fell_at:.2f} s. Plain replay won't work: make the "
      f"motion gentler, or train it:\n              {train}"
    )
  elif wobble_at is not None:
    print(
      f"verdict       WOBBLES from t = {wobble_at:.2f} s but stays up. Replay may work on "
      f"the robot; a trained policy will track it more tightly:\n              {train}"
    )
  else:
    who = "the policy" if checkpoint else "plain replay"
    print(f"verdict       HOLDS UP: {who} follows the reference closely.")
  if report_file is not None:
    result = {
      "motion": str(motion_file.resolve()),
      "checkpoint": str(checkpoint.resolve()) if checkpoint else None,
      "seed": seed, "randomize_dynamics": randomize,
      "mass_scale": mass_scale, "com_offset_m": [com_x, com_y, com_z],
      "loops": loops, "frames": n_frames,
      "reference_resets": reference_resets,
      "verdict": "FALLS" if fell_at is not None else "WOBBLES" if wobble_at is not None else "HOLDS UP",
      "worst_position_error_m": worst_pos, "worst_orientation_error_deg": worst_tilt,
      "mean_position_error_m": float(np.mean([s["position_error_m"] for s in samples])),
      "rms_position_error_m": float(np.sqrt(np.mean([s["position_error_m"] ** 2 for s in samples]))),
      "p95_position_error_m": float(np.percentile([s["position_error_m"] for s in samples], 95)),
      "mean_orientation_error_deg": float(np.mean([s["orientation_error_deg"] for s in samples])),
      "p95_orientation_error_deg": float(np.percentile([s["orientation_error_deg"] for s in samples], 95)),
      "rms_joint_error_rad": float(np.sqrt(np.mean([s["joint_error_rms_rad"] ** 2 for s in samples]))),
      "worst_joint_error_rad": max(s["joint_error_max_rad"] for s in samples),
      "worst_tracked_body_error_m": max(s["tracked_body_error_max_m"] for s in samples),
      "peak_joint_speed_rad_s": max(s["joint_speed_max_rad_s"] for s in samples),
      "peak_motor_torque_nm": max(s["motor_torque_max_nm"] for s in samples),
      "deployment_target_clamp": deployment_target_clamp,
      "fraction_steps_requesting_target_outside_limits": float(np.mean([s["target_limit_violation_max_rad"] > 1e-6 for s in samples])),
      "mean_joint_target_violation_fraction": float(np.mean([s["target_limit_violation_fraction"] for s in samples])),
      "max_requested_target_violation_rad": max(s["target_limit_violation_max_rad"] for s in samples),
      "peak_body_tilt_from_upright_deg": max(s["body_tilt_from_upright_deg"] for s in samples),
      "minimum_root_height_m": min(s["root_height_m"] for s in samples),
      "fraction_frames_without_foot_contact": float(np.mean([s["foot_contact_count"] == 0 for s in samples])),
      "failure_criteria": (["position_tracking"] if worst_pos > FALL_POS_M else [])
                          + (["orientation_tracking"] if worst_tilt > FALL_TILT_DEG else []),
      "fell_at_s": fell_at, "wobble_at_s": wobble_at,
      "approved_for_hardware": False,
      "scope": "Course verdict uses root tracking thresholds, not a physical fall detector; joint/body/effort metrics are reported separately. Randomized mode varies dynamics, not external pushes.",
      "samples": samples,
    }
    report_file.parent.mkdir(parents=True, exist_ok=True)
    report_file.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
  env.close()
  if video_file is not None:
    # These are recorded physics states, not a visual replay of the reference.
    # Set MUJOCO_GL=egl before starting this process on a headless GPU host.
    import imageio.v3 as iio
    from mjlab.tasks.tracking.config.pupper import motion_design as md
    from mjlab.tasks.tracking.config.pupper import unimate as U

    actual = md.Motion(fps=1 / dt, root_pos=np.asarray([p[0] for p in video_poses]),
                       root_quat=np.asarray([p[1] for p in video_poses]),
                       joint_pos=np.asarray([p[2] for p in video_poses]))
    with np.load(motion_file, allow_pickle=False) as source:
      frames = np.asarray([p[3] for p in video_poses])
      reference = md.Motion(fps=1 / dt, root_pos=source["body_pos_w"][frames, 0],
                           root_quat=source["body_quat_w"][frames, 0],
                           joint_pos=source["joint_pos"][frames])
    panels = [U.Fit(m, np.empty((0, 19, 3)), {}, {}) for m in (reference, actual)]
    video_file.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(video_file, U.render(panels, labels=["Reference", "Physics rollout"], skeleton=False), fps=1 / dt)


if __name__ == "__main__":
  tyro.cli(main, config=mjlab.TYRO_FLAGS)
