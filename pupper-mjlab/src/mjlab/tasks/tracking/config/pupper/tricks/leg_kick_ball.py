"""Kick a ball with the leg kick in simulation: paw speed at contact, ball speed after.

The tracking task has no ball, so this adds one (the header-kick task's nominal
ball: 0.075 m radius, 0.10 kg, friction 0.6, solref (0.01, 0.5), condim 3) to the
scene that play_motion.py uses, with the same nominal robot. It plays the kick
either open loop with the robot's trick-player gains (as play_motion.py does) or
with a trained tracking policy (--checkpoint). The paw and the ball are recorded
at every 4 ms physics step, not only at the 50 Hz control rate.

    uv run python -m mjlab.tasks.tracking.config.pupper.tricks.leg_kick_ball --side right
    ... --side right --ball-dx 0.02     # ball 2 cm farther than planned
    ... --side right --no-ball True     # check: matches play_motion.py's error timeline
    ... --side right --motion-file .../leg_kick.npz --checkpoint .../model_1500.pt

The ball starts where leg_kick.strike() puts it (plus --ball-dx/--ball-dy). With
the clip library (leg_kick.npz), the kick clip starts after one stand_idle loop.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Literal

import mujoco
import numpy as np
import torch
import tyro

import mjlab
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import PUPPER_ACTION_SCALE
from mjlab.entity import EntityCfg
from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.play_motion import REPLAY_KD, REPLAY_KP
from mjlab.tasks.tracking.config.pupper.tricks import leg_kick as lk

HERE = Path(__file__).resolve().parent
BALL_MASS = 0.10
BALL_FRICTION = (0.6, 0.005, 0.0001)
BALL_SOLREF = (0.01, 0.5)


def ball_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec()
  body = spec.worldbody.add_body(name="ball")
  body.add_freejoint(name="ball_joint")
  body.add_geom(
    name="ball_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=(lk.BALL_RADIUS, 0, 0),
    mass=BALL_MASS, condim=3, friction=BALL_FRICTION, solref=BALL_SOLREF,
    priority=2, contype=1, conaffinity=1,
  )
  return spec


def main(
  side: Literal["right", "left"] = "right",
  motion_file: Path | None = None,
  checkpoint: Path | None = None,
  ball_dx: float = 0.0,
  ball_dy: float = 0.0,
  no_ball: bool = False,
  report_file: Path | None = None,
) -> dict:
  """motion_file defaults to leg_kick_<side>.npz; with a clip library the kick
  clip named kick_<side> is located from the npz clip table."""
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import RslRlVecEnvWrapper
  from mjlab.tasks.pupper.deployment_contract import compose_pupper_com_randomization
  from mjlab.tasks.tracking.config.pupper.env_cfgs import pupper_tracking_env_cfg
  from mjlab.tasks.tracking.mdp import MotionCommand

  leg = "front_r" if side == "right" else "front_l"
  motion_file = (motion_file or HERE / f"leg_kick_{side}.npz").resolve()
  clips = {c["name"]: c for c in md.read_clips(motion_file)}
  kick_start = clips[f"kick_{side}"]["start"] / lk.FPS if clips else 0.0
  motion = lk.kick_right()
  if side == "left":
    motion = lk.mirror(motion)
  plan = lk.strike(motion, leg)
  ball0 = np.array(plan["ball_centre_start_frame_m"]) + np.array([ball_dx, ball_dy, 0.0])
  if no_ball:
    ball0 = np.array([5.0, 0.0, lk.BALL_RADIUS])

  # The scene and dynamics of play_motion.py (nominal robot; trick-player gains
  # for open-loop replay, training gains for a policy).
  cfg = pupper_tracking_env_cfg(motion_file=str(motion_file), play=True)
  cfg.seed, cfg.scene.num_envs, cfg.terminations = 42, 1, {}
  for name in ("encoder_bias", "foot_friction", "pd_gains"):
    cfg.events.pop(name, None)
  cfg.events["base_com"].params["ranges"] = {i: (0.0, 0.0) for i in range(3)}
  cfg.events["inertial"].params["alpha_range"] = (0.0, 0.0)
  compose_pupper_com_randomization(cfg)
  actuator = cfg.scene.entities["robot"].articulation.actuators[0]
  assert isinstance(actuator, BuiltinPositionActuatorCfg)
  if checkpoint is None:
    actuator.stiffness, actuator.damping = REPLAY_KP, REPLAY_KD
  cfg.scene.entities["ball"] = EntityCfg(
    spec_fn=ball_spec, init_state=EntityCfg.InitialStateCfg(pos=tuple(ball0))
  )

  device = "cpu"
  env = ManagerBasedRlEnv(cfg=cfg, device=device)
  wrapped = RslRlVecEnvWrapper(env)
  robot, ball = env.scene["robot"], env.scene["ball"]
  cmd = env.command_manager.get_term("motion")
  assert isinstance(cmd, MotionCommand)
  n_frames = int(cmd.motion.time_step_total)
  default = robot.data.default_joint_pos

  def replay(obs):
    del obs
    nxt = torch.clamp(cmd.time_steps + 1, max=n_frames - 1)
    return (cmd.motion.joint_pos[nxt] - default) / PUPPER_ACTION_SCALE

  policy = replay
  if checkpoint is not None:
    from dataclasses import asdict

    from mjlab.tasks.registry import load_rl_cfg
    from mjlab.tasks.tracking.rl import MotionTrackingOnPolicyRunner

    runner = MotionTrackingOnPolicyRunner(
      wrapped, asdict(load_rl_cfg("Mjlab-Tracking-Flat-Pupper-v3")), device=device
    )
    runner.load(str(checkpoint), map_location=device)
    policy = runner.get_inference_policy(device=device)

  def put_ball(pos) -> None:
    state = torch.zeros(1, 13, device=device)
    state[0, :3] = torch.tensor(pos, dtype=torch.float32)
    state[0, 3] = 1.0
    ball.write_root_state_to_sim(state)

  # Start at frame 0, as play_motion.py does. The ball waits out of the way and
  # is put down at rest 0.5 s before the kick clip starts (with a clip library,
  # the other kick plays first and would touch it).
  wrapped.reset()
  cmd.reset_to_frame(torch.arange(1, device=device), 0)
  place_step = max(0, round((kick_start - 0.5) * lk.FPS))
  put_ball(ball0 if place_step == 0 else (5.0, 0.0, lk.BALL_RADIUS))
  env.scene.write_data_to_sim()
  env.sim.forward()
  env.sim.sense()
  cmd.update_relative_body_poses()
  obs = wrapped.get_observations()

  # Record after every physics substep of env.step().
  site = robot.find_sites(f"leg_{leg}_3_foot_site")[0][0]
  rows = []
  sim_step = env.sim.step

  def recorded_step():
    sim_step()
    env.sim.forward()  # fresh site positions for the record
    rows.append(
      np.concatenate(
        [
          robot.data.site_pos_w[0, site].cpu().numpy(),
          ball.data.root_link_pos_w[0].cpu().numpy(),
          ball.data.root_link_lin_vel_w[0].cpu().numpy(),
          robot.data.root_link_pos_w[0].cpu().numpy(),
          [-float(robot.data.projected_gravity_b[0, 2])],
        ]
      )
    )

  env.sim.step = recorded_step
  with torch.inference_mode():
    for step in range(n_frames - 1):
      if step == place_step and place_step > 0:
        put_ball(ball0)
      obs, *_ = wrapped.step(policy(obs))
  env.close()

  rec = np.array(rows)
  dt = env.physics_dt
  t = (np.arange(len(rec)) + 1) * dt - kick_start  # time from the kick clip start
  paw, bpos, bvel = rec[:, :3], rec[:, 3:6], rec[:, 6:9]
  root, upright = rec[:, 9:12], rec[:, 12]
  paw_vel = np.gradient(paw, dt, axis=0)
  # Root at the end of control step k against reference frame k + 1 (the
  # command plays the file frame by frame, clip library included).
  per_step = slice(env.cfg.decimation - 1, None, env.cfg.decimation)
  ref = np.load(motion_file)["body_pos_w"][:, 0]
  root_err = np.linalg.norm(root[per_step] - ref[1 : len(root[per_step]) + 1], axis=1)
  swing = (t > plan["strike_time_s"] - 0.3) & (t < plan["strike_time_s"] + 0.3)
  i_peak = int(np.flatnonzero(swing)[np.argmax(paw_vel[swing, 0])])
  tilt = np.degrees(np.arccos(np.clip(upright, -1, 1)))
  result = {
    "side": side,
    "motion_file": motion_file.name,
    "controller": checkpoint.name if checkpoint else "open-loop replay",
    "planned": plan,
    "ball_start_m": np.round(ball0, 3).tolist(),
    "paw_peak_forward_speed_m_s": round(float(paw_vel[i_peak, 0]), 2),
    "paw_peak_time_s": round(float(t[i_peak]), 3),
    "paw_at_peak_m": np.round(paw[i_peak], 3).tolist(),
    "worst_root_error_m": round(float(root_err.max()), 4),
    "worst_tilt_from_upright_deg": round(float(tilt.max()), 1),
    "root_error_every_0.5s_m": [round(float(root_err[i]), 4) for i in range(24, len(root_err), 25)],
  }
  if not no_ball:
    moved = (np.linalg.norm(bvel[:, :2], axis=1) > 0.05) & (t > -0.4)
    if moved.any():
      i0 = int(np.argmax(moved))
      after = min(i0 + int(round(0.10 / dt)), len(t) - 1)
      v = bvel[after]
      result.update(
        {
          "contact_time_s": round(float(t[i0]), 3),
          "paw_velocity_at_contact_m_s": np.round(paw_vel[max(i0 - 1, 0)], 2).tolist(),
          "paw_at_contact_m": np.round(paw[max(i0 - 1, 0)], 3).tolist(),
          "ball_speed_0.1s_after_m_s": round(float(np.linalg.norm(v[:2])), 2),
          "ball_direction_0.1s_after_deg": round(float(math.degrees(math.atan2(v[1], v[0]))), 1),
          "ball_peak_speed_m_s": round(float(np.linalg.norm(bvel[:, :2], axis=1).max()), 2),
          "ball_peak_height_m": round(float(bpos[:, 2].max()), 3),
        }
      )
    else:
      result["contact_time_s"] = None  # the paw never moved the ball: a miss
  print(json.dumps(result, indent=2))
  if report_file is not None:
    report_file.parent.mkdir(parents=True, exist_ok=True)
    report_file.write_text(json.dumps(result, indent=2) + "\n")
  return result


if __name__ == "__main__":
  tyro.cli(main, config=mjlab.TYRO_FLAGS)
