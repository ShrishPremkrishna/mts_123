"""Pupper v3 header-kick task: sprint at a ball, hit it with the head bumper, stop.

Built on the flat Pupper velocity config, so the robot model, action latency,
IMU latency, observation noise and robot domain randomization are the same as
in the walking tasks. The task replaces the velocity command with the ``kick``
command (ball position and aim in the deploy frame's command slot), adds the
ball and the head geoms, and defines its own rewards and terminations. See
README.md in this folder for the design and the deploy contract.
"""

from __future__ import annotations

import functools
import math

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import PUPPER_DECIMATION
from mjlab.entity import EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.pupper.pupper_env_cfg import pupper_flat_env_cfg
from mjlab.tasks.pupper_kick import mdp
from mjlab.tasks.velocity import mdp as velocity_mdp

##
# Ball. Nominal values for a light foam ball; replace with the measured ball.
##

BALL_RADIUS: float = 0.075  # m (15 cm diameter)
BALL_MASS: float = 0.10  # kg
# (sliding, torsional, rolling) friction; priority 2 makes these the contact
# values for every ball pair (floor, legs, head). condim 3 uses sliding only.
BALL_FRICTION: tuple[float, float, float] = (0.6, 0.005, 0.0001)
# Contact (time constant [s], damping ratio). 0.5 gives a drop rebound e ~ 0.43.
BALL_SOLREF: tuple[float, float] = (0.01, 0.5)

# Per-reset randomization of the ball, relative to the nominal values above.
BALL_MASS_SCALE: tuple[float, float] = (0.5, 2.0)
BALL_RADIUS_SCALE: tuple[float, float] = (0.85, 1.15)
BALL_SLIDE_FRICTION: tuple[float, float] = (0.3, 1.0)
# Contact damping ratio: e ~ 0.43 (0.5) to e ~ 0.18 (0.8). Restitution is unmeasured.
BALL_DAMPING_RATIO: tuple[float, float] = (0.5, 0.8)
# Rolling resistance as a constant deceleration [m/s^2] (see mdp.get_ball_spec):
# a ball hit at 1.5 m/s rolls approximately 1.9 m (0.6) to 11 m (0.1).
BALL_ROLL_DECEL: tuple[float, float] = (0.1, 0.6)

# Robot additions to the inherited walking-task randomization.
EFFORT_LIMIT_SCALE: tuple[float, float] = (0.8, 1.0)
JOINT_RESET_OFFSET: float = 0.05  # rad, uniform per joint at reset

# Posture limits for hardware plausibility (see the matching reward terms).
# 0.10 in CPU run 2, which then sprinted crouched (mean base 0.105 m, 18 deg nose-up).
MIN_BASE_HEIGHT: float = 0.12  # m; standing height is 0.145
JOINT_SPEED_LIMIT: float = 20.0  # rad/s; the PD model has no torque-speed curve

EPISODE_S: float = 5.0
# A lunge pitches the body further than a trot (the walking tasks use 30 deg).
# Set the kick controller's max_body_angle on the robot just above this.
FELL_OVER_LIMIT: float = math.radians(35.0)

# Weights are per second (scaled by dt); event terms carry their total reward.
REWARD_WEIGHTS: dict[str, float] = {
  "approach_velocity": 2.0,
  "kick_alignment": 1.0,
  "header": 3.0,  # event: first bumper contact
  "kick_quality": 5.0,  # event: 5 x (along - 0.6 x across) [m/s], 0.3 s after the header
  "head_recontact": -5.0,  # bumper still on the ball after 0.1 s (pushing)
  "leg_ball_contact": -5.0,  # after the header (before it, a contact is a foul)
  "post_kick_stop": 3.0,
  "foul": -3.0,  # event: foul termination
  "fell": -10.0,  # fell_over or body_ground termination
  "low_base": -40.0,  # per metre below MIN_BASE_HEIGHT
  "joint_speed": -0.02,  # per (rad/s)^2 above JOINT_SPEED_LIMIT
  "abduction_deviation": -1.0,
  "missed": -2.0,
  "upright": 1.5,  # 0.5 in CPU run 2 (see MIN_BASE_HEIGHT)
  "lin_vel_z_l2": -0.2,
  "ang_vel_xy_l2": -0.05,
  "action_rate_l2": -0.02,
  "foot_slip": -0.1,
  "joint_torques_l2": -2.0e-4,
  "knee_ground_contact": -1.0,
  "self_collision_r": -1.0,
  "self_collision_l": -1.0,
}

_HEAD_PATTERN = r"^(" + "|".join(mdp.HEAD_GEOMS) + r")$"
_LEG_ROOTS = r"^leg_(front|back)_(r|l)_1$"
_CHASSIS_PATTERN = r"^(" + "|".join(mdp.CHASSIS_GEOMS) + r")$"
_BODY_PATTERN = r"^(" + "|".join(mdp.HEAD_GEOMS + mdp.CHASSIS_GEOMS) + r")$"


def _ball_contact_sensor(name: str, primary: ContactMatch) -> ContactSensorCfg:
  return ContactSensorCfg(
    name=name,
    primary=primary,
    secondary=ContactMatch(mode="body", pattern="ball", entity="ball"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    # One control step of substeps, so a short impact is not missed.
    history_length=PUPPER_DECIMATION,
  )


def _scene(cfg: ManagerBasedRlEnvCfg) -> None:
  robot = cfg.scene.entities["robot"]
  robot.spec_fn = mdp.get_kick_robot_spec
  robot.collisions = tuple(robot.collisions) + (mdp.HEAD_COLLISION,)

  cfg.scene.entities["ball"] = EntityCfg(
    spec_fn=functools.partial(
      mdp.get_ball_spec,
      radius=BALL_RADIUS,
      mass=BALL_MASS,
      friction=BALL_FRICTION,
      solref=BALL_SOLREF,
    ),
    init_state=EntityCfg.InitialStateCfg(pos=(0.6, 0.0, BALL_RADIUS)),
  )

  # The foot height scan only fed the dropped foot-clearance terms; raycasts are
  # the most expensive sensor on a CPU run.
  sensors = [s for s in (cfg.scene.sensors or ()) if s.name != "foot_height_scan"]
  sensors += [
    _ball_contact_sensor(
      "ball_head_contact", ContactMatch(mode="geom", pattern=_HEAD_PATTERN, entity="robot")
    ),
    _ball_contact_sensor(
      "ball_leg_contact", ContactMatch(mode="subtree", pattern=_LEG_ROOTS, entity="robot")
    ),
    _ball_contact_sensor(
      "ball_torso_contact",
      ContactMatch(mode="geom", pattern=_CHASSIS_PATTERN, entity="robot"),
    ),
    ContactSensorCfg(
      name="body_ground_contact",
      primary=ContactMatch(mode="geom", pattern=_BODY_PATTERN, entity="robot"),
      secondary=ContactMatch(mode="body", pattern="terrain"),
      fields=("found", "force"),
      reduce="netforce",
      num_slots=1,
      history_length=PUPPER_DECIMATION,
    ),
  ]
  cfg.scene.sensors = tuple(sensors)
  # Room for the ball's floor and body contacts on top of the legs'.
  cfg.sim.nconmax = 64


def _observations(cfg: ManagerBasedRlEnvCfg) -> None:
  # Actor: the deployable 36-dim frame; the command slot carries the kick target.
  cfg.observations["actor"] = ObservationGroupCfg(
    terms={
      "policy": ObservationTermCfg(
        func=mdp.pupper_actor_obs,
        params={
          "command_name": "kick",
          "add_noise": True,
          "imu_latency_distribution": mdp.PUPPER_IMU_LATENCY_DIST,
        },
      ),
    },
    concatenate_terms=True,
    enable_corruption=False,
    history_length=20,
  )
  critic = cfg.observations["critic"]
  critic.terms.pop("command", None)
  critic.terms.pop("foot_height", None)
  critic.terms["kick_command"] = ObservationTermCfg(
    func=mdp.generated_commands, params={"command_name": "kick"}
  )
  critic.terms["kick_state"] = ObservationTermCfg(
    func=mdp.kick_privileged, params={"command_name": "kick"}
  )


def _events(cfg: ManagerBasedRlEnvCfg) -> None:
  ev = cfg.events
  # The kick command places the robot and the ball together.
  ev.pop("reset_base", None)
  ev.pop("push_robot", None)
  ev["reset_robot_joints"].params["position_range"] = (-JOINT_RESET_OFFSET, JOINT_RESET_OFFSET)
  ev["effort_limits"] = EventTermCfg(
    mode="startup",
    func=dr.effort_limits,
    params={
      "asset_cfg": SceneEntityCfg("robot", actuator_names=(".*",)),
      "effort_limit_range": EFFORT_LIMIT_SCALE,
      "operation": "scale",
    },
  )
  # Ball properties change every episode (they run before the command places
  # the ball, so the new radius sets the spawn height).
  ev["ball_mass"] = EventTermCfg(
    mode="reset",
    func=dr.pseudo_inertia,
    params={
      "asset_cfg": SceneEntityCfg("ball", body_names=("ball",)),
      "alpha_range": tuple(0.5 * math.log(s) for s in BALL_MASS_SCALE),
    },
  )
  ev["ball_radius"] = EventTermCfg(
    mode="reset",
    func=dr.geom_size,
    params={
      "asset_cfg": SceneEntityCfg("ball", geom_names=("ball_geom",)),
      "ranges": BALL_RADIUS_SCALE,
      "operation": "scale",
      "axes": [0],
    },
  )
  ev["ball_slide_friction"] = EventTermCfg(
    mode="reset",
    func=dr.geom_friction,
    params={
      "asset_cfg": SceneEntityCfg("ball", geom_names=("ball_geom",)),
      "ranges": BALL_SLIDE_FRICTION,
      "operation": "abs",
      "axes": [0],
    },
  )
  ev["ball_restitution"] = EventTermCfg(
    mode="reset",
    func=mdp.geom_damping_ratio,
    params={
      "asset_cfg": SceneEntityCfg("ball", geom_names=("ball_geom",)),
      "ranges": BALL_DAMPING_RATIO,
    },
  )


def _rewards(cfg: ManagerBasedRlEnvCfg) -> None:
  w = REWARD_WEIGHTS
  kick = {"command_name": "kick"}
  old = cfg.rewards
  cfg.rewards = {
    "approach_velocity": RewardTermCfg(
      func=mdp.approach_velocity, weight=w["approach_velocity"], params=dict(kick)
    ),
    "kick_alignment": RewardTermCfg(
      func=mdp.kick_alignment, weight=w["kick_alignment"], params=dict(kick)
    ),
    "header": RewardTermCfg(func=mdp.header, weight=w["header"], params=dict(kick)),
    "kick_quality": RewardTermCfg(
      func=mdp.kick_quality, weight=w["kick_quality"], params=dict(kick)
    ),
    "head_recontact": RewardTermCfg(
      func=mdp.head_recontact, weight=w["head_recontact"], params=dict(kick)
    ),
    "leg_ball_contact": RewardTermCfg(
      func=mdp.ball_contact,
      weight=w["leg_ball_contact"],
      params={"sensor_name": "ball_leg_contact"},
    ),
    "post_kick_stop": RewardTermCfg(
      func=mdp.post_kick_stop, weight=w["post_kick_stop"], params=dict(kick)
    ),
    "fell": RewardTermCfg(
      func=mdp.termination_event,
      weight=w["fell"],
      params={"term_name": ("fell_over", "body_ground")},
    ),
    "low_base": RewardTermCfg(
      func=mdp.base_below, weight=w["low_base"], params={"min_height": MIN_BASE_HEIGHT}
    ),
    "joint_speed": RewardTermCfg(
      func=mdp.joint_speed_excess, weight=w["joint_speed"], params={"limit": JOINT_SPEED_LIMIT}
    ),
    "foul": RewardTermCfg(
      func=mdp.termination_event, weight=w["foul"], params={"term_name": "foul"}
    ),
    "missed": RewardTermCfg(
      func=mdp.termination_event, weight=w["missed"], params={"term_name": "missed_ball"}
    ),
    "upright": RewardTermCfg(
      func=velocity_mdp.upright,
      weight=w["upright"],
      params={
        "std": math.sqrt(0.2),
        "asset_cfg": SceneEntityCfg("robot", body_names=("base_link",)),
      },
    ),
    "lin_vel_z_l2": RewardTermCfg(func=mdp.lin_vel_z_l2, weight=w["lin_vel_z_l2"]),
    "ang_vel_xy_l2": RewardTermCfg(func=mdp.ang_vel_xy_l2, weight=w["ang_vel_xy_l2"]),
    "action_rate_l2": RewardTermCfg(func=mdp.action_rate_l2, weight=w["action_rate_l2"]),
    "joint_torques_l2": RewardTermCfg(
      func=mdp.joint_torques_l2, weight=w["joint_torques_l2"]
    ),
  }
  # Walking-task terms, definitions kept: new name -> walking-task name.
  # abduction_angle penalizes the _2 joints (abduction; _1 is hip pitch).
  reused = {
    "knee_ground_contact": "knee_ground_contact",
    "self_collision_r": "self_collision_r",
    "self_collision_l": "self_collision_l",
    "abduction_deviation": "abduction_angle",
    "foot_slip": "foot_slip",
  }
  for name, old_name in reused.items():
    term = old[old_name]
    term.weight = w[name]
    if "command_name" in term.params:  # foot_slip gates on a command norm
      term.params["command_name"] = "kick"
      term.params["command_threshold"] = 0.0
    cfg.rewards[name] = term


def _terminations(cfg: ManagerBasedRlEnvCfg) -> None:
  cfg.terminations = {
    "time_out": TerminationTermCfg(func=mdp.time_out, time_out=True),
    "fell_over": TerminationTermCfg(
      func=mdp.bad_orientation, params={"limit_angle": FELL_OVER_LIMIT}
    ),
    "body_ground": TerminationTermCfg(
      func=mdp.body_ground, params={"sensor_name": "body_ground_contact"}
    ),
    # A finished attempt is a true terminal state: no reward follows it.
    "kick_done": TerminationTermCfg(func=mdp.kick_done, params={"command_name": "kick"}),
    "foul": TerminationTermCfg(func=mdp.foul, params={"command_name": "kick"}),
    "missed_ball": TerminationTermCfg(
      func=mdp.missed_ball, params={"command_name": "kick"}
    ),
  }


def pupper_header_kick_env_cfg(
  play: bool = False, perception: mdp.BallPerceptionCfg | None = None
) -> ManagerBasedRlEnvCfg:
  """Header kick on flat ground. ``play`` runs every episode at full difficulty."""
  cfg = pupper_flat_env_cfg(play=play)
  cfg.episode_length_s = EPISODE_S

  cfg.commands = {
    "kick": mdp.KickCommandCfg(
      initial_level=1.0 if play else 0.0,
      adaptive_curriculum=not play,
      rolling_decel=BALL_ROLL_DECEL,
      perception=perception if perception is not None else mdp.BallPerceptionCfg(),
    )
  }
  _scene(cfg)
  _observations(cfg)
  _events(cfg)
  _rewards(cfg)
  _terminations(cfg)
  cfg.curriculum = {}
  cfg.viewer.body_name = "base_link"
  return cfg
