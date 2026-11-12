"""Header-kick MDP terms (extends the Pupper MDP)."""

from mjlab.tasks.pupper.mdp import *  # noqa: F401, F403

from .events import geom_damping_ratio  # noqa: F401
from .kick_command import (  # noqa: F401
  BallPerceptionCfg,
  KickCommand,
  KickCommandCfg,
  contact_any,
  heading_frame,
  perfect_perception,
)
from .rewards import (  # noqa: F401
  approach_velocity,
  ball_contact,
  base_below,
  body_ground,
  foul,
  head_recontact,
  header,
  kick_alignment,
  kick_done,
  joint_speed_excess,
  kick_privileged,
  kick_quality,
  missed_ball,
  post_kick_stop,
  termination_event,
)
from .scene import (  # noqa: F401
  CHASSIS_GEOMS,
  HEAD_BLOCK,
  HEAD_BUMPER,
  HEAD_COLLISION,
  HEAD_FRONT_X,
  HEAD_GEOMS,
  TORSO_SHELL,
  get_ball_spec,
  get_kick_robot_spec,
)
