"""Ball entity and head-bumper geometry for the header-kick task.

The stock Pupper model collides only through its knee and foot spheres. A header
needs a contact surface on the front of the body, so the kick robot adds three
boxes to ``base_link``: contype 2, conaffinity 1|2. They touch the ball (contype
and conaffinity 1|2) and the floor (contype 1), so the chassis cannot sink
through the floor. Explicit body-pair excludes stop them from touching the leg
spheres, which also have contype 1. A first version left the boxes off the
floor, and the policy learned a crawl with the chassis inside the floor.
"""

from __future__ import annotations

import mujoco

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import get_spec
from mjlab.utils.spec_config import CollisionCfg

# Collision bit shared by the head geoms and the ball (bit 1 is the legs/floor).
HEAD_COLLISION_BIT = 2
# Leg bodies that carry collision spheres (knee on _2, foot on _3).
_LEG_COLLISION_BODIES = tuple(
  f"leg_{fb}_{side}_{k}" for fb in ("front", "back") for side in ("r", "l") for k in (2, 3)
)

# Names of the added base_link geoms. A header is a ball contact on HEAD_GEOMS.
HEAD_BUMPER = "head_bumper"
HEAD_BLOCK = "head_block"
TORSO_SHELL = "torso_shell"
HEAD_GEOMS = (HEAD_BUMPER,)
# Ball contact on these is a foul (side/top of the chassis, not the bumper face).
CHASSIS_GEOMS = (HEAD_BLOCK, TORSO_SHELL)

# Box geometry in the base_link frame: name -> (center, half-size), metres.
# The chassis mesh spans x -0.108..0.151, |y| <= 0.057, z -0.032..0.04 at the
# front (measured from BodyV4v70_001). The bumper is a 2 cm foam pad on the front
# face, as wide as the chassis. Its bottom edge (z = -0.09) is 0.055 m above the
# floor at the 0.145 m standing height, below the nominal ball center (0.075 m),
# so the contact normal stays near horizontal for the whole radius range. It
# reaches the floor at approximately 20 deg of nose-down pitch.
HEAD_BOX_GEOMS: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {
  HEAD_BUMPER: ((0.161, 0.0, -0.03), (0.01, 0.057, 0.06)),
  HEAD_BLOCK: ((0.131, 0.0, 0.004), (0.02, 0.057, 0.036)),
  TORSO_SHELL: ((0.0015, 0.0, 0.004), (0.1095, 0.057, 0.036)),
}

# Front of the bumper along base_link x [m].
HEAD_FRONT_X = HEAD_BOX_GEOMS[HEAD_BUMPER][0][0] + HEAD_BOX_GEOMS[HEAD_BUMPER][1][0]

# Applied after PUPPER_COLLISION, whose disable_other_geoms=True zeroes these
# boxes; this config re-enables them (ball and floor, see module docstring).
HEAD_COLLISION = CollisionCfg(
  geom_names_expr=tuple(rf"^{n}$" for n in HEAD_BOX_GEOMS),
  contype=HEAD_COLLISION_BIT,
  conaffinity=1 | HEAD_COLLISION_BIT,
  condim=3,
  friction=(0.6, 0.005, 0.0001),
  disable_other_geoms=False,
)


def get_kick_robot_spec() -> mujoco.MjSpec:
  """Pupper v3 with the head bumper, head block and torso shell on base_link."""
  spec = get_spec()
  base = spec.body("base_link")
  for name, (pos, half) in HEAD_BOX_GEOMS.items():
    visible = name == HEAD_BUMPER
    base.add_geom(
      name=name,
      type=mujoco.mjtGeom.mjGEOM_BOX,
      size=half,
      pos=pos,
      # The bumper is drawn (group 1, like the visual meshes); the chassis boxes
      # coincide with the mesh, so they stay in the hidden collision group.
      group=1 if visible else 3,
      rgba=(0.15, 0.15, 0.15, 1.0) if visible else (0.5, 0.5, 0.5, 0.3),
      contype=HEAD_COLLISION_BIT,
      conaffinity=1 | HEAD_COLLISION_BIT,
      # base_link has an explicit <inertial>, so geom mass does not change it.
      density=0.0,
    )
  # Leg spheres have contype 1, which the boxes accept; keep the legs free.
  for leg in _LEG_COLLISION_BODIES:
    spec.add_exclude(bodyname1="base_link", bodyname2=leg)
  return spec


def get_ball_spec(
  radius: float,
  mass: float,
  friction: tuple[float, float, float],
  solref: tuple[float, float],
) -> mujoco.MjSpec:
  """A free solid sphere. Priority 2 makes its contact parameters win on every pair.

  condim 3, not 6: with the Pupper solver options (2 iterations, pyramidal cone,
  impratio 10) MuJoCo over-damps rolling friction by an order of magnitude (a
  1 m/s ball stopped in under 1 s at 0.002 m). The kick command applies rolling
  resistance as an explicit force instead (``KickCommandCfg.rolling_decel``).
  """
  spec = mujoco.MjSpec()
  body = spec.worldbody.add_body(name="ball")
  body.add_freejoint(name="ball_joint")
  body.add_geom(
    name="ball_geom",
    type=mujoco.mjtGeom.mjGEOM_SPHERE,
    size=(radius, 0.0, 0.0),
    mass=mass,
    rgba=(1.0, 0.45, 0.05, 1.0),
    condim=3,
    friction=friction,
    solref=solref,
    priority=2,
    contype=1 | HEAD_COLLISION_BIT,
    conaffinity=1 | HEAD_COLLISION_BIT,
  )
  return spec
