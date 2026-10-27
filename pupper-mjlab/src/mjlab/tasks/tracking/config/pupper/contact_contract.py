"""Opt-in restoration of the tracking task's intended torso and leg ground hulls.

The inherited walking collision configuration disables every non-ball geometry,
including the hulls added by get_spec_with_hulls. Preserve legacy defaults for
reproducing trained artifacts; explicitly select this contract for new experiments.
"""
import dataclasses

from mjlab.utils.spec_config import CollisionCfg


def enable_tracking_ground_hulls(cfg) -> dict:
  from mjlab.tasks.tracking.config.pupper.env_cfgs import get_spec_with_hulls, HULL_BODIES

  robot = cfg.scene.entities["robot"]
  if robot.spec_fn is not get_spec_with_hulls:
    raise ValueError("Tracking ground hulls require the Pupper tracking model with body hulls")
  robot.collisions = tuple(dataclasses.replace(c, disable_other_geoms=False)
                           for c in robot.collisions) + (
    CollisionCfg(geom_names_expr=(r".*_hull$",), contype=0, conaffinity=1, condim=3,
                 friction=(0.8, 0.02, 0.01), disable_other_geoms=False),
  )
  return {"contract": "tracking_ground_hulls_v1", "hull_count": len(HULL_BODIES),
          "contype": 0, "conaffinity": 1,
          "scope": "Restore intended torso/leg contacts with terrain and active ball colliders; hulls do not collide with each other"}
