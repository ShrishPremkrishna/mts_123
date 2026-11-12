"""Ball randomization events with no built-in mjlab equivalent."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.envs.mdp.dr._core import _randomize_model_field
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


@requires_model_fields("geom_solref")
def geom_damping_ratio(
  env: "ManagerBasedRlEnv",
  env_ids: torch.Tensor | None,
  ranges: tuple[float, float],
  asset_cfg: SceneEntityCfg,
) -> None:
  """Set the contact damping ratio (solref[1]) of the matched geoms.

  For the ball (which wins every contact by priority) this sets restitution:
  with the Pupper solver options and a 0.01 s time constant, 0.5 gives a drop
  rebound of e ~ 0.43 and 0.8 gives e ~ 0.18.
  """
  _randomize_model_field(
    env,
    env_ids,
    "geom_solref",
    entity_type="geom",
    ranges=ranges,
    operation="abs",
    asset_cfg=asset_cfg,
    axes=[1],
    valid_axes=[0, 1],
  )
