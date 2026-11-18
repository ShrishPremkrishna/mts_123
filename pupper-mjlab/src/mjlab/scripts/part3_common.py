"""Shared Part 3 (Opt Lab 2, multi-gait distillation) simulation helpers.

Simulation only. Three things every Part 3 rollout needs and that the Part 2
harness (:mod:`mjlab.scripts.part2_eval`) does not do by itself:

* **Lab 4 domain randomization, checked.** The spec asks for the Bumpy pass "with
  your lab 4 DR ranges". The team's Lab 4 record
  (``lab4_runs/shrish_stablegait/training-settings.json``) kept the course
  notebook's defaults, and those are exactly the task defaults in
  ``tasks/pupper/pupper_env_cfg.py``. :func:`check_lab4_dr` asserts that on the
  live config instead of trusting it, so a drifted default fails loudly.
* **The checked Bumpy terrain contract.** With the torso hull on
  (``--locomotion-torso-contact``, a standard flag of every run here) the stock
  0.05 m Perlin heightfield overflows MuJoCo-Warp's per-pair triangle capacity
  and silently truncates collisions (``configs/optional2/README.md``). The
  reviewed fix is a 0.10 m grid plus an accumulated overflow audit
  (``foundation/scripts/bumpy_checked_cli.py``). :func:`rollout_conditions`
  applies the same contract to evaluation rollouts, and a nonzero overflow
  fails the run.
* **DR off.** ``torso_support.nominal_mechanics`` (the repo's diagnostic) drops
  the sampled friction / encoder bias / COM / PD gains / inertia events and keeps
  observation noise, latency and reset randomness.

The context manager patches ``pupper_locomotion.settings`` and
``mjlab.envs.ManagerBasedRlEnv`` for the duration of one rollout; the Part 2
``_Rollout`` looks both up at call time, so its code is reused unchanged.
"""

from __future__ import annotations

import contextlib
import json
import math
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]  # pupper-mjlab/
FLAT_TASK = "Mjlab-MixedGaits-Flat-Pupper-v3"
BUMPY_TASK = "Mjlab-MixedGaits-Bumpy-Pupper-v3"
CHECKED_BUMPY_RESOLUTION_M = 0.10

# Lab 4 DR ranges: the team's recorded Lab 4 settings (= course notebook defaults).
LAB4_DR = {
  "kp_range": (0.6, 1.1),
  "kd_range": (0.8, 1.5),
  "foot_friction": (0.6, 1.4),
}
LAB4_DR_SOURCE = "lab4_runs/shrish_stablegait/training-settings.json"

TORSO_SENSOR = "p3_torso_ground_contact"

CONDITIONS = {
  # name: (task, dr_on)
  "flat_dr": (FLAT_TASK, True),
  "flat_nodr": (FLAT_TASK, False),
  "bumpy_dr": (BUMPY_TASK, True),
  "bumpy_nodr": (BUMPY_TASK, False),
}


def lab4_dr_from_file(path: Path | None = None) -> dict[str, tuple[float, float]]:
  """The DR ranges recorded in the team's Lab 4 run settings."""
  path = path or (REPO_ROOT / LAB4_DR_SOURCE)
  rec = json.loads(Path(path).read_text())["domain_randomization"]
  return {
    "kp_range": tuple(rec["kp_multiplier"]),
    "kd_range": tuple(rec["kd_multiplier"]),
    "foot_friction": tuple(rec["friction"]),
  }


def check_lab4_dr(cfg) -> dict[str, Any]:
  """Assert the env config randomizes PD gains and foot friction with Lab 4 ranges."""
  recorded = lab4_dr_from_file()
  if recorded != LAB4_DR:
    raise ValueError(f"Lab 4 record {recorded} != LAB4_DR constant {LAB4_DR}")
  ev = cfg.events
  missing = [n for n in ("pd_gains", "foot_friction") if n not in ev]
  if missing:
    raise ValueError(f"DR events missing from config: {missing}")
  live = {
    "kp_range": tuple(float(v) for v in ev["pd_gains"].params["kp_range"]),
    "kd_range": tuple(float(v) for v in ev["pd_gains"].params["kd_range"]),
    "foot_friction": tuple(float(v) for v in ev["foot_friction"].params["ranges"]),
  }
  bad = {k: (live[k], LAB4_DR[k]) for k in LAB4_DR if not _close(live[k], LAB4_DR[k])}
  if bad:
    raise ValueError(f"DR ranges differ from Lab 4 ({LAB4_DR_SOURCE}): {bad}")
  return dict(
    contract="part3_lab4_dr_v1",
    source=LAB4_DR_SOURCE,
    ranges={k: list(v) for k, v in live.items()},
    other_dr_events=sorted(set(ev) & {"encoder_bias", "base_com", "inertial"}),
  )


def _close(a, b, tol=1e-9):
  return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b, strict=True))


def add_torso_contact_sensor(cfg) -> bool:
  """Read-only torso-hull/terrain contact sensor (works on plane and heightfield)."""
  from mjlab.sensor import ContactMatch, ContactSensorCfg
  from mjlab.tasks.pupper.locomotion_contact_contract import (
    get_spec_with_torso_contact,
  )

  if cfg.scene.entities["robot"].spec_fn is not get_spec_with_torso_contact:
    return False
  if any(s.name == TORSO_SENSOR for s in cfg.scene.sensors or ()):
    return True
  cfg.scene.sensors = tuple(cfg.scene.sensors or ()) + (
    ContactSensorCfg(
      name=TORSO_SENSOR,
      primary=ContactMatch(mode="geom", pattern=r"^base_link_hull$", entity="robot"),
      secondary=ContactMatch(mode="body", pattern="terrain"),
      fields=("found",),
      reduce="netforce",
      num_slots=1,
    ),
  )
  return True


@contextlib.contextmanager
def rollout_conditions(
  args,
  *,
  dr: bool = True,
  checked_bumpy: bool = True,
  audit_path: Path | None = None,
  torso_sensor: bool = False,
  render: bool = False,
  viewer: dict | None = None,
):
  """Patch the Part 2 rollout's config/env construction for one Part 3 condition.

  Records what was applied in ``args.part3_condition`` (which ``provenance``
  serializes into every output).
  """
  import mjlab.envs as envs
  from mjlab.scripts import pupper_locomotion as cli

  original_settings = cli.settings
  original_env = envs.ManagerBasedRlEnv
  record: dict[str, Any] = dict(task=args.task, dr=dr)
  args.part3_condition = record
  audits: list = []

  def settings(s_args, play):
    cfg, agent = original_settings(s_args, play)
    record["lab4_dr"] = check_lab4_dr(cfg)  # checked even when DR is then removed
    if not dr:
      from mjlab.tasks.pupper.torso_support import nominal_mechanics

      record["nominal_mechanics"] = nominal_mechanics(cfg)
    terrain = cfg.scene.terrain
    bumpy = terrain is not None and terrain.terrain_type == "generator"
    record["terrain"] = "bumpy" if bumpy else "flat"
    if bumpy and checked_bumpy:
      from mjlab.tasks.pupper.heightfield_contract import apply_perlin_resolution

      record["heightfield"] = apply_perlin_resolution(cfg, CHECKED_BUMPY_RESOLUTION_M)
    elif bumpy:
      record["heightfield"] = "UNCHECKED stock 0.05 m grid (diagnostic only)"
    if torso_sensor:
      record["torso_sensor"] = add_torso_contact_sensor(cfg)
    if viewer:
      for k, v in viewer.items():
        setattr(cfg.viewer, k, v)
    return cfg, agent

  class CheckedEnv(original_env):
    def __init__(self, cfg, *a, **kw):
      if render:
        kw["render_mode"] = "rgb_array"
      super().__init__(cfg, *a, **kw)
      if record.get("terrain") == "bumpy" and audit_path is not None:
        from mjlab.tasks.pupper.heightfield_contract import PhysicsOverflowAudit

        audits.append(PhysicsOverflowAudit(self.sim, audit_path, check_every=200))

    def close(self):
      try:
        while audits:
          record["overflow_audit"] = _summarize_audit(audits.pop().close())
      finally:
        super().close()

  cli.settings = settings
  envs.ManagerBasedRlEnv = CheckedEnv
  try:
    yield record
  finally:
    cli.settings = original_settings
    envs.ManagerBasedRlEnv = original_env


def _summarize_audit(report):
  return {k: report[k] for k in ("contract", "physics_steps_observed", "zero_overflow", "affected_worlds")}


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float | None, float | None]:
  """95 % Wilson score interval for a binomial rate."""
  if n <= 0:
    return (None, None)
  p = k / n
  den = 1 + z * z / n
  c = (p + z * z / (2 * n)) / den
  h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
  return (max(0.0, c - h), min(1.0, c + h))
