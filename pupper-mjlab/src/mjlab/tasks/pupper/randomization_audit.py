"""Read actual initialized Pupper physics fields; configuration alone is not proof."""
from __future__ import annotations
import hashlib
from pathlib import Path
import numpy as np
from mjlab.actuator import BuiltinPositionActuator


def snapshot_domain_randomization(env):
  robot = env.scene["robot"]
  n = env.num_envs
  def cpu(value):
    return value.detach().cpu().numpy().copy()
  def defaults(field, ids):
    values = env.sim.get_default_field(field)
    if field in env.sim.per_world_default_fields:
      return values[:, ids]
    return values[ids].unsqueeze(0).expand(n, *values[ids].shape)
  body_ids = robot.indexing.body_ids
  base_id = robot.indexing.root_body_id
  actual_mass = env.sim.model.body_mass[:, body_ids]
  base_com = env.sim.model.body_ipos[:, base_id]
  base_default = defaults("body_ipos", [base_id])[:, 0]
  arrays = dict(body_mass_kg=cpu(actual_mass),
                body_mass_ratio=cpu(actual_mass / defaults("body_mass", body_ids)),
                base_com_offset_m=cpu(base_com-base_default),
                encoder_bias_rad=cpu(robot.data.encoder_bias),
                body_names=np.asarray(robot.body_names), joint_names=np.asarray(robot.joint_names))
  friction_event = env.event_manager.get_term_cfg("foot_friction") if "foot_friction" in env.cfg.events else None
  if friction_event is not None:
    local_ids = friction_event.params["asset_cfg"].geom_ids
    geom_ids = robot.indexing.geom_ids[local_ids]
    arrays["foot_tangential_friction"] = cpu(env.sim.model.geom_friction[:, geom_ids, 0])
    arrays["foot_geom_priority"] = cpu(env.sim.model.geom_priority[geom_ids])
    external_ids = sorted(set(range(env.sim.mj_model.ngeom))-set(robot.indexing.geom_ids.detach().cpu().tolist()))
    arrays["external_geom_priority"] = np.asarray(env.sim.mj_model.geom_priority[external_ids]).copy()
  kp, kd, names = [], [], []
  unsupported = []
  for actuator in robot.actuators:
    if hasattr(actuator, "stiffness") and actuator.stiffness is not None:
      kp.append(cpu(actuator.stiffness)); kd.append(cpu(actuator.damping))
      names.extend(actuator.target_names)
    elif isinstance(actuator, BuiltinPositionActuator):
      ids = actuator.global_ctrl_ids
      kp.append(cpu(env.sim.model.actuator_gainprm[:, ids, 0]))
      kd.append(cpu(-env.sim.model.actuator_biasprm[:, ids, 2]))
      names.extend(actuator.target_names)
    else:
      unsupported.append(type(actuator).__name__)
  if kp:
    arrays["pd_stiffness"] = np.concatenate(kp, axis=1)
    arrays["pd_damping"] = np.concatenate(kd, axis=1)
    arrays["pd_target_names"] = np.asarray(names)
  fields = {}
  for name, value in arrays.items():
    if value.dtype.kind not in "biuf" or name.endswith("_geom_priority"):
      continue
    fields[name] = dict(shape=list(value.shape), minimum=np.min(value, axis=0).tolist(),
                       maximum=np.max(value, axis=0).tolist(), mean=np.mean(value, axis=0).tolist(),
                       std=np.std(value, axis=0).tolist())
  events = [dict(name=name, mode=event.mode, function=f"{event.func.__module__}.{event.func.__name__}")
            for name, event in env.cfg.events.items() if event is not None]
  warnings = []
  if unsupported:
    warnings.append(f"Gain audit not implemented for actuator types: {unsupported}")
  order = [e["name"] for e in events if e["mode"] == "startup"]
  from mjlab.envs.mdp import dr
  if "base_com" in order and "inertial" in order and order.index("base_com") < order.index("inertial") and env.cfg.events["inertial"].func is dr.pseudo_inertia:
    warnings.append("Legacy event order: pseudo_inertia overwrites the earlier base_com offset; sampled COM robustness is absent.")
  return dict(worlds=n, event_order=events, realized_fields=fields, warnings=warnings,
              physics_randomization_events_configured=any(e["name"] in ("foot_friction","base_com","inertial","pd_gains","encoder_bias") for e in events),
              sampling="startup parameters are sampled once per world, not re-sampled each reset",
              observation_noise={group:{name:term.params.get("add_noise") for name,term in cfg.terms.items()
                                       if term is not None and "add_noise" in term.params}
                                 for group,cfg in env.cfg.observations.items()},
              imu_latency_distributions={group:{name:list(term.params["imu_latency_distribution"]) for name,term in cfg.terms.items()
                                               if term is not None and "imu_latency_distribution" in term.params}
                                         for group,cfg in env.cfg.observations.items()},
              geom_priorities={name:value.tolist() for name,value in arrays.items() if name.endswith("_geom_priority")},
              actuator_delays=[dict(type=type(a).__name__, physics_steps_min=a.cfg.delay_min_lag,
                                   physics_steps_max=a.cfg.delay_max_lag) for a in robot.actuators],
              scope="initialized simulator fields; does not establish policy robustness or hardware transfer"), arrays


def write_randomization_snapshot(env, directory):
  report, arrays = snapshot_domain_randomization(env)
  path = Path(directory) / "randomization_snapshot.npz"
  np.savez_compressed(path, **arrays)
  report["snapshot"] = path.name
  report["snapshot_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
  return report
