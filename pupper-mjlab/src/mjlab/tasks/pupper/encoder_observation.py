"""Explicit sensor-coordinate contract for future Pupper locomotion experiments.

Legacy locomotion actors see true q even though actions are in encoder coordinates.
The opt-in measured mode sees q+b. Both modes keep command conversion u-b, fixed
reference/default poses, velocity, noise draws and IMU latency unchanged.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

MODES = ('legacy_true', 'deployment_measured')
CHECKPOINT_KEY = 'pupper_encoder_observation'
_PERMISSION_ATTR = '_pupper_encoder_observation_load_permission'
_CONTEXT_ATTR = '_pupper_encoder_observation_context'
_LOADED_ATTR = '_pupper_encoder_observation_loaded'
LOAD_KEY = 'pupper_encoder_observation_source_checkpoint'
_ACTORS = {'pupper_actor_obs', 'pupper_gait_actor_obs', 'pupper_gait_shared_obs',
           'mixed_gait_actor_obs', 'zero_reference_actor_obs', 'jump_actor_obs',
           'mixed_jump_actor_obs'}


def validate_mode(mode):
  if mode not in MODES:
    raise ValueError(f'Encoder observation mode must be one of {MODES}')
  return mode


def contract(mode):
  return {'contract': 'pupper_encoder_observation_v1', 'mode': validate_mode(mode),
          'joint_position': 'q+b-default' if mode == 'deployment_measured' else 'q-default',
          'position_command_to_physical_target': 'u-b',
          'bias_definition': 'measured_position_minus_true_position',
          'fixed_reference_and_default_unchanged': True}


def validate_contract(value):
  if (not isinstance(value, dict) or type(value.get('fixed_reference_and_default_unchanged')) is not bool
      or value != contract(value.get('mode'))):
    raise ValueError('Invalid Pupper encoder observation contract')
  return dict(value)


def _actor_term(cfg):
  group = getattr(cfg, 'observations', {}).get('actor')
  term = group.terms.get('policy') if group is not None else None
  func = getattr(term, 'func', None)
  if (getattr(func, '__name__', '') not in _ACTORS or
      not getattr(func, '__module__', '').startswith(('mjlab.tasks.pupper.', 'mjlab.tasks.pupper_gait.'))):
    return None
  return term


def metadata(env_or_cfg):
  cfg = getattr(env_or_cfg, 'cfg', env_or_cfg)
  term = _actor_term(cfg)
  return None if term is None else contract(term.params.get('encoder_observation_mode', 'legacy_true'))


def apply_mode(cfg, mode):
  """Change only the actor and an identical shared-actor critic frame, if present."""
  mode = validate_mode(mode)
  term = _actor_term(cfg)
  if term is None:
    raise ValueError('Encoder observation modes support Pupper locomotion actors only; tracking already uses measured joints')
  term.params['encoder_observation_mode'] = mode
  # Shared actor/critic terms use one per-step cache. Privileged critic terms
  # remain true state; never add encoder bias to privileged physics observations.
  if term.func.__name__ == 'pupper_gait_shared_obs':
    critic = cfg.observations.get('critic')
    other = critic.terms.get('policy') if critic is not None else None
    if other is None or other.func is not term.func:
      raise ValueError('Shared actor frame requires its matching cached critic term')
    other.params['encoder_observation_mode'] = mode
  return contract(mode)


def _sha(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _checkpoint_snapshot(path):
  """Bind metadata to one immutable byte buffer; reject concurrent replacement."""
  import io
  import torch
  raw = Path(path).read_bytes()
  digest = hashlib.sha256(raw).hexdigest()
  state = torch.load(io.BytesIO(raw), map_location='cpu', weights_only=False)
  if _sha(path) != digest:
    raise RuntimeError('Checkpoint changed while reading encoder observation metadata')
  infos = state.get('infos')
  if infos is not None and not isinstance(infos, dict):
    raise ValueError('Checkpoint infos must be a mapping or None')
  tagged = infos is not None and CHECKPOINT_KEY in infos
  return (validate_contract(infos[CHECKPOINT_KEY]) if tagged else None), digest


def checkpoint_contract(path):
  """Untagged historical checkpoints retain legacy compatibility, never rewritten."""
  return _checkpoint_snapshot(path)[0]


def configure(cfg, *, checkpoint=None, requested_mode=None, allow_change=False, purpose):
  """Infer a stamped mode or deliberately authorize an exact-checkpoint change."""
  if purpose not in ('training', 'evaluation', 'export'):
    raise ValueError('Unknown encoder observation configuration purpose')
  if requested_mode is not None:
    validate_mode(requested_mode)
  if allow_change and (checkpoint is None or requested_mode is None or purpose == 'export'):
    raise ValueError('Mode change requires an exact checkpoint, explicit mode and training/evaluation purpose')
  inherited, digest = _checkpoint_snapshot(checkpoint) if checkpoint is not None else (None, None)
  previous = inherited['mode'] if inherited else 'legacy_true'
  mode = previous if requested_mode is None else requested_mode
  changed = checkpoint is not None and mode != previous
  if changed and not allow_change:
    raise ValueError('Checkpoint encoder observation mode differs; use explicit --allow-encoder-observation-mode-change for this separate experiment')
  current = apply_mode(cfg, mode)
  permission = None
  if changed:
    permission = {'checkpoint_sha256': digest, 'from_mode': previous, 'to_mode': mode, 'purpose': purpose}
  setattr(cfg, _PERMISSION_ATTR, permission)
  context = {'contract': current, 'purpose': purpose, 'checkpoint_sha256': digest,
          'checkpoint_training_mode': previous if checkpoint is not None else None,
          'checkpoint_mode_tagged': inherited is not None,
          'active_observation_mode': mode, 'mode_changed_explicitly': changed,
          'mode_change_permission': permission}
  setattr(cfg, _CONTEXT_ATTR, context)
  return context


def check_load(env, path):
  expected, digest = _checkpoint_snapshot(path)
  cfg = getattr(env, 'cfg', env)
  context = getattr(cfg, _CONTEXT_ATTR, None)
  if isinstance(context, dict) and context.get('checkpoint_sha256') not in (None, digest):
    raise ValueError('Configured exact-checkpoint identity changed before loading')
  previous = expected['mode'] if expected else 'legacy_true'
  actual = metadata(env)
  if actual is None:
    if expected is not None:
      raise ValueError('Tagged encoder checkpoint requires a supported Pupper actor environment')
    return None
  changed = actual['mode'] != previous
  permission = getattr(env.cfg, _PERMISSION_ATTR, None)
  if changed and (not isinstance(permission, dict) or permission.get('checkpoint_sha256') != digest
      or permission.get('from_mode') != previous or permission.get('to_mode') != actual['mode']
      or permission.get('purpose') not in ('training', 'evaluation')):
    raise ValueError('Checkpoint encoder observation mode differs without an exact-checkpoint mode-change permission')
  return {'checkpoint_sha256': digest, 'checkpoint_training_mode': previous,
          'checkpoint_mode_tagged': expected is not None,
          'active_observation_mode': actual['mode'], 'mode_changed_explicitly': changed}


def record_successful_load(env, receipt, log_dir=None, *, path=None):
  """Record only after the real actor/optimizer loader has succeeded."""
  if receipt is None:
    return
  if path is not None and _sha(path) != receipt['checkpoint_sha256']:
    raise RuntimeError('Checkpoint changed during actual runner load; refusing to train or export')
  setattr(env.cfg, _LOADED_ATTR, dict(receipt))
  context = getattr(env.cfg, _CONTEXT_ATTR, None)
  if isinstance(context, dict):
    context.update(receipt, checkpoint_load_verified=True)
  if log_dir is not None:
    import json
    target = Path(log_dir) / 'encoder-checkpoint-load.json'
    target.write_text(json.dumps(receipt, indent=2) + '\n')


def checkpoint_infos(env, infos):
  current = metadata(env)
  if current is None:
    if isinstance(infos, dict) and CHECKPOINT_KEY in infos:
      raise ValueError('Cannot save stale encoder metadata from an unsupported environment')
    return infos
  if infos is not None and not isinstance(infos, dict):
    raise ValueError('Checkpoint infos must be a mapping or None')
  result = dict(infos or {})
  loaded = getattr(env.cfg, _LOADED_ATTR, None)
  if CHECKPOINT_KEY in result and validate_contract(result[CHECKPOINT_KEY]) != current:
    # Some callers pass the loaded checkpoint's infos back to save(). Rewrite
    # only this one stale mode after a verified, explicitly authorized load.
    if (not isinstance(loaded, dict) or not loaded.get('mode_changed_explicitly')
        or result[CHECKPOINT_KEY]['mode'] != loaded['checkpoint_training_mode']
        or current['mode'] != loaded['active_observation_mode']):
      raise ValueError('Checkpoint infos disagree with active encoder observation mode')
  result[CHECKPOINT_KEY] = current
  if loaded is not None:
    result[LOAD_KEY] = dict(loaded)
  return result
