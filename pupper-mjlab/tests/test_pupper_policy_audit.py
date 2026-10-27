"""Saved training YAML is comparison data, never a Python execution request."""
import builtins

import pytest
import yaml

from mjlab.scripts.pupper_policy_audit import load_training_config


def test_known_training_tags_load_without_importing_or_resolving_python_names(monkeypatch):
  text = """
tuple: !!python/tuple [1, 2, null]
function: !!python/name:mjlab.tasks.pupper_gait.mdp.gait.pupper_gait_actor_obs ''
factory: !!python/name:mjlab.entity.entity.%3Clambda%3E ''
indices: !!python/object/apply:builtins.slice [null, 12, 2]
transmission: !!python/object/apply:mjlab.actuator.actuator.TransmissionType [joint]
origin: !!python/object/apply:mjlab.viewer.viewer_config.OriginType [4]
"""
  def forbidden_import(*args, **kwargs):
    raise AssertionError('training YAML must not import a referenced Python name')
  with monkeypatch.context() as patch:
    patch.setattr(builtins, '__import__', forbidden_import)
    result = load_training_config(text)
  assert result == dict(tuple=[1, 2, None],
      function='mjlab.tasks.pupper_gait.mdp.gait.pupper_gait_actor_obs',
      factory='mjlab.entity.entity.<lambda>', indices='slice(None, 12, 2)',
      transmission='joint', origin='OriginType.ASSET_BODY')


@pytest.mark.parametrize('payload', [
  "x: !!python/object/apply:os.system ['touch SHOULD_NOT_EXIST']",
  "x: !!python/object/apply:subprocess.Popen [['echo', 'unexpected']]",
  "x: !!python/object/new:tuple [[1, 2]]",
  "x: !!python/object:arbitrary.Type {}",
  "x: !!python/name:os.system ''",
  "x: !!python/name:mjlab.valid.function 'nonempty argument'",
  "x: !!python/object/apply:builtins.slice [null, null, not_an_integer]",
  "x: !!python/object/apply:builtins.slice {args: [null, null, null]}",
  "x: !!python/object/apply:mjlab.actuator.actuator.TransmissionType [unknown]",
  "x: !!python/object/apply:mjlab.viewer.viewer_config.OriginType [99]",
  "x: !!python/object/apply:mjlab.viewer.viewer_config.OriginType [true]",
])
def test_unknown_or_executable_tags_are_rejected(payload, tmp_path, monkeypatch):
  monkeypatch.chdir(tmp_path)
  with pytest.raises((ValueError, yaml.constructor.ConstructorError)):
    load_training_config(payload)
  assert not (tmp_path / 'SHOULD_NOT_EXIST').exists()


def test_plain_wandb_configuration_remains_supported():
  assert load_training_config('env_cfg:\n  value:\n    decimation: 5\n') == {
    'env_cfg': {'value': {'decimation': 5}}}
  with pytest.raises(ValueError, match='mapping'):
    load_training_config('[1, 2]')
