"""Joystick coverage and actual launch-file wiring, without ROS or execution."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import re
import sys
from types import ModuleType, SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / 'gemini_pupper/robot'
sys.path.insert(0, str(ROBOT))
from controller_coverage import ESTOP_DEFAULTS, combined_course_config, estop_parameters
from tricks import WALK_CONTROLLER, load_tricks


@pytest.fixture
def course():
    return yaml.safe_load((ROOT / 'pupper_gait_deploy/config.yaml').read_text())


def test_every_real_catalog_controller_is_covered_without_new_face_button_mapping(course):
    original = deepcopy(course)
    names = [WALK_CONTROLLER]
    for trick in load_tricks().values():
        if trick.kind == 'policy': names.append(trick.controller)
        if trick.walks: names.append(trick.walk_controller)
    combined = combined_course_config(course, names)
    actual = estop_parameters(combined)
    baseline = original['estop_controller']['ros__parameters']
    expected_prefix = ['neural_controller_default', 'neural_controller_three_legged',
                       'neural_controller', 'neural_controller_student']
    assert actual['controller_names'][:4] == baseline['controller_names'] == expected_prefix
    assert actual['switch_button_indices'] == baseline['switch_button_indices'] == [0, 1, 2, 3]
    assert actual['controller_names'] == expected_prefix + ['neural_controller_default_alt'] + names
    # joy_utils uses the first four entries for buttons; switch-away and estop
    # iterate the whole list. Every added actor is consequently deactivated for
    # each face-button target, without acquiring a face-button activation path.
    for target in actual['controller_names'][:len(actual['switch_button_indices'])]:
        deactivate = [name for name in actual['controller_names'] if name != target]
        assert set(names + ['neural_controller_default_alt']) <= set(deactivate)
    assert set(names) <= set(actual['controller_names'])
    assert course == original
    # All control settings, paths, gains and velocity limits are unchanged.
    combined['estop_controller'] = deepcopy(original['estop_controller'])
    assert combined == original


def test_explicit_stop_indices_retain_actual_reviewed_cpp_defaults(course):
    source = (ROOT / 'pupperv3-monorepo/ros2_ws/src/joy_utils/src/estop_controller.cpp').read_text()
    for key, value in ESTOP_DEFAULTS.items():
        assert re.search(r'declare_parameter<int>\(\s*"' + key + r'",\s*' + str(value) + r'\)', source)
    params = estop_parameters(combined_course_config(course, ['gemini_walk']))
    assert params['estop_index'] == 12 and params['estop_release_index'] == 9
    course['estop_controller']['ros__parameters'].update(estop_index=11, estop_release_index=8)
    params = estop_parameters(combined_course_config(course, ['gemini_walk']))
    assert params['estop_index'] == 11 and params['estop_release_index'] == 8


def test_absent_alternate_is_not_invented_or_existing_alternate_duplicated(course):
    base = course['estop_controller']['ros__parameters']
    base['controller_names'].append('neural_controller_default_alt')
    assert estop_parameters(combined_course_config(course, ['gemini_walk']))['controller_names'].count('neural_controller_default_alt') == 1
    base['controller_names'].pop()
    del course['controller_manager']['ros__parameters']['neural_controller_default_alt']
    assert 'neural_controller_default_alt' not in estop_parameters(combined_course_config(course, ['gemini_walk']))['controller_names']


@pytest.mark.parametrize('names', [[], ['same', 'same'], ['bad/name'], [''], ['1name'], [None],
                                  ['neural_controller_default'], ['joint_state_broadcaster']])
def test_invalid_or_colliding_additions_are_rejected(course, names):
    before = deepcopy(course)
    with pytest.raises(ValueError): combined_course_config(course, names)
    assert course == before


@pytest.mark.parametrize('fault', ['duplicate_name', 'invalid_name', 'undeclared_name', 'empty_buttons',
    'too_many_buttons', 'duplicate_button', 'negative_button', 'boolean_button', 'stop_overlap',
    'duplicate_stops', 'negative_stop', 'bad_alternate_type'])
def test_invalid_baseline_mapping_is_rejected(course, fault):
    params = course['estop_controller']['ros__parameters']
    if fault == 'duplicate_name': params['controller_names'][1] = params['controller_names'][0]
    elif fault == 'invalid_name': params['controller_names'][0] = '/namespaced'
    elif fault == 'undeclared_name': params['controller_names'][0] = 'missing'
    elif fault == 'empty_buttons': params['switch_button_indices'] = []
    elif fault == 'too_many_buttons': params['switch_button_indices'] = list(range(5))
    elif fault == 'duplicate_button': params['switch_button_indices'][1] = 0
    elif fault == 'negative_button': params['switch_button_indices'][0] = -1
    elif fault == 'boolean_button': params['switch_button_indices'][0] = False
    elif fault == 'stop_overlap': params['estop_index'] = 0
    elif fault == 'duplicate_stops': params['estop_index'] = 9
    elif fault == 'negative_stop': params['estop_release_index'] = -1
    else: course['controller_manager']['ros__parameters']['neural_controller_default_alt']['type'] = 'other'
    with pytest.raises(ValueError): combined_course_config(course, ['gemini_walk'])


def test_runtime_contract_never_fills_in_missing_stop_parameters(course):
    with pytest.raises(ValueError, match='explicitly'): estop_parameters(course)


@pytest.fixture
def launch_modules(monkeypatch, tmp_path):
    """Capture launch descriptions; never execute a Node or ExecuteProcess."""
    class Item:
        def __init__(self, *args, **kwargs): self.args, self.kwargs = args, kwargs
    modules = {
        'launch': ['LaunchDescription'],
        'launch.actions': ['DeclareLaunchArgument', 'ExecuteProcess', 'IncludeLaunchDescription'],
        'launch.launch_description_sources': ['PythonLaunchDescriptionSource'],
        'launch.substitutions': ['Command', 'FindExecutable', 'PathJoinSubstitution', 'LaunchConfiguration',
                                 'PythonExpression', 'TextSubstitution', 'IfElseSubstitution'],
        'launch.conditions': ['IfCondition', 'UnlessCondition'],
        'launch_ros': [], 'launch_ros.actions': ['Node'],
        'launch_ros.parameter_descriptions': ['ParameterFile'],
        'launch_ros.substitutions': ['FindPackageShare'],
        'ament_index_python': [], 'ament_index_python.packages': [],
    }
    for name, symbols in modules.items():
        module = ModuleType(name)
        for symbol in symbols: setattr(module, symbol, type(symbol, (Item,), {}))
        monkeypatch.setitem(sys.modules, name, module)
    share = tmp_path / 'share'
    (share / 'launch').mkdir(parents=True)
    (share / 'launch/config.yaml').write_bytes((ROOT / 'pupper_gait_deploy/config.yaml').read_bytes())
    monkeypatch.setattr(sys.modules['ament_index_python.packages'], 'get_package_share_directory', lambda name: str(share), raising=False)
    def load(relative, name):
        spec = importlib.util.spec_from_file_location(name, ROOT / relative)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return SimpleNamespace(load=load, share=share)


def test_course_launch_default_is_unchanged_and_override_reaches_all_config_consumers(launch_modules):
    module = launch_modules.load('pupper_gait_deploy/launch.py', 'course_launch_coverage_test')
    entities = module.generate_launch_description().args[0]
    declarations = {e.kwargs['name']: e for e in entities if type(e).__name__ == 'DeclareLaunchArgument'}
    default = declarations['config_file'].kwargs['default_value']
    assert type(default).__name__ == 'PathJoinSubstitution'
    assert default.args[0][0].args == ('neural_controller',)
    assert default.args[0][1:] == ['launch', 'config.yaml']
    assert declarations['sim'].kwargs['default_value'] == 'False'
    assert declarations['teleop'].kwargs['default_value'] == 'True'
    covered = set()
    for entity in entities:
        if type(entity).__name__ != 'Node': continue
        if entity.kwargs['executable'] in ('joy_node', 'teleop_node', 'ros2_control_node', 'estop_controller'):
            parameter = entity.kwargs['parameters'][0]
            assert type(parameter).__name__ == 'ParameterFile' and parameter.kwargs['allow_substs'] is True
            assert type(parameter.args[0]).__name__ == 'LaunchConfiguration'
            assert parameter.args[0].args == ('config_file',)
            covered.add(entity.kwargs['executable'])
        if entity.kwargs['executable'] == 'spawner' and entity.kwargs['arguments'][0].startswith('neural_controller'):
            assert '--inactive' in entity.kwargs['arguments']
    assert covered == {'joy_node', 'teleop_node', 'ros2_control_node', 'estop_controller'}


def test_gemini_passes_generated_file_explicitly_and_validates_all_names_before_actions(launch_modules, tmp_path, monkeypatch):
    module = launch_modules.load('gemini_pupper/robot/launch/gemini_pupper.launch.py', 'gemini_launch_coverage_test')
    monkeypatch.setattr(module, 'TRICK_PARAMS', tmp_path / 'runtime/tricks.yaml')
    monkeypatch.setattr(module, 'COMBINED_COURSE_CONFIG', tmp_path / 'runtime/course.yaml')
    tricks = {'finite': SimpleNamespace(kind='policy', controller='trick_finite', walks=False),
              'interactive': SimpleNamespace(kind='policy', controller='trick_interactive', walks=True, walk_controller='walk_interactive')}
    monkeypatch.setattr(module, 'load_tricks', lambda: tricks)
    monkeypatch.setattr(module, 'controller_params', lambda trick, base: deepcopy(base))
    monkeypatch.setattr(module, 'walk_policy_params', lambda trick, base: deepcopy(base))
    monkeypatch.setattr(module, 'validate_locomotion_contract', lambda *args: None)
    monkeypatch.setattr(module, 'follower_config', lambda: 'disabled')
    monkeypatch.setattr(module, 'person_follower', lambda: [])
    monkeypatch.setattr(module, 'camera_white_balance', lambda: [])
    manifests = []
    monkeypatch.setattr(module, 'write_launch_manifest', lambda *args, **kwargs: manifests.append((args, kwargs)))
    entities = module.generate_launch_description().args[0]
    include = next(e for e in entities if type(e).__name__ == 'IncludeLaunchDescription')
    assert dict(include.kwargs['launch_arguments']) == {'teleop': 'False', 'config_file': str(module.COMBINED_COURSE_CONFIG)}
    teleop = next(e for e in entities if e.kwargs.get('executable') == 'teleop_node')
    assert teleop.kwargs['parameters'] == [str(module.COMBINED_COURSE_CONFIG)]
    assert teleop.kwargs['remappings'] == [('cmd_vel', 'teleop_cmd_vel')]
    combined = yaml.safe_load(module.COMBINED_COURSE_CONFIG.read_text())
    assert estop_parameters(combined)['controller_names'][-4:] == ['gemini_walk', 'trick_finite', 'trick_interactive', 'walk_interactive']
    args, kwargs = manifests[0]
    assert list(args[0]) == ['gemini_walk', 'trick_finite', 'trick_interactive', 'walk_interactive']
    assert kwargs == {'course_config_path': launch_modules.share / 'launch/config.yaml', 'combined_config_path': module.COMBINED_COURSE_CONFIG}
    assert yaml.safe_load((launch_modules.share / 'launch/config.yaml').read_text())['estop_controller']['ros__parameters']['controller_names'] == estop_parameters(combined)['controller_names'][:4]
    # Dictionary construction must not hide a duplicate normalized trick name.
    tricks['interactive'].controller = 'trick_finite'
    with pytest.raises(ValueError, match='duplicate'): module.generate_launch_description()
