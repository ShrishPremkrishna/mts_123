"""Keep actual driver target bounds distinct from model joint limits."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('description_inventory', Path(__file__).parents[1] / 'scripts/inspect_robot_description.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_driver_limits_are_not_mechanical_limits():
    result = module.inspect_description('''<robot><joint name="leg"><limit lower="-3" upper="3"/></joint>
      <ros2_control><joint name="leg"><param name="position_min">-2</param>
      <param name="position_max">2</param><state_interface name="position"/></joint></ros2_control></robot>''')
    assert not result['errors']
    assert result['driver_joints']['leg']['parameters'] == {'position_min': -2, 'position_max': 2}
    assert result['driver_joints']['leg']['state_interfaces'] == ['position']


def test_unevaluated_xacro_and_nonfinite_are_visible_gaps():
    result = module.inspect_description('''<robot><joint name="leg"><param name="position_min">${lower}</param>
      <param name="position_max">nan</param></joint></robot>''')
    assert len(result['errors']) == 3
    assert result['driver_joints']['leg']['parameters'] == {}


def test_malformed_and_duplicate_declarations_are_not_successes():
    assert module.inspect_description('<robot>')['errors']
    joint = '<joint name="leg"><param name="position_min">-1</param><param name="position_max">1</param></joint>'
    assert module.inspect_description('<robot>' + joint * 2 + '</robot>')['errors'] == ['leg: duplicate driver declaration']


def test_class_driver_contract_matches_actual_xml():
    import json
    root = Path(__file__).parents[2]
    contract = json.loads((root / 'foundation/configs/class-driver-joint-limits.json').read_text())
    source = root / contract['sources'][0]['path']
    parsed = module.inspect_description(source.read_text())
    assert parsed['sha256'] == contract['sources'][0]['sha256']
    assert not parsed['errors']
    assert {name: [item['parameters']['position_min'], item['parameters']['position_max']]
            for name, item in parsed['driver_joints'].items()} == contract['joint_limits']
