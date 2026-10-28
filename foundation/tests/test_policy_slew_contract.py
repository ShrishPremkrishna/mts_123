import copy
import importlib.util
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('slew_policy_validator',ROOT/'pupper_gait_deploy/download_policy.py')
validator=importlib.util.module_from_spec(spec);spec.loader.exec_module(validator)

def policy():
    return dict(required_controller_features=['position_target_slew_v1'],action_types=['position']*12,
                joint_lower_limits=[-2.]*12,joint_upper_limits=[2.]*12,
                position_target_slew=dict(contract='pupper_position_target_slew_v1',max_rate_rad_s=15.,max_step_s=.02,
                  joint_names=validator.EXPECTED_JOINT_NAMES.copy(),lower_limits=[-1.]*12,upper_limits=[1.]*12,driver_limits_sha256='a'*64))

def test_opt_in_metadata_requires_verified_capability_and_legacy_remains_optional():
    assert validator.position_slew_problems({})==[]
    assert validator.position_slew_problems(policy())==[]
    value=policy();value.pop('required_controller_features')
    assert validator.position_slew_problems(value)
    value=policy();value.pop('position_target_slew')
    assert validator.position_slew_problems(value)

@pytest.mark.parametrize('defect',['rate_bool','rate_large','rate_nan','step_zero','step_large','bounds_nan','bounds_shape','bounds_loose','bounds_reversed','names','sha','actions','missing_actions','unknown_feature','duplicate_feature','feature_type','block_type'])
def test_malformed_slew_contract_cannot_reach_controller(defect):
    p=policy();b=p['position_target_slew']
    if defect=='rate_bool':b['max_rate_rad_s']=True
    elif defect=='rate_large':b['max_rate_rad_s']=15.01
    elif defect=='rate_nan':b['max_rate_rad_s']=float('nan')
    elif defect=='step_zero':b['max_step_s']=0
    elif defect=='step_large':b['max_step_s']=.0201
    elif defect=='bounds_nan':b['lower_limits'][0]=float('nan')
    elif defect=='bounds_shape':b['lower_limits'].pop()
    elif defect=='bounds_loose':b['lower_limits'][0]=-3
    elif defect=='bounds_reversed':b['lower_limits'][0]=1.5
    elif defect=='names':b['joint_names'].reverse()
    elif defect=='sha':b['driver_limits_sha256']='not-a-sha'
    elif defect=='actions':p['action_types'][0]='effort'
    elif defect=='missing_actions':p.pop('action_types')
    elif defect=='unknown_feature':p['required_controller_features'].append('other')
    elif defect=='duplicate_feature':p['required_controller_features']*=2
    elif defect=='feature_type':p['required_controller_features']='position_target_slew_v1'
    elif defect=='block_type':p['position_target_slew']=None
    assert validator.position_slew_problems(p)


def test_installer_stages_slew_dependency_build_and_runtime_declarations(tmp_path):
    # Exercise the actual installer in a disposable monorepo, including its
    # overlay layout. Missing source/header/package mappings break Pi rebuilds.
    import xml.etree.ElementTree as ET
    installer_spec=importlib.util.spec_from_file_location('slew_install',ROOT/'pupper_gait_deploy/install.py')
    installer=importlib.util.module_from_spec(installer_spec);installer_spec.loader.exec_module(installer)
    assert installer.install(tmp_path, dry_run=False)
    package=tmp_path/'ros2_ws/src/neural_controller'
    for relative in ['include/neural_controller/position_target_slew.hpp',
                     'test/test_position_target_slew.cpp','package.xml','CMakeLists.txt']:
        assert (package/relative).is_file()
    manifest=ET.parse(package/'package.xml').getroot()
    assert 'rcl_interfaces' in [v.text for v in manifest.findall('depend')]
    assert (package/'include/neural_controller/position_target_slew.hpp').read_bytes()==(
        ROOT/'pupper_gait_deploy/src/position_target_slew.hpp').read_bytes()
