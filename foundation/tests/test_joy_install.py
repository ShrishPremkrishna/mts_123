"""The normal installer cannot silently omit the immutable joystick dependency."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

def installer():
    spec = importlib.util.spec_from_file_location('joy_contract_installer', ROOT / 'pupper_gait_deploy/install.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_installer_copies_complete_joy_package_inputs_to_disposable_target(tmp_path):
    module = installer()
    assert module.install(tmp_path, dry_run=False)
    package = tmp_path / 'ros2_ws/src/joy_utils'
    for relative in module.JOY_FILES:
        assert (package / relative).read_bytes() == (module.JOY_SOURCE / relative).read_bytes()
    assert 'rcl_interfaces' in (package / 'package.xml').read_text()
    assert 'test_estop_parameter_contract.cpp' in (package / 'CMakeLists.txt').read_text()


def test_missing_joy_dependency_refuses_before_writing_any_existing_target(tmp_path, monkeypatch):
    module = installer()
    sentinel = tmp_path / 'ros2_ws/src/neural_controller/src/neural_controller.cpp'
    sentinel.parent.mkdir(parents=True)
    sentinel.write_text('preserved working baseline')
    before = {str(path.relative_to(tmp_path)): path.read_bytes() for path in tmp_path.rglob('*') if path.is_file()}
    monkeypatch.setattr(module, 'JOY_SOURCE', tmp_path / 'missing_source')
    assert not module.install(tmp_path, dry_run=False)
    after = {str(path.relative_to(tmp_path)): path.read_bytes() for path in tmp_path.rglob('*') if path.is_file()}
    assert after == before


def test_already_in_place_source_does_not_self_copy_or_create_backup(tmp_path, monkeypatch):
    module = installer()
    package = tmp_path / 'ros2_ws/src/joy_utils'
    for relative in module.JOY_FILES:
        path = package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((module.JOY_SOURCE / relative).read_bytes())
    monkeypatch.setattr(module, 'JOY_SOURCE', package)
    assert module.install(tmp_path, dry_run=False)
    assert not list(package.rglob('*.backup'))
