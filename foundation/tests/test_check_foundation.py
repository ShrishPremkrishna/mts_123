"""Runner orchestration checks: no ROS, network, model or physics jobs."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import pytest

PATH = Path(__file__).parents[1] / 'scripts/check_foundation.py'
spec = importlib.util.spec_from_file_location('check_foundation', PATH)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def test_check_plan_routes_incompatible_environments_without_dropping_groups():
    checks = dict(runner.build_checks('/api/bin/python', '/numeric/bin/python'))
    assert len(checks) == 11
    assert checks['tool_queue'][0] == checks['floor_input'][0] == checks['course_tools'][0] == '/api/bin/python'
    assert checks['classical'][0] == checks['motion_export'][0] == checks['course_follow'][0] == '/numeric/bin/python'
    assert checks['controller_cpp'] == ['bash', 'pupper_gait_deploy/test/run_host_test.sh']
    assert len([p for p in checks['motion_export'] if p.startswith('pupper-mjlab/tests/')]) == 23


def test_known_environment_discovery_and_venv_symlinks_preserved(tmp_path):
    venv = tmp_path / '.venv/bin/python'
    venv.parent.mkdir(parents=True)
    venv.symlink_to(sys.executable)
    assert runner.interpreter_candidates(tmp_path, 'numerical', None)[0] == venv
    assert runner.interpreter_candidates(tmp_path, 'api', None)[0] == tmp_path / runner.API_ENV
    assert runner.interpreter_candidates(tmp_path, 'api', str(venv)) == [venv]


def test_explicit_missing_interpreter_does_not_fall_back(tmp_path):
    selected, attempts = runner.select_interpreter(tmp_path, 'api', str(tmp_path / 'missing'), tmp_path, {}, 1)
    assert selected is None and len(attempts) == 1 and attempts[0]['error'] == 'Not an executable file'


def test_actual_import_preflight_preserves_module_failure(tmp_path, monkeypatch):
    monkeypatch.setitem(runner.REQUIRED, 'api', ('json', 'pupper_intentionally_missing_dependency'))
    selected, attempts = runner.select_interpreter(tmp_path, 'api', sys.executable, tmp_path, runner.check_environment(tmp_path), 5)
    assert selected is None
    assert attempts[0]['runtime']['modules']['json']
    assert attempts[0]['runtime']['module_files']['json'] == json.__file__
    assert 'pupper_intentionally_missing_dependency' not in attempts[0]['runtime']['module_files']
    assert 'ModuleNotFoundError' in attempts[0]['runtime']['errors']['pupper_intentionally_missing_dependency']
    monkeypatch.setitem(runner.REQUIRED, 'api', ('json',))
    selected, attempts = runner.select_interpreter(tmp_path, 'api', sys.executable, tmp_path, runner.check_environment(tmp_path), 5)
    assert selected == str(Path(sys.executable).absolute()) and attempts[0]['runtime']['errors'] == {}


@pytest.mark.parametrize('source_tree, accepted', [('current', True), ('old', False)])
def test_numerical_preflight_checks_checkout_of_editable_install(tmp_path, monkeypatch, source_tree, accepted):
    root = tmp_path / 'current'
    root.mkdir()
    source = tmp_path / source_tree / 'pupper-mjlab/src'
    (source / 'mjlab').mkdir(parents=True)
    (source / 'mjlab/__init__.py').write_text('')
    env = runner.check_environment(tmp_path)
    env['PYTHONPATH'] = str(source)
    monkeypatch.setitem(runner.REQUIRED, 'numerical', ('mjlab',))
    interpreter, attempts = runner.select_interpreter(root, 'numerical', sys.executable, tmp_path, env, 5)
    assert bool(interpreter) == accepted
    if not accepted:
        assert str(source) in attempts[0]['error']
        assert str(root / 'pupper-mjlab/src/mjlab/__init__.py') in attempts[0]['error']


def test_explicit_output_failure_never_silently_redirects(tmp_path):
    exists = tmp_path / 'exists'
    exists.mkdir()
    (exists / 'preserved').write_text('evidence')
    with pytest.raises(RuntimeError, match='new writable --output'):
        runner.prepare_output(tmp_path, exists, 'stamp')
    assert (exists / 'preserved').read_text() == 'evidence'


def test_unwritable_default_falls_back_and_is_reported(tmp_path, monkeypatch):
    def denied(_):
        raise PermissionError('read-only checkout')
    monkeypatch.setattr(runner, '_new_output', denied)
    fallback = tmp_path / 'fallback'
    def create(**_):
        fallback.mkdir()
        return str(fallback)
    monkeypatch.setattr(runner.tempfile, 'mkdtemp', create)
    output, reason = runner.prepare_output(tmp_path, None, 'stamp')
    assert output == fallback and 'read-only checkout' in reason


def test_run_environment_uses_writable_caches_and_no_bytecode(tmp_path, monkeypatch):
    monkeypatch.setenv('MPLCONFIGDIR', '/unwritable/global')
    monkeypatch.setenv('PUPPER_REFERENCE_DIR', '/wrong-policy-input')
    env = runner.check_environment(tmp_path)
    assert env['MPLCONFIGDIR'].startswith(str(tmp_path))
    assert 'PUPPER_REFERENCE_DIR' not in env
    assert env['FORCE_CPU'] == '1' and env['CUDA_VISIBLE_DEVICES'] == '' and env['PYTHONDONTWRITEBYTECODE'] == '1'


def test_missing_command_is_an_evidenced_failure(tmp_path):
    row = runner.execute(['/not/a/program'], tmp_path, os.environ.copy(), tmp_path/'failure.log', 1)
    assert row['returncode'] == 127 and 'FileNotFoundError' in row['error'] and not row['timed_out']


@pytest.mark.skipif(os.name != 'posix', reason='POSIX process-group cleanup')
def test_timeout_stops_spawned_child_even_if_it_ignores_term(tmp_path):
    marker = tmp_path / 'escaped'
    child = 'import signal,time,pathlib;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(.8);pathlib.Path('+repr(str(marker))+').write_text("escaped")'
    parent = 'import subprocess,sys,time;subprocess.Popen([sys.executable,"-c",'+repr(child)+']);print("started",flush=True);time.sleep(30)'
    row = runner.execute([sys.executable, '-c', parent], tmp_path, os.environ.copy(), tmp_path/'timeout.log', .2)
    assert row['returncode'] == 124 and row['timed_out'] and 'started' in (tmp_path/'timeout.log').read_text()
    time.sleep(.85)
    assert not marker.exists()


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf'])
def test_timeouts_must_be_positive_finite(value):
    with pytest.raises(Exception):
        runner.positive_seconds(value)


@pytest.fixture
def main_rig(tmp_path, monkeypatch):
    root = tmp_path/'repo'
    (root/'foundation/configs').mkdir(parents=True)
    (root/'foundation/configs/class-driver-joint-limits.json').write_text('{"sources":[]}')
    monkeypatch.setattr(runner.subprocess, 'run', lambda *a, **kw: SimpleNamespace(stdout='revision\n'))
    monkeypatch.setattr(runner, 'source_hashes', lambda root: {'fixed.py':'hash'})
    monkeypatch.setattr(runner, 'select_interpreter', lambda root, kind, *a: (f'/{kind}/python', [{'runtime':{'errors':{}}}]))
    monkeypatch.setattr(runner, 'build_checks', lambda *a: [('one', ['example']), ('two', ['example'])])
    calls = []
    def execute(command, root, env, log, timeout):
        calls.append(str(log));log.write_text('test result\n')
        return {'command':command, 'returncode':0, 'log':str(log), 'seconds':0, 'timed_out':False}
    monkeypatch.setattr(runner, 'execute', execute)
    return root, tmp_path/'out', calls


def test_main_records_both_interpreters_and_actual_commands(main_rig):
    root, output, calls = main_rig
    assert runner.main(['--repo',str(root),'--output',str(output)]) == 0
    report = json.loads((output/'checks.json').read_text())
    assert report['interpreters'] == {'api':'/api/python','numerical':'/numerical/python'}
    assert report['passed'] and len(report['checks']) == len(calls) == 2
    assert report['runner']['path'] == str(PATH.absolute()) and report['runner']['sha256']


def test_preflight_failure_starts_no_suites_and_keeps_receipt(main_rig, monkeypatch):
    root, output, calls = main_rig
    monkeypatch.setattr(runner, 'select_interpreter', lambda *a: (None,[{'error':'missing fastapi'}]))
    assert runner.main(['--repo',str(root),'--output',str(output)]) == 2
    report = json.loads((output/'checks.json').read_text())
    assert report['status'] == 'preflight_failed' and report['checks'] == calls == []
    assert 'no dependencies were installed' in report['error']


def test_failed_check_is_not_hidden_and_later_check_still_runs(main_rig, monkeypatch):
    root, output, calls = main_rig
    def execute(command, root, env, log, timeout):
        calls.append(str(log));return {'command':command,'returncode':1 if len(calls)==1 else 0}
    monkeypatch.setattr(runner, 'execute', execute)
    assert runner.main(['--repo',str(root),'--output',str(output)]) == 1
    r = json.loads((output/'checks.json').read_text())
    assert [x['returncode'] for x in r['checks']] == [1,0] and not r['passed']


def test_mutated_or_added_sources_invalidate_otherwise_passing_run(main_rig, monkeypatch):
    root, output, _ = main_rig
    snapshots = iter([{'fixed.py':'old'}, {'fixed.py':'new','added.py':'extra'}])
    monkeypatch.setattr(runner, 'source_hashes', lambda root: next(snapshots))
    assert runner.main(['--repo',str(root),'--output',str(output)]) == 1
    r = json.loads((output/'checks.json').read_text())
    assert r['changed_source_during_checks'] == ['fixed.py'] and r['added_source_during_checks'] == ['added.py']


def test_interruption_writes_nonpassing_receipt(main_rig, monkeypatch):
    root, output, _ = main_rig
    def interrupt(*args):raise KeyboardInterrupt
    monkeypatch.setattr(runner, 'execute', interrupt)
    assert runner.main(['--repo',str(root),'--output',str(output)]) == 130
    r = json.loads((output/'checks.json').read_text())
    assert r['status'] == 'interrupted' and not r['passed']


def test_test_overrides_and_provider_keys_are_not_inherited(tmp_path, monkeypatch):
    keys = ('PYTHONPATH','PYTHONHOME','PYTEST_ADDOPTS','PYTEST_PLUGINS','GEMINI_API_KEY','GOOGLE_API_KEY','OPENAI_API_KEY','ANTHROPIC_API_KEY')
    for key in keys:monkeypatch.setenv(key,'test-only-value')
    env=runner.check_environment(tmp_path)
    assert all(key not in env for key in keys)
    assert env['PYTHONNOUSERSITE']=='1'


def test_atomic_report_failure_preserves_previous_receipt(tmp_path, monkeypatch):
    output=tmp_path/'checks.json'
    runner.write_report(output,{'passed':False,'status':'running'})
    old=output.read_bytes()
    def fail(*args):raise OSError('simulated failed atomic replacement')
    monkeypatch.setattr(Path,'replace',fail)
    with pytest.raises(OSError):runner.write_report(output,{'passed':True})
    assert output.read_bytes()==old


def test_missing_source_is_preserved_for_mutation_comparison(tmp_path, monkeypatch):
    monkeypatch.setattr(runner,'watched_files',lambda root:['missing.cpp'])
    assert runner.source_hashes(tmp_path)=={'missing.cpp':None}


def test_malformed_source_manifest_failure_gets_receipt(main_rig, monkeypatch):
    root,output,calls=main_rig
    def bad(_):raise KeyError('sources')
    monkeypatch.setattr(runner,'source_hashes',bad)
    assert runner.main(['--repo',str(root),'--output',str(output)])==2
    r=json.loads((output/'checks.json').read_text())
    assert r['status']=='runner_error' and 'KeyError' in r['error'] and not calls


def test_final_save_failure_returns_runner_error(main_rig,monkeypatch,capsys):
    root,output,_=main_rig;original=runner.write_report
    def write(path,report):
        if report['status']=='passed':raise OSError('disk full')
        original(path,report)
    monkeypatch.setattr(runner,'write_report',write)
    assert runner.main(['--repo',str(root),'--output',str(output)])==2
    assert 'Cannot save final receipt' in capsys.readouterr().err


@pytest.mark.skipif(os.name!='posix',reason='POSIX signal cleanup')
def test_sigterm_to_runner_stops_active_check_and_saves_interruption(tmp_path):
    root=tmp_path/'repo';(root/'foundation/configs').mkdir(parents=True)
    (root/'foundation/configs/class-driver-joint-limits.json').write_text('{"sources":[]}')
    out=tmp_path/'out';started=tmp_path/'started';escaped=tmp_path/'escaped'
    child='import pathlib,time;pathlib.Path('+repr(str(started))+').write_text("ready");time.sleep(1);pathlib.Path('+repr(str(escaped))+').write_text("bad")'
    launcher=tmp_path/'launch.py'
    launcher.write_text('import importlib.util,sys\nfrom types import SimpleNamespace\ns=importlib.util.spec_from_file_location("runner",'+repr(str(PATH))+');r=importlib.util.module_from_spec(s);s.loader.exec_module(r)\nr.subprocess.run=lambda *a,**k:SimpleNamespace(stdout="revision")\nr.source_hashes=lambda root:{}\nr.select_interpreter=lambda *a:(sys.executable,[])\nr.build_checks=lambda *a:[("signal_check",[sys.executable,"-c",'+repr(child)+'])]\nraise SystemExit(r.main(["--repo",'+repr(str(root))+',"--output",'+repr(str(out))+']))\n')
    with (tmp_path/'launcher.log').open('w') as log:
        p=subprocess.Popen([sys.executable,str(launcher)],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+5
            while not started.exists() and p.poll() is None and time.monotonic()<deadline:time.sleep(.02)
            assert started.exists()
            p.terminate();assert p.wait(timeout=5)==130
            receipt=json.loads((out/'checks.json').read_text())
            assert receipt['status']=='interrupted' and receipt['interrupted_signal']=='SIGTERM' and not receipt['passed']
            time.sleep(1.05);assert not escaped.exists()
        finally:
            if p.poll() is None:p.kill();p.wait()
