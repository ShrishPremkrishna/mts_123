"""Local fixtures only: no ROS, network requests or process signals."""
from contextlib import nullcontext
from copy import deepcopy
import importlib.util
import json
import shutil
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import persistent_session as session
SOURCE_ROOT = HERE.parents[1]


READY_SPAWNERS = '''[spawner-7] [INFO] [spawner_motion]: Loaded neural_controller_default
[spawner-7] [INFO] [spawner_motion]: Loaded gemini_walk
[spawner-7] [INFO] [spawner_motion]: Loaded trick_sit_stay_shake
[spawner-6] [INFO] [spawner_feedback]: Configured and activated joint_state_broadcaster
[spawner-6] [INFO] [spawner_feedback]: Configured and activated imu_sensor_broadcaster
[INFO] [spawner-6]: process has finished cleanly [pid 36]
[INFO] [spawner-7]: process has finished cleanly [pid 37]
'''


class Life:
    SCHEMA = 'gemini_owned_processes_v1'
    def __init__(self, run):
        self.HERE = run / 'app/robot'; self.PROC = run / 'proc'; self.PROC.mkdir()
        self.receipt = {'session_id': 'a' * 32, 'status': 'running', 'processes': {
            'manager': {'pid': 20, 'role': 'manager'}, 'launch': {'pid': 19, 'role': 'launch'}},
            'pending': {}, 'events': []}
        self.states = {'manager': 'running', 'launch': 'running'}
        self.spawned = []; self.sent = []; self.closed = False
        self.native_log = 'Finished homing!\n' + READY_SPAWNERS
    def coordinator(self):
        return SimpleNamespace(execution_lock=lambda *a: nullcontext())
    def check_support(self, run): pass
    def boot_id(self): return 'fixture-boot'
    def save(self, run, receipt): self.receipt = deepcopy(receipt)
    def load(self, run): return deepcopy(self.receipt)
    def require_starting(self, receipt):
        if receipt['status'] != 'starting': raise ValueError('startup canceled')
    def state(self, target, tag): return self.states.get(target['role'], 'exited')
    def spawn(self, run, role, expected, command):
        self.spawned.append((role, expected, command))
        self.receipt['processes'][role] = {'pid': 30 + len(self.spawned), 'role': role}
        self.states[role] = 'running'
        if role == 'launch': (run / 'robot.log').write_text(self.native_log)
    def refresh(self, run, receipt):
        receipt['processes']['manager'] = {'pid': 20, 'role': 'manager'}
        self.states['manager'] = 'running'; self.save(run, receipt)
    def operate(self, run, action, args, expected):
        assert action == 'ready'; assert expected == self.receipt['session_id']
        self.receipt['status'] = 'running'
    def terminate(self, run, receipt, role, target):
        self.sent.append(role); self.states[role] = 'exited'
        receipt['events'].append({'event': 'term_' + role})
    def cleanup(self, run): self.closed = True


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.run = Path(self.tmp.name)
        self.life = Life(self.run)
        topics = self.run / 'topics.json'; topics.write_text('["/joint_states","/llm_cmd_vel"]')
        self.plan = {'session_id': 'a' * 32, 'controllers': ['neural_controller_default', 'gemini_walk', 'trick_sit_stay_shake'], 'app_root': str(self.run / 'app'),
            'environment': {'PORT': '18090'}, 'components': {
                'launch': {'path': '/selected/lab6_session.launch.py'},
                'telemetry_topics': {'path': str(topics)}}}
        session.write(self.run / 'native-readiness.json', {'session_id': 'a' * 32, 'homing_finished': True})
        self.network = patch.object(session, 'http', return_value={
            'status': 'running', 'tools': True, 'runtime': 'robot', 'success': True}).start()
        self.port = patch.object(session, 'port_free').start()
        self.owned = patch.object(session, 'owned_api').start()
        self.maps = patch.object(session, 'check_loaded').start()
        self.addCleanup(patch.stopall)

    def attachment_fixture(self):
        self.run = self.run.resolve()
        self.plan['app_root'] = str(Path(self.plan['app_root']).resolve())
        old = Path(self.plan['app_root']); new = self.run / 'new-app'
        mutable = ['gemini_pupper/robot/server.py', 'gemini_pupper/robot/robot_api.py',
                   'gemini_pupper/robot/tool_registry.py', 'gemini_pupper/lab/part2_walk.py',
                   'gemini_pupper/web/dist/index.html']
        extra = ['gemini_pupper/tricks/sit_stay_shake/policy.json',
                 'gemini_pupper/tricks/sit_stay_shake/trick.yaml',
                 'follow_me_pupper/robot/follow_service.py', 'follow_me_pupper/robot/yolov8n_seg.hef']
        for relative in (*session.API_FIXED_FILES, *mutable, *extra):
            path = old / relative; path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('unchanged fixture: ' + relative)
        shutil.copytree(old, new)
        (new / 'gemini_pupper/robot/server.py').write_text('reviewed new API server')
        (new / 'gemini_pupper/robot/robot_api.py').write_text('reviewed application queue change')
        (new / 'gemini_pupper/web/dist/index.html').write_text('reviewed new browser')
        self.plan['components']['lifecycle'] = session.pinned(old / 'gemini_pupper/robot/session_processes.py')
        for role in ('native', 'imu'):
            library = self.run / (role + '.so'); library.write_text('selected ' + role)
            self.plan['components'][role] = session.pinned(library)
        original_app = old / 'app-stage-manifest.json'
        original_app.write_text(json.dumps({'final_selected_files': [
            {'path': str(path.relative_to(old)), 'sha256': session.sha(path)}
            for path in sorted(old.rglob('*')) if path.is_file() and path != original_app]}))
        self.plan['components']['app_manifest'] = session.pinned(original_app)
        config = self.run / 'persistent_controllers.yaml'; config.write_text('original exact controller parameters')
        self.plan['generated_config'] = session.pinned(config)
        native_runtime = self.run / 'runtime-manifest.json'
        catalog = {str(old / relative): session.sha(old / relative) for relative in extra[:2]}
        runtime = {'schema': 'pupper_persistent_session_v1', 'skills': ['sit_stay_shake'],
                   'motion_mode': 'class_run', 'motion_controllers': self.plan['controllers'],
                   'configuration': str(config), 'catalog': {'sit_stay_shake': catalog},
                   'follower': {'mode': 'lab5', 'service_dir': str(old / 'follow_me_pupper/robot'),
                                'always_detect': False, 'use_color': False, 'avoid_obstacles': False},
                   'assets': {str(old / relative): session.sha(old / relative)
                              for relative in (*session.API_FIXED_FILES, *mutable[:-1], *extra)}}
        runtime['assets'][str(config)] = session.sha(config)
        session.write(native_runtime, runtime)
        self.plan['runtime_manifest'] = session.pinned(native_runtime)
        self.plan['environment'].update(PUPPER_SESSION_SKILLS='sit_stay_shake',
            PUPPER_SESSION_MOTION_MODE='class_run', PUPPER_FOLLOWER_MODE='lab5',
            FOLLOW_ME_DIR=str(old / 'follow_me_pupper/robot'), FOLLOW_ALWAYS_DETECT='false',
            FOLLOW_USE_COLOR='false', FOLLOW_AVOID_OBSTACLES='false',
            PUPPER_SESSION_MANIFEST=str(native_runtime), PUPPER_PROBE_SESSION_ID=self.plan['session_id'])
        session.write(self.run / 'selected-session.json', self.plan)
        manifest = new / 'app-stage-manifest.json'
        def refresh_manifest():
            entries = [{'path': str(path.relative_to(new)), 'sha256': session.sha(path)}
                       for path in sorted(new.rglob('*')) if path.is_file() and path != manifest]
            manifest.write_text(json.dumps({'destination_remote_path': str(new),
                'baseline_remote_path': str(old), 'final_selected_files': entries}))
            return session.sha(manifest)
        digest = refresh_manifest()
        return SimpleNamespace(old=old, new=new, manifest=manifest, digest=digest,
                               refresh=refresh_manifest, runtime=runtime)

    def test_explicit_api_attachment_preserves_native_and_uses_original_profile(self):
        fixture = self.attachment_fixture()
        plan_before = (self.run / 'selected-session.json').read_bytes()
        runtime_before = Path(self.plan['runtime_manifest']['path']).read_bytes()
        manager = deepcopy(self.life.receipt['processes']['manager'])
        native_launch = deepcopy(self.life.receipt['processes']['launch'])
        original_spawn = self.life.spawn
        environments = []
        def spawn(*args):
            environments.append(dict(os.environ)); original_spawn(*args)
        with patch.object(self.life, 'spawn', side_effect=spawn), patch.dict(os.environ, self.plan['environment']):
            result = session.start_api(self.run, self.plan, self.life,
                app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
            self.assertEqual(os.environ['PUPPER_SESSION_MANIFEST'], self.plan['runtime_manifest']['path'])
        self.assertTrue(result['api_ready']); self.assertFalse(result['motor_launch'])
        self.assertEqual([row[0] for row in self.life.spawned], ['server'])
        self.assertEqual(self.life.spawned[0][1], str(fixture.new / 'gemini_pupper/robot/server.py'))
        self.assertEqual(self.life.receipt['processes']['manager'], manager)
        self.assertEqual(self.life.receipt['processes']['launch'], native_launch)
        self.assertEqual(self.life.receipt['session_id'], self.plan['session_id'])
        self.assertEqual(environments[0]['FOLLOW_ME_DIR'], self.plan['environment']['FOLLOW_ME_DIR'])
        self.assertEqual(environments[0]['PUPPER_PROBE_SESSION_ID'], self.plan['session_id'])
        self.assertEqual((self.run / 'selected-session.json').read_bytes(), plan_before)
        self.assertEqual(Path(self.plan['runtime_manifest']['path']).read_bytes(), runtime_before)
        selection = result['api_application']
        document = session.verify_api_attachment(selection, self.plan)
        self.assertEqual(document['native_parent']['lifecycle'], self.plan['components']['lifecycle'])
        self.assertEqual(document['rollback_app_root'], str(fixture.old))
        derived = json.loads(Path(document['runtime_manifest']['path']).read_text())
        self.assertEqual(derived['configuration'], fixture.runtime['configuration'])
        self.assertEqual(derived['follower'], fixture.runtime['follower'])
        self.assertEqual(derived['motion_controllers'], fixture.runtime['motion_controllers'])
        self.assertTrue(set(fixture.runtime['assets']) <= set(derived['assets']))
        self.assertTrue(all(str(fixture.new) in path for path in derived['catalog']['sit_stay_shake']))
        # Exercise the unchanged real startup validator, rather than only assert
        # our dictionary layout: fresh catalog paths remain exact pinned bytes.
        spec = importlib.util.spec_from_file_location('attachment_runtime_validator',
            SOURCE_ROOT / 'gemini_pupper/robot/runtime_config.py')
        runtime_module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = runtime_module
        try:
            spec.loader.exec_module(runtime_module)
            trick = SimpleNamespace(path=fixture.new / 'gemini_pupper/tricks/sit_stay_shake/policy.json',
                walk_path=None, controller='trick_sit_stay_shake', walks=False)
            with patch.dict(os.environ, environments[0]):
                self.assertEqual(runtime_module.validate_session_catalog({'sit_stay_shake': trick}), derived)
        finally:
            sys.modules.pop(spec.name, None)

    def test_api_attachment_refuses_model_descriptor_contract_or_follower_changes(self):
        fixture = self.attachment_fixture()
        for relative in ('gemini_pupper/tricks/sit_stay_shake/policy.json',
                         'gemini_pupper/tricks/sit_stay_shake/trick.yaml',
                         'gemini_pupper/robot/tricks.py', 'gemini_pupper/robot/runtime_config.py',
                         'foundation/hardware/lab6-session.yaml',
                         'follow_me_pupper/robot/follow_service.py'):
            with self.subTest(relative=relative):
                path = fixture.new / relative; original = path.read_bytes()
                path.write_bytes(b'changed incompatible input')
                digest = fixture.refresh()  # Even a valid new manifest cannot authorize it.
                with self.assertRaises(ValueError):
                    session.start_api(self.run, self.plan, self.life,
                        app_manifest=fixture.manifest, app_manifest_sha256=digest)
                self.assertEqual(self.life.spawned, [])
                self.assertFalse((self.run / 'api-attachments').exists())
                self.assertEqual(self.life.receipt['status'], 'running')
                path.write_bytes(original)

    def test_api_attachment_rejects_alive_api_unknown_port_and_mapping_mismatch_before_writes(self):
        fixture = self.attachment_fixture()
        self.life.receipt['processes']['server'] = {'role': 'server'}
        with self.assertRaisesRegex(ValueError, 'API receipt exists'):
            session.start_api(self.run, self.plan, self.life,
                app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
        self.life.receipt['processes'].pop('server')
        self.life.receipt['pending']['server'] = {'gate': {'pid': 99}}
        with self.assertRaisesRegex(ValueError, 'API receipt exists'):
            session.start_api(self.run, self.plan, self.life,
                app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
        self.life.receipt['pending'].clear()
        self.port.side_effect = OSError('unknown listener')
        with self.assertRaises(OSError):
            session.start_api(self.run, self.plan, self.life,
                app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
        self.port.side_effect = None; self.maps.side_effect = ValueError('different native map')
        with self.assertRaisesRegex(ValueError, 'different native map'):
            session.start_api(self.run, self.plan, self.life,
                app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
        self.assertEqual(self.life.spawned, [])
        self.assertFalse((self.run / 'api-attachments').exists())

    def test_api_attachment_records_failure_without_native_cleanup_and_can_roll_back_explicitly(self):
        fixture = self.attachment_fixture()
        original_spawn = self.life.spawn
        def exited(*args):
            original_spawn(*args); self.life.states['server'] = 'exited'
        with patch.object(self.life, 'spawn', side_effect=exited):
            with self.assertRaisesRegex(ValueError, 'API exited before readiness'):
                session.start_api(self.run, self.plan, self.life,
                    app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
        selection = self.life.receipt['api_application']
        self.assertEqual(self.life.states['manager'], 'running')
        self.assertFalse(self.life.closed); self.assertEqual(self.life.sent, [])
        session.stop_api(self.run, self.plan, self.life)
        self.assertEqual(self.life.receipt['retired_servers'][-1]['api_application'], selection)
        self.assertNotIn('api_application', self.life.receipt)
        result = session.start_api(self.run, self.plan, self.life)  # original default is explicit rollback
        self.assertTrue(result['api_ready']); self.assertIsNone(result['api_application'])
        self.assertEqual(self.life.spawned[-1][1], str(fixture.old / 'gemini_pupper/robot/server.py'))
        self.assertEqual([row[0] for row in self.life.spawned], ['server', 'server'])
        self.assertEqual(self.life.states['manager'], 'running')

    def test_api_attachment_is_immutable_and_detects_modified_new_source(self):
        fixture = self.attachment_fixture()
        selection, _ = session.prepare_api_attachment(self.run, self.plan, fixture.manifest, fixture.digest)
        before = Path(selection['path']).read_bytes()
        repeated, _ = session.prepare_api_attachment(self.run, self.plan, fixture.manifest, fixture.digest)
        self.assertEqual(selection, repeated)
        self.assertEqual(Path(selection['path']).read_bytes(), before)
        (fixture.new / 'gemini_pupper/robot/server.py').write_text('unreviewed replacement')
        with self.assertRaises(ValueError): session.verify_api_attachment(selection, self.plan)
        self.assertEqual(self.life.spawned, [])

    def test_modified_attached_source_cannot_prevent_explicit_owned_api_stop(self):
        fixture = self.attachment_fixture()
        session.start_api(self.run, self.plan, self.life,
            app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
        selection = self.life.receipt['api_application']
        (fixture.new / 'gemini_pupper/robot/server.py').write_text('changed after launch')
        with self.assertRaises(ValueError): session.verify_api_attachment(selection, self.plan)
        stopped = session.stop_api(self.run, self.plan, self.life)
        self.assertTrue(stopped['api_exited']); self.assertTrue(stopped['native_session_retained'])
        self.assertEqual(self.life.sent, ['server'])
        self.assertEqual(self.life.states['manager'], 'running')
        self.assertEqual(self.life.receipt['retired_servers'][-1]['api_application'], selection)

    def test_app_selection_cannot_be_misread_as_native_start_override(self):
        with patch.object(sys, 'argv', ['persistent_session.py', 'start-native',
                '--run-dir', str(self.run), '--app-manifest', '/candidate/manifest.json',
                '--app-manifest-sha256', 'a' * 64]), patch.object(session, 'load') as load:
            with self.assertRaises(SystemExit) as caught:
                session.main()
        self.assertEqual(caught.exception.code, 2)
        load.assert_not_called(); self.assertEqual(self.life.spawned, [])

    def test_api_attachment_cannot_omit_loaded_application_source_from_new_pins(self):
        fixture = self.attachment_fixture()
        manifest = json.loads(fixture.manifest.read_text())
        manifest['final_selected_files'] = [row for row in manifest['final_selected_files']
            if row['path'] != 'gemini_pupper/robot/robot_api.py']
        fixture.manifest.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'omits preserved source'):
            session.start_api(self.run, self.plan, self.life,
                app_manifest=fixture.manifest, app_manifest_sha256=session.sha(fixture.manifest))
        self.assertEqual(self.life.spawned, [])
        self.assertFalse((self.run / 'api-attachments').exists())

    def test_unconfirmed_api_spawn_retains_attachment_and_refuses_guessed_rollback(self):
        fixture = self.attachment_fixture()
        def pending(*_args):
            self.life.receipt['pending']['server'] = {'gate': {'pid': 99}}
            raise ValueError('spawn identity not confirmed')
        with patch.object(self.life, 'spawn', side_effect=pending):
            with self.assertRaisesRegex(ValueError, 'spawn identity not confirmed'):
                session.start_api(self.run, self.plan, self.life,
                    app_manifest=fixture.manifest, app_manifest_sha256=fixture.digest)
        self.assertIn('api_application', self.life.receipt)
        self.assertIn('server', self.life.receipt['pending'])
        with self.assertRaisesRegex(ValueError, 'No confirmed API identity'):
            session.stop_api(self.run, self.plan, self.life)
        self.assertEqual(self.life.sent, [])
        self.assertEqual(self.life.states['manager'], 'running')
        self.assertFalse(self.life.closed)

    def test_same_python_binary_with_different_virtualenv_is_refused(self):
        first = self.run / 'venv-a/bin/python'; second = self.run / 'venv-b/bin/python'
        first.parent.mkdir(parents=True); second.parent.mkdir(parents=True)
        first.symlink_to(Path(sys.executable).resolve()); second.symlink_to(Path(sys.executable).resolve())
        self.assertEqual(first.resolve(), second.resolve())
        with patch.object(sys, 'executable', str(first)), patch.object(sys, 'prefix', str(first.parent.parent)):
            plan = {'interpreter': session.interpreter_identity()}
            session.check_interpreter(plan)
        with patch.object(sys, 'executable', str(second)), patch.object(sys, 'prefix', str(second.parent.parent)):
            with self.assertRaisesRegex(ValueError, 'virtualenv prefix'):
                session.check_interpreter(plan)
        with patch.object(sys, 'executable', str(first)), patch.object(sys, 'prefix', str(second.parent.parent)):
            with self.assertRaisesRegex(ValueError, 'virtualenv prefix'):
                session.check_interpreter(plan)

    def test_api_start_never_starts_native_or_camera_and_reuses_manager(self):
        result = session.start_api(self.run, self.plan, self.life)
        self.assertTrue(result['api_ready'])
        self.assertEqual([r[0] for r in self.life.spawned], ['server'])
        self.assertEqual(self.life.receipt['processes']['manager']['pid'], 20)
        self.assertFalse(result['motor_launch'])

    def test_existing_api_or_unknown_port_refuses_before_any_spawn(self):
        self.life.receipt['processes']['server'] = {'role': 'server'}
        with self.assertRaisesRegex(ValueError, 'API receipt exists'):
            session.start_api(self.run, self.plan, self.life)
        self.life.receipt['processes'].pop('server')
        self.port.side_effect = OSError('bound')
        with self.assertRaises(OSError): session.start_api(self.run, self.plan, self.life)
        self.assertEqual(self.life.spawned, [])

    def test_missing_native_identity_or_wrong_readiness_refuses_api(self):
        self.life.states['manager'] = 'exited'
        with self.assertRaisesRegex(ValueError, 'not ready'):
            session.start_api(self.run, self.plan, self.life)
        self.assertEqual(self.life.spawned, [])

    def test_api_stop_then_explicit_restart_retains_native_and_old_receipt(self):
        session.start_api(self.run, self.plan, self.life)
        old = self.life.receipt['processes']['server']
        result = session.stop_api(self.run, self.plan, self.life)
        self.assertTrue(result['native_session_retained'])
        self.assertEqual(self.life.sent, ['server'])
        self.assertEqual(self.life.receipt['retired_servers'][0]['target'], old)
        self.assertTrue(self.life.receipt['retired_servers'][0]['events'])
        session.start_api(self.run, self.plan, self.life)
        self.assertEqual([r[0] for r in self.life.spawned], ['server', 'server'])
        self.assertEqual(self.life.states['manager'], 'running')

    def test_wrong_api_port_never_receives_http_but_owned_ingress_stops(self):
        session.start_api(self.run, self.plan, self.life); self.network.reset_mock()
        self.owned.side_effect = ValueError('foreign port')
        with self.assertRaisesRegex(ValueError, 'acknowledgment unavailable'):
            session.stop_api(self.run, self.plan, self.life)
        self.network.assert_not_called(); self.assertEqual(self.life.sent, ['server'])
        self.assertEqual(self.life.states['manager'], 'running')

    def test_failed_stop_ack_is_not_success(self):
        session.start_api(self.run, self.plan, self.life)
        self.network.return_value = {'success': False}
        with self.assertRaisesRegex(ValueError, 'not acknowledged'):
            session.stop_api(self.run, self.plan, self.life)
        self.assertEqual(self.life.sent, ['server'])

    def test_homing_then_delayed_plugin_and_spawner_completion_share_one_deadline(self):
        self.life.native_log = 'Finished homing!\n'
        clock = SimpleNamespace(now=0., waits=0)
        self.maps.side_effect = [False, True, True, True]
        def sleep(seconds):
            self.assertEqual(self.life.receipt['status'], 'starting')
            self.assertFalse((self.run / 'native-readiness.json').exists())
            clock.now += seconds; clock.waits += 1
            if clock.waits == 1:
                # Plugin appears, but only the first controller has loaded.
                log = 'Finished homing!\n[spawner-7] [INFO] [spawner_motion]: Loaded neural_controller_default\n'
            elif clock.waits == 2:
                # All Loaded lines still do not prove final configuration completed.
                log = 'Finished homing!\n' + READY_SPAWNERS.rsplit('[INFO] [spawner-7]', 1)[0]
            else:
                log = 'Finished homing!\n' + READY_SPAWNERS
            (self.run / 'robot.log').write_text(log)
        (self.run / 'native-readiness.json').unlink()  # Normally absent before first startup.
        with patch.object(session, 'time', SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep)):
            result = session.start_native(self.run, self.plan, self.life)
        self.assertEqual(clock.waits, 3)
        self.assertEqual(self.maps.call_count, 4)
        self.assertTrue(result['controller_spawners_finished'])
        self.assertEqual([r[0] for r in self.life.spawned], ['camera', 'recorder', 'launch'])
        self.assertEqual(self.life.receipt['status'], 'running')

    def test_native_start_preserves_one_camera_recorder_and_current_launch(self):
        result = session.start_native(self.run, self.plan, self.life)
        self.assertTrue(result['homing_finished'])
        self.assertEqual([r[0] for r in self.life.spawned], ['camera', 'recorder', 'launch'])
        self.assertEqual(self.life.spawned[0][2].count('camera_node'), 1)
        self.assertEqual(self.life.spawned[2][2], ['ros2', 'launch', '/selected/lab6_session.launch.py'])
        self.maps.assert_called_once()

    def test_stale_homing_log_cannot_authorize_new_start(self):
        (self.run / 'robot.log').write_text('Finished homing!')
        with self.assertRaisesRegex(ValueError, 'stale readiness'):
            session.start_native(self.run, self.plan, self.life)
        self.assertEqual(self.life.spawned, [])

    def test_unrelated_system_exe_permission_does_not_block_cmdline_inventory(self):
        system = self.life.PROC / '1'; system.mkdir()
        (system / 'cmdline').write_bytes(b'/sbin/init\0')
        original = Path.resolve
        def guarded(path, *args, **kwargs):
            if path.name == 'exe': raise PermissionError('ptrace denied')
            return original(path, *args, **kwargs)
        with patch.object(Path, 'resolve', guarded):
            result = session.start_native(self.run, self.plan, self.life)
        self.assertTrue(result['homing_finished'])

    def test_global_cmdline_controller_and_camera_owners_block_before_spawn(self):
        proc = self.life.PROC / '42'; proc.mkdir()
        for executable in ('/opt/ros/lib/ros2_control_node', '/opt/ros/lib/camera_node'):
            with self.subTest(executable=executable):
                (proc / 'cmdline').write_bytes(executable.encode() + b'\0--ros-args\0')
                with self.assertRaisesRegex(ValueError, 'Existing controller/camera owner'):
                    session.start_native(self.run, self.plan, self.life)
        self.assertEqual(self.life.spawned, [])

    def test_cmdline_permission_denial_is_not_silently_ignored(self):
        proc = self.life.PROC / '43'; proc.mkdir()
        original = Path.read_bytes
        def denied(path):
            if path == proc / 'cmdline': raise PermissionError('owned process unreadable')
            return original(path)
        with patch.object(Path, 'read_bytes', denied), patch.object(self.life, 'save') as save:
            with self.assertRaises(PermissionError):
                session.start_native(self.run, self.plan, self.life)
        save.assert_not_called(); self.assertEqual(self.life.spawned, [])

    def test_disappeared_pid_is_ignored_before_start(self):
        (self.life.PROC / '44').mkdir()  # vanished cmdline during the scan
        result = session.start_native(self.run, self.plan, self.life)
        self.assertTrue(result['homing_finished'])

    def test_native_unknown_port_refuses_before_camera_or_motion(self):
        self.port.side_effect = OSError('unknown owner')
        with self.assertRaises(OSError): session.start_native(self.run, self.plan, self.life)
        self.assertEqual(self.life.spawned, [])

    def test_native_failure_is_failure_keeps_receipt_and_never_starts_api(self):
        self.life.native_log = 'IMU startup check failed: unsettled\n'
        with self.assertRaisesRegex(ValueError, 'startup failed'):
            session.start_native(self.run, self.plan, self.life)
        self.assertEqual(self.life.receipt['status'], 'starting')
        self.assertNotIn('server', [r[0] for r in self.life.spawned])
        self.assertEqual(self.life.sent, [])

    def test_receipt_from_other_session_refused(self):
        self.life.receipt['session_id'] = 'b' * 32
        with self.assertRaisesRegex(ValueError, 'does not match'):
            session.start_api(self.run, self.plan, self.life)
        self.assertEqual(self.life.spawned, [])

    def test_shutdown_delegates_existing_owned_cleanup_only(self):
        with patch.object(session, 'load', return_value=(self.plan, self.life)), \
             patch.object(sys, 'argv', ['persistent_session.py', 'shutdown', '--run-dir', str(self.run)]):
            session.main()
        self.assertTrue(self.life.closed); self.assertEqual(self.life.spawned, [])
        self.network.assert_not_called()

    def test_prepared_status_does_not_spawn_or_contact_api(self):
        self.plan['rollback_app'] = '/rollback'
        with patch.object(session, 'load', return_value=(self.plan, self.life)), \
             patch.object(sys, 'argv', ['persistent_session.py', 'status', '--run-dir', str(self.run)]):
            session.main()
        self.assertEqual(self.life.spawned, []); self.network.assert_not_called()

    def test_prepare_is_nonactuating_and_preserves_follower_selection(self):
        app = self.run / 'release'; app.mkdir()
        lifecycle = app / 'gemini_pupper/robot/session_processes.py'
        lifecycle.parent.mkdir(parents=True); lifecycle.write_text('# fixture')
        manifest = self.run / 'app-manifest.json'
        manifest.write_text(json.dumps({'destination_remote_path': str(app),
            'baseline_remote_path': '/preserved/rollback', 'final_selected_files': [
                {'path': str(lifecycle.relative_to(app)), 'sha256': session.sha(lifecycle)}]}))
        for name in ('native.so', 'imu.so', 'launch.py', 'lab6-session.yaml'):
            (self.run / name).write_text(name)
        (self.run / 'floor_stock').mkdir()
        (self.run / 'floor_stock/baseline-expanded.urdf').write_text('fixture urdf')
        args = SimpleNamespace(app_manifest=manifest, app_manifest_sha256=session.sha(manifest),
            native_library=self.run / 'native.so', native_sha256=session.sha(self.run / 'native.so'),
            imu_library=self.run / 'imu.so', imu_sha256=session.sha(self.run / 'imu.so'),
            launch=self.run / 'launch.py', config=self.run / 'lab6-session.yaml', urdf=self.run / 'floor_stock/baseline-expanded.urdf',
            run_dir=self.run / 'prepared', port=18090, telemetry_topics=self.run / 'topics.json')
        def prepare_profile(config, urdf):
            (args.run_dir / 'runtime-manifest.json').write_text('{}')
            output = args.run_dir / 'controllers.yaml'; output.write_text('controllers: []')
            return output, ['fixture_controller']
        with patch.dict(os.environ, {'PUPPER_SESSION_SKILLS': '', 'ROS_DOMAIN_ID': '168', 'PUPPER_FOLLOWER_MODE': 'lab5', 'FOLLOW_ME_DIR': '/reviewed/follower', 'FOLLOW_ALWAYS_DETECT': 'true', 'ROS_LOCALHOST_ONLY': '1',
             'PUPPER_ONBOARD_BOOTSTRAP_DIR': '/run/user/1000/onboard',
             'GEMINI_KEY_FILE': '/private/operator/api_key'}), \
             patch.object(session, 'module', return_value=SimpleNamespace(prepare_session_profile=prepare_profile)), \
             patch.object(session.subprocess, 'Popen') as spawn:
            result = session.prepare(args)
        spawn.assert_not_called(); self.assertFalse(result['motor_launch'])
        saved = json.loads((args.run_dir / 'selected-session.json').read_text())
        self.assertEqual(saved['environment']['PUPPER_FOLLOWER_MODE'], 'lab5')
        self.assertEqual(saved['environment']['FOLLOW_ME_DIR'], '/reviewed/follower')
        self.assertEqual(saved['environment']['PUPPER_SESSION_MOTION_MODE'], 'walking')
        self.assertEqual(saved['environment']['FOLLOW_ALWAYS_DETECT'], 'true')
        self.assertEqual(saved['environment']['ROS_LOCALHOST_ONLY'], '1')
        self.assertEqual(saved['rollback_app'], '/preserved/rollback')
        self.assertEqual(saved['environment']['PUPPER_ONBOARD_BOOTSTRAP_DIR'], '/run/user/1000/onboard')
        self.assertEqual(saved['environment']['GEMINI_KEY_FILE'], '/private/operator/api_key')
        with patch.dict(os.environ, {'PUPPER_ONBOARD_BOOTSTRAP_DIR': '/wrong/dir', 'GEMINI_KEY_FILE': '/wrong/key'}), \
             patch.object(session, 'pinned'), patch.object(session, 'module', return_value=self.life):
            session.load(args.run_dir, verify=False)
            self.assertEqual(os.environ['PUPPER_ONBOARD_BOOTSTRAP_DIR'], '/run/user/1000/onboard')
            self.assertEqual(os.environ['GEMINI_KEY_FILE'], '/private/operator/api_key')




class SelectedMappingTests(unittest.TestCase):
    def test_only_absent_library_is_pending_wrong_or_deleted_library_fails(self):
        plan = {'components': {'native': {'path': '/selected/libnative.so'},
                               'imu': {'path': '/selected/libimu.so'}}}
        def line(path): return '1-2 r-xp 0 00:00 1 ' + path + '\n'
        with patch.object(Path, 'read_text', return_value=line('/selected/libimu.so')):
            self.assertFalse(session.check_loaded(plan, {'pid': 1}, allow_pending=True))
            with self.assertRaisesRegex(ValueError, 'not mapped'):
                session.check_loaded(plan, {'pid': 1})
        for path in ('/wrong/libnative.so', '/selected/libnative.so (deleted)'):
            with self.subTest(path=path), patch.object(Path, 'read_text', return_value=line(path)):
                with self.assertRaisesRegex(ValueError, 'Unexpected or deleted'):
                    session.check_loaded(plan, {'pid': 1}, allow_pending=True)
        with patch.object(Path, 'read_text', side_effect=PermissionError('maps unavailable')):
            with self.assertRaises(PermissionError):
                session.check_loaded(plan, {'pid': 1}, allow_pending=True)

if __name__ == '__main__': unittest.main()
