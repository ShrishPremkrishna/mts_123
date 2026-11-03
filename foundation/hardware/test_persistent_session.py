"""Local fixtures only: no ROS, network requests or process signals."""
from contextlib import nullcontext
from copy import deepcopy
import importlib.util
import json
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
        with patch.dict(os.environ, {'PUPPER_SESSION_SKILLS': '', 'ROS_DOMAIN_ID': '168', 'PUPPER_FOLLOWER_MODE': 'lab5', 'FOLLOW_ME_DIR': '/reviewed/follower', 'FOLLOW_ALWAYS_DETECT': 'true', 'ROS_LOCALHOST_ONLY': '1'}), \
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
