"""The fake server establishes only stub ROS before importing the actual app."""
import json
from pathlib import Path
import subprocess
import sys


ROBOT_DIR = Path(__file__).resolve().parents[1] / "robot"


def test_fresh_process_fake_entrypoint_ignores_external_bind_and_never_imports_camera():
    code = r'''
import json, os, socket, sys
os.environ['HOST'] = '0.0.0.0'
sys.modules['cv2'] = None
sys.modules['fisheye_converter'] = None

def forbidden(*args, **kwargs):
    raise AssertionError('fake entrypoint tried network or subprocess access')
socket.socket.connect = forbidden
socket.socket.bind = forbidden
import subprocess
subprocess.Popen = forbidden
subprocess.run = forbidden
import uvicorn

def check(app, **kwargs):
    import server, test_tools, robot_api
    assert app is server.app and server.FAKE_MODE
    assert robot_api.Node is test_tools._Node
    assert kwargs == {'host': '127.0.0.1', 'port': 8123}
    assert 'cv2' in sys.modules and sys.modules['cv2'] is None
    print(json.dumps(server.health()))
uvicorn.run = check
import fake_server
fake_server.main(['--port', '8123'])
'''
    result = subprocess.run([sys.executable, "-c", code], cwd=ROBOT_DIR,
                            text=True, capture_output=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == {"status": "running", "tools": False,
                                       "runtime": "fake", "hardware_connected": False,
                                       "command_epoch": None, "command_epoch_required": True}


def test_fake_entrypoint_refuses_existing_runtime_instead_of_overwriting_it():
    code = '''
import sys, types
sys.modules['rclpy'] = types.ModuleType('rclpy')
import fake_server
try:
    fake_server.main([])
except RuntimeError as error:
    assert 'fresh process' in str(error)
else:
    raise AssertionError('existing ROS binding was silently replaced')
assert 'test_tools' not in sys.modules and 'server' not in sys.modules
'''
    result = subprocess.run([sys.executable, "-c", code], cwd=ROBOT_DIR,
                            text=True, capture_output=True, timeout=15)
    assert result.returncode == 0, result.stderr
