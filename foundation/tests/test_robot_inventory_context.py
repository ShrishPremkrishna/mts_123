"""Exercise the shell inventory's ROS path with an uninitialized default context."""
import json
import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/robot_inventory.sh'

# rclpy's global spin helpers use its default executor unless one is provided.
# This stub keeps that default context uninitialized, as the real inventory does,
# and only delivers replies/samples through an executor using the custom context.
ROS_STUB = r'''
import atexit, json, os, sys, time, types
from pathlib import Path
trace = []
atexit.register(lambda: Path(os.environ['INVENTORY_ROS_TRACE']).write_text(json.dumps(trace)))
def module(name, **values):
    value = types.ModuleType(name)
    value.__dict__.update(values)
    sys.modules[name] = value
    return value
class Context:
    def __init__(self): self.initialized = False; trace.append('context_created')
    def ok(self): return self.initialized
    def shutdown(self): self.initialized = False; trace.append('context_shutdown')
def init(*, args, context):
    assert isinstance(context, Context)
    context.initialized = True
    trace.append('custom_context_initialized')
def default_spin(*args, **kwargs):
    raise RuntimeError('default context is uninitialized; explicit-context executor required')
class Request:
    def __init__(self, **values): self.__dict__.update(values)
class Service: pass
Service.Request = Request
class Future:
    def __init__(self, reply): self.reply = reply; self.ready = False
    def done(self): return self.ready
    def result(self): assert self.ready; return self.reply
class Client:
    def __init__(self, path): self.path = path
    def wait_for_service(self, *, timeout_sec): return True
    def call_async(self, request):
        trace.append('read_service:' + self.path)
        if self.path.endswith('/get_parameters'):
            assert request.names == ['robot_description']
            return Future(types.SimpleNamespace(values=[types.SimpleNamespace(type=4, string_value='<robot/>')]))
        assert self.path in ['/controller_manager/list_controllers', '/controller_manager/list_hardware_interfaces']
        return Future(types.SimpleNamespace())
class Node:
    def __init__(self, context): self.context = context; self.subscriptions = []
    def get_node_names_and_namespaces(self):
        if os.environ.get('INVENTORY_ROS_FAIL_GRAPH') == '1': raise RuntimeError('graph fixture failure')
        return [('fixture', '/')]
    def get_topic_names_and_types(self): return []
    def get_service_names_and_types(self): return []
    def create_client(self, kind, path): return Client(path)
    def create_subscription(self, kind, topic, receive, qos):
        self.subscriptions.append((topic, receive)); return object()
    def destroy_node(self): trace.append('node_destroyed')
def create_node(name, *, context, enable_rosout):
    assert context.initialized and enable_rosout is False
    trace.append('node_created'); return Node(context)
class Executor:
    def __init__(self, *, context):
        assert context.initialized
        self.context = context; self.node = None; trace.append('executor_custom_context')
    def add_node(self, node):
        assert node.context is self.context
        self.node = node; trace.append('node_added')
    def spin_until_future_complete(self, future, *, timeout_sec):
        assert self.node is not None and self.context.ok()
        future.ready = True; trace.append('executor_future')
    def spin_once(self, *, timeout_sec):
        assert self.node is not None and self.context.ok()
        stamp = types.SimpleNamespace(sec=1, nanosec=0)
        for topic, receive in self.node.subscriptions:
            if topic == '/joint_states':
                receive(types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp), name=['leg'], position=[.1], velocity=[.2], effort=[.3]))
            elif topic == '/camera/image_raw/compressed':
                receive(types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp), format='jpeg', data=b'private_pixels'))
        trace.append('executor_sample'); time.sleep(timeout_sec)
    def remove_node(self, node):
        assert self.node is node
        self.node = None; trace.append('node_removed')
    def shutdown(self, *, timeout_sec): trace.append('executor_shutdown'); return True
module('rclpy', init=init, create_node=create_node, spin_once=default_spin, spin_until_future_complete=default_spin)
module('rclpy.context', Context=Context)
module('rclpy.executors', SingleThreadedExecutor=Executor)
module('rclpy.qos', qos_profile_sensor_data=object())
for package, child, names in [('sensor_msgs', 'msg', ['JointState','Imu','CompressedImage']), ('geometry_msgs', 'msg', ['Twist']), ('controller_manager_msgs', 'srv', ['ListControllers','ListHardwareInterfaces']), ('rcl_interfaces', 'srv', ['GetParameters'])]:
    module(package); module(package+'.'+child, **{name:Service for name in names})
module('rosidl_runtime_py'); module('rosidl_runtime_py.convert', message_to_ordereddict=lambda msg: {'fixture_read': True})
'''


@pytest.mark.parametrize('fail_graph', [False, True])
def test_ros_inventory_uses_custom_executor_and_cleans_up(tmp_path, fail_graph):
    stub = tmp_path / 'stub'; stub.mkdir()
    (stub / 'sitecustomize.py').write_text(ROS_STUB)
    empty = tmp_path / 'empty'; empty.mkdir()
    trace_path = tmp_path / 'trace.json'
    env = dict(os.environ, PYTHONPATH=str(stub), PYTHONDONTWRITEBYTECODE='1',
               INVENTORY_ROS_TRACE=str(trace_path), INVENTORY_ROS_FAIL_GRAPH=str(int(fail_graph)))
    args = ['bash', str(SCRIPT), '--repo-root', str(empty), '--ros', '--sample-seconds', '.01']
    for component in ('monorepo', 'gemini', 'follower', 'deployment'):
        args.extend(['--' + component + '-dir', str(empty)])
    result = subprocess.run(args, capture_output=True, text=True, env=env, timeout=15, check=True)
    report = json.loads(result.stdout); trace = json.loads(trace_path.read_text())
    if fail_graph:
        assert report['ros']['error'] == 'RuntimeError: graph fixture failure'
    else:
        assert 'error' not in report['ros']
        assert trace.count('executor_future') == 4
        assert report['ros']['samples']['/joint_states']['last']['positions'] == [.1]
        assert report['ros']['samples']['/camera/image_raw/compressed']['last'] == {'format':'jpeg','encoded_bytes':14}
        assert 'private_pixels' not in result.stdout
    assert trace[:4] == ['context_created', 'custom_context_initialized', 'executor_custom_context', 'node_created']
    assert trace[-4:] == ['node_removed', 'executor_shutdown', 'node_destroyed', 'context_shutdown']
