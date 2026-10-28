#!/usr/bin/env bash
# Read-only inventory. No launch, publish, controller switch, configuration edit,
# credential read, camera-image output, or network download. JSON goes to stdout.
set -eu
inventory_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec python3 - "$inventory_repo_root" "$@" <<'PY'
import argparse
import datetime
import hashlib
import importlib.metadata
import io
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

parser = argparse.ArgumentParser(description='Read-only Pupper inventory; --ros observes the already running local ROS graph.')
parser.add_argument('repo', type=Path)
parser.add_argument('--repo-root', type=Path, help='explicit discovered working checkout to inspect; defaults to this script checkout')
for name in ('monorepo', 'gemini', 'follower', 'deployment'):
    parser.add_argument('--' + name + '-dir', type=Path, help='explicit discovered component directory; fallback candidates are labeled')
parser.add_argument('--ros', action='store_true', help='sample existing ROS topics and read controller state; never launch or command')
parser.add_argument('--sample-seconds', type=float, default=3.0)
parser.add_argument('--contact-export', type=Path, help='Optional Lab 1 exported model directory to hash; model is never executed')
args = parser.parse_args()
sys.path.insert(0, str(args.repo / 'foundation/scripts'))
from inspect_robot_description import inspect_description
from robot_artifact_backup import fingerprint_file
tool_repo = args.repo
if args.repo_root:
    args.repo = args.repo_root.expanduser().absolute()
if not 0 < args.sample_seconds <= 15:
    parser.error('--sample-seconds must be more than zero and at most 15')

def read(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, UnicodeError):
        return None

def command(argv, timeout=8):
    if not shutil.which(argv[0]):
        return {'available': False}
    try:
        result = subprocess.run(argv, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        return {'returncode': result.returncode, 'stdout': result.stdout[:16000].strip(),
                'stderr': result.stderr[:2000].strip(), 'truncated': len(result.stdout) > 16000}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'error': type(exc).__name__}

component_sources = {}
def component(label, name, variable=None):
    explicit = getattr(args, label + '_dir', None)
    if explicit is not None:
        candidate, origin = explicit.expanduser().absolute(), 'explicit argument'
    elif variable and os.environ.get(variable):
        candidate, origin = Path(os.environ[variable]).expanduser().absolute(), 'explicit environment ' + variable
    elif (args.repo / name).is_dir():
        candidate, origin = args.repo / name, 'unverified in-checkout candidate'
    else:
        candidate, origin = Path.home() / name, 'unverified home-directory candidate'
    component_sources[label] = origin
    return candidate

monorepo = component('monorepo', 'pupperv3-monorepo', 'PUPPER_MONOREPO_DIR')
gemini = component('gemini', 'gemini_pupper')
follower = component('follower', 'follow_me_pupper', 'FOLLOW_ME_DIR')
deploy = component('deployment', 'pupper_gait_deploy', 'PUPPER_GAIT_DEPLOY_DIR')
report = {
    'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'scope': 'Local read-only inventory of explicit paths or labeled candidates; paths are not proof of the active installation. No hardware capability or safety approval is implied.',
    'inventory_tool_root': str(tool_repo),
    'host': {'system': platform.system(), 'release': platform.release(), 'machine': platform.machine(),
             'python': platform.python_version(), 'model': (read('/proc/device-tree/model') or '').rstrip('\0'),
             'uptime': read('/proc/uptime'), 'loadavg': read('/proc/loadavg'),
             'thermal_zones': {str(p.parent): read(p) for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')}},
    'versions': {}, 'repos': {}, 'components': {}, 'file_hashes': [],
    'hardware': {'hailo_devices': sorted(str(p) for p in Path('/dev').glob('hailo*')),
                 'video_devices': sorted(str(p) for p in Path('/dev').glob('video*')),
                 'throttling': command(['vcgencmd', 'get_throttled']),
                 'temperature': command(['vcgencmd', 'measure_temp']),
                 'hailort': command(['hailortcli', '--version']),
                 'battery': 'not sampled; record physical/current approved battery readout separately'},
    'runtime_choices': {name: os.environ.get(name) for name in (
        'ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
        'PUPPER_FOLLOWER_MODE', 'FOLLOW_ALWAYS_DETECT', 'FOLLOW_USE_COLOR', 'FOLLOW_AVOID_OBSTACLES')},
}
for package in ('numpy', 'torch', 'mujoco', 'onnxruntime', 'hailort', 'rclpy', 'fastapi', 'google-genai'):
    try:
        report['versions'][package] = importlib.metadata.version(package)
    except importlib.metadata.PackageNotFoundError:
        report['versions'][package] = None
for name, directory in {'foundation': args.repo, 'monorepo': monorepo, 'gemini': gemini,
                        'follower': follower, 'deployment': deploy}.items():
    report['components'][name] = {'path': str(directory), 'exists': directory.is_dir(),
        'path_source': component_sources.get(name, 'explicit repo-root argument' if args.repo_root else 'script checkout'),
        'active_installation_verified': False}
    if directory.is_dir():
        report['repos'][name] = {'commit': command(['git', '-C', str(directory), 'rev-parse', 'HEAD']),
                                 'branch': command(['git', '-C', str(directory), 'branch', '--show-current']),
                                 'changes': command(['git', '-C', str(directory), 'status', '--porcelain'])}

report['components']['ros_install'] = {'path': str(monorepo / 'ros2_ws/install/local_setup.bash'),
                                      'exists': (monorepo / 'ros2_ws/install/local_setup.bash').is_file()}
report['components']['web_build'] = {'exists': (gemini / 'web/dist/index.html').is_file()}
report['components']['hailo_model'] = {'paths': [str(p) for p in (follower / 'robot/yolov8n_seg.hef', follower / 'yolov8n_seg.hef') if p.is_file()]}
report['components']['new_lab6_trained_policies'] = {
    name: (gemini / 'tricks' / name / 'policy.json').is_file() for name in ('sit_stay_shake', 'spider_walk')}
report['driver_descriptions'] = {}

# Hash allowlisted policy/configuration areas. Never read credentials, .env, logs,
# shell history, remote URLs, API responses or arbitrary configuration directories.
# Symlinks (including ancestor links), special files and common credential payloads
# are refused. Explicitly select reviewed real targets of symlink-install files.
areas = [(gemini / 'tricks', {'.json', '.yaml', '.npz'}),
         (gemini / 'robot/launch', {'.yaml', '.py'}),
         (monorepo / 'ros2_ws/src/neural_controller/policies', {'.json', '.yaml'}),
         (monorepo / 'ros2_ws/src/neural_controller/launch', {'.json', '.yaml', '.py'}),
         (monorepo / 'ros2_ws/install/neural_controller/share/neural_controller/launch', {'.json', '.yaml', '.py'}),
         (monorepo / 'ros2_ws/install/neural_controller/lib', {'.so'}),
         (deploy / 'policies', {'.json', '.yaml'})]
areas += [(monorepo / 'ros2_ws/src/pupper_v3_description/description', {'.xacro', '.urdf'}),
          (monorepo / 'ros2_ws/install/pupper_v3_description/share/pupper_v3_description/description', {'.xacro', '.urdf'}),
          (monorepo / 'ros2_ws/install/control_board_hardware_interface/lib', {'.so'})]
files = {p for directory, suffixes in areas if directory.is_dir()
         for p in directory.rglob('*') if p.is_file() and p.suffix in suffixes}
for relative in ('src/control_board_hardware_interface.cpp', 'src/rt/rt_spi.cpp'):
    p = monorepo / 'ros2_ws/src/control_board_hardware_interface' / relative
    if p.is_file(): files.add(p)
for p in (gemini / 'robot/camera_params.yaml', follower / 'robot/camera_params.yaml',
          args.repo / 'ik_heuristic_walking_lab/ik.yaml', args.repo / 'ik_heuristic_walking_lab/walking.yaml',
          args.repo / 'pd_control_lab/pd_control.yaml'):
    if p.is_file(): files.add(p)
for p in (follower / 'robot/yolov8n_seg.hef', follower / 'yolov8n_seg.hef'):
    if p.is_file(): files.add(p)
if args.contact_export:
    report['components']['contact_export'] = {'path': str(args.contact_export),
        'model_exists': (args.contact_export / 'contact.pt').is_file(),
        'metadata_exists': (args.contact_export / 'metadata.json').is_file()}
    for name in ('contact.pt', 'metadata.json'):
        p = args.contact_export / name
        if p.is_file(): files.add(p)
for p in sorted(files):
    try:
        description = io.BytesIO() if p.suffix in ('.xacro', '.urdf') else None
        captured = fingerprint_file(p.absolute(), destination=description)
        report['file_hashes'].append({'path': str(p), 'bytes': captured['bytes'], 'sha256': captured['sha256']})
        if description is not None:
            parsed = inspect_description(description.getvalue().decode('utf-8'))
            if parsed['driver_joints'] or parsed['errors']:
                report['driver_descriptions'][str(p)] = parsed
    except (OSError, ValueError, UnicodeError) as exc:
        report['file_hashes'].append({'path': str(p), 'error': type(exc).__name__, 'reason': str(exc)})

if args.ros:
    ros = report['ros'] = {'note': 'Observes existing graph only; no launch, command publisher or parameter writes.'}
    node = context = None
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState, Imu, CompressedImage
        from geometry_msgs.msg import Twist
        from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
        from rcl_interfaces.srv import GetParameters
        from rosidl_runtime_py.convert import message_to_ordereddict
        context = Context()
        rclpy.init(args=[], context=context)
        node = rclpy.create_node('foundation_readonly_inventory', context=context, enable_rosout=False)
        time.sleep(0.25)
        ros['nodes'] = node.get_node_names_and_namespaces()
        ros['topics'] = node.get_topic_names_and_types()
        ros['services'] = node.get_service_names_and_types()
        ros['robot_descriptions'] = {}
        # Only this named parameter is read. Do not enumerate arbitrary parameters.
        for owner in ('/controller_manager', '/robot_state_publisher'):
            client = node.create_client(GetParameters, owner + '/get_parameters')
            if not client.wait_for_service(timeout_sec=0.5):
                ros['robot_descriptions'][owner] = {'error': 'parameter service unavailable'}
                continue
            future = client.call_async(GetParameters.Request(names=['robot_description']))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            response = future.result() if future.done() else None
            if response and len(response.values) == 1 and response.values[0].type == 4:
                ros['robot_descriptions'][owner] = inspect_description(response.values[0].string_value)
            else:
                ros['robot_descriptions'][owner] = {'error': 'robot_description unavailable as string'}
        for suffix, service_type in [('list_controllers', ListControllers), ('list_hardware_interfaces', ListHardwareInterfaces)]:
            client = node.create_client(service_type, '/controller_manager/' + suffix)
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(service_type.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                ros[suffix] = message_to_ordereddict(future.result()) if future.done() and future.result() is not None else {'error': 'no response'}
            else:
                ros[suffix] = {'error': 'service unavailable'}
        samples, subscriptions = {}, []
        def receive(topic, kind, msg):
            entry = samples.setdefault(topic, {'receipt_times': [], 'header_stamps': [], 'last': {}})
            entry['receipt_times'].append(time.monotonic())
            if hasattr(msg, 'header'):
                entry['header_stamps'].append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
            if kind == 'camera':
                entry['last'] = {'format': msg.format, 'encoded_bytes': len(msg.data)}
            elif kind == 'joints':
                entry['last'] = {'names': list(msg.name), 'positions': list(msg.position),
                                 'velocities': list(msg.velocity), 'efforts': list(msg.effort)}
            else:
                entry['last'] = message_to_ordereddict(msg)
        topics = [('/joint_states', JointState, 'joints'),
                  ('/imu_sensor_broadcaster/imu', Imu, 'imu'),
                  ('/camera/image_raw/compressed', CompressedImage, 'camera'),
                  ('/cmd_vel', Twist, 'command'), ('/teleop_cmd_vel', Twist, 'command'),
                  ('/llm_cmd_vel', Twist, 'command')]
        for topic, message_type, kind in topics:
            samples[topic] = {'receipt_times': [], 'header_stamps': [], 'last': {}}
            subscriptions.append(node.create_subscription(message_type, topic,
                lambda msg, topic=topic, kind=kind: receive(topic, kind, msg), qos_profile_sensor_data))
        deadline = time.monotonic() + args.sample_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0., deadline-time.monotonic())))
        ros['observation_seconds'] = args.sample_seconds
        ros['samples'] = {}
        for topic, entry in samples.items():
            receipts, stamps = entry.pop('receipt_times'), entry.pop('header_stamps')
            intervals = [b-a for a, b in zip(receipts, receipts[1:])]
            entry.update(count=len(receipts), estimated_receipt_hz=(len(receipts)-1)/(receipts[-1]-receipts[0]) if len(receipts)>1 and receipts[-1]>receipts[0] else None,
                         maximum_receipt_gap=max(intervals) if intervals else None,
                         first_header_stamp=stamps[0] if stamps else None, last_header_stamp=stamps[-1] if stamps else None,
                         nonincreasing_stamps=sum(b <= a for a, b in zip(stamps, stamps[1:])))
            ros['samples'][topic] = entry
    except Exception as exc:
        ros['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None: node.destroy_node()
        if context is not None and context.ok(): context.shutdown()
else:
    report['ros'] = {'sampled': False, 'reason': 'Pass --ros after sourcing the existing approved ROS environment on the robot.'}
def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return f'nonfinite: {value}'
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    return value
print(json.dumps(finite_json(report), indent=2, default=str, allow_nan=False))
PY
