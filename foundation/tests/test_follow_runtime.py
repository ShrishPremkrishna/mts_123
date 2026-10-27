"""Perception freshness and device ownership regressions; no ROS or Hailo required."""
import ast
import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for relative in ('follow_me_pupper/lab', 'follow_me_pupper/robot'):
    sys.path.insert(0, str(ROOT / relative))
from lab_checks import FakeDetection, FakeTime, load_follow_me
from perception_safety import fresh_frame
import target_lock

fm = load_follow_me()


def node(now=10):
    obj = object.__new__(fm.StateMachineNode)
    obj.state, obj.tracking_enabled = fm.State.TRACK, True
    obj.lock = target_lock.Lock()
    obj.target_pos, obj.target_bottom = .1, 300
    obj.last_frame_time = obj.last_detection_time = FakeTime(now)
    obj.obstacle_pos, obj.avoid_done_time = None, None
    obj.phase_start, obj.avoid_dir = FakeTime(now), 1
    obj.sent = []
    obj.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime(now))
    obj.command_publisher = SimpleNamespace(publish=obj.sent.append)
    obj.get_logger = lambda: SimpleNamespace(info=lambda _: None, warning=lambda _: None, error=lambda _: None)
    return obj


@pytest.fixture(autouse=True)
def twist(monkeypatch):
    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.)
            self.angular = SimpleNamespace(z=0.)
    monkeypatch.setattr(fm, 'Twist', Twist)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -5.])
def test_bad_detection_cannot_become_motion(bad):
    obj = node()
    obj.target_class_id = 0
    det = FakeDetection(300, 100, 400, 400, class_id=0)
    det.bbox.size_x = bad
    assert obj.process_detections([det]) is None
    obj.target_pos = float('nan')
    assert obj.motion_command() == (0., 0.)


def test_close_target_aborts_detour_before_another_forward_command():
    obj = node()
    obj.state = fm.State.AVOID_PASS
    obj.target_bottom = target_lock.STOP_BOTTOM
    obj.timer_callback()
    assert obj.state == fm.State.TRACK
    assert obj.sent[-1].linear.x == obj.sent[-1].angular.z == 0


def test_detour_end_with_lost_target_does_not_resume_stale_forward_motion():
    obj = node()
    obj.state, obj.phase_start = fm.State.AVOID_RETURN, FakeTime(9)
    obj.last_detection_time = FakeTime(1)
    yaw, forward = obj.detour_command(FakeTime(10))
    assert obj.state == fm.State.SEARCH and forward == 0


def test_clock_rewind_stops_and_unknown_target_does_not_chase_previous_person():
    obj = node()
    obj.last_frame_time = FakeTime(11)
    obj.timer_callback()
    assert obj.sent[-1].linear.x == obj.sent[-1].angular.z == 0
    obj.class_name_to_id = {'person': 0}
    obj.target_object = 'person'
    assert obj.start_tracking('not a class') is False
    assert not obj.tracking_enabled
    assert obj.state == fm.State.IDLE


def test_follow_yaw_stays_inside_default_policy_command_range():
    obj = node()
    obj.target_pos = .5
    assert obj.motion_command()[0] == -2.


def test_corrupt_detection_frame_does_not_refresh_watchdog():
    obj = node()
    old_frame = obj.last_frame_time
    bad = FakeDetection(300, 100, 400, 400, class_id=0)
    bad.bbox.size_y = float('nan')
    obj.detection_callback(SimpleNamespace(detections=[bad]))
    assert obj.last_frame_time is old_frame


def viewer_classes():
    """Load the actual viewer worker/source classes without its optional UI imports."""
    path = ROOT / 'follow_me_pupper/robot/viser_camera.py'
    tree = ast.parse(path.read_text())
    classes = [item for item in tree.body if isinstance(item, ast.ClassDef)
               and item.name in ('RosSource', 'DetectionWorker')]
    scope = dict(threading=threading, time=time, fresh_frame=fresh_frame)
    exec(compile(ast.Module(classes, type_ignores=[]), str(path), 'exec'), scope)
    return scope


def test_viewer_consumes_each_ros_image_once_and_rejects_old_frames():
    cls = viewer_classes()['RosSource']
    source = object.__new__(cls)
    source.lock = threading.Lock()
    source.frame = np.zeros((3, 4, 3))
    source.frame_received_at = time.monotonic()
    assert source.read().shape == (3, 4, 3)
    assert source.read() is None  # a dead camera cannot keep /detections alive
    source.frame = np.zeros((3, 4, 3))
    source.frame_received_at = time.monotonic() - 2
    assert source.read() is None


def test_viewer_drops_delayed_inference_and_worker_owns_detector_until_finished():
    cls = viewer_classes()['DetectionWorker']
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    reports = []
    class Detector:
        last_inference_ms = 1
        def infer(self, frame):
            entered.set()
            assert release.wait(4)
            return ['a stale detection']
        def close(self): closed.set()
    worker = cls(Detector(), on_result=lambda *args: reports.append(args))
    worker.submit(np.zeros((3, 4, 3)))
    assert entered.wait(1)
    try:
        assert worker.close() is False  # timed join must not close an in-use device
        assert not closed.is_set()
    finally:
        release.set()
        worker.thread.join(1)
    assert closed.is_set() and reports == []


@pytest.fixture
def service(monkeypatch):
    stubs = {
        'cv2': {'imdecode': lambda *a: np.zeros((2, 2, 3)), 'IMREAD_COLOR': 1,
                'cvtColor': lambda x, *a: x, 'COLOR_BGR2RGB': 1},
        'rclpy.executors': {'ExternalShutdownException': RuntimeError},
        'rclpy.logging': {'LoggingSeverity': SimpleNamespace(WARN=30)},
        'rclpy.signals': {'SignalHandlerOptions': SimpleNamespace(NO=0)},
        'sensor_msgs.msg': {'CompressedImage': object},
        'std_srvs.srv': {'Trigger': object},
        'vision_msgs.msg': {'Detection2D': object, 'ObjectHypothesisWithPose': object,
                            'Detection2DArray': lambda: SimpleNamespace(header=SimpleNamespace(), detections=[])},
        'fisheye_converter': {'equirect_height': lambda *args: 572,
                              'fisheye_to_equirectangular': lambda frame, *args, **kwargs: frame,
                              'load_camera_model': lambda *args: SimpleNamespace(width=2, height=2)},
    }
    for name, attributes in stubs.items():
        mod = ModuleType(name)
        mod.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, mod)
    spec = importlib.util.spec_from_file_location('follow_service_under_test', ROOT / 'follow_me_pupper/robot/follow_service.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def service_object(module):
    obj = object.__new__(module.FollowService)
    obj.running, obj.tracking_enabled, obj.tracking_generation = True, True, 1
    obj.state_lock, obj.image_lock = threading.Lock(), threading.Lock()
    obj.image_ready, obj.wake = threading.Event(), threading.Event()
    obj.camera_model = SimpleNamespace(width=2, height=2)
    obj.get_logger = lambda: SimpleNamespace(error=lambda _: None, warning=lambda _: None)
    obj.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
    obj.send_eyes = lambda *args: None
    obj.target_class_id = 0
    return obj


def test_service_stop_does_not_close_a_running_detector_or_allow_second_owner(service):
    obj = service_object(service)
    obj.pipeline = SimpleNamespace(join=lambda **kwargs: None, is_alive=lambda: True)
    obj.image_sub = object()
    obj.destroy_subscription = lambda sub: None
    obj.detector = SimpleNamespace(close=lambda: pytest.fail('must not close while inference runs'))
    obj.stop_pipeline()
    assert not obj.running and obj.image_sub is None
    assert 'still stopping' in obj.start_pipeline()


def test_service_drops_old_result_and_previous_follow_session(service):
    obj = service_object(service)
    clock = [10.]
    service.time = SimpleNamespace(monotonic=lambda: clock[0])
    obj.detection_callback = lambda _: pytest.fail('stale result must not refresh tracking')
    def slow(frame):
        clock[0] += 2
        return []
    obj.process(SimpleNamespace(data=b'123'), 10., SimpleNamespace(infer=slow))
    clock[0] = 10.
    def new_session(frame):
        obj.tracking_generation += 1
        return []
    obj.process(SimpleNamespace(data=b'123'), 10., SimpleNamespace(infer=new_session))


def test_service_rejects_duplicate_camera_stamps(service):
    obj = service_object(service)
    obj.last_image_stamp = None
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=1)))
    obj.on_image(msg)
    first = obj.latest_image
    obj.on_image(msg)
    assert obj.latest_image is first
    obj.on_image(SimpleNamespace())
    assert obj.latest_image is first
