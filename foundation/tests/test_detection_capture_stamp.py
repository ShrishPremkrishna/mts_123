"""Actual viewer classes with mocked ROS/Hailo: image/stamp stay paired."""
import ast
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]


class Clock:
    value = 100.0

    def monotonic(self):
        return self.value

    def time(self):
        return self.value


@pytest.fixture
def viewer(monkeypatch):
    safety_spec = importlib.util.spec_from_file_location(
        "capture_stamp_safety", ROOT / "follow_me_pupper/robot/perception_safety.py")
    safety = importlib.util.module_from_spec(safety_spec)
    safety_spec.loader.exec_module(safety)
    path = ROOT / "follow_me_pupper/robot/viser_camera.py"
    tree = ast.parse(path.read_text())
    selected = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))
                and item.name in ("RosSource", "DetectionWorker", "DetectionPublisher",
                                  "detection_projection")]
    projection_tree = ast.parse((ROOT / "follow_me_pupper/robot/fisheye_converter.py").read_text())
    selected = [item for item in projection_tree.body
                if isinstance(item, ast.FunctionDef) and item.name == "equirect_height"] + selected
    clock = Clock()
    cv2 = SimpleNamespace(
        IMREAD_COLOR=1, COLOR_BGR2RGB=1,
        imdecode=lambda arr, _: np.full((2, 3, 3), int(arr[0]), np.uint8),
        cvtColor=lambda image, _: image)
    scope = dict(threading=threading, time=clock, np=np, cv2=cv2,
                 fresh_frame=safety.fresh_frame, image_stamp_ns=safety.image_stamp_ns,
                 MAX_FRAME_AGE_S=safety.MAX_FRAME_AGE_S,
                 DETECTION_PUBLISH_WIDTH=700, DETECTION_PUBLISH_HEIGHT=572,
                 DETECTION_H_FOV_DEG=220.0)
    exec(compile(ast.Module(selected, type_ignores=[]), str(path), "exec"), scope)

    def header():
        return SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="")

    def detection():
        return SimpleNamespace(header=header(), id="", results=[],
                               bbox=SimpleNamespace(center=SimpleNamespace(
                                   position=SimpleNamespace(x=0, y=0)), size_x=0, size_y=0))

    messages = ModuleType("vision_msgs.msg")
    messages.Detection2DArray = lambda: SimpleNamespace(header=header(), detections=[])
    messages.Detection2D = detection
    messages.ObjectHypothesisWithPose = lambda: SimpleNamespace(
        hypothesis=SimpleNamespace(class_id="", score=0))
    monkeypatch.setitem(sys.modules, "vision_msgs.msg", messages)
    strings = ModuleType("std_msgs.msg")
    strings.String = object
    monkeypatch.setitem(sys.modules, "std_msgs.msg", strings)
    return SimpleNamespace(**scope, clock=clock)


def source(viewer):
    obj = object.__new__(viewer.RosSource)
    obj.lock = threading.Lock()
    obj.frame = obj.frame_received_at = obj.frame_stamp = None
    obj.last_image_stamp = obj.last_read_at = obj.last_read_stamp = None
    obj.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_500_000_000)))
    return obj


def message(stamp, pixel=1):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return SimpleNamespace(data=bytes([pixel]),
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))


def publisher(viewer, now_ns=10_500_000_000):
    messages = []
    node = SimpleNamespace(
        now_ns=now_ns, create_publisher=lambda *args: SimpleNamespace(publish=messages.append),
        create_subscription=lambda *args: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=node.now_ns))
    obj = viewer.DetectionPublisher(node, ["person"])
    return obj, messages


def test_atomic_snapshot_keeps_selected_image_stamp_after_new_camera_callback(viewer):
    camera = source(viewer)
    camera._on_image(message(10_123_456_789, pixel=1))
    selected, received_at, stamp = camera.read_sample()
    viewer.clock.value = 100.1
    camera._on_image(message(10_223_456_789, pixel=2))
    assert (selected[0, 0, 0], received_at, stamp) == (1, 100, 10_123_456_789)
    latest, received_at, stamp = camera.read_sample()
    assert (latest[0, 0, 0], received_at, stamp) == (2, 100.1, 10_223_456_789)
    assert camera.read_sample() is None


def test_slow_older_decode_cannot_overwrite_newer_image_and_metadata(viewer):
    camera = source(viewer)
    entered, release = threading.Event(), threading.Event()
    decode = viewer.cv2.imdecode

    def slow_decode(arr, mode):
        if arr[0] == 1:
            entered.set()
            assert release.wait(2)
        return decode(arr, mode)

    viewer.cv2.imdecode = slow_decode
    thread = threading.Thread(target=camera._on_image, args=(message(10_000_000_001, 1),))
    thread.start()
    try:
        assert entered.wait(1)
        viewer.clock.value = 100.1
        camera._on_image(message(10_000_000_002, 2))
    finally:
        release.set()
        thread.join(1)
    image, received_at, stamp = camera.read_sample()
    assert (image[0, 0, 0], received_at, stamp) == (2, 100.1, 10_000_000_002)


def test_source_rejects_repeated_and_reordered_stamps_without_new_evidence(viewer):
    camera = source(viewer)
    camera._on_image(message(10_000_000_002, 2))
    assert camera.read()[0, 0, 0] == 2  # legacy API still yields an array
    camera._on_image(message(10_000_000_002, 3))
    camera._on_image(message(10_000_000_001, 4))
    assert camera.read_sample() is None
    assert camera.last_image_stamp == 10_000_000_002


def test_future_capture_stamp_cannot_poison_source_highwater(viewer):
    camera = source(viewer)
    camera._on_image(message(10_400_000_000, 1))
    assert camera.read()[0, 0, 0] == 1
    camera._on_image(message(999_000_000_000, 9))
    assert camera.last_image_stamp == 10_400_000_000
    assert camera.read_sample() is None
    camera._on_image(message(10_500_000_000, 2))
    image, _, stamp = camera.read_sample()
    assert image[0, 0, 0] == 2 and stamp == 10_500_000_000


def test_stale_source_header_is_rejected_before_initial_highwater(viewer):
    camera = source(viewer)
    camera._on_image(message(8_000_000_000, 1))
    assert camera.last_image_stamp is None and camera.read_sample() is None
    camera._on_image(message(10_500_000_000, 2))
    assert camera.read()[0, 0, 0] == 2


@pytest.mark.parametrize("node", [None, SimpleNamespace(), SimpleNamespace(
    get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=None)))])
def test_unknown_source_ros_clock_cannot_admit_or_advance_capture(viewer, node):
    camera = source(viewer)
    camera.node = node
    camera._on_image(message(10_500_000_000))
    assert camera.last_image_stamp is None and camera.read_sample() is None


def test_source_clock_exception_fails_closed_and_recovers_without_poisoning(viewer):
    camera = source(viewer)
    healthy_node = camera.node

    def unavailable():
        raise RuntimeError("clock unavailable")

    camera.node = SimpleNamespace(get_clock=unavailable)
    camera._on_image(message(10_500_000_000))
    assert camera.last_image_stamp is None
    camera.node = healthy_node
    camera._on_image(message(10_500_000_000))
    assert camera.read_sample() is not None


def test_publisher_keeps_exact_source_nanoseconds_for_nonempty_and_empty_results(viewer):
    pub, messages = publisher(viewer)
    image = np.zeros((2, 3, 3), np.uint8)
    detection = SimpleNamespace(box=(0, 0, 2, 1), mask=None, class_id=0, score=0.9)
    assert pub.publish([detection], image, capture_stamp_ns=10_123_456_789, received_at=100) == 1
    msg = messages[-1]
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (10, 123_456_789)
    assert msg.detections[0].header.stamp == msg.header.stamp
    assert pub.publish([], image, capture_stamp_ns=10_223_456_789, received_at=100) == 0
    assert len(messages) == 2 and messages[-1].detections == []
    assert messages[-1].header.stamp.nanosec == 223_456_789


@pytest.mark.parametrize("stamp,receipt", [
    (None, 100), (True, 100), (-1, 100), (10.0, 100),
    (10_000_000_000, None), (10_000_000_000, float("nan")),
    (10_000_000_000, 98), (10_000_000_000, 101),
    (8_000_000_000, 100), (11_000_000_000, 100),
])
def test_missing_invalid_delayed_or_future_metadata_publishes_nothing(viewer, stamp, receipt):
    pub, messages = publisher(viewer)
    result = pub.publish([], np.zeros((2, 3, 3)), capture_stamp_ns=stamp, received_at=receipt)
    assert result is None and messages == []


def test_publisher_cannot_turn_duplicate_or_reordered_results_into_new_empty_frames(viewer):
    pub, messages = publisher(viewer)
    image = np.zeros((2, 3, 3))
    assert pub.publish([], image, capture_stamp_ns=10_200_000_000, received_at=100) == 0
    for stamp in (10_200_000_000, 10_100_000_000):
        assert pub.publish([], image, capture_stamp_ns=stamp, received_at=100) is None
    assert len(messages) == 1


def test_epoch_scale_future_nanosecond_and_stale_boundary_are_not_rounded_fresh(viewer):
    now_ns = 1_800_000_000_123_456_789
    pub, messages = publisher(viewer, now_ns=now_ns)
    image = np.zeros((2, 3, 3))
    for stamp in (now_ns + 1, now_ns - 1_000_000_001):
        assert pub.publish([], image, capture_stamp_ns=stamp, received_at=100) is None
    assert messages == []
    assert pub.publish([], image, capture_stamp_ns=now_ns, received_at=100) == 0
    assert messages[0].header.stamp.nanosec == 123_456_789


def test_selected_snapshot_survives_inference_while_source_advances(viewer):
    camera = source(viewer)
    camera._on_image(message(10_123_456_789, 1))
    image, received_at, stamp = camera.read_sample()
    entered, release, published = threading.Event(), threading.Event(), threading.Event()
    pub, messages = publisher(viewer)

    class Model:
        last_inference_ms = 1

        def infer(self, frame):
            entered.set()
            assert release.wait(2)
            assert frame[0, 0, 0] == 1
            return []

        def close(self):
            pass

    def report(detections, frame, **metadata):
        pub.publish(detections, frame, **metadata)
        published.set()

    worker = viewer.DetectionWorker(Model(), on_stamped_result=report)
    try:
        assert worker.submit(image, received_at, capture_stamp_ns=stamp)
        assert entered.wait(1)
        camera._on_image(message(10_223_456_789, 2))
        release.set()
        assert published.wait(1)
        assert messages[0].header.stamp.nanosec == 123_456_789
        assert camera.last_image_stamp == 10_223_456_789
    finally:
        release.set()
        assert worker.close()


def test_worker_latest_pending_image_keeps_its_own_stamp_and_rejects_old_submits(viewer):
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    reports = []

    class Model:
        last_inference_ms = 1

        def infer(self, frame):
            if frame[0, 0, 0] == 1:
                entered.set()
                assert release.wait(2)
            return []

        def close(self):
            pass

    def report(detections, frame, **metadata):
        reports.append((int(frame[0, 0, 0]), metadata))
        if len(reports) == 2:
            finished.set()

    worker = viewer.DetectionWorker(Model(), on_stamped_result=report)
    try:
        assert not worker.submit(np.zeros((2, 3, 3)))  # stamped path requires source evidence
        assert worker.submit(np.full((2, 3, 3), 1), 100, capture_stamp_ns=1)
        assert entered.wait(1)
        assert worker.submit(np.full((2, 3, 3), 2), 100, capture_stamp_ns=2)
        assert worker.submit(np.full((2, 3, 3), 3), 100, capture_stamp_ns=3)
        assert not worker.submit(np.full((2, 3, 3), 9), 100, capture_stamp_ns=3)
        assert not worker.submit(np.full((2, 3, 3), 9), 100, capture_stamp_ns=2)
        release.set()
        assert finished.wait(1)
        assert [(pixel, data["capture_stamp_ns"]) for pixel, data in reports] == [(1, 1), (3, 3)]
    finally:
        release.set()
        assert worker.close()


def test_stalled_inference_and_inference_failure_do_not_publish_healthy_empty_frames(viewer):
    entered, release, failed = threading.Event(), threading.Event(), threading.Event()
    reports = []

    class Model:
        last_inference_ms = 1

        def infer(self, frame):
            if frame[0, 0, 0] == 1:
                entered.set()
                assert release.wait(2)
                return []
            failed.set()
            raise RuntimeError("test accelerator error")

        def close(self):
            pass

    worker = viewer.DetectionWorker(Model(), on_stamped_result=lambda *a, **k: reports.append((a, k)))
    try:
        worker.submit(np.full((2, 3, 3), 1), 100, capture_stamp_ns=1)
        assert entered.wait(1)
        viewer.clock.value = 102
        worker.submit(np.full((2, 3, 3), 2), 102, capture_stamp_ns=2)
        release.set()
        assert failed.wait(1)
    finally:
        release.set()
        assert worker.close()
    assert reports == []


def test_unstamped_picam_worker_retains_legacy_two_argument_callback(viewer):
    reported = threading.Event()
    calls = []

    class Model:
        last_inference_ms = 1

        def infer(self, frame):
            return []

        def close(self):
            pass

    def report(*args):
        calls.append(args)
        reported.set()

    worker = viewer.DetectionWorker(Model(), on_result=report)
    try:
        worker.submit(np.zeros((2, 3, 3)))
        assert reported.wait(1)
        assert len(calls) == 1 and len(calls[0]) == 2
    finally:
        assert worker.close()


@pytest.mark.parametrize("display_fov,display_width", [(90, 700), (180, 350), (300, 1050), (220, 350)])
def test_display_fov_and_scale_do_not_change_detection_projection_or_box_contract(
        viewer, display_fov, display_width):
    calls = []
    raw = np.zeros((1050, 1400, 3), np.uint8)
    displayed = np.ones((viewer.equirect_height(display_width, display_fov), display_width, 3), np.uint8)

    def project(image, model, width, height, *, h_fov_deg):
        calls.append((image is raw, width, height, h_fov_deg))
        return np.zeros((height, width, 3), np.uint8)

    viewer.detection_projection.__globals__["fisheye_to_equirectangular"] = project
    detector_input = viewer.detection_projection(
        raw, object(), 700, displayed=displayed, display_fov=display_fov)
    assert calls == [(True, 700, 572, 220.0)]
    assert detector_input is not displayed
    pub, messages = publisher(viewer)
    detection = SimpleNamespace(box=(280, 100, 420, 500), mask=None, class_id=0, score=0.9)
    assert pub.publish([detection], detector_input,
                       capture_stamp_ns=10_400_000_000, received_at=100) == 1
    box = messages[0].detections[0].bbox
    assert (box.center.position.x, box.center.position.y, box.size_x, box.size_y) == (350, 300, 140, 400)


def test_matching_default_display_projection_is_reused_without_second_warp(viewer):
    displayed = np.zeros((572, 700, 3), np.uint8)

    def redundant_warp(*args, **kwargs):
        pytest.fail("the matching default display should be reused")

    viewer.detection_projection.__globals__["fisheye_to_equirectangular"] = redundant_warp
    assert viewer.detection_projection(
        np.zeros((1050, 1400, 3)), object(), 700,
        displayed=displayed, display_fov=220.0) is displayed


def test_raw_camera_view_still_gets_fixed_detector_projection(viewer):
    calls = []

    def project(image, model, width, height, *, h_fov_deg):
        calls.append((width, height, h_fov_deg))
        return np.zeros((height, width, 3), np.uint8)

    viewer.detection_projection.__globals__["fisheye_to_equirectangular"] = project
    frame = viewer.detection_projection(np.zeros((1050, 1400, 3)), object(), 700)
    assert frame.shape[:2] == (572, 700) and calls == [(700, 572, 220.0)]
