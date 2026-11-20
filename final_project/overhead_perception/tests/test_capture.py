"""Camera capture against a fake camera that can be missing, drop out and come back,
the way a wireless phone camera does. No real camera is touched."""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fieldcam.capture import FrameGrabber, is_live  # noqa: E402


class FakeWorld:
    """The 'phone': whether it is reachable right now, and every handle opened on it."""

    def __init__(self, available=True):
        self.available, self.handles, self.opens = available, [], 0

    def open(self, _source):
        self.opens += 1
        cap = FakeCap(self)
        self.handles.append(cap)
        return cap


class FakeCap:
    def __init__(self, world):
        self.world, self.opened, self.released, self.k = world, world.available, False, 0

    def isOpened(self):
        return self.opened and not self.released

    def set(self, *_a):
        return True

    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: 64, cv2.CAP_PROP_FRAME_HEIGHT: 48}.get(prop, 0)

    def read(self):
        time.sleep(0.01)
        if not self.world.available or self.released:
            return False, None
        self.k += 1
        return True, np.full((48, 64, 3), self.k % 255, np.uint8)

    def release(self):
        self.released = True


def wait_until(cond, timeout=6.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_live_or_file():
    assert is_live(1) and is_live("http://192.168.1.20:8080/video") and is_live("rtsp://phone/stream")
    assert not is_live("clip.mov") and not is_live("out/run1.mp4")


def test_file_frame_clock_uses_presentation_time():
    world = FakeWorld()
    with FrameGrabber("run.mov", 64, 48, 30, open_fn=world.open) as grab:
        # Unequal frame intervals are ordinary for phone video.
        stamp = None
        for position_ms in (100, 139, 183):
            assert grab.read()[0] is not None
            world.handles[0].get = lambda prop: position_ms
            stamp = grab.file_time_s(stamp)
            assert stamp == position_ms / 1000
        for bad_ms in (183, 100, -1, float("nan"), float("inf")):
            world.handles[0].get = lambda prop: bad_ms
            with pytest.raises(ValueError, match="presentation timestamp"):
                grab.file_time_s(stamp)
        assert grab.file_time_s(stamp, constant_fps=25) == 2 / 25


@pytest.mark.parametrize(("options", "clock", "fps"), [
    ([], "video_presentation", 20),
    (["--replay-fps", "10"], "assumed_constant_fps", 10),
])
def test_file_clock_reaches_published_states(tmp_path, monkeypatch, options, clock, fps):
    from scripts import run_perception

    path = tmp_path / "short.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 20, (64, 48))
    assert writer.isOpened()
    for _ in range(3):
        writer.write(np.zeros((48, 64, 3), np.uint8))
    writer.release()
    states = []
    sink = SimpleNamespace(source_id="test", source_session="test", send=states.append, close=lambda: None)
    def publisher(_host, _port, *, live):
        assert not live
        return sink

    monkeypatch.setattr(run_perception, "StatePublisher", publisher)
    monkeypatch.setattr(run_perception, "OPEN_CAMERAS", [])
    monkeypatch.setattr(sys, "argv", ["run_perception", "--source", str(path), "--no-gui", *options])
    try:
        run_perception.main()
    finally:
        for grab in run_perception.OPEN_CAMERAS:
            grab.close()
    assert [s["captured_at"] for s in states] == pytest.approx([0, 1 / fps, 2 / fps])
    assert all(s["replay_clock"] == clock and s["stamp"] == s["captured_at"] for s in states)


def test_waits_for_a_camera_that_is_not_there_yet():
    world, said = FakeWorld(available=False), []
    with pytest.raises(RuntimeError, match="lock it"):
        FrameGrabber(1, 64, 48, 30, wait=0, open_fn=world.open, say=said.append)   # no patience: fail with advice
    t0 = time.time()

    def appear_later(source):
        if time.time() - t0 > 1.5:
            world.available = True
        return world.open(source)

    grab = FrameGrabber(1, 64, 48, 30, wait=20, open_fn=appear_later, say=said.append)
    assert time.time() - t0 > 1.4 and any("waiting for camera" in m for m in said)
    frame, _, n = grab.read()
    assert frame is not None and grab.status == "ok" and (grab.width, grab.height) == (64, 48)
    grab.close()
    assert world.handles[-1].released


def test_dropout_is_reported_then_recovered():
    world = FakeWorld()
    grab = FrameGrabber(1, 64, 48, 30, stall_s=0.3, open_fn=world.open, say=lambda _m: None)
    frame, _, n = grab.read()
    assert frame is not None and grab.status == "ok" and grab.lost_for == 0.0
    first = world.handles[-1]

    world.available = False                       # the phone drops off the network
    assert wait_until(lambda: grab.status == "lost")
    n = grab._n
    got, _, n2 = grab.read(n, timeout=0.3)
    assert got is None and n2 == n                # a lost feed hands out nothing, not the old frame again
    assert grab.lost_for > 0.3
    assert wait_until(lambda: first.released)     # it let go of the dead camera

    world.available = True                        # the phone is back
    assert wait_until(lambda: grab.status == "ok" and grab.reconnects == 1)
    fresh, _, n3 = grab.read(n, timeout=2.0)
    assert fresh is not None and n3 != n          # new frames flow again, from a new handle
    assert world.handles[-1] is not first and world.opens >= 2
    grab.close()
    assert world.handles[-1].released


def test_a_brief_hiccup_is_not_a_dropout():
    world = FakeWorld()
    grab = FrameGrabber(1, 64, 48, 30, stall_s=1.0, open_fn=world.open, say=lambda _m: None)
    grab.read()
    world.available = False
    time.sleep(0.3)                               # shorter than stall_s
    world.available = True
    time.sleep(0.2)
    assert grab.status == "ok" and grab.reconnects == 0 and world.opens == 1
    grab.close()


# ---- choosing the camera by name --------------------------------------------------
MAC = {"index": 0, "name": "MacBook Pro Camera", "model": "MacBook Pro Camera", "built_in": True}
IPHONE = {"index": 1, "name": "Ryan’s iPhone Camera", "model": "iPhone14,8", "built_in": False}
PIXEL = {"index": 1, "name": "Pixel 8", "model": "UVC Camera", "built_in": False}
GENERIC = {"index": 1, "name": "USB Video Device", "model": "UVC Camera VendorID_1234", "built_in": False}


def test_what_counts_as_a_camera_name():
    from fieldcam.capture import is_camera_name

    assert is_camera_name("phone") and is_camera_name("iPhone") and is_camera_name("Pixel 8")
    assert not is_camera_name(1) and not is_camera_name("1") and not is_camera_name("sim")
    assert not is_camera_name("clip.mov") and not is_camera_name("runs/match1.mp4")
    assert not is_camera_name("http://192.168.1.20:8080/video")
    assert is_live("phone") and not is_live("clip.mov")


def test_find_camera_by_name():
    from fieldcam.capture import find_camera

    assert find_camera("phone", [MAC, IPHONE]) is IPHONE              # the phone, not the laptop's own camera
    assert find_camera("phone", [MAC, PIXEL]) is PIXEL                # an Android phone in webcam mode
    assert find_camera("phone", [MAC, GENERIC]) is GENERIC            # unknown name: any camera that is not built in
    assert find_camera("phone", [MAC]) is None                        # only the laptop camera: no phone, do not use it
    assert find_camera("iphone", [MAC, IPHONE]) is IPHONE             # part of a name, any case
    assert find_camera("ryan's iphone", [MAC, IPHONE]) is IPHONE      # straight apostrophe matches the curly one
    assert find_camera("iPhone14,8", [MAC, IPHONE]) is IPHONE         # or the model
    assert find_camera("macbook", [MAC, IPHONE]) is MAC
    assert find_camera("gopro", [MAC, IPHONE]) is None


def test_named_camera_is_found_again_under_a_new_number():
    """The phone is number 1. It drops out, and when it comes back another camera has taken
    number 1, so the phone is now number 2. Asking for it by name must follow it."""
    world, opened = FakeWorld(), []
    cams = {"list": [MAC, IPHONE]}

    def open_fn(index):
        opened.append(index)
        return world.open(index)

    grab = FrameGrabber("phone", 64, 48, 30, stall_s=0.3, open_fn=open_fn, cameras_fn=lambda: cams["list"],
                        say=lambda _m: None)
    assert opened == [1] and grab.device["name"] == IPHONE["name"]
    assert grab.read()[0] is not None

    world.available = False                                   # unplugged
    cams["list"] = [MAC]                                      # and gone from the list
    assert wait_until(lambda: grab.status == "lost")
    time.sleep(1.2)
    assert opened == [1]                                      # it did NOT fall back to the laptop camera
    webcam = dict(GENERIC, index=1, name="Desk Webcam", model="UVC")
    cams["list"] = [MAC, webcam, dict(IPHONE, index=2)]       # plugged back in, now third in the list
    world.available = True
    assert wait_until(lambda: grab.status == "ok" and grab.reconnects == 1)
    assert opened[-1] == 2 and grab.device["index"] == 2
    grab.close()

    with pytest.raises(RuntimeError, match="USB"):            # no phone at all, no patience: fail with advice
        FrameGrabber("phone", 64, 48, 30, wait=0, open_fn=open_fn, cameras_fn=lambda: [MAC], say=lambda _m: None)
