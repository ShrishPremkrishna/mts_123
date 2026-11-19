"""Camera capture against a fake camera that can be missing, drop out and come back,
the way a wireless phone camera does. No real camera is touched."""
import sys
import time
from pathlib import Path

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
