"""Live camera capture in a child process that the eye respawns (fc/eye.py LiveSource, Nov 28).

Nov 27: the phone camera dropped mid-session and only an app restart brought it back; in 042513Z the feed
froze for 27 s (identical pictures at full rate) and never counted as lost. Here a fake camera (FakeCamera,
driven by a control file the child reads every frame) stalls, freezes, hangs, comes back at another size, as
another device, and the source must recover on its own, keep the picture size the calibration was made at,
never fall back to another camera, and say what is going on."""
import importlib.util
import json
import math
import os
import time

import pytest

from fc.core import RobotCfg, load_field

HAVE_CV2 = importlib.util.find_spec("cv2") is not None
pytestmark = pytest.mark.skipif(not HAVE_CV2, reason="needs OpenCV (.venv-fc)")
if HAVE_CV2:
    from fc import eye as eye_mod
    from fc.eye import Eye, LiveSource, SyntheticSource, find_device

BLUE = RobotCfg("blue", "blue", 10, "", "stock")


class Ctl:
    """The fake camera's control file."""

    def __init__(self, path, **base):
        self.path, self.base = path, base
        self.set("ok")

    def set(self, mode, **kw):
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"mode": mode, **self.base, **kw}))
        os.replace(tmp, self.path)

    @property
    def source(self):
        return f"fake:{self.path}"


def until(cond, timeout, step=0.05):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if cond():
            return True
        time.sleep(step)
    return cond()


def frames_for(src, seconds):
    n, t_end = 0, time.monotonic() + seconds
    while time.monotonic() < t_end:
        f, _ = src.read()
        n += f is not None
    return n


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def test_find_device_locks_to_the_first_camera_and_skips_desk_view():
    cams = [{"index": 0, "name": "Shrish’s iPhone Desk View Camera", "model": "iPhone15,2", "built_in": False},
            {"index": 1, "name": "Shrish’s iPhone Camera", "model": "iPhone15,2", "built_in": False},
            {"index": 2, "name": "FaceTime HD Camera", "model": "", "built_in": True},
            {"index": 3, "name": "OBS Virtual Camera", "model": "", "built_in": False}]
    dev, _ = find_device("phone", cameras=cams)
    assert dev["name"] == "Shrish’s iPhone Camera"           # not the Desk View crop
    dev, _ = find_device("phone", lock="Shrish's iPhone Camera", cameras=cams)   # apostrophe kind ignored
    assert dev["index"] == 1
    # the phone is gone: a reconnect must NOT take the virtual camera that "phone" would fall back to
    dev, why = find_device("phone", lock="Shrish’s iPhone Camera", cameras=cams[2:])
    assert dev is None and "not connected" in why and "OBS Virtual Camera" in why
    assert find_device("phone", cameras=cams[2:])[0]["name"] == "OBS Virtual Camera"   # first connection only
    assert find_device("Desk View", cameras=cams)[0]["index"] == 0                     # asked for by name
    assert find_device("1")[0]["index"] == 1


def test_respawns_on_stall_freeze_and_hang_and_frees_the_child(tmp_path):
    ctl = Ctl(tmp_path / "cam.json", size=[320, 180])
    src = LiveSource(ctl.source, 320, 180, wait=10, say=lambda m: None)
    try:
        assert src.state == "ok" and src.lock_name == "Fake iPhone Camera"
        assert frames_for(src, 0.5) >= 10
        for mode, why in (("stall", "no new frame"), ("freeze", "picture frozen"), ("hang", "no new frame")):
            n = src.reconnects
            ctl.set(mode)
            assert until(lambda: src.state == "reconnecting", 3.0), (mode, src.message)
            assert why in src.reason and src.message.startswith("camera reconnecting"), src.message
            ctl.set("ok")
            assert until(lambda: src.reconnects == n + 1 and src.state == "ok", 6.0), (mode, src.message)
            assert src.message.startswith("camera back after") and frames_for(src, 0.3) >= 5
        pid = src.proc.pid
    finally:
        src.close()
    assert until(lambda: not alive(pid), 2.0), "capture child left running (would hold the phone)"


def test_waits_for_a_camera_that_is_not_there_yet(tmp_path):
    import threading
    ctl = Ctl(tmp_path / "cam.json", size=[320, 180])
    ctl.set("absent")
    said, out = [], {}
    th = threading.Thread(target=lambda: out.update(src=LiveSource(ctl.source, 320, 180, wait=15, say=said.append)))
    th.start()
    time.sleep(1.5)
    assert "src" not in out
    ctl.set("ok")
    th.join(10)
    src = out["src"]
    try:
        assert src.state == "ok" and src.respawns >= 2 and src.reconnects == 0   # several attempts, then streaming
        assert any("-> Fake iPhone Camera" in m for m in said), said
    finally:
        src.close()


def test_reconnect_keeps_the_pinned_size_and_device(tmp_path):
    ctl = Ctl(tmp_path / "cam.json", size=[320, 180])
    src = LiveSource(ctl.source, 320, 180, wait=10, say=lambda m: None)
    try:
        ctl.set("stall")
        assert until(lambda: src.state == "reconnecting", 3.0)
        src.read()                                      # the last frame before the stall, not yet taken
        ctl.set("ok", size=[640, 480])                  # 4:3: another view, refused
        assert until(lambda: "gives 640x480" in src.message, 6.0), src.message
        assert frames_for(src, 0.5) == 0
        ctl.set("ok", size=[320, 180], name="OBS Virtual Camera")   # another camera: never taken
        assert until(lambda: "Fake iPhone Camera not connected" in src.message, 6.0), src.message
        assert frames_for(src, 0.5) == 0 and src.state == "reconnecting"
        ctl.set("ok", size=[640, 360])                  # same device, same aspect, other size: scaled to the pin
        assert until(lambda: src.state == "ok", 8.0), src.message
        f, _ = src.read()
        assert f.shape == (180, 320, 3) and src.camera_size == [640, 360] and "scaled to 320x180" in src.message
        assert src.health()["reconnects"] == 1 and src.health()["lock"] == "Fake iPhone Camera"
    finally:
        src.close()


def test_added_latency_and_cpu_per_frame(tmp_path):
    """Child receipt -> frame in the eye's hands, and the eye process's CPU per 1080p frame (copy out of shared
    memory + wake-up). Measured Nov 28 (M-series Mac): p50 ~1.4 ms, ~1.1 ms CPU/frame; bounds are loose."""
    import resource
    ctl = Ctl(tmp_path / "cam.json")
    src = LiveSource(ctl.source, 1920, 1080, wait=10, say=lambda m: None)
    try:
        time.sleep(0.5)
        r0 = resource.getrusage(resource.RUSAGE_SELF)
        lat, t_end = [], time.monotonic() + 3.0
        while time.monotonic() < t_end:
            f, t = src.read()
            if f is not None:
                lat.append(time.monotonic() - t)
        r1 = resource.getrusage(resource.RUSAGE_SELF)
    finally:
        src.close()
    lat.sort()
    cpu = (r1.ru_utime + r1.ru_stime - r0.ru_utime - r0.ru_stime) / len(lat)
    assert len(lat) > 60
    assert lat[len(lat) // 2] < 0.006 and cpu < 0.006, (lat[len(lat) // 2], cpu)


def test_eye_keeps_its_calibration_across_a_reconnect(tmp_path):
    """The eye serves poses again after the camera comes back (here at 1280x720, scaled to the pinned 1920x1080)
    with the same calibration: no re-fit, state unverified until the floor tags confirm it."""
    import cv2
    field = load_field()
    syn = Eye(field, [BLUE], {}, source="synthetic")
    s = SyntheticSource(syn.cfg, field, [BLUE], 1920, 1080)
    frame, _ = s.read()
    x0, y0, _ = s.truth["robots"]["blue"]
    cv2.imwrite(str(tmp_path / "field.png"), frame)
    ctl = Ctl(tmp_path / "cam.json", image=str(tmp_path / "field.png"))
    params = {"eye_tag_height": 0.18, "eye_tag_offset_x": 0.0, "eye_yaw_offset_deg": 0.0, "eye_floor_size": 0.15}
    eye = Eye(field, [BLUE], params, source=ctl.source, cal_path=tmp_path / "eye_cal.json", ball_color_path=None,
              robot_colors_path=None, robots_dir=None)
    eye.start()

    def pose():
        w = eye.latest()
        o = w.robots.get("blue") if w else None
        return o.pose if o is not None and time.monotonic() - o.t < 0.5 else None
    try:
        assert until(lambda: pose() is not None and eye.calmon["state"] == "ok", 15.0), eye.message
        assert isinstance(eye.src, LiveSource)
        how, saved = eye.cal.how, eye.cal.saved
        ctl.set("stall")
        assert until(lambda: eye.message.startswith("camera reconnecting"), 4.0), eye.message
        assert eye.status()["camera"]["state"] == "reconnecting"
        ctl.set("ok", size=[1280, 720])
        assert until(lambda: eye.src.reconnects == 1, 8.0), eye.message
        assert until(lambda: pose() is not None, 5.0), eye.message
        p = pose()
        assert math.hypot(p.x - x0, p.y - y0) < 0.02
        assert (eye.cal.how, eye.cal.saved) == (how, saved) and eye.calmon["resolves"] == 0   # no re-fit
        events = " | ".join(e[1] for e in eye.calmon["events"])
        assert "cal unverified: camera back after" in events and "scaled to 1920x1080" in events, events
        assert until(lambda: eye.calmon["state"] == "ok", 5.0), eye.message   # floor tags verified it
    finally:
        eye.close()


def test_eye_default_cal_path_is_used_for_a_fake_live_camera_only_when_given(tmp_path):
    """Guard for the tests above: a fake camera is a live source, so without cal_path it would write the
    site's config/eye_cal.json."""
    assert Eye(load_field(), [BLUE], {}, source="fake:/nonexistent.json").cal_path == eye_mod.EYE_CAL_PATH
