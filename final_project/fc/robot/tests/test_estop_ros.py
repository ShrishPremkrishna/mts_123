"""E-stop end to end with real rclpy: drive_bridge.py (subprocess) against a fake controller manager.

Measures the latency from the signed estop packet leaving the sender to (a) /emergency_stop arriving and
(b) the SwitchController request arriving at the manager, and checks latch, acks, zero output, restart
behaviour and the fallen cutoff. Needs ROS 2 Jazzy with controller_manager_msgs (skipped otherwise):
  docker run --rm --init --network none -v "$PWD/fc/robot:/src:ro" cs123-ros-jazzy-slew-check:latest \
    bash -c 'source /opt/ros/jazzy/setup.bash && cd /src && python3 -m pytest -q -s -p no:cacheprovider tests/test_estop_ros.py'
"""
import json
import math
import os
import socket
import statistics
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
cm = pytest.importorskip("controller_manager_msgs.srv")
from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Empty  # noqa: E402

ROBOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROBOT)
import drive_bridge as db  # noqa: E402

KEY = bytes(range(32))
LEGS = "neural_controller_default neural_controller_liedown"
DOMAIN = "93"


class FakeManager:
    def __init__(self):
        os.environ["ROS_DOMAIN_ID"] = DOMAIN
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()
        self.node = rclpy.create_node("fake_controller_manager")
        self.states = {"neural_controller_default": "active", "neural_controller_liedown": "inactive",
                       "joint_state_broadcaster": "active"}
        self.switches, self.estops, self.twists = [], [], []
        self.node.create_service(cm.ListControllers, "/controller_manager/list_controllers", self.on_list)
        self.node.create_service(cm.SwitchController, "/controller_manager/switch_controller", self.on_switch)
        self.node.create_subscription(Empty, "/emergency_stop", lambda m: self.estops.append(time.monotonic()), 10)
        self.node.create_subscription(Twist, "/llm_cmd_vel", lambda m: self.twists.append((time.monotonic(), m.linear.x)), 10)
        self.imu = self.node.create_publisher(Imu, "/imu_test", 10)
        self.run = True
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        while self.run:
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def on_list(self, req, res):
        from controller_manager_msgs.msg import ControllerState
        for n, s in self.states.items():
            c = ControllerState()
            c.name, c.state = n, s
            res.controller.append(c)
        return res

    def on_switch(self, req, res):
        self.switches.append((time.monotonic(), list(req.deactivate_controllers), req.strictness,
                              list(req.activate_controllers)))
        for n in req.deactivate_controllers:
            if n in self.states:
                self.states[n] = "inactive"
        for n in req.activate_controllers:
            if n in self.states:
                self.states[n] = "active"
        res.ok = True
        return res

    def close(self):
        self.run = False
        self.thread.join(timeout=2)
        self.node.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture(scope="module")
def mgr():
    m = FakeManager()
    yield m
    m.close()


@pytest.fixture
def env(tmp_path):
    key = tmp_path / "link.key"
    key.write_text(KEY.hex())
    key.chmod(0o600)
    return tmp_path


def start_bridge(tmp, port, extra=()):
    log = open(tmp / ("bridge-%d.log" % port), "w")
    e = {**os.environ, "ROS_DOMAIN_ID": DOMAIN, "ROS_LOCALHOST_ONLY": "1"}
    p = subprocess.Popen([sys.executable, "-u", os.path.join(ROBOT, "drive_bridge.py"), "--robot-id", "blue",
                          "--key-file", str(tmp / "link.key"), "--state-file", "", "--port", str(port),
                          "--bind", "127.0.0.1", "--no-mux-guard", "--imu-topic", "/imu_test",
                          "--walker", "neural_controller_default", "--legs", LEGS, "--latch-file", str(tmp / "estop.json"),
                          *extra], env=e, stdout=log, stderr=subprocess.STDOUT)
    return p


class Sender:
    def __init__(self, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("127.0.0.1", 0))
        self.s.settimeout(0.5)
        self.port, self.seq, self.sid = port, 0, "t%d" % port

    def send(self, vx=0.0, estop=False):
        self.seq += 1
        m = {"v": 2, "robot": "blue", "sid": self.sid, "seq": self.seq, "t": time.time(), "vx": vx, "vy": 0.0,
             "wz": 0.0, "ttl": 0.25}
        if estop:
            m["estop"] = True
        t = time.monotonic()
        self.s.sendto(db.sign(KEY, m), ("127.0.0.1", self.port))
        return t

    def ack(self):
        try:
            a = json.loads(self.s.recv(4096))
        except socket.timeout:
            return None
        assert db.verify(KEY, a)
        return a


def wait(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


def warm(snd, mgr, bridge_log):
    """Bridge up, graph discovered (it has polled list_controllers), driving at 0.2 m/s."""
    assert wait(lambda: (snd.send(0.2), snd.ack())[1] is not None, 15), open(bridge_log).read()
    n = len(mgr.twists)
    assert wait(lambda: len(mgr.twists) > n + 5, 5)
    time.sleep(1.5)                                   # >= 2 list_controllers polls at 2 Hz


def test_link_estop_latency_latch_and_restart(mgr, env):
    lat_es, lat_sw = [], []
    for trial in range(5):
        mgr.states["neural_controller_default"] = "active"
        port = 19870 + trial
        p = start_bridge(env, port)
        try:
            snd = Sender(port)
            warm(snd, mgr, env / ("bridge-%d.log" % port))
            n_sw, n_es = len(mgr.switches), len(mgr.estops)
            t0 = snd.send(estop=True)
            assert wait(lambda: len(mgr.switches) > n_sw, 2.0), open(env / ("bridge-%d.log" % port)).read()
            assert wait(lambda: len(mgr.estops) > n_es, 1.0)
            lat_es.append((mgr.estops[n_es] - t0) * 1000)
            lat_sw.append((mgr.switches[n_sw][0] - t0) * 1000)
            assert "neural_controller_default" in mgr.switches[n_sw][1] and mgr.switches[n_sw][2] == 1  # BEST_EFFORT
            assert all(n in LEGS.split() for n in mgr.switches[n_sw][1])
            a = snd.ack()
            assert a and a["estopped"] is True and a["hold"] == "estopped"
            for _ in range(5):                         # the laptop's resend burst: idempotent
                snd.send(estop=True)
                assert snd.ack()["estopped"] is True
            time.sleep(0.3)
            assert len(mgr.switches) == n_sw + 1       # one R3 action, confirmed, no retries
            snd.send(vx=0.3)                           # motion while latched -> zero only
            snd.ack()
            t1 = time.monotonic()
            assert wait(lambda: any(t > t1 for t, _ in mgr.twists), 1.0)
            assert all(x == 0.0 for t, x in mgr.twists if t > t1)
            assert wait(lambda: json.load(open(env / "estop.json"))["r3"]["confirmed"] is True, 3.0)
            latch = json.load(open(env / "estop.json"))
            assert latch["reason"] == "link" and (os.stat(env / "estop.json").st_mode & 0o777) == 0o600
        finally:
            p.terminate()
            p.wait(timeout=10)
        if trial == 0:                                 # restart with the latch present: still latched
            p = start_bridge(env, 19880)
            try:
                snd = Sender(19880)
                acks = []
                assert wait(lambda: (snd.send(0.2), acks.append(snd.ack()))[1] is None and acks[-1] is not None, 15)
                assert acks[-1]["estopped"] is True and acks[-1]["bridge"]["estop_reason"] == "link"
            finally:
                p.terminate()
                p.wait(timeout=10)
        os.remove(env / "estop.json")                 # = fcctl ready --confirm (re-arm)
    print("\nLINK E-STOP LATENCY (ms, packet sent -> received at the fake manager, 5 trials): "
          "/emergency_stop median %.1f max %.1f; SwitchController median %.1f max %.1f"
          % (statistics.median(lat_es), max(lat_es), statistics.median(lat_sw), max(lat_sw)))
    assert max(lat_es) < 50 and max(lat_sw) < 100


def test_fallen_cutoff(mgr, env):
    mgr.states["neural_controller_default"] = "active"
    p = start_bridge(env, 19890)
    try:
        snd = Sender(19890)
        warm(snd, mgr, env / "bridge-19890.log")
        n_sw = len(mgr.switches)

        def imu(tilt_deg, seconds):
            s, c = math.sin(math.radians(tilt_deg) / 2), math.cos(math.radians(tilt_deg) / 2)
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                m = Imu()
                m.orientation.x, m.orientation.w = s, c
                mgr.imu.publish(m)
                time.sleep(0.01)
        imu(30.0, 1.0)                                 # leaning, not fallen
        imu(60.0, 0.3)                                 # a short knock-over spike
        imu(10.0, 0.2)
        assert len(mgr.switches) == n_sw
        t0 = time.monotonic()
        imu(70.0, 1.0)
        assert len(mgr.switches) == n_sw + 1, open(env / "bridge-19890.log").read()
        dt = mgr.switches[n_sw][0] - t0
        assert 0.45 < dt < 0.7, dt
        assert wait(lambda: json.load(open(env / "estop.json")).get("reason") == "fallen", 2.0)
        snd.send(0.2)
        a = snd.ack()
        assert a["estopped"] is True and a["bridge"]["estop_reason"] == "fallen"
        print("\nFALLEN CUTOFF: SwitchController %.0f ms after the tilt passed 45 deg (hold 500 ms)" % (dt * 1000))
    finally:
        p.terminate()
        p.wait(timeout=10)


def test_park_keeps_the_link_estop_armed_during_the_lie_down(mgr, env):
    """fcctl down parks the bridge (SIGUSR1), hands the legs to the lie-down controller and polls the latch:
    motion is never published again, and a link E-STOP deactivates the lie-down controller mid-blend."""
    import signal
    mgr.states.update({"neural_controller_default": "active", "neural_controller_liedown": "inactive"})
    p = start_bridge(env, 19895)
    log = env / "bridge-19895.log"
    try:
        snd = Sender(19895)
        warm(snd, mgr, log)
        p.send_signal(signal.SIGUSR1)
        assert wait(lambda: "PARKED pid=%d" % p.pid in open(log).read(), 2.0), open(log).read()
        snd.send(vx=0.3)
        a = snd.ack()
        assert a["hold"] == "parked" and a["bridge"]["parked"] is True
        t1 = time.monotonic()
        assert wait(lambda: any(t > t1 for t, _ in mgr.twists), 1.0)
        assert all(x == 0.0 for t, x in mgr.twists if t > t1)
        # the STRICT handover fcctl does: walker off, lie-down on (the bridge sees it at its next 2 Hz poll)
        mgr.states.update({"neural_controller_default": "inactive", "neural_controller_liedown": "active"})
        time.sleep(1.2)
        n_sw = len(mgr.switches)
        snd.send(estop=True)
        assert wait(lambda: len(mgr.switches) > n_sw, 2.0), open(log).read()
        assert "neural_controller_liedown" in mgr.switches[n_sw][1]
        assert wait(lambda: mgr.states["neural_controller_liedown"] == "inactive", 1.0)
        assert wait(lambda: os.path.exists(env / "estop.json"), 2.0)    # what fcctl's lowering wait polls
        assert json.load(open(env / "estop.json"))["reason"] == "link"
    finally:
        p.terminate()
        p.wait(timeout=10)
        if os.path.exists(env / "estop.json"):
            os.remove(env / "estop.json")


def _frozen_imu(mgr, seconds, hz=250):
    """Messages keep arriving, values bit-identical (the RCA 2026-11-24 freeze)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        m = Imu()
        m.orientation.w = 1.0
        m.linear_acceleration.z = 9.81
        mgr.imu.publish(m)
        time.sleep(1.0 / hz)


def test_imu_freeze_lies_down_then_legs_off_and_latches(mgr, env):
    mgr.states.update({"neural_controller_default": "active", "neural_controller_liedown": "inactive"})
    p = start_bridge(env, 19897, ("--liedown", "neural_controller_liedown", "--liedown-wait", "1.0"))
    log = env / "bridge-19897.log"
    pub = None
    try:
        snd = Sender(19897)
        warm(snd, mgr, log)
        n_sw = len(mgr.switches)
        t0 = time.monotonic()
        pub = threading.Thread(target=_frozen_imu, args=(mgr, 3.5), daemon=True)
        pub.start()
        assert wait(lambda: len(mgr.switches) > n_sw, 3.0), open(log).read()
        t_sw, off, strict, on = mgr.switches[n_sw]
        assert on == ["neural_controller_liedown"] and off == ["neural_controller_default"], mgr.switches[n_sw]
        assert strict == cm.SwitchController.Request.STRICT
        assert 1.1 < t_sw - t0 < 1.8, t_sw - t0                 # frozen 0.5 s -> bad, + 0.75 s sustained
        snd.send(0.3)
        a = snd.ack()
        assert a["estopped"] is True and a["imu_lie"]["phase"] in ("switching", "lowering")
        assert a["bridge"]["estop_reason"] == "imu_frozen"
        assert json.load(open(env / "estop.json"))["reason"] == "imu_frozen"   # latched at once
        assert not [t for t in mgr.estops if t > t0]           # no /emergency_stop during the gentle lowering
        assert wait(lambda: len(mgr.switches) > n_sw + 1, 3.0), open(log).read()
        t_off, off2, _, on2 = mgr.switches[n_sw + 1]           # then legs off, lie-down included
        assert "neural_controller_liedown" in off2 and not on2 and 0.9 < t_off - t_sw < 1.6, (t_off - t_sw)
        assert wait(lambda: mgr.states["neural_controller_liedown"] == "inactive", 1.0)
        assert wait(lambda: (json.load(open(env / "estop.json")).get("r3") or {}).get("confirmed"), 3.0)
        all_twists = [x for t, x in mgr.twists if t > t_sw + 0.05]
        assert all(x == 0.0 for x in all_twists)
        print("\nIMU LIE: switch %.0f ms after the freeze began; legs off %.0f ms later"
              % ((t_sw - t0) * 1000, (t_off - t_sw) * 1000))
    finally:
        if pub is not None:
            pub.join(timeout=5)                                 # no IMU traffic into the next test's bridge
        p.terminate()
        p.wait(timeout=10)
        if os.path.exists(env / "estop.json"):
            os.remove(env / "estop.json")


def test_imu_freeze_without_liedown_does_r3_at_once(mgr, env):
    mgr.states.update({"neural_controller_default": "active", "neural_controller_liedown": "inactive"})
    p = start_bridge(env, 19898)
    log = env / "bridge-19898.log"
    pub = None
    try:
        snd = Sender(19898)
        warm(snd, mgr, log)
        n_sw = len(mgr.switches)
        t0 = time.monotonic()
        pub = threading.Thread(target=_frozen_imu, args=(mgr, 2.5), daemon=True)
        pub.start()
        assert wait(lambda: len(mgr.switches) > n_sw, 3.0), open(log).read()
        _, off, strict, on = mgr.switches[n_sw]
        assert not on and "neural_controller_default" in off and strict == cm.SwitchController.Request.BEST_EFFORT
        assert [t for t in mgr.estops if t > t0]               # the R3 action: /emergency_stop too
        assert json.load(open(env / "estop.json"))["reason"] == "imu_frozen"
    finally:
        if pub is not None:
            pub.join(timeout=5)                                 # no IMU traffic into the next test's bridge
        p.terminate()
        p.wait(timeout=10)
        if os.path.exists(env / "estop.json"):
            os.remove(env / "estop.json")
