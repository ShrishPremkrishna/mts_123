"""heat.py: CPU governor/frequency, fan and top-process sampling from fake /proc and /sys trees. Pure."""
import os
import sys

ROBOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROBOT)
import heat  # noqa: E402


def w(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def stat(pid, comm, utime, stime):
    return "%d (%s) S 1 1 1 0 -1 4194560 0 0 0 0 %d %d 0 0 20 0 1 0 100 0 0\n" % (pid, comm, utime, stime)


def test_parse_stat_handles_spaces_and_parens():
    assert heat.parse_stat(stat(7, "a (b) c", 10, 5)) == ("a (b) c", 15)


def test_label():
    assert heat.label("camera_node", ["/opt/ros/jazzy/lib/camera_ros/camera_node", "--ros-args"]) == "camera_node"
    assert heat.label("python3", ["/usr/bin/python3", "-u", "/home/pi/x/drive_bridge.py", "--x"]) == "python3 drive_bridge.py"
    assert heat.label("kworker/0:1", []) == "kworker/0:1"


def test_top_rates_and_order():
    a = {1: (100, "a"), 2: (50, "b"), 3: (0, "c"), 4: (10, "gone")}
    b = {1: (150, "a"), 2: (60, "b"), 3: (0, "c"), 5: (99, "new")}
    rows = heat.top(a, b, 0.5, n=5, clk=100)
    assert rows == [{"pid": 1, "pct": 100.0, "cmd": "a"}, {"pid": 2, "pct": 20.0, "cmd": "b"}]
    assert heat.top(a, b, 0.5, n=1)[0]["pid"] == 1 and heat.top(a, b, 0.0) == []


def test_cpu_fan_and_snapshot(tmp_path):
    sysr, proc = str(tmp_path / "sys"), str(tmp_path / "proc")
    d = os.path.join(sysr, heat.CPU0)
    for k, v in (("scaling_governor", "ondemand"), ("scaling_cur_freq", "2400000"), ("scaling_min_freq", "1500000"),
                 ("scaling_max_freq", "2400000")):
        w(os.path.join(d, k), v + "\n")
    cpu = heat.cpu_info(sysr)
    assert cpu["governor"] == "ondemand" and (cpu["cur_mhz"], cpu["min_mhz"], cpu["max_mhz"]) == (2400, 1500, 2400)
    assert heat.fan_info(sysr) is None
    f = os.path.join(sysr, heat.FAN_DEV)
    for k, v in (("type", "pwm-fan"), ("cur_state", "4"), ("max_state", "4")):
        w(os.path.join(f, k), v)
    w(os.path.join(sysr, heat.FAN_HWMON, "hwmon3", "fan1_input"), "9765\n")
    assert heat.fan_info(sysr) == {"type": "pwm-fan", "state": 4, "max_state": 4, "rpm": 9765}
    w(os.path.join(proc, "12", "stat"), stat(12, "camera_node", 300, 20))
    w(os.path.join(proc, "12", "cmdline"), "/opt/ros/jazzy/lib/camera_ros/camera_node\0--ros-args\0")
    w(os.path.join(proc, "self", "stat"), "ignored")
    w(os.path.join(proc, "13", "stat"), "garbage")
    assert heat.snapshot(proc) == {12: (320, "camera_node")}
    assert heat.cpu_info(str(tmp_path / "none"))["governor"] is None


def test_sample_on_this_host_is_json_shaped():
    r = heat.sample(n=3, interval=0.05, sys_root="/nonexistent") if os.path.isdir("/proc") else None
    if r is not None:
        assert set(r) == {"cpu", "fan", "top", "top_interval_s"} and len(r["top"]) <= 3
