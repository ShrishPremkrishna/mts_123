"""rosq.py pure helpers: settle logic, topology problems, DDS participant count, missing-mux reason."""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import rosq  # noqa: E402

UDP = """  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode ref pointer drops
  1: 00000000:1B58 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 1 2 0 0
  2: 00000000:1D02 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 1 2 0 0
  3: 00000000:1B63 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 1 2 0 0
  4: 00000000:2694 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 1 2 0 0
"""


def test_dds_participant_ids_from_proc_net_udp():
    # 0x1B58 = 7000 (id 0), 0x1B63 = 7011 (id 11); 0x1D02 = 7426 (a well-known port), 0x2694 = 9876: ignored
    assert rosq.dds_participant_ids(UDP) == [0, 11]
    assert rosq.dds_participant_ids("header only\n") == []


def test_dds_summary_warns_near_the_initial_peers_limit():
    assert rosq.dds_summary([0, 1, 11]) == {"participants": 3, "max_id": 11, "warn": None}
    assert "too many ROS processes" in rosq.dds_summary(list(range(30)))["warn"]
    assert rosq.dds_summary([])["max_id"] is None


def test_topology_problems():
    assert rosq.topology(["cmd_vel_mux"], ["cmd_vel_mux"], ["fc_drive_bridge"], ["teleop_twist_joy_node"], "/t") == []
    p = rosq.topology([], [], [], [], "/t")
    assert p[0].startswith("/cmd_vel publishers []") and "not subscribed" in p[1]
    p = rosq.topology(["cmd_vel_mux", "x"], ["cmd_vel_mux"], ["fc_drive_bridge", "api"], ["a", "b"], "/t")
    assert len(p) == 3


def test_settle_waits_for_complete_and_quiet():
    s = rosq.Settle(min_s=2.0, quiet_s=1.5, max_s=8.0)
    assert not s.update(0.1, ("a",), False)
    assert not s.update(2.5, ("a",), False)          # incomplete: keep waiting
    assert not s.update(3.0, ("a", "mux"), True)     # complete but just changed
    assert not s.update(4.0, ("a", "mux"), True)
    assert s.update(4.6, ("a", "mux"), True)         # unchanged for 1.6 s


def test_settle_respects_min_and_gives_up_at_max():
    s = rosq.Settle(min_s=2.0, quiet_s=1.5, max_s=8.0)
    assert not s.update(0.0, ("m",), True)
    assert not s.update(1.9, ("m",), True)           # quiet long enough but before min_s
    assert s.update(2.0, ("m",), True)
    s = rosq.Settle(min_s=2.0, quiet_s=1.5, max_s=8.0)
    assert not s.update(7.9, ("a",), False)
    assert s.update(8.0, ("a",), False)


def test_missing_mux_reason():
    assert rosq.missing_mux_reason(["cmd_vel_mux"], True, 2.0) is None
    assert rosq.missing_mux_reason([], False, 8.0) == "no cmd_vel_mux process running"
    assert "not discovered after 8.0 s" in rosq.missing_mux_reason([], True, 8.0)
    assert "not discovered" in rosq.missing_mux_reason([], None, 8.0)


def test_mux_process_running(tmp_path):
    (tmp_path / "self").mkdir()
    for pid, argv in (("10", b"/usr/bin/python3\0x.py\0"), ("11", b"/ws/lib/cmd_vel_mux/cmd_vel_mux_node\0--ros-args\0")):
        (tmp_path / pid).mkdir()
        (tmp_path / pid / "cmdline").write_bytes(argv)
    assert rosq.mux_process_running(str(tmp_path)) is True
    (tmp_path / "11" / "cmdline").write_bytes(b"/opt/ros/jazzy/bin/ros2\0run\0cmd_vel_mux\0cmd_vel_mux_node\0")
    assert rosq.mux_process_running(str(tmp_path)) is False   # the `ros2 run` wrapper is not the node
    assert rosq.mux_process_running(str(tmp_path / "nope")) is None
