"""Exercise Pi operator terminal reporting with a memory-only gateway."""
import json
import os
import sys
import time
from types import SimpleNamespace as NS

import pytest

import fc_operator


def test_board_error_only_uses_current_activation_and_native_driver(tmp_path):
    native = "[ros2_control_node-2] [{}] [1795216388.020664228] [ControlBoardHardwareInterface]: {}\n"
    log = tmp_path / "robot.log"
    log.write_text(native.format("ERROR", "older fault") + native.format("INFO", "Successfully activated!") +
                   "[ros2_control_node-2] [ERROR] [1.0] [controller_manager]: other diagnostic\n")
    assert fc_operator.board_error(tmp_path) is None
    with log.open("a") as out:
        out.write(native.format("ERROR", "IMU fault phase=read reason=invalid or discontinuous joint exchange "
                                "sampled_mono_us=3250071194 quaternion={seq=42 valid=1}"))
    assert fc_operator.board_error(tmp_path) == "IMU fault phase=read reason=invalid or discontinuous joint exchange"


def test_failed_stand_stops_before_inspecting_board_and_retains_native_cause(tmp_path, monkeypatch, capsys):
    import software_start

    owner = "a" * 32
    (tmp_path / "selected-session.json").write_text(json.dumps({"session_id":"current-run"}))
    actions = []
    def inspect(run, *, stand_since):
        assert stand_since is not None
        assert actions == ["request Stop", "acknowledge Stop"]
        return "IMU fault phase=read reason=invalid or discontinuous joint exchange"

    gateway = NS(state=dict(instance_id="instance", epoch=1, sequence=3, fc_operator_control=True,
                            fc_operator_owner=owner, latched=True, **dict.fromkeys(software_start.BLOCKED,False)),
                 observed_at=time.monotonic(), stop_publisher=NS(get_subscription_count=lambda:1),
                 request_stop=lambda _:actions.append("request Stop"),
                 poll_stop=lambda:actions.append("acknowledge Stop") or dict(confirmed=True, gateway_epoch=2))
    monkeypatch.setattr(fc_operator, "board_error", inspect)
    monkeypatch.setattr(software_start, "attach", lambda *a,**kw:gateway)
    future = NS(done=lambda:True, result=lambda:NS(success=False, message="switch refused by controller manager"))
    client = NS(service_is_ready=lambda:True, call_async=lambda _:future)
    node = NS(create_publisher=lambda *a:NS(get_subscription_count=lambda:1),create_client=lambda *a:client,
              destroy_node=lambda:None)
    for name, module in {
        "rclpy":NS(init=lambda:None,create_node=lambda *a,**kw:node,spin_once=lambda *a,**kw:None,
                   ok=lambda:True,try_shutdown=lambda:None),
        "rclpy.qos":NS(QoSProfile=NS,ReliabilityPolicy=NS(BEST_EFFORT=1)),
        "controller_manager_msgs.srv":NS(ListControllers=object),
        "joy_utils.srv":NS(FcStartMotion=NS(Request=NS)),
        "std_msgs.msg":NS(String=str),
    }.items():
        monkeypatch.setitem(sys.modules,name,module)
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd) as inp, os.fdopen(write_fd,"w") as out:
        monkeypatch.setattr(sys,"stdin",inp)
        out.write(json.dumps(dict(action="stand", supported=True, challenge="instance 1 3")) + "\n")
        out.flush()
        with pytest.raises(ValueError,match="switch refused"):
            fc_operator.serve(tmp_path,"pupper-1",owner)
    terminal = json.loads(capsys.readouterr().out.splitlines()[-1])
    disk = json.loads((tmp_path / "fc-operator-failure.json").read_text())
    assert terminal["confirmed"] and terminal["reason"] == disk["reason"]
    assert "switch refused by controller manager" in terminal["reason"]
    assert "invalid or discontinuous joint exchange" in terminal["reason"]
    assert disk["session_id"] == "current-run" and disk["phase"] == "stand" and disk["gateway_epoch"] == 2
    assert not (tmp_path / "fc-operator-standing.json").exists()


def test_joint_warning_is_scoped_to_claim_or_matching_terminal_pair(tmp_path):
    log = tmp_path / "robot.log"
    def row(level, when, msg):
        return f"[ros2_control_node-2] [{level}] [{when:.9f}] [ControlBoardHardwareInterface]: {msg}\n"
    sample = "FC joint feedback phase=unclaimed predicate=position_rad joint=knee observed=.616 min=-2.69 max=.61"
    gate = "Supported Home retained; a fresh leg claim requires qualified primary and alternate IMU agreement and current, in-range joint feedback"
    issue = "IMU fault phase=read reason=position_rad"
    warning = row("WARN", 4.0, sample)
    log.write_text(warning + row("WARN", 4.019, gate))
    assert sample in fc_operator.board_error(tmp_path, stand_since=3.98)
    assert fc_operator.board_error(tmp_path, stand_since=4.01) == gate
    assert fc_operator.board_error(tmp_path) is None
    log.write_text(warning + row("WARN", 5., gate))
    assert fc_operator.board_error(tmp_path, stand_since=3.98) == gate
    post = sample.replace("unclaimed", "after_exchange")
    log.write_text(warning + row("ERROR", 6., issue + " sampled_mono_us=123") + row("WARN", 6.001, post))
    assert fc_operator.board_error(tmp_path) == issue + ". " + post
    unrelated = "IMU fault phase=read reason=stale quaternion receipt"
    log.write_text(warning + row("ERROR", 6., unrelated) + row("WARN", 6.001, post))
    assert fc_operator.board_error(tmp_path) == unrelated
