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
    def inspect(run):
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
        "rclpy":NS(init=lambda **kw:None,create_node=lambda *a,**kw:node,spin_once=lambda *a,**kw:None,
                   ok=lambda:True,try_shutdown=lambda:None),
        "rclpy.qos":NS(QoSProfile=NS,ReliabilityPolicy=NS(BEST_EFFORT=1)),
        "rclpy.signals":NS(SignalHandlerOptions=NS(NO=0)),
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


def test_termination_after_start_submits_stop_on_the_live_node_and_restores_handlers(tmp_path, monkeypatch, capsys):
    import signal
    import software_start

    owner = "b" * 32
    (tmp_path / "selected-session.json").write_text(json.dumps({"session_id":"current-run"}))
    actions = []
    gateway = NS(state=dict(instance_id="instance", epoch=1, sequence=3, fc_operator_control=True,
                            fc_operator_owner=owner, latched=True, **dict.fromkeys(software_start.BLOCKED,False)),
                 observed_at=time.monotonic(), stop_publisher=NS(get_subscription_count=lambda:1),
                 request_stop=lambda reason:actions.append("request Stop: " + reason),
                 poll_stop=lambda:actions.append("acknowledge Stop") or dict(confirmed=True, gateway_epoch=2))
    monkeypatch.setattr(fc_operator, "board_error", lambda run: None)
    monkeypatch.setattr(software_start, "attach", lambda *a,**kw:gateway)
    pending = NS(done=lambda:False)
    client = NS(service_is_ready=lambda:True, call_async=lambda _:actions.append("Start") or pending)
    node = NS(create_publisher=lambda *a:NS(get_subscription_count=lambda:1),create_client=lambda *a:client,
              destroy_node=lambda:None)
    def spin_once(*a, **kw):
        # The rclpy context stays valid: no rclpy signal handler may shut it down first.
        if actions == ["Start"]:
            actions.append("SIGTERM")
            os.kill(os.getpid(), signal.SIGTERM)
    init = []
    for name, module in {
        "rclpy":NS(init=lambda **kw:init.append(kw),create_node=lambda *a,**kw:node,spin_once=spin_once,
                   ok=lambda:True,try_shutdown=lambda:None),
        "rclpy.qos":NS(QoSProfile=NS,ReliabilityPolicy=NS(BEST_EFFORT=1)),
        "rclpy.signals":NS(SignalHandlerOptions=NS(NO="no handlers")),
        "controller_manager_msgs.srv":NS(ListControllers=object),
        "joy_utils.srv":NS(FcStartMotion=NS(Request=NS)),
        "std_msgs.msg":NS(String=str),
    }.items():
        monkeypatch.setitem(sys.modules,name,module)
    before = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd) as inp, os.fdopen(write_fd,"w") as out:
        monkeypatch.setattr(sys,"stdin",inp)
        out.write(json.dumps(dict(action="stand", supported=True, challenge="instance 1 3")) + "\n")
        out.flush()
        with pytest.raises(KeyboardInterrupt):
            fc_operator.serve(tmp_path,"pupper-1",owner)
    assert init == [{"signal_handler_options": "no handlers"}]
    assert actions[:3] == ["Start", "SIGTERM", "request Stop: KeyboardInterrupt"] and actions[3] == "acknowledge Stop"
    terminal = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert terminal["event"] == "operator_stop" and terminal["confirmed"]
    assert {sig: signal.getsignal(sig) for sig in before} == before
