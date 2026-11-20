"""Offline operator controls and deadline checks for the FC guarded path."""
from types import SimpleNamespace

import pytest

from final_project.behaviors.fc_executor_bridge import COMBINED_LIMITS
from final_project.behaviors.operator_drive import OperatorDrive
from final_project.behaviors import combined_fc_runner as runner
from final_project.behaviors.test_combined_fc_runner import path_fixture
from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.model import Command


def controls(robot="pupper-1"):
    core = SimpleNamespace(executor=Executor(kick_side="left"), limits=COMBINED_LIMITS, robot_id=robot)
    result = OperatorDrive(core)
    result.observe_path(True, 1)
    return result


def send(control, now, action, *, delay=0, stable=True, **payload):
    token = control.offer(now)
    message = {**({"epoch": "keyboard-game"} if action == "drive" else {}), **payload, "command": action, "ticket": token["ticket"], "robot_id": token["robot_id"]}
    control.accept(message, now=now + delay, stable=stable)
    return message


def axes(forward=0, left=0, turn=0):
    return dict(forward=forward, left=left, turn=turn)


@pytest.mark.parametrize("robot", ["pupper-1", "pupper-2"])
def test_key_response_expires_from_robot_ticket_even_with_delayed_network(robot):
    c = controls(robot)
    assert c.mode == "autonomous" and c.command(1) is None
    send(c, 1, "mode", mode="teleop")
    c.match_epoch = "keyboard-game"
    assert not c.core.executor.armed and c.command(1).vx == 0
    send(c, 1.12, "drive", **axes())
    message = send(c, 1.23, "drive", delay=.19, **axes(forward=.6, left=.8, turn=-1))
    cmd = c.command(1.42)
    assert (cmd.vx, cmd.vy, cmd.wz) == pytest.approx((.6*.22, .8*.18, -.65))
    assert cmd.robot_id == robot and cmd.expires_at == pytest.approx(1.48)
    with pytest.raises(ValueError, match="consumed"):
        c.accept(message, now=1.43, stable=True)
    assert c.command(1.49).vx == 0


def test_foreign_failed_and_out_of_order_tickets_preserve_owner_and_latest_intent():
    c = controls()
    old, new = c.offer(1)["ticket"], c.offer(1.12)["ticket"]
    mode = {"command": "mode", "mode": "teleop", "ticket": new, "robot_id": "pupper-2"}
    with pytest.raises(ValueError, match="different robot"):
        c.accept(mode, now=1.14, stable=True)
    c.accept({**mode, "robot_id": "pupper-1"}, now=1.15, stable=True)
    with pytest.raises(ValueError, match="expired"):
        c.consume_ticket({"ticket":old, "robot_id":"pupper-1"}, 1.16)
    with pytest.raises(ValueError, match="expired"):
        send(c, 1.30, "drive", delay=.26, **axes(forward=1))
    with pytest.raises(ValueError, match="normalized"):
        send(c, 1.60, "drive", **axes(forward=1, left=1))


def test_mode_and_hold_require_new_keys_and_stable_walking_output():
    c = controls()
    with pytest.raises(ValueError, match="completed kick"):
        send(c, 1, "mode", mode="teleop", stable=False)
    send(c, 1.12, "mode", mode="teleop")
    c.match_epoch = "keyboard-game"
    with pytest.raises(ValueError, match="release"):
        send(c, 1.24, "drive", **axes(forward=1))
    send(c, 1.36, "drive", **axes())
    send(c, 1.48, "drive", **axes(forward=1))
    c.observe_output(c.command(1.48))
    c.observe_path(False, 1)
    assert c.command(1.51).vx == 0
    with pytest.raises(ValueError, match="zero walking"):
        send(c, 1.60, "mode", mode="autonomous")
    c.observe_path(True, 1)
    with pytest.raises(ValueError, match="release"):
        send(c, 1.75, "drive", **axes(forward=1))
    send(c, 1.87, "drive", **axes())
    send(c, 1.99, "mode", mode="autonomous")
    assert c.command(2) is None and not c.core.executor.armed


@pytest.mark.parametrize("expire_in_encoder", [True, False])
def test_camera_expiring_at_final_sink_publishes_zero_and_preserves_walker(expire_in_encoder):
    path, now, sent, *_ = path_fixture()
    cmd = Command(.1, 0, 0, 10.15)
    if expire_in_encoder:
        encode = path.core.deadline.encode
        def delayed(command):
            now[0] += .015
            return encode(command)
        path.core.deadline.encode = delayed
    else:
        now[0] += .015
    assert path.write(cmd)
    assert path.core.executor.external_holds["pupper-1"]
    assert len(sent) == 1 and runner.protocol.decode(sent[0]).is_zero
    path.native_error = "native binding was lost"
    with pytest.raises(RuntimeError, match="native binding"):
        path.write(cmd)
    assert len(sent) == 1


def test_operator_arm_requires_current_ticket_in_either_mode():
    path, now, *_ = path_fixture()
    c = OperatorDrive(path.core)
    c.observe_path(True, 2)
    gateway = SimpleNamespace(stop_capable=True, state={"instance_id": "i"},
                              stop_publisher=SimpleNamespace(get_subscription_count=lambda: 1))
    runtime = SimpleNamespace(robot=SimpleNamespace(gateway_stop_anchor=lambda: {"instance_id": "i"}))
    arm = {"command": "arm", "epoch": "match", "robot_id": path.core.robot_id}
    with pytest.raises(ValueError, match="expired"):
        runner.operator_action(arm, path.core, path, runtime, gateway, c)
    packet = {**arm, "ticket": c.offer(now[0])["ticket"]}
    assert runner.operator_action(packet, path.core, path, runtime, gateway, c)["accepted"]
    with pytest.raises(ValueError, match="consumed"):
        runner.operator_action(packet, path.core, path, runtime, gateway, c)
    now[0] += .12
    send(c, now[0], "mode", mode="teleop")
    now[0] += .12
    with pytest.raises(ValueError, match="kickoff"):
        send(c, now[0], "drive", **axes(forward=1))
    now[0] += .12
    path.check = lambda _: None
    arm["epoch"] = "keyboard-game"
    reply = runner.operator_action({**arm, "ticket": c.offer(now[0])["ticket"]}, path.core, path, runtime, gateway, c)
    assert reply["accepted"] and c.match_epoch == arm["epoch"] and not path.core.executor.armed
    now[0] += .12
    with pytest.raises(ValueError, match="kickoff"):
        send(c, now[0], "drive", epoch="old-game", **axes(forward=1))
    assert c.command(now[0]).vx == 0
