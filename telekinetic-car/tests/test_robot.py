"""Exercise the ROS sink's safety decisions using a clock and in-memory endpoints."""

import json
from types import SimpleNamespace as NS

import pytest

from gemini_pupper.robot.stop_status import StopStatus
from final_project.behaviors.tools.movement_trial import applied_command
from pupper_bci import robot


def endpoint(name, topic_type, gid):
    return NS(node_name=name, node_namespace="/", topic_type=topic_type, endpoint_gid=[gid])


class Harness:
    def __init__(self):
        self.now = 10.0
        self.sequence = 0
        self.sent = []
        self.process_alive = True
        self.stock_problem = None
        pubs = {
            robot.INPUT: [endpoint(robot.NODE, robot.TWIST_TYPE, 1)],
            robot.OUTPUT: [endpoint("cmd_vel_mux", robot.TWIST_TYPE, 2)],
            robot.JOY: [endpoint("joy_node", "sensor_msgs/msg/Joy", 3)],
            robot.GATEWAY: [endpoint("estop_controller", "std_msgs/msg/String", 4)],
            robot.OBSERVATION: [endpoint(robot.WALKER, "std_msgs/msg/Float32MultiArray", 5)],
        }
        subs = {
            robot.INPUT: [endpoint("cmd_vel_mux", robot.TWIST_TYPE, 6)],
            robot.OUTPUT: [endpoint(robot.WALKER, robot.TWIST_TYPE, 7)],
        }
        self.pubs, self.subs = pubs, subs
        node = NS(get_publishers_info_by_topic=pubs.get, get_subscriptions_info_by_topic=subs.get,
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=round(self.now * 1e9))))
        stock = NS(poll=lambda: None, problem=lambda **_: self.stock_problem)
        self.sink = robot.RobotSink(node, NS(spin_once=lambda **_: None), NS(publish=self.sent.append),
                                   lambda: NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0)),
                                   stock, StopStatus(), applied_command, lambda: self.process_alive,
                                   enable_button=4, clock=lambda: self.now)

    def gateway(self, **overrides):
        self.sequence += 1
        status = dict(protocol="pupper_stop_v1", instance_id="a" * 32,
                      sequence=self.sequence, epoch=1, latched=False, fault=False,
                      stop_required=False, in_flight=False, unknown_remote_submission=False,
                      shutting_down=False)
        status.update(overrides)
        self.sink.on_gateway(NS(data=json.dumps(status)))

    def joy(self, held=False, *, age=0.0, r3=False):
        buttons = [0] * 13
        buttons[4], buttons[12] = int(held), int(r3)
        sec, nsec = divmod(round((self.now - age) * 1e9), 1_000_000_000)
        self.sink.on_joy(NS(buttons=buttons, header=NS(stamp=NS(sec=sec, nanosec=nsec))))

    def observation(self, forward=0.0):
        data = [0.0] * 960
        data[6] = forward
        self.sink.on_observation(NS(data=data))

    def tick(self, *, joy=False, observation=0.0, step=0.05):
        self.now += step
        self.gateway()
        if joy is not None:
            self.joy(joy)
        self.observation(observation)
        return self.sink.poll()

    def discover(self, *, held=False):
        assert not self.tick(joy=held)
        assert not self.tick(joy=held, step=robot.QUIET_PERIOD + 0.01)

    def engage(self):
        self.discover()
        assert self.tick(joy=True)
        self.sink.publish(0.15, -0.35)


def test_initial_held_button_requires_release_and_continuing_motion_needs_one_poll_per_command():
    fake = Harness()
    fake.discover(held=True)
    assert not fake.tick(joy=True)
    assert fake.sent == []
    assert not fake.tick(joy=False)
    assert fake.tick(joy=True)
    fake.sink.publish(0.15, 0.35)
    assert (fake.sent[-1].linear.x, fake.sent[-1].linear.y, fake.sent[-1].angular.z) == (0.15, 0.0, 0.35)
    with pytest.raises(RuntimeError, match="recent held-enable poll"):
        fake.sink.publish(0.15, 0.0)
    assert fake.sent[-1].linear.x == fake.sent[-1].angular.z == 0


def test_release_is_immediate_and_repress_awaits_zero_feedback():
    fake = Harness()
    fake.engage()
    fake.now += 0.01
    fake.joy(False)
    assert fake.sent[-1].linear.x == fake.sent[-1].angular.z == 0
    assert not fake.tick(joy=True, observation=0.15)
    assert fake.tick(joy=True)
    fake.sink.publish(0.10, 0.0)
    assert fake.sink.close()
    assert fake.sent[-1].linear.x == 0
    assert not fake.sink.close()


def test_stale_joy_drops_enable_and_held_reconnect_cannot_rearm():
    fake = Harness()
    fake.engage()
    assert not fake.tick(joy=None, step=robot.JOY_AGE + 0.01)
    assert fake.sent[-1].linear.x == 0
    assert not fake.tick(joy=True)
    assert not fake.tick(joy=False)
    assert fake.tick(joy=True)


def test_old_joy_delivery_only_gets_remaining_stamp_age():
    fake = Harness()
    fake.discover()
    fake.now += 0.14
    fake.gateway()
    fake.observation()
    fake.joy(True, age=0.13)
    assert fake.sink.poll()
    fake.now += 0.025
    with pytest.raises(RuntimeError, match="recent held-enable poll"):
        fake.sink.publish(0.1, 0.0)
    assert fake.sent == []


@pytest.mark.parametrize("loss,match", [
    ("gateway", "epoch changed"), ("invalid_gateway", "malformed"),
    ("stale_gateway", "status is stale"), ("stop", "operator Stop"),
    ("observation", "observation is missing"), ("runtime", "runtime changed"),
    ("stock", "stock lost"), ("final_topic", "competing ROS endpoints"),
])
def test_authority_loss_latches_and_zeroes_while_owning_private_input(loss, match):
    fake = Harness()
    fake.engage()
    fake.now += robot.GATEWAY_AGE + 0.01
    if loss != "stale_gateway":
        fake.gateway(epoch=2 if loss == "gateway" else 1)
    if loss == "invalid_gateway":
        fake.sink.on_gateway(NS(data="malformed"))
        fake.gateway()
    if loss != "observation":
        fake.observation()
    fake.joy(True, r3=loss == "stop")
    fake.process_alive = loss != "runtime"
    fake.stock_problem = "stock lost" if loss == "stock" else None
    if loss == "final_topic":
        fake.pubs[robot.OUTPUT].append(endpoint("wrong", robot.TWIST_TYPE, 8))
    with pytest.raises(RuntimeError, match=match):
        fake.sink.poll()
    assert fake.sent[-1].linear.x == fake.sent[-1].angular.z == 0
    with pytest.raises(RuntimeError, match=match):
        fake.sink.poll()


def test_competing_private_input_means_no_cleanup_write():
    fake = Harness()
    fake.engage()
    fake.pubs[robot.INPUT].append(endpoint("other", robot.TWIST_TYPE, 8))
    with pytest.raises(RuntimeError, match="competing ROS endpoints"):
        fake.sink.poll()
    assert not fake.sink.close()
    assert len(fake.sent) == 1


def test_continuity_and_zero_feedback_precede_admission():
    fake = Harness()
    fake.stock_problem = "stock not active"
    fake.discover()
    assert fake.sink.waiting_reason == "stock not active"
    fake.stock_problem = None
    assert not fake.tick(observation=0.13, joy=True)
    assert "zero" in fake.sink.waiting_reason
    assert not fake.tick(joy=True)
    assert not fake.tick(joy=False)
    assert fake.tick(joy=True)


@pytest.mark.parametrize("forward,yaw", [(-0.01, 0.0), (0.16, 0.0), (0.1, 0.36),
                                             (float("nan"), 0.0), (0.0, float("inf"))])
def test_out_of_envelope_publish_fails_closed(forward, yaw):
    fake = Harness()
    fake.engage()
    with pytest.raises(RuntimeError, match="BCI walk limits"):
        fake.sink.publish(forward, yaw)
    assert fake.sent[-1].linear.x == fake.sent[-1].angular.z == 0


def test_idle_zero_and_close_emit_nothing():
    fake = Harness()
    fake.discover()
    fake.sink.publish(0.0, 0.0)
    fake.sink.close()
    assert fake.sent == []


def test_profile_rejects_long_mux_timeout_and_remapped_path():
    config = {
        "cmd_vel_mux": {"ros__parameters": {"inputs": [robot.INPUT], "timeout_ms": 150}},
        "estop_controller": {"ros__parameters": {
            "switch_button_indices": [0], "controller_names": [robot.WALKER],
            "estop_index": 12, "estop_release_index": 9,
        }},
        robot.WALKER: {"ros__parameters": {"game_buttons": False, "cmd_vel_timeout_s": 0.25, "speed_cap": 0.49}},
        "joy_node": {"ros__parameters": {"autorepeat_rate": 50.0}},
    }
    robot._check_config(config)
    mux = config["cmd_vel_mux"]["ros__parameters"]
    mux["timeout_ms"] = 500
    with pytest.raises(ValueError, match="150 ms"):
        robot._check_config(config)
    mux.update(timeout_ms=150, inputs=[robot.INPUT, "/teleop_cmd_vel"])
    with pytest.raises(ValueError, match="private mux"):
        robot._check_config(config)


@pytest.mark.parametrize("button", [0, 1, 2, 3, 9, 12, -1, 32, True])
def test_reserved_or_invalid_deadman_indices_are_rejected(button):
    with pytest.raises(ValueError, match="button index"):
        robot._check_enable_button(button)
