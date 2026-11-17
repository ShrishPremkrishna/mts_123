"""Minimal stand-ins for rclpy and the message/service types the kick nodes use.

Installed into ``sys.modules`` by ``install()`` so the nodes import unchanged on
a machine without ROS. Publishes, service calls and controller switches are
recorded; ``FakeRos`` drives the clock, timers and subscriptions. This checks
the nodes' logic and wiring, not ROS transport or timing.
"""

from __future__ import annotations

import sys
import types
from collections import defaultdict
from dataclasses import dataclass, field


class _Msg:
  def __init__(self, **kwargs):
    for k, v in kwargs.items():
      setattr(self, k, v)


def _vec():
  return _Msg(x=0.0, y=0.0, z=0.0)


class Twist(_Msg):
  def __init__(self):
    super().__init__(linear=_vec(), angular=_vec())


class Header(_Msg):
  def __init__(self, t=0.0):
    sec = int(t)
    super().__init__(stamp=_Msg(sec=sec, nanosec=int(round((t - sec) * 1e9))), frame_id="")


class PointStamped(_Msg):
  def __init__(self, x=0.0, y=0.0, t=0.0):
    super().__init__(header=Header(t), point=_Msg(x=x, y=y, z=0.0))


def _quat(yaw):
  import math

  return _Msg(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class PoseStamped(_Msg):
  def __init__(self, x=0.0, y=0.0, yaw=0.0, t=0.0):
    super().__init__(header=Header(t), pose=_Msg(position=_Msg(x=x, y=y, z=0.0), orientation=_quat(yaw)))


class Imu(_Msg):
  def __init__(self, yaw=0.0):
    super().__init__(orientation=_quat(yaw))


class Joy(_Msg):
  def __init__(self, buttons=()):
    super().__init__(buttons=list(buttons), axes=[])


class String(_Msg):
  def __init__(self, data=""):
    super().__init__(data=data)


class Empty(_Msg):
  pass


class Trigger:
  class Request(_Msg):
    pass

  class Response(_Msg):
    def __init__(self):
      super().__init__(success=False, message="")


class ListControllers:
  class Request(_Msg):
    pass


class GetParameters:
  class Request(_Msg):
    pass


class ParameterType:
  PARAMETER_NOT_SET = 0
  PARAMETER_BOOL = 1
  PARAMETER_INTEGER = 2
  PARAMETER_DOUBLE = 3
  PARAMETER_STRING = 4


def parameter_value(value):
  kind = {bool: 1, int: 2, float: 3, str: 4}.get(type(value), 0)
  return _Msg(type=kind, bool_value=value if kind == 1 else False,
              double_value=value if kind == 3 else 0.0,
              string_value=value if kind == 4 else "")


class SwitchController:
  class Request(_Msg):
    BEST_EFFORT = 1
    STRICT = 2

    def __init__(self):
      super().__init__(activate_controllers=[], deactivate_controllers=[], strictness=0,
                       activate_asap=False)


class Future:
  """Resolved by FakeRos.tick(), like a real executor running done callbacks later."""

  def __init__(self, compute):
    self._compute, self._callbacks, self._result, self._done = compute, [], None, False
    self.cancelled = False
    self._error = None

  def done(self):
    return self._done

  def cancel(self):
    self.cancelled = self._done = True

  def add_done_callback(self, cb):
    self._callbacks.append(cb)

  def result(self):
    if self._error:
      raise self._error
    return self._result

  def resolve(self):
    if self.cancelled:
      return
    try:
      self._result = self._compute()
    except Exception as exc:
      self._error = exc
    self._done = True
    for cb in self._callbacks:
      cb(self)


@dataclass
class FakeRos:
  t: float = 1000.0
  published: dict = field(default_factory=lambda: defaultdict(list))
  subs: dict = field(default_factory=lambda: defaultdict(list))
  services: dict = field(default_factory=dict)
  switch_calls: list = field(default_factory=list)
  timers: list = field(default_factory=list)
  pending: list = field(default_factory=list)
  publishers: dict = field(default_factory=lambda: defaultdict(int))
  # Scripted robot side: controller states and whether the gateway accepts switches.
  controllers: dict = field(default_factory=lambda: {
    "neural_controller_default": "active", "neural_controller_student": "inactive"})
  switch_ok: bool = True
  switch_answers: bool = True  # False: switch requests are never answered
  unavailable: set = field(default_factory=set)
  held_services: set = field(default_factory=set)
  held: list = field(default_factory=list)
  controller_types: dict = field(default_factory=dict)
  controller_params: dict = field(default_factory=lambda: {
    "max_vx_forward": 0.49, "max_vx_backward": 0.3, "speed_cap": 0.0,
    "game_buttons": False, "model_path": ""})

  def list_response(self):
    return _Msg(controller=[_Msg(name=n, state=st,
                  type=self.controller_types.get(n, "neural_controller/NeuralController"),
                  claimed_interfaces=[]) for n, st in self.controllers.items()])

  def params_response(self, req):
    return _Msg(values=[parameter_value(self.controller_params.get(n)) for n in req.names])

  def switch_response(self, req):
    self.switch_calls.append(req)
    if self.switch_ok:
      for n in req.deactivate_controllers:
        self.controllers[n] = "inactive"
      for n in req.activate_controllers:
        self.controllers[n] = "active"
    return _Msg(ok=self.switch_ok)

  def publish(self, topic, msg):
    for cb in self.subs[topic]:
      cb(msg)

  def call(self, name):
    return self.services[name](Trigger.Request(), Trigger.Response())

  def tick(self, n=1):
    for _ in range(n):
      pending, self.pending = self.pending, []
      for future in pending:
        future.resolve()
      period = self.timers[0][0]
      self.t += period
      for _, cb in self.timers:
        cb()


ROS = FakeRos()


class _Param:
  def __init__(self, value):
    self.value = value


class Parameter:
  def __init__(self, name, type_=None, value=None):
    self.name, self.value = name, value


class Node:
  OVERRIDES: dict = {}  # parameter values "given on the command line"

  def __init__(self, name):
    self._params = {}

  def declare_parameter(self, name, value):
    self._params.setdefault(name, Node.OVERRIDES.get(name, value))

  def get_parameter(self, name):
    return _Param(self._params[name])

  def set_parameters(self, params):
    for p in params:
      self._params[p.name] = p.value

  def create_publisher(self, msg_type, topic, qos):
    topic = topic if topic.startswith("/") else "/" + topic
    ROS.publishers[topic] += 1
    return _Msg(publish=lambda msg, topic=topic: ROS.published[topic].append(msg))

  def count_publishers(self, topic):
    return ROS.publishers[topic]

  def create_subscription(self, msg_type, topic, cb, qos):
    ROS.subs[topic].append(cb)

  def create_service(self, srv_type, name, cb):
    ROS.services[name] = cb

  def create_client(self, srv_type, name):
    def call_async(req, srv_type=srv_type):
      if srv_type is ListControllers:
        future = Future(ROS.list_response)
      elif srv_type is GetParameters:
        future = Future(lambda: ROS.params_response(req))
      else:
        future = Future(lambda req=req: ROS.switch_response(req))
        if not ROS.switch_answers:
          return future  # lost: never resolved
      if name in ROS.held_services:
        ROS.held.append(future)
      else:
        ROS.pending.append(future)
      return future

    def remove(future):
      if future in ROS.pending:
        ROS.pending.remove(future)
    return _Msg(service_is_ready=lambda: name not in ROS.unavailable,
                call_async=call_async, remove_pending_request=remove)

  def create_timer(self, period, cb):
    ROS.timers.append((period, cb))

  def get_clock(self):
    return _Msg(now=lambda: _Msg(nanoseconds=int(round(ROS.t * 1e9))))

  def get_logger(self):
    log = lambda *a, **k: None  # noqa: E731
    return _Msg(info=log, warning=log, error=log)

  def destroy_node(self):
    pass


def install(overrides: dict | None = None) -> FakeRos:
  """Reset the fake world and register the stand-in modules."""
  global ROS
  ROS = FakeRos()
  mods = {
    "rclpy": dict(init=lambda **k: None, ok=lambda: True, shutdown=lambda: None,
                  try_shutdown=lambda: None, spin=lambda n: None),
    "rclpy.node": dict(Node=Node),
    "rclpy.parameter": dict(Parameter=Parameter),
    "rclpy.executors": dict(ExternalShutdownException=RuntimeError),
    "rclpy.signals": dict(SignalHandlerOptions=_Msg(NO=0)),
    "geometry_msgs.msg": dict(Twist=Twist, PointStamped=PointStamped, PoseStamped=PoseStamped),
    "std_msgs.msg": dict(String=String, Empty=Empty),
    "std_srvs.srv": dict(Trigger=Trigger),
    "sensor_msgs.msg": dict(Imu=Imu, Joy=Joy),
    "controller_manager_msgs.srv": dict(SwitchController=SwitchController, ListControllers=ListControllers),
    "rcl_interfaces.srv": dict(GetParameters=GetParameters),
    "rcl_interfaces.msg": dict(ParameterType=ParameterType),
  }
  for name in ("rclpy", "geometry_msgs", "std_msgs", "std_srvs", "sensor_msgs", "controller_manager_msgs", "rcl_interfaces"):
    sys.modules[name] = types.ModuleType(name)
  for name, attrs in mods.items():
    module = types.ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module
  sys.modules["rclpy"].node = sys.modules["rclpy.node"]
  Node.OVERRIDES = dict(overrides or {})
  return ROS
