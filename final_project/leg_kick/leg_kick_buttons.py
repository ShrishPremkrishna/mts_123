"""Leg kick on the square slot (neural_controller_student), for robot trials.

Each SQUARE activation of the slot plays one kick, WAIT_S seconds after the
activation (the slot's init and fade-in take 2 s + 2 s). The node then hands the
legs back to the walker:

  * exit: EXIT_DELAY_S after the kick clip returns to stand_idle, a STRICT switch
    activates neural_controller_default and deactivates the slot. After
    SWITCH_RETRIES failed tries, the node publishes /emergency_stop.
  * watchdog: the trick policy trained only with its feet on the ground. When the
    robot is lifted, it oscillates (4 to 5.7 rad/s on all legs in the stand test).
    While the slot is active outside a kick clip, the mean over the last
    WATCH_WINDOW_S of the largest joint speed per sample above WATCH_RAD_S
    switches to the walker and cancels the kick.

Side: R1 or R2 selects the right kick, L1 or L2 the left kick (ds4 joy indices
4 L1, 5 R1, 6 L2, 7 R2). The node logs each button press.

    python3 leg_kick_buttons.py [right|left] [WAIT_S]
"""

from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass, field

WAIT_S = 5.5
EXIT_DELAY_S = 0.5
GAP_S = 0.5  # no clip status for this long: the slot is inactive
# Window mean of the per-sample max |joint velocity|. 2026-11-16 bags: outside kicks
# on the floor <= 0.4 rad/s; the stand-test shake 2.35 to 3.06 rad/s.
WATCH_RAD_S = 1.5
WATCH_WINDOW_S = 0.3
WATCH_GRACE_S = 1.0  # after a kick clip ends, settling is allowed
SWITCH_RETRIES = 2
RIGHT_BUTTONS, LEFT_BUTTONS = (5, 7), (4, 6)


@dataclass
class KickLogic:
  """ROS-free decisions. Feed it events with times in seconds; it returns actions:
  ("kick", side), ("exit", reason)."""

  side: str = "right"
  wait_s: float = WAIT_S
  active_since: float | None = None
  last_status_t: float = -1e9
  clip: str | None = None
  kick_sent: bool = False
  kick_ended_t: float | None = None
  exit_requested: bool = False
  window: deque = field(default_factory=deque)
  log: list = field(default_factory=list)

  def status(self, t: float, clip: str) -> list:
    out = []
    if t - self.last_status_t > GAP_S:  # a new activation
      self.active_since, self.kick_sent, self.kick_ended_t = t, False, None
      self.exit_requested, self.clip = False, None
      self.window.clear()
      self.log.append(f"slot active; kick_{self.side} in {self.wait_s} s")
    self.last_status_t = t
    if clip != self.clip:
      if self.clip and self.clip.startswith("kick") and clip == "stand_idle":
        self.kick_ended_t = t
      if self.clip is not None:
        self.log.append(f"clip {clip}")
      self.clip = clip
    return out

  def joints(self, t: float, max_speed: float) -> list:
    """max_speed: largest |joint velocity| in this /joint_states sample."""
    if not self.active() or self.exit_requested:
      return []
    in_kick = self.clip is not None and self.clip.startswith("kick")
    settling = self.kick_ended_t is not None and t - self.kick_ended_t < WATCH_GRACE_S
    if in_kick or settling:
      self.window.clear()
      return []
    self.window.append((t, max_speed))
    while self.window[0][0] < t - WATCH_WINDOW_S:
      self.window.popleft()
    if t - self.window[0][0] < 0.8 * WATCH_WINDOW_S:
      return []  # not a full window yet
    mean = sum(v for _, v in self.window) / len(self.window)
    if mean > WATCH_RAD_S:
      self.exit_requested = True
      self.kick_sent = True  # cancel a pending kick
      return [("exit", f"joint speed {mean:.1f} rad/s (window mean) outside a kick")]
    return []

  def tick(self, t: float) -> list:
    out = []
    if self.active_since is not None and t - self.last_status_t > GAP_S:
      self.log.append("slot inactive")
      self.active_since = None
      return out
    if not self.active() or self.exit_requested:
      return out
    if not self.kick_sent and t - self.active_since > self.wait_s:
      self.kick_sent = True
      out.append(("kick", self.side))
    if self.kick_ended_t is not None and t - self.kick_ended_t >= EXIT_DELAY_S:
      self.exit_requested = True
      out.append(("exit", "kick done"))
    return out

  def button(self, index: int) -> None:
    if index in RIGHT_BUTTONS + LEFT_BUTTONS:
      self.side = "right" if index in RIGHT_BUTTONS else "left"
      self.log.append(f"side -> {self.side}")

  def active(self) -> bool:
    return self.active_since is not None


def main() -> None:
  import rclpy
  from controller_manager_msgs.srv import SwitchController
  from rclpy.node import Node
  from sensor_msgs.msg import Joy, JointState
  from std_msgs.msg import Empty, String

  slot, walker = "neural_controller_student", "neural_controller_default"

  class LegKickNode(Node):
    def __init__(self, side: str, wait_s: float):
      super().__init__("leg_kick_buttons")
      self.logic = KickLogic(side=side, wait_s=wait_s)
      self.clip_pub = self.create_publisher(String, f"/{slot}/clip", 10)
      self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
      self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
      self.create_subscription(String, f"/{slot}/clip_status", self.on_status, 10)
      self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
      self.create_subscription(Joy, "/joy", self.on_joy, 10)
      self.create_timer(0.02, self.on_tick)
      self.buttons: list = []
      self.tries = 0
      self.say(f"ready: side {side}, kick {wait_s} s after each square activation; "
               f"exit {EXIT_DELAY_S} s after the kick; watchdog {WATCH_RAD_S} rad/s over {WATCH_WINDOW_S} s")

    def say(self, msg: str) -> None:
      print(time.strftime("%H:%M:%S"), msg, flush=True)

    def flush(self) -> None:
      for line in self.logic.log:
        self.say(line)
      self.logic.log.clear()

    def act(self, actions: list) -> None:
      for kind, arg in actions:
        if kind == "kick":
          self.clip_pub.publish(String(data=f"kick_{arg}"))
          self.say(f"sent kick_{arg}")
        elif kind == "exit":
          self.say(f"exit to {walker}: {arg}")
          self.tries = 0
          self.request_switch()
      self.flush()

    def request_switch(self) -> None:
      self.tries += 1
      if not self.switch.service_is_ready():
        return self.switch_failed("controller_manager switch service not ready")
      req = SwitchController.Request()
      req.activate_controllers = [walker]
      req.deactivate_controllers = [slot]
      req.strictness = SwitchController.Request.STRICT
      req.timeout.sec = 1
      self.switch.call_async(req).add_done_callback(self.on_switched)

    def on_switched(self, future) -> None:
      res = future.result()
      if res is not None and res.ok:
        self.say(f"switched to {walker}")
      else:
        self.switch_failed("switch refused")

    def switch_failed(self, why: str) -> None:
      self.say(f"{why} (try {self.tries})")
      if self.tries <= SWITCH_RETRIES:
        self.request_switch()
      else:
        self.estop_pub.publish(Empty())
        self.say("published /emergency_stop (R3 state; Options releases it)")

    def on_status(self, m) -> None:
      self.act(self.logic.status(time.time(), m.data.split(" ")[0]))

    def on_joints(self, m) -> None:
      if m.velocity:
        self.act(self.logic.joints(time.time(), max(abs(v) for v in m.velocity)))

    def on_joy(self, m) -> None:
      pressed = [i for i, b in enumerate(m.buttons)
                 if b and (i >= len(self.buttons) or not self.buttons[i])]
      self.buttons = list(m.buttons)
      for i in pressed:
        self.say(f"button {i}")
        self.logic.button(i)
      self.flush()

    def on_tick(self) -> None:
      self.act(self.logic.tick(time.time()))

  side = sys.argv[1] if len(sys.argv) > 1 else "right"
  wait_s = float(sys.argv[2]) if len(sys.argv) > 2 else WAIT_S
  rclpy.init()
  node = LegKickNode(side, wait_s)
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass


if __name__ == "__main__":
  main()
