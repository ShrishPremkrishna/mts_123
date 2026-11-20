"""Drive the production camera guard and selected native policies on loopback DDS."""
import json
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from final_project.behaviors import body_command_guard as guard
from final_project.behaviors import command_deadline as protocol
from final_project.behaviors import command_path_evidence as evidence
from final_project.behaviors import combined_fc_runner as runner
from final_project.behaviors.fc_executor_bridge import DeadlineEncoder
from final_project.behaviors.test_body_command_guard import Timeline
from final_project.pupper_fc.model import Command


def main():
    out = Path("/evidence")
    rclpy.init()
    gate = guard.create_ros_node(arena_ball_contact=True, command_contract="fc_deadline_v1")
    field = rclpy.create_node(runner.NAME)
    detector = rclpy.create_node("viser_camera")
    spin = SingleThreadedExecutor()
    for node in (gate, field, detector):
        spin.add_node(node)
    request = field.create_publisher(String, protocol.REQUEST_TOPIC, 1)
    camera = detector.create_publisher(String, guard.FRAME_TOPIC, 1)
    core = SimpleNamespace(clock=time.monotonic, executor=SimpleNamespace(armed=False),
                           deadline=DeadlineEncoder())
    path = runner.GuardedPath(core, field, lambda data: request.publish(String(data=data)))
    field.create_subscription(String, evidence.GUARD_STATUS_TOPIC, lambda m: path.guard_status(m.data), 1)
    field.create_subscription(String, evidence.NATIVE_AUTHORITY_TOPIC, lambda m: path.native_status(m.data),
                              runner.native_authority_qos())
    frame = Timeline()
    initial_generation = None
    authority_publishers = []
    absent_during_kick = False
    phase = "initial bind"
    deadline = time.monotonic() + 22
    next_frame = 0.
    try:
        while time.monotonic() < deadline:
            spin.spin_once(timeout_sec=.002)
            now = time.monotonic()
            if now < next_frame:
                continue
            next_frame = now + .025
            frame.ros = detector.get_clock().now().nanoseconds
            camera.publish(String(data=json.dumps(frame.frame())))
            native = field.get_publishers_info_by_topic(evidence.NATIVE_AUTHORITY_TOPIC)
            move = False
            if phase == "initial bind" and path.check(False) is None:
                initial_generation = path.native["generation"]
                core.executor.armed = True
                authority_publishers = [(i.node_name, i.topic_type, i.qos_profile.reliability.name,
                                         i.qos_profile.durability.name) for i in native]
                assert authority_publishers == [("gemini_walk", "std_msgs/msg/String", "BEST_EFFORT", "VOLATILE")]
                phase = "first walking"
            if phase == "first walking":
                if (out / "walk-before").exists():
                    path.clip_started()
                    assert path.check(False)[0] == "native"  # Previous binding is already fenced.
                    (out / "play").touch()
                    phase = "kick active"
                else:
                    move = path.check(False) is None
            if phase == "kick active":
                if not native and path.check(True) is None:
                    absent_during_kick = True
                if absent_during_kick and (out / "clip-playing").exists():
                    (out / "restore").touch()
                    phase = "return bind"
            if phase == "return bind" and path.restored_walker_ready():
                assert path.native["generation"] > initial_generation
                assert path.native["bound_source_instance"] == core.deadline.source_instance
                phase = "returned walking"
            if phase == "returned walking":
                if (out / "walk-after").exists():
                    break
                move = path.check(False) is None
            path.write(Command(vx=.16 if move else 0., expires_at=now + .14))
        passed = (out / "walk-after").exists() and phase == "returned walking"
        result = dict(passed=passed, phase=phase, first_generation=initial_generation, native=path.native,
                      saw_inactive_endpoints=absent_during_kick, authority_publishers=authority_publishers,
                      hardware=False, source=core.deadline.source_instance, guard_error=path.guard_error,
                      native_error=path.native_error)
        for name in ("walk-before", "clip-playing", "walk-after"):
            if (out / name).is_file():
                result[name] = (out / name).read_text().strip()
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        assert passed, (result, path.check(phase in ("kick active", "return bind")))
    finally:
        (out / "done").touch()
        spin.shutdown()
        for node in (field, detector, gate):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
