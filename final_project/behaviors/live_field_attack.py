#!/usr/bin/env python3
"""One live, capture-leased soccer attempt for a selected fc_walk_kick runner.

On the Pi this process exclusively owns the fieldcam UDP receiver. With
--publish it sends the original normalized frame followed by an attack lease
only for a simultaneously measured striker, ball and configured opponent goal.
The runner selects the kick foot and owns arm, gait, kick and Stop. With
--approach-only --side, this source emits only MOVE to a ball-clear standoff on
that paw line. An unsafe segment or lost pair withdraws the frame immediately.
By default report the actual field observations without loading ROS.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time


class LiveFieldPlay:
    """Produce a single capture-leased action for each admitted observation."""

    def __init__(self, robot_id, field, max_age, approach_side=None):
        self.robot_id, self.field, self.max_age = robot_id, field, max_age
        self.approach_side = approach_side
        self.goal_name = "right" if field.attack_sign == 1 else "left"

    def inside(self, point, margin=0.):
        return (abs(point["x"]) <= self.field.length / 2 - margin
                and abs(point["y"]) <= self.field.width / 2 - margin)

    def plan(self, frame, now):
        captured = frame["captured_at"]
        if captured is None or not 0 <= now - captured < self.max_age:
            return None, "capture revoked or expired"
        pose, ball = frame["robots"].get(self.robot_id), frame["ball"]
        goal = frame["goals"].get(self.goal_name)
        if pose is None or pose["captured_at"] != captured or ball is None or goal is None:
            return None, "same-frame striker, ball and opponent goal required"
        if not self.inside(pose, self.field.robot_radius) or not self.inside(ball):
            return None, "robot body or ball centre outside the field"
        # Reuse the camera sequence. Lease renewals keep one attempt identity
        # throughout the epoch; the handoff retires after its native receipts.
        event = {"event": "intent", "producer_id": frame["producer_id"], "epoch": frame["epoch"],
                 "clock": frame["clock"], "seq": frame["seq"], "robot_id": self.robot_id,
                 "kind": "attack", "intent_id": "live-field-one-shot",
                 "issued_at": captured, "valid_until": captured + self.max_age}
        if self.approach_side is None:
            return event, "paired attack"
        # MOVE steers directly. Keep the entire observed straight segment clear
        # of the ball and include the move controller's arrival tolerance.
        from final_project.pupper_fc.controller import HOME_TOLERANCE, _segment_distance, defensive_clearance
        from final_project.pupper_fc.model import Ball
        from final_project.pupper_fc.paw_approach import paw_pose
        clearance = defensive_clearance(self.field)
        if math.hypot(goal["x"] - ball["x"], goal["y"] - ball["y"]) < 1e-6:
            return None, "ball coincides with the goal"
        target = paw_pose(Ball(**ball), (goal["x"], goal["y"]), self.approach_side,
                          forward_m=clearance + HOME_TOLERANCE)
        point = {"x": target.x, "y": target.y, "yaw": target.yaw}
        if not self.inside(point, self.field.robot_radius):
            return None, "standoff falls outside the robot-centre bounds"
        if _segment_distance(pose["x"], pose["y"], target.x, target.y, ball["x"], ball["y"]) < clearance:
            return None, "direct standoff approach would enter ball clearance"
        event.update(kind="move", intent_id="live-field-standoff", target=point)
        return event, "paired standoff move"

    def events(self, frame, now):
        intent, reason = self.plan(frame, now)
        if intent is not None:
            return frame, intent, reason
        # Ordinary MOVE does not require the ball once admitted. Fence its older
        # lease on the first missing pair/unsafe target; never mint a capture.
        if frame["captured_at"] is not None:
            frame = {**frame, "captured_at": None, "robots": {}, "ball": None, "goals": {}}
        return frame, None, reason


class LiveMatchPlay(LiveFieldPlay):
    """Apply the one camera referee's decision using only this Pi's capture-bound observations."""
    def __init__(self, robot_id, field, max_age, epoch, roster):
        super().__init__(robot_id, field, max_age)
        from final_project.pupper_fc.tactics import Tactics
        self.epoch, self.roster = epoch, sorted(roster)
        self.tactics = Tactics(field, robot_id)
        self.number, self.started, self.reacquire_after = 1, None, None
        self.goal_notified = self.attempt_capture = self.in_flight = None
        self.ready_to_arm = False

    def completion(self, receipt, now):
        if (not isinstance(receipt, dict) or self.started is None or receipt.get("robot_id") != self.robot_id
                or receipt.get("epoch") != self.epoch or receipt.get("attempt_id") != self.started):
            return False
        if receipt.get("state") == "engaged":
            issued = receipt.get("issued_at")
            if type(issued) not in (int, float) or not self.attempt_capture <= issued <= now or self.reacquire_after is not None:
                return False
            self.in_flight = self.started
            return True
        completed = receipt.get("completed_at")
        if (receipt.get("state", "completed") != "completed" or type(completed) not in (int, float)
                or not self.attempt_capture <= completed <= now or self.reacquire_after is not None):
            return False
        self.reacquire_after, self.in_flight = completed, None
        return True

    def events(self, frame, now):
        from final_project.pupper_fc.model import Ball
        from final_project.pupper_fc.match_director import directive_ok, goal_geometry
        self.ready_to_arm = False
        captured = frame["captured_at"]
        info = frame.get("fc_match")
        pose, ball = frame["robots"].get(self.robot_id), frame["ball"]
        def hold(reason):
            return {**frame, "captured_at": None, "robots": {}, "ball": None, "goals": {}}, None, reason
        if (captured is None or not 0 <= now - captured < self.max_age
                or not directive_ok(info, self.epoch, self.roster, frame.get("source", {}).get("seq"))
                or pose is None or pose["captured_at"] != captured or not self.inside(pose, self.field.robot_radius)):
            return hold("match directive, participant pose or capture not ready")
        lines = info.get("goal_lines")
        if not goal_geometry(lines, self.field.length, self.field.width, self.field.ball_radius):
            return hold("calibrated goal posts required")
        frame = {**frame, "goals": {key: {"x": lines[end]["x"], "y": (lines[end]["y_min"] + lines[end]["y_max"]) / 2}
                                   for key, end in (("left", "west"), ("right", "east"))}}
        phase = info["phase"]
        if phase == "pending":
            poses = [frame["robots"].get(robot) for robot in self.roster]
            if (not ball or not self.inside(ball) or not all(row and row["captured_at"] == captured
                    and self.inside(row, self.field.robot_radius) for row in poses)):
                return hold("waiting for measured kickoff positions")
            self.ready_to_arm = True
            return frame, None, "kickoff observations ready; awaiting all arms"
        event = {"event":"intent", "epoch":self.epoch, "producer_id":frame["producer_id"], "clock":frame["clock"],
                 "seq":frame["seq"], "robot_id":self.robot_id, "kind":"attack",
                 "intent_id":"referee-" + str(info["rally"]), "issued_at":captured, "valid_until":captured + self.max_age}
        if self.in_flight is not None:
            if any(r not in frame["robots"] for r in self.roster):
                return hold("fresh participant poses required throughout a kick")
            event["intent_id"] = self.in_flight
            if phase == "goal":
                frame["robots"] = {**frame["robots"], self.robot_id: {**pose, "goal": info["goal_sign"] * self.field.attack_sign}}
            # Only the already dispatched native job may retain its identity here. A missing ball
            # still stops an unconfirmed swing according to the Executor's original checks.
            return frame, event, "native kick owns legs until walker return"
        if phase == "holding":
            return hold("shared match referee holds")
        if phase == "goal":
            from final_project.pupper_fc.controller import HOME_FRACTION
            # Report even if our first packet arrives after the reset ball was placed.
            new_goal = self.goal_notified != info["rally"]
            self.goal_notified = info["rally"]
            sign = info["goal_sign"] * self.field.attack_sign if new_goal or not info["reset_ready"] else 0
            if self.started is not None:
                event["intent_id"] = self.started
            if not sign:
                # A Pi that already consumed Resume must hold its home pose until every teammate is ready.
                event.update(kind="move", intent_id="reset-home-" + str(info["rally"]),
                             target={"x": -self.field.attack_sign * HOME_FRACTION * self.field.length,
                                     "y": 0., "yaw": math.atan2(0., self.field.attack_sign)})
            robots = {**frame["robots"], self.robot_id: {**pose, "goal": sign}}
            return {**frame, "robots": robots}, event, "scored; returning for reset"
        if (ball is None or not self.inside(ball) or any(row not in frame["robots"] for row in self.roster)
                or self.goal_name not in frame["goals"]):
            return hold("current participants, ball and target required")
        if info["attacker"] != self.robot_id:
            own_goal = lines["west" if self.field.attack_sign == 1 else "east"]
            target = self.tactics.guard_pose(Ball(**ball), own_goal)
            if target is None:
                return hold("no ball-clear defensive pose")
            event.update(kind="defend", intent_id="guard-" + str(info["role_cycle"]), target=vars(target))
            return frame, event, "guard own goal"
        if self.reacquire_after is not None:
            if captured <= self.reacquire_after:
                return hold("waiting for a ball measured after walker recovery")
            self.number += 1
            self.reacquire_after = None
        attempt = f"r{info['rally']}-p{info['role_cycle']}-k{self.number}"
        if self.started != attempt:
            self.started, self.attempt_capture = attempt, captured
        event["intent_id"] = self.started
        return frame, event, "paired attack"



def stream(receiver, play, duration, *, publish=None, trace=None, pump=None):
    end = time.monotonic() + duration
    frames = paired = revocations = 0
    latest = None
    report_at = 0.
    reported_ready = False
    try:
        while time.monotonic() < end:
            if pump is not None and not pump():
                break
            for frame in receiver.poll():
                now = time.monotonic()
                outgoing, intent, reason = play.events(frame, now)
                if publish is not None:
                    publish(outgoing)
                    if intent is not None:
                        publish(intent)
                if trace is not None:
                    trace.write(json.dumps({"at": now, "frame": frame, "forwarded_capture": outgoing["captured_at"],
                                            "intent": intent, "reason": reason},
                                           allow_nan=False) + "\n")
                frames += 1
                paired += int(intent is not None)
                revocations += int(outgoing["captured_at"] is None)
                latest = {"seq": frame["seq"], "captured_at": frame["captured_at"],
                          "paired": intent is not None, "ready_to_arm": getattr(play, "ready_to_arm", False), "reason": reason, "target": intent.get("target") if intent else None,
                          "pose": frame["robots"].get(play.robot_id),
                          "ball": frame["ball"], "opponent_goal": frame["goals"].get(play.goal_name)}
            now = time.monotonic()
            ready = bool(latest and latest["ready_to_arm"])
            if now >= report_at or ready != reported_ready:
                reported_ready = ready
                print(json.dumps({"mode": "publish" if publish else "observe",
                                  "reason": receiver.core.last_reason, "frames": frames,
                                  "pairs": paired, "revocations": revocations, "latest": latest},
                                 allow_nan=False), flush=True)
                report_at = now + 1.
            time.sleep(.003)
    finally:
        # An orderly source exit clears the runner frame immediately. On a
        # lost process or connection the original capture's lease still expires.
        revoke = receiver.core.revoke("live field attack source closed")
        if revoke is not None and publish is not None:
            publish(revoke)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release", type=Path, required=True,
                        help="selected release root providing the installed ingress and field contract")
    parser.add_argument("--field-config", type=Path, required=True,
                        help="same full-precision geometry and attack signs as the runner")
    parser.add_argument("--source-host", required=True)
    parser.add_argument("--source-id", default="fieldcam")
    parser.add_argument("--source-session", required=True)
    parser.add_argument("--source-robot", default="striker")
    parser.add_argument("--calibration-revision", required=True, type=int)
    parser.add_argument("--producer-id", required=True)
    parser.add_argument("--epoch", required=True)
    parser.add_argument("--robot-id", required=True, choices=("pupper-1", "pupper-2"))
    parser.add_argument("--match-tracks", help="JSON camera-track to team-ID map for a camera-directed match")
    parser.add_argument("--stop-on-eof", action="store_true", help="SSH input closes the field producer")
    parser.add_argument("--approach-only", action="store_true", help="issue only MOVE to a ball-clear standoff")
    parser.add_argument("--side", choices=("right", "left"), help="approach-only standoff's paw line")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--duration", type=float, default=60.)
    parser.add_argument("--trace", type=Path, help="new JSONL file for original normalized frames and leases")
    parser.add_argument("--publish", action="store_true", help="publish on the selected runner's ROS domain")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600 or not 1 <= args.port <= 65535:
        parser.error("duration must be in (0, 3600] seconds and port in [1, 65535]")
    if args.approach_only != bool(args.side):
        parser.error("--approach-only and its --side must be supplied together")
    # This file may be staged beside an existing verified release. Resolve all
    # shared code from that release, including when the file is invoked directly.
    release = args.release.resolve(strict=True)
    sys.path.insert(0, str(release))
    from final_project.behaviors import fc_executor_bridge as bridge, overhead_ingress as ingress
    for module in (bridge, ingress):
        if Path(module.__file__).resolve().parent != release / "final_project/behaviors":
            parser.error("ingress or executor import is outside the selected release")
    config = json.loads(args.field_config.read_text())
    field = bridge.load_fields(config)[args.robot_id]
    play = LiveFieldPlay(args.robot_id, field, ingress.MAX_FRAME_AGE_S, args.side)
    tracks = json.loads(args.match_tracks) if args.match_tracks is not None else {args.source_robot: args.robot_id}
    if args.match_tracks is not None:
        if args.approach_only or not isinstance(tracks, dict) or args.robot_id not in tracks.values():
            parser.error("continuous match needs the participant track and cannot select a standoff")
        play = LiveMatchPlay(args.robot_id, field, ingress.MAX_FRAME_AGE_S, args.epoch, tracks.values())
    core = ingress.IngressCore(source_id=args.source_id, source_session=args.source_session,
        producer_id=args.producer_id, epoch=args.epoch,
        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        robot_map=tracks, calibration_revision=args.calibration_revision,
        field=config["field"])
    receiver = ingress.UDPIngress(core, port=args.port, source_host=args.source_host)
    trace = node = None
    try:
        trace = args.trace.open("x", buffering=1) if args.trace else None
        publish = None
        if args.publish:
            import rclpy
            from std_msgs.msg import String
            rclpy.init(args=[])
            node = rclpy.create_node("fc_live_field_attack")
            # Reliable, volatile, same writer in frame/intent order. Each pair
            # needs two history entries; subscriptions use KEEP_LAST depth 8.
            publisher = node.create_publisher(String, bridge.EVENT_TOPIC, 8)

            def publish(event):
                publisher.publish(String(data=json.dumps(event, allow_nan=False)))

            if args.match_tracks is not None:
                def kick_result(message):
                    try:
                        play.completion(json.loads(message.data), time.monotonic())
                    except (ValueError, TypeError):
                        pass
                node.create_subscription(String, "/fc/kick_result", kick_result, 4)

        exiting = False
        def shutdown(signum, context):
            nonlocal exiting
            exiting = True

        previous = {sig: signal.signal(sig, shutdown) for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)}

        def pump():
            if exiting:
                return False
            if node is not None and args.match_tracks is not None:
                rclpy.spin_once(node, timeout_sec=0)
            if args.stop_on_eof:
                import select
                if select.select([sys.stdin], [], [], 0)[0] and not sys.stdin.buffer.read1(128):
                    return False
            return True

        print(json.dumps({"event": "source_open", "mode": "publish" if publish else "observe",
                          "source": core.identity(), "epoch": args.epoch, "producer_id": args.producer_id,
                          "robot_id": args.robot_id, "goal": play.goal_name, "approach_only": args.approach_only,
                          "side": args.side, "field": config["field"]}),
              flush=True)
        try:
            stream(receiver, play, args.duration, publish=publish, trace=trace, pump=pump)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    finally:
        receiver.close()
        if trace is not None:
            trace.close()
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
