"""Device-free soccer rehearsal with production kickoff, field subprocesses, UDP and ROS DDS.

Each Pi has its own ROS domain. The synthetic camera reports measured positions of a
2-D walker. The guard, Stop gateway, controller peers and native clip feedback are
memory fixtures. HTTP/WebSocket and physical-controller checks run separately.
"""
import asyncio
from contextlib import ExitStack
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
BLUE, RED = "pupper-1", "pupper-2"
SIGNS = {BLUE: 1, RED: -1}
TRACKS = {BLUE: "striker", RED: "keeper"}
OWNER = "bench-operator"


class Camera:
    """Deliver scripted ball observations and positions integrated from actual runner decisions."""
    def __init__(self, field):
        self.field = field
        self.session = "bench-camera"
        self.director, self.publishers = None, ()
        self.seq, self.captured = 0, 0.
        self.match, self.ball_x = None, 0.

    def frame(self, players):
        self.seq += 1
        self.captured = time.time()
        measured = dict(visible=True, measured_this_frame=True, measured_at=self.captured, age_s=0., vx=0., vy=0.)
        frame = dict(seq=self.seq, calibration_valid=True, calibration_revision=1, field=self.field,
                     captured_at=self.captured, stamp=self.captured, frame_age_s=0.,
                     ball=dict(measured, x=self.ball_x, y=0.), robots={},
                     goals={name:dict(x=sign*self.field["length"]/2, y_min=-self.field["goal_width"]/2, y_max=self.field["goal_width"]/2)
                            for name, sign in (("west", -1), ("east", 1))})
        for p in players:
            pose = p.sim.robot
            frame["robots"][TRACKS[p.robot]] = dict(measured, x=pose.x, y=pose.y, yaw=pose.yaw, wz=0.)
        if self.director:
            self.match = frame["fc_match"] = self.director.update(frame, now=self.captured)
            for pub in self.publishers:
                pub.send(frame)

    def snapshot(self):
        return dict(live=True, source_session=self.session, match=self.match,
                    frame=dict(seq=self.seq, captured_at=self.captured, calibration_valid=True, calibration_revision=1),
                    visible=dict(ball=True, robots=dict(striker=True, keeper=True)))

    def set_match(self, director, pubs=()):
        self.director, self.publishers, self.match = director, pubs, None

    def commit_start(self):
        self.director.commit_start()

    def resume(self):
        self.director.resume()


class Player:
    """The production BridgeCore, operator actions and handoff behind a local peer adapter."""
    def __init__(self, stack, robot, fields, camera, domain, attacker):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from final_project.behaviors import combined_fc_runner as combined, fc_executor_bridge as bridge
        from final_project.behaviors.operator_drive import OperatorDrive
        from final_project.pupper_fc.kick_handoff import KickHandoff
        from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, PAW_LEFT_M
        from final_project.pupper_fc.simulation import Simulator, SimulatedKick
        self.robot, self.domain, self.owner, self.closed = robot, domain, OWNER, False
        self.host, self.offer = robot, {"mode": "autonomous", "ready": True}
        self.camera, self.reject_after = camera, None
        self.port = 19500 + domain
        boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.core = bridge.BridgeCore(robot, "wire", boot, fields, kick_side="right",
                                      paw_forward_m=COMBINED_PAW_FORWARD_M["right"])
        self.handoff = KickHandoff(robot)
        self.backend = SimulatedKick(self.handoff)
        self.sim = Simulator(fields[robot], bridge.COMBINED_LIMITS, bumper_contact=False)
        sign = SIGNS[robot]
        self.sim.move_robot(-sign * (.35 if attacker else .87), -sign * PAW_LEFT_M["right"], 0 if sign == 1 else math.pi)
        self.sim.move_ball(0., 0.)
        self.controls = OperatorDrive(self.core)
        self.controls.observe_path(True, 1)
        self.path = SimpleNamespace(check=lambda _: None)
        self.gateway = SimpleNamespace(stop_capable=True, state={"instance_id":"bench"},
                                       stop_publisher=SimpleNamespace(get_subscription_count=lambda: 1))
        self.runtime = SimpleNamespace(handoff=self.handoff, velocity_sink=lambda cmd: None,
            robot=SimpleNamespace(gateway_stop_anchor=lambda: self.gateway.state, current=None,
                                  commands=SimpleNamespace(empty=lambda:True)))
        self.context = Context()
        self.context.init(args=[], domain_id=domain)
        stack.callback(self.context.try_shutdown)
        self.node = rclpy.create_node("fc_bench_runner_" + robot[-1], context=self.context)
        stack.callback(self.node.destroy_node)
        self.spin = SingleThreadedExecutor(context=self.context)
        self.spin.add_node(self.node)
        stack.callback(self.spin.shutdown)
        self.node.create_subscription(String, bridge.EVENT_TOPIC, self.ingest, 32)
        self.result = self.node.create_publisher(String, "/fc/kick_result", 4)
        self.actions, self.receipts, self.kinds = [], [], set()
        self.missed_goal_frames = 0
        self.events = self.pending_frames = self.pending_armed_frames = self.revokes = 0
        self.first_motion = self.last_return = self.new_capture = self.after_return_x = None
        self.walked_between_m = 0.
        self.nonzero_between = False
        self.phase = None

    async def request(self, action, **fields):
        from final_project.behaviors import combined_fc_runner as combined
        if action == "arm" and self.reject_after:
            # Let the first participant arm and receive several pending camera frames.
            async with asyncio.timeout(4):
                while not self.reject_after():
                    await asyncio.sleep(.01)
            raise ValueError("injected missing zero command")
        if self.robot == RED and action == "arm":
            await asyncio.sleep(.10)
        while True:
            now = time.monotonic()
            offer = self.controls.offer(now, combined.resume_cycle(self.core, now))
            if offer and (action != "resume" or offer["resume_ready"]):
                break
            await asyncio.sleep(.01)
        request = dict(command=action, robot_id=self.robot, ticket=offer["ticket"], **fields)
        result = combined.operator_action(request, self.core, self.path, self.runtime, self.gateway, self.controls)
        assert result["accepted"], result
        self.actions.append((action, fields.get("epoch"), result.get("state")))
        return result

    def ingest(self, message):
        event = json.loads(message.data)
        self.events += 1
        # Drop Red's goal pulse between the ROS subscription and the executor. On
        # reconnection the first admitted camera frame already has the ball replaced.
        info = event.get("fc_match") or {}
        if self.robot == RED and info.get("phase") == "goal" and not info["reset_ready"]:
            self.missed_goal_frames += 1
            return
        self.core.ingest(event)
        director = self.camera.director
        if director and director.pending_start:
            assert event["event"] != "intent", event
            if event.get("captured_at") is not None:
                self.pending_frames += 1
                self.pending_armed_frames += int(self.core.executor.armed)
        if event["event"] == "intent":
            self.kinds.add(event["kind"])
            if self.last_return and event["intent_id"] == "r1-p0-k2" and self.new_capture is None:
                assert event["issued_at"] > self.last_return
                self.new_capture = event["issued_at"]
        elif event["captured_at"] is None:
            self.revokes += 1

    def tick(self, dt):
        from std_msgs.msg import String
        from final_project.behaviors.combined_fc_runner import kick_event
        for _ in range(5):
            self.spin.spin_once(timeout_sec=0)
        now = time.monotonic()
        prior = self.handoff.active
        step = self.core.executor.kick_step(self.handoff, now)
        self.phase, self.reason = step.decision.phase, step.decision.reason
        from final_project.behaviors.combined_fc_runner import resume_cycle
        self.offer["resume_ready"] = resume_cycle(self.core, now) is not None
        command = step.decision.command
        moving = any((command.vx, command.vy, command.wz))
        director = self.camera.director
        if (director and director.pending_start) or not self.core.executor.armed:
            assert not moving and step.request is None, (self.robot, self.phase)
        if moving and self.first_motion is None:
            self.first_motion = now
        if self.last_return and len(self.receipts) == 1 and moving:
            self.nonzero_between = True
        assert step.cancel is None, (self.robot, self.phase, step.cancel)
        edge = kick_event(prior, self.handoff, step, now)
        if edge:
            assert director.observe_kick(edge)
            self.result.publish(String(data=json.dumps(edge)))
            if edge["state"] == "completed":
                self.receipts.append(edge["attempt_id"])
                if len(self.receipts) == 1:
                    self.last_return, self.after_return_x = now, self.sim.robot.x
                    # Simulated clip has no ball impulse. Move the ball 24 cm, then integrate
                    # the next approach solely from the runner's commands.
                    self.camera.ball_x = .24 * SIGNS[self.robot]
                elif len(self.receipts) == 2:
                    self.walked_between_m = abs(self.sim.robot.x - self.after_return_x)
        self.backend.update(step, now)
        sim_command = replace(command, robot_id=BLUE, expires_at=self.sim.time + max(0., command.expires_at - now))
        self.sim.step(sim_command, dt)


async def rehearse(roster, base_domain, root):
    from final_project.behaviors.fc_executor_bridge import load_fields
    from final_project.pupper_fc.field_transport import FieldFeed
    from final_project.pupper_fc.lobby import Lobby, PROFILE
    field = {**json.loads(PROFILE.read_text())["field"], "robot_radius": .16, "bumper_offset": .19}
    config = {"field":field, "attack_signs":SIGNS}
    geometry = root / "field.json"
    geometry.write_text(json.dumps(config))
    camera = Camera(field)
    players, sources = {}, []
    run_log = root / "sources.log"
    async def describe(host, robot):
        return dict(robot_host="127.0.0.1", field_config=config, port=players[robot].port)
    async def open_source(host, robot, info, epoch, session, revision, tracks):
        player = players[robot]
        command = [sys.executable, str(ROOT / "final_project/behaviors/live_field_attack.py"),
                   "--release", str(ROOT), "--field-config", str(geometry), "--source-host", "127.0.0.1",
                   "--source-session", session, "--calibration-revision", str(revision), "--producer-id", "wire",
                   "--epoch", epoch, "--robot-id", robot, "--match-tracks", json.dumps(tracks),
                   "--port", str(player.port), "--stop-on-eof", "--duration", "60", "--publish"]
        proc = await asyncio.create_subprocess_exec(*command, stdin=asyncio.subprocess.PIPE,
                   stdout=asyncio.subprocess.PIPE, stderr=log, env={**os.environ, "ROS_DOMAIN_ID":str(player.domain)})
        sources.append(proc)
        return FieldFeed(proc, robot, epoch, session)
    lobby = Lobby(describe_peer=describe, field_factory=open_source, settings=root/"no-operator-settings")
    lobby.camera = camera
    lobby.peers = players
    started = time.monotonic()
    with ExitStack() as stack:
        log = stack.enter_context(run_log.open("w"))
        for i, robot in enumerate(roster):
            players[robot] = Player(stack, robot, load_fields(config), camera, base_domain+i, i == 0)
        lead = players[roster[0]]
        last_frame = last_tick = time.monotonic()
        camera.frame(players.values())
        async def clock():
            nonlocal last_frame, last_tick
            while True:
                now = time.monotonic()
                if now - last_frame >= .032:
                    last_frame = now
                    camera.frame(players.values())
                for player in players.values():
                    player.tick(min(.2, now - last_tick))
                last_tick = now
                await asyncio.sleep(.004)
        ticker = asyncio.create_task(clock())
        async def until(predicate, limit=12):
            async with asyncio.timeout(limit):
                while not predicate():
                    if ticker.done():
                        ticker.result()
                    await asyncio.sleep(.008)
        try:
            result = {"roster":roster}
            if len(roster) == 2:
                players[RED].reject_after = lambda: players[BLUE].pending_armed_frames >= 3
                try:
                    await lobby.start(OWNER)
                    raise AssertionError("start with rejected Red arm passed")
                except ValueError as exc:
                    assert "injected" in str(exc), exc
                assert lobby.phase == "idle" and all(not p.core.executor.armed for p in players.values())
                assert all(p.first_motion is None and p.actions[-1][2] == "standby" for p in players.values())
                assert players[BLUE].pending_armed_frames > 0
                players[RED].reject_after = None
                result["rejected_start_standby"] = True

            speeds = []
            for trial in range(2):
                for p in players.values():
                    p.first_motion = None
                launched = time.monotonic()
                await lobby.start(OWNER)
                await until(lambda: lead.first_motion is not None)
                speeds.append(round(lead.first_motion-launched, 3))
                if trial == 0:
                    ending = time.monotonic()
                    await lobby.end(OWNER)
                    result["end_to_standby_s"] = round(time.monotonic() - ending, 3)
                    assert all(p.actions[-1][2] == "standby" and not p.core.executor.armed for p in players.values())
            result["start_to_first_nonzero_s"], result["warm_restart_to_first_nonzero_s"] = speeds
            if len(roster) == 1:
                await until(lambda: len(lead.receipts) >= 2, 24)
                assert lead.receipts == ["r1-p0-k1", "r1-p0-k2"]
                assert lead.nonzero_between and lead.walked_between_m > .16 and lead.new_capture is not None
                result.update(kick_completions=lead.receipts,
                              observed_ball_relocation_m=.24, integrated_second_approach_m=round(lead.walked_between_m,3),
                              new_capture_after_first_completion_s=round(lead.new_capture-lead.last_return, 3))
            else:
                await until(lambda: "attack" in lead.kinds and "defend" in players[RED].kinds)
                # Script a measured whole-ball crossing, then keep the ball outside and try Resume.
                plane = field["length"]/2
                for offset in (-.25, -.17, -.09, -.01, .07, .15):
                    camera.ball_x = plane+offset
                    await asyncio.sleep(.065)
                assert camera.match["score"] == {BLUE:1, RED:0} and camera.match["phase"] == "goal"
                try:
                    await lobby.resume(OWNER)
                    raise AssertionError("Resume with the ball outside passed")
                except ValueError as exc:
                    assert "ball" in str(exc)
                await until(lambda: players[RED].missed_goal_frames > 0)
                camera.ball_x = 0.
                await until(lambda: camera.match["reset_ready"])
                assert not lobby.state()["resume_ready"]
                try:
                    await lobby.resume(OWNER)
                    raise AssertionError("Resume before the walker reached home passed")
                except ValueError as exc:
                    assert "home positions" in str(exc) and lobby.phase == "playing"
                try:
                    await until(lambda: lobby.state()["resume_ready"], 17)
                except TimeoutError as exc:
                    raise AssertionError([(p.robot,p.phase,p.reason,p.sim.robot,p.core.executor.frame_seq,
                                           p.core.executor.frame) for p in players.values()]) from exc
                await lobby.resume(OWNER)
                await until(lambda: camera.match["phase"] == "playing" and camera.match["rally"] == 2)
                result.update(score=camera.match["score"], early_resume_held_match=True, missed_red_goal_frames=players[RED].missed_goal_frames,
                              rally_after_replacement_and_resume=2)
            revokes = [p.revokes for p in players.values()]
            # Closing an individual field's input exercises the production source's orderly revoke
            # and the lobby monitor's automatic End. The connected runners can be reused.
            sources[-1].stdin.close()
            await until(lambda: lobby.phase == "idle", 4)
            assert all(p.actions[-1][2] == "standby" for p in players.values())
            assert all(s.returncode == 0 for s in sources)
            await until(lambda: all(p.revokes > n for p,n in zip(players.values(),revokes)))
            result.update(eof_standby=True, ros_events={p.robot:p.events for p in players.values()},
                          pending_frames={p.robot:p.pending_frames for p in players.values()},
                          wall_s=round(time.monotonic()-started,3))
            print(json.dumps(result), flush=True)
        finally:
            ticker.cancel()
            try:
                await ticker
            except asyncio.CancelledError:
                pass
            finally:
                await lobby.end(OWNER)
                for proc in sources:
                    if proc.returncode is None:
                        proc.terminate()
                        await proc.wait()


def main():
    if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1":
        raise SystemExit("Run in the device-free, network-disabled ROS test container.")
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--roster", choices=("blue","red","both","all"), default="all")
    args = parser.parse_args()
    for idx, (name, roster) in enumerate((("blue",[BLUE]), ("red",[RED]), ("both",[BLUE, RED]))):
        if args.roster in ("all",name):
            with tempfile.TemporaryDirectory() as temp:
                asyncio.run(rehearse(roster, 100+idx*2, Path(temp)))


if __name__ == "__main__":
    main()
