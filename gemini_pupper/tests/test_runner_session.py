"""The combined runner's session loop over the fake-ROS walker, real queue kick and strict board file.

Lobby-format ticketed requests drive autonomy, the native kick receipts, end of
match, teleop kickoff and expiring drive through one CombinedRuntime. Guard,
native-authority and board evidence are synthetic; nothing here is physical.
"""
import asyncio
from contextlib import asynccontextmanager
import json
import os
from types import SimpleNamespace

from test_fc_walk_kick import advance, field_clock_rig  # Installs the fake ROS modules first.
from final_project.behaviors import board_imu as imu
from final_project.behaviors import combined_fc_runner as runner
from final_project.behaviors.fc_executor_bridge import BridgeCore, DEADLINE_CONTRACT
from final_project.behaviors.operator_drive import OperatorDrive
from final_project.behaviors.test_command_path_evidence import CLOCK, SOURCE, authority, graph, status
from final_project.pupper_fc.model import Field
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, PAW_LEFT_M
from fc_walk_kick import CombinedRuntime

ROBOT = "pupper-1"
PITCH = {"length": 3.2, "width": 2.0}


class Bench:
    """Synthetic guard, native-authority and board publications on the rig clock."""

    def __init__(self, rig, tmp_path):
        self.rig, self.robot = rig, rig.ts
        ns = lambda: round(self.robot.now() * 1e9)
        self.ns = ns
        self.core = BridgeCore(ROBOT, "field", "bench", {ROBOT: Field(), "pupper-2": Field(attack_sign=-1)},
                               clock=self.robot.now, monotonic_ns=ns, command_contract=DEADLINE_CONTRACT,
                               source_instance=SOURCE, command_clock_id=CLOCK, kick_side="left",
                               paw_forward_m=COMBINED_PAW_FORWARD_M["left"])
        node, pubs, subs, endpoint = graph()
        pubs[runner.protocol.REQUEST_TOPIC] = [endpoint(runner.NAME)]
        pubs[runner.evidence.NATIVE_AUTHORITY_TOPIC] = [endpoint("gemini_walk")]
        pubs["/fc/obstacle_detections"] = [endpoint("viser_camera")]
        subs[runner.protocol.ADMITTED_TOPIC] = [endpoint("gemini_walk")]
        self.published = []
        self.path = runner.GuardedPath(self.core, node, self.published.append)
        self.board = tmp_path / "fc-board-health.bin"
        self.balance = imu.MatchImu(imu.BoardReceipt(self.board, os.getpid()), lambda: (self.robot.now(), 0., 0.),
                                    clock_ns=ns)
        self.controls = OperatorDrive(self.core)
        self.gateway = SimpleNamespace(stop_capable=True, state=None,
                                       stop_publisher=SimpleNamespace(get_subscription_count=lambda: 1))
        self.events, self.kicks = [], []
        self.session = runner.RunnerSession(self.core, self.path, self.balance, self.gateway, self.events.append,
                                            controls=self.controls, publish_kick=self.kicks.append,
                                            clock=self.robot.now)
        self.runtime = CombinedRuntime(self.robot, self.core.executor, ROBOT, velocity_sink=self.path.write,
                                       path_problem=self.session.runtime_problem,
                                       walker_ready=self.path.restored_walker_ready)
        self.session.attach(self.runtime)
        self.board_seq = self.guard_seq = self.field_seq = 0
        self.start_ns = ns()
        self.walker_activations = len(rig.robot.activations("gemini_walk"))
        self.generation, self.activation_ns, self.bound = 2, self.start_ns, SOURCE

    def publish_evidence(self):
        n = self.ns()
        self.board_seq += 2
        self.board.write_bytes(imu.RECORD.pack(self.board_seq, imu.MAGIC, os.getpid(), 1_000, 1, 0, 0, imu.HEALTHY,
                                               n, n - 1_000_000, n - 1_000_000, 0, 0, 1, 1, 5, 2,
                                               n - 1_000_000, n - 1_000_000, 0, 0, 3, 1, 0))
        self.guard_seq += 1
        self.session.on_guard(json.dumps(status(
            sampled_at_monotonic_ns=n, sequence=self.guard_seq, source_seq=self.guard_seq + 1,
            source_issued_mono_ns=n - 5_000_000, source_expires_mono_ns=n + 140_000_000,
            clearance_mono_ns=self.start_ns - 1, post_clear_zero_seq=1)))
        activations = len(self.rig.robot.activations("gemini_walk"))
        if activations != self.walker_activations:
            # The restored walker is a new native activation, bound on its first command.
            self.walker_activations, self.generation = activations, self.generation + 1
            self.activation_ns, self.bound = n, ""
        elif not self.bound and self.published:
            self.bound = SOURCE
        self.path.native_status(json.dumps(authority(generation=self.generation, activation_mono_ns=self.activation_ns,
                                                     bound_source_instance=self.bound, sampled_mono_ns=n)))

    def field(self, epoch, pose, kind="attack"):
        now = self.robot.now()
        self.field_seq += 1
        common = dict(producer_id="field", clock=self.core.clock_name, epoch=epoch, seq=self.field_seq)
        self.core.ingest({**common, "event": "intent", "robot_id": ROBOT, "kind": kind, "intent_id": kind + "-1",
                          "issued_at": now, "valid_until": now + .3,
                          **({"target": {"x": pose[0] + .3, "y": pose[1], "yaw": 0.}} if kind == "move" else {})})
        self.core.ingest({**common, "event": "frame", "captured_at": now,
                          "robots": {ROBOT: {"x": pose[0], "y": pose[1], "yaw": 0., "captured_at": now}},
                          "ball": {"x": 0., "y": 0.}, "goals": {"right": {"x": PITCH["length"] / 2, "y": 0.}}})

    def offers(self):
        return [e for e in self.events if e.get("event") == "controls"]

    async def request(self, command, **fields):
        """Like the lobby Peer: wait for a newly issued ticket, then send one request."""
        issued = len(self.offers())
        while len(self.offers()) == issued:
            await self.tick()
        offer = self.offers()[-1]
        ended = self.session.handle(dict(command=command, robot_id=ROBOT, ticket=offer["ticket"], **fields))
        reply = [e for e in self.events if e.get("event") == "operator"][-1]
        assert reply["command"] == command and reply["ticket"] == offer["ticket"]
        return ended, reply

    def last_command(self):
        return runner.protocol.decode(self.published[-1])

    async def tick(self, epoch=None, pose=None, kind="attack"):
        self.publish_evidence()
        if epoch is not None:
            self.field(epoch, pose, kind)
        step = self.session.tick()
        await advance(self.rig)
        return step


@asynccontextmanager
async def opened_bench(monkeypatch, tmp_path):
    async with field_clock_rig(monkeypatch) as rig:
        b = Bench(rig, tmp_path)
        await b.runtime.open()
        b.gateway.state = b.robot.gateway_stop_anchor()
        try:
            yield b
        finally:
            b.runtime.stop("test closed")
            await b.session.close(b.robot.now())
            closing = asyncio.create_task(b.runtime.close())
            for _ in range(1000):  # Native cleanup sleeps on the rig clock.
                if closing.done():
                    break
                await advance(rig)
            await closing
            b.balance.reader.close()


async def kick_until(b, states):
    staged = (-COMBINED_PAW_FORWARD_M["left"], -PAW_LEFT_M["left"])
    walked = False
    for i in range(1000):  # The rig's clip and walker return take about 8.5 s.
        await b.tick("match-1", (-.9, staged[1]) if i < 8 else staged)
        walked |= not b.last_command().is_zero
        if any(k["state"] in states for k in b.kicks):
            return walked
    raise AssertionError(f"no {states} receipt: {b.kicks}, stop={b.runtime.stop_reason}")


def test_autonomous_kick_then_teleop_share_one_runtime_and_report_distinct_evidence(monkeypatch, tmp_path):
    async def case():
        async with opened_bench(monkeypatch, tmp_path) as b:
            rig = b.rig
            for _ in range(11):
                await b.tick()
            offer = [e for e in b.events if e.get("event") == "controls"][-1]
            assert offer["ready"] and offer["imu"]["state"] == "healthy" and offer["imu"]["generation"] == 1
            assert not any(e.get("event") == "kick" for e in b.events)

            _, reply = await b.request("arm", epoch="match-1", mode="autonomous", pitch=PITCH)
            assert reply["accepted"] and b.core.executor.armed
            assert b.core.executor.controllers[ROBOT].field.length == PITCH["length"]
            assert await kick_until(b, ("completed", "retired", "aborted"))
            assert b.runtime.stop_reason is None, b.runtime.stop_reason
            assert [k["state"] for k in b.kicks] == ["engaged", "clip_started", "completed"]
            assert all(k["physical"] == runner.NO_CONTACT and k["epoch"] == "match-1" for k in b.kicks)
            assert [e["state"] for e in b.events if e.get("event") == "kick"] == ["engaged", "clip_started", "completed"]
            assert len(rig.robot.activations("trick_leg_kick")) == 1
            assert rig.robot.controllers["gemini_walk"] == "active" and b.generation == 3

            staged = (-COMBINED_PAW_FORWARD_M["left"], -PAW_LEFT_M["left"])
            for _ in range(3):
                await b.tick("match-1", staged, kind="move")
            _, reply = await b.request("pause")
            assert reply["state"] == "standby" and not b.core.executor.armed
            _, reply = await b.request("mode", mode="teleop")
            assert not reply["accepted"]  # Fresh board pairs and a lapsed walking lease are both pending.
            for _ in range(10):
                await b.tick()
            _, reply = await b.request("mode", mode="teleop")
            assert reply["accepted"], reply
            await b.tick()
            _, reply = await b.request("arm", epoch="keyboard-1", mode="teleop")
            assert reply["accepted"] and not b.core.executor.armed and b.controls.match_epoch == "keyboard-1"
            await b.tick()
            _, reply = await b.request("drive", epoch="keyboard-1", forward=0, left=0, turn=0)
            assert reply["accepted"]
            await b.tick()
            _, reply = await b.request("drive", epoch="keyboard-1", forward=1, left=0, turn=0)
            assert reply["accepted"]
            issued = b.robot.now()  # The consumed ticket was issued at or before this request.
            await b.tick()
            drive = b.last_command()
            assert drive.vx == b.core.limits.vx and drive.expires_mono_ns <= round((issued + .25) * 1e9)
            for _ in range(15):
                await b.tick()  # No renewed key press: the ticket-bound command lapses to zero.
            assert b.last_command().is_zero and b.runtime.stop_reason is None
            assert len(rig.robot.activations("trick_leg_kick")) == 1

            assert b.session.handle({"command": "stop"}) is True
    asyncio.run(case())


def test_stop_during_a_kick_aborts_without_success_and_cannot_rearm(monkeypatch, tmp_path):
    async def case():
        async with opened_bench(monkeypatch, tmp_path) as b:
            for _ in range(11):
                await b.tick()
            _, reply = await b.request("arm", epoch="match-1", mode="autonomous")
            assert reply["accepted"]
            await kick_until(b, ("clip_started",))
            assert b.rig.robot.controllers["trick_leg_kick"] == "active"
            assert b.session.handle({"command": "stop"}) is True
            b.runtime.stop("operator Stop or input closed")  # What run()'s terminate() does next.
            await b.session.close(b.robot.now())
            assert [k["state"] for k in b.kicks] == ["engaged", "clip_started", "aborted"]
            assert "operator Stop" in b.kicks[-1]["reason"] and not b.core.executor.armed
            b.session.handle({"command": "arm", "robot_id": ROBOT, "ticket": b.offers()[-1]["ticket"],
                              "epoch": "match-2", "mode": "autonomous"})
            reply = [e for e in b.events if e.get("event") == "operator"][-1]
            assert not reply["accepted"] and not b.core.executor.armed
            for _ in range(5):
                await b.tick("match-1", (-.9, 0.))
            assert b.last_command().is_zero and len(b.kicks) == 3
    asyncio.run(case())


def test_production_session_and_kick_import_without_ros_or_test_modules():
    import subprocess
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    source = """
import sys
from final_project.behaviors.combined_fc_runner import RunnerSession
from fc_walk_kick import CombinedRuntime
from operator_kick import kick_in_place
from motion_contract import CALL_STOPS, ACTIVATION_GATE, Result
assert 'rclpy' not in sys.modules and 'robot_api' not in sys.modules
assert not any(k.startswith('test_') for k in sys.modules)
assert Result(True, 'receipt').as_dict() == {'success': True, 'message': 'receipt'}
"""
    result = subprocess.run([sys.executable, '-c', source], cwd=root, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
