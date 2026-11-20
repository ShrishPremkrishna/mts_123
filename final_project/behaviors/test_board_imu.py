"""Real board wire format and match handlers with deterministic IMU inputs; no hardware."""
import asyncio
import os
from types import SimpleNamespace

import pytest

from final_project.behaviors import board_imu as imu
from final_project.behaviors import combined_fc_runner as runner
from final_project.behaviors.operator_drive import OperatorDrive
from final_project.behaviors.test_combined_fc_runner import path_fixture
from final_project.pupper_fc.model import Command


class Trial:
    def __init__(self, tmp_path):
        self.path, self.now, self.sent, *_ = path_fixture()
        self.path.check = lambda during: None  # Independent guard/graph tests cover this boundary.
        self.core = self.path.core
        self.file = tmp_path / "fc-board-health.bin"
        self.reader = imu.BoardReceipt(self.file, os.getpid(), "field_redundant_imu")
        self.ros = self.now[0]
        self.gate = imu.MatchImu(self.reader, lambda: (self.ros, 0., 0.), clock_ns=self.clock_ns)
        self.controls = OperatorDrive(self.core)
        self.controls.observe_path(True, self.path.last_walk_generation)
        self.gateway = SimpleNamespace(stop_capable=True, state={"instance_id":"bench"},
                                       stop_publisher=SimpleNamespace(get_subscription_count=lambda: 1))
        self.stops, self.edges, self.outputs, self.releases = [], [], [], []
        self.owners = {"gemini_walk":"active", "trick_leg_kick":"inactive"}
        async def read_owners():
            return dict(self.owners)
        self.runtime = SimpleNamespace(robot=SimpleNamespace(gateway_stop_anchor=lambda: self.gateway.state,
                                                             stops=0, fallen=None, motion_block_reason=lambda: None,
                                                             controller_states=read_owners, leg_controllers=tuple(self.owners),
                                                             current=None, commands=SimpleNamespace(empty=lambda: True)),
                                       stop_reason=None, stop=self.stops.append, handoff=SimpleNamespace(active=None),
                                       velocity_sink=self.outputs.append)
        self.path.balance = self.gate
        self.path.sync_balance = lambda: runner.sync_match_balance(self.gate, self.core, self.path, self.runtime,
                                                                   self.controls, self.edges.append)
        self.seq, self.incident, self.status = 0, 0, 1
        self.advance(3)

    def clock_ns(self):
        return round(self.now[0] * 1e9)

    def advance(self, count=1, **edits):
        for _ in range(count):
            self.now[0] += .015
            self.ros = self.now[0]
            self.seq += 2
            n = self.clock_ns()
            self.path.native["sampled_mono_ns"] = n
            self.gate.observe_owners(self.gate.hold_id, n, self.owners, self.runtime.robot.leg_controllers)
            row = [self.seq, imu.MAGIC, os.getpid(), 1_000_000_000, 1, 0, self.incident, self.status,
                   n, n-1_000_000, n-1_000_000, 0, 0, self.seq%256, self.seq%256, 5, 2,
                   n-1_000_000, n-1_000_000, 0, 0, 3, 3, 0]
            for key, value in edits.items():
                row[int(key[1:])] = value
            self.file.write_bytes(imu.RECORD.pack(*row))
            self.path.sync_balance()

    def packet(self, action, **fields):
        # Issue on the robot clock, leaving the full normal ticket expiry intact.
        self.controls.next_issue = self.now[0]
        offer = self.controls.offer(self.now[0])
        return dict(command=action, ticket=offer["ticket"], robot_id=self.core.robot_id, **fields)

    def action(self, action, **fields):
        return runner.operator_action(self.packet(action, **fields), self.core, self.path, self.runtime,
                                      self.gateway, self.controls, balance=self.gate)

    def ready(self):
        self.status, self.incident = 2, self.incident + 1
        self.advance()
        assert not self.core.executor.armed
        self.status = 3
        self.advance(3)
        assert self.gate.state == "ready_to_resume"

    def resume(self, release=None, **edits):
        async def accept(generation, incident):
            self.releases.append((generation, incident))
        fields = dict(epoch="new", previous_epoch="game", hold_id=self.gate.hold_id,
                      board_session=self.gate.session, generation=self.gate.generation)
        fields.update(edits)
        return asyncio.run(runner.imu_resume_action(self.packet("imu_resume", **fields), self.core,
                           self.path, self.runtime, self.gateway, self.controls, self.gate, release or accept))


@pytest.fixture
def trial(tmp_path):
    t = Trial(tmp_path)
    try:
        yield t
    finally:
        t.reader.close()


@pytest.mark.parametrize("manual", [False, True])
def test_loss_clears_authority_and_explicit_resume_uses_new_epoch(trial, manual):
    t = trial
    if manual:
        t.controls.mode = "teleop"
    assert t.gate.state == "healthy" and t.action("arm", epoch="game")["accepted"]
    t.controls.pending = object()
    t.ready()
    assert t.controls.pending is None and t.controls.match_epoch is None
    assert not any((t.outputs[-1].vx, t.outputs[-1].vy, t.outputs[-1].wz)) and not t.stops
    assert any(edge["imu"]["state"] == "paused" for edge in t.edges)
    assert t.action("imu_pause", epoch="game")["hold_id"] == t.gate.hold_id
    with pytest.raises(ValueError):
        t.action("arm", epoch="bypass")
    for wrong in (dict(previous_epoch="foreign"), dict(hold_id="earlier"), dict(generation=2),
                  dict(board_session="other"), dict(epoch="game")):
        with pytest.raises(ValueError):
            t.resume(**wrong)
    assert not t.releases
    result = t.resume()
    assert result["state"] == "armed" and result["imu"]["epoch"] == "new" and t.releases == [(1, 1)]
    assert t.core.executor.armed != manual and t.core.executor.epoch == "new"
    if manual:
        assert t.controls.match_epoch == "new" and t.controls.needs_neutral
    previous = dict(event="intent", epoch="game", clock=t.core.clock_name, producer_id=t.core.producer_id,
                    robot_id=t.core.robot_id, seq=90, kind="attack", intent_id="old",
                    issued_at=t.now[0], valid_until=t.now[0]+.1)
    assert not t.core.ingest(previous)["accepted"]


def test_missed_native_incident_and_stale_components_always_pause(trial):
    t = trial
    t.action("arm", epoch="game")
    t.incident, t.status = 1, 3  # Did not receive an intermediate backup packet.
    t.advance()
    assert t.gate.hold_id and not t.core.executor.armed
    for _ in range(12):
        # Even freshly written board and ROS heartbeats cannot replace a stale gyro receipt.
        t.advance(w10=t.clock_ns()-150_000_000, w18=t.clock_ns()-150_000_000)
        assert t.gate.state == "paused"
    t.advance(3)
    assert t.gate.state == "ready_to_resume"


def test_native_rejection_or_fault_during_reply_never_rearms(trial):
    t = trial
    t.action("arm", epoch="game")
    t.ready()
    async def reject(*unused):
        raise ValueError("native veto")
    with pytest.raises(ValueError, match="native veto"):
        t.resume(reject)
    assert not t.core.executor.armed
    async def interrupted(*unused):
        t.incident += 1
        t.advance()
    with pytest.raises(ValueError):
        t.resume(interrupted)
    assert not t.core.executor.armed
    t.advance(3)
    t.advance(w5=1, w7=4)
    assert t.gate.state == "home_required"
    t.advance(5)
    assert t.gate.state == "home_required" and not t.core.executor.armed


def test_interrupted_clip_requests_existing_hard_stop(trial):
    t = trial
    t.action("arm", epoch="game")
    t.runtime.handoff.active = object()
    t.incident, t.status = 1, 2
    t.advance()
    assert len(t.stops) == 1 and not t.core.executor.armed


def test_stop_generation_during_native_ack_blocks_resume(trial):
    t = trial
    t.action("arm", epoch="game")
    t.ready()
    async def stopped(*unused):
        t.runtime.robot.stops += 1
    with pytest.raises(ValueError):
        t.resume(stopped)
    assert t.gate.state == "paused" and not t.core.executor.armed


def test_deactivated_walker_cannot_become_recovery_ready_again_in_same_match(trial):
    t = trial
    t.action("arm", epoch="game")
    t.ready()
    t.owners["gemini_walk"] = "inactive"
    t.advance()
    assert t.gate.state == "paused" and t.gate.walker_lost
    t.owners["gemini_walk"] = "active"
    t.advance(4)
    assert t.gate.state == "paused"
    with pytest.raises(ValueError):
        t.resume()
    assert not t.releases


def test_nonzero_native_command_cannot_outlive_original_imu_deadline(trial):
    t = trial
    t.action("arm", epoch="game")
    t.path.frame_deadline = t.now[0] + .3
    t.path.native["sampled_mono_ns"] = t.clock_ns()
    t.ros = t.now[0] - .08
    t.path.write(Command(.1,0,0,t.now[0]+.15,t.core.robot_id))
    packet = runner.protocol.decode(t.sent[-1])
    assert packet.vx == .1 and packet.expires_mono_ns <= round(t.ros*1e9) + imu.AGE_NS
    t.now[0] += .025
    t.path.write(Command(.1,0,0,t.now[0]+.15,t.core.robot_id))
    assert runner.protocol.decode(t.sent[-1]).is_zero and not t.core.executor.armed


@pytest.mark.parametrize("supersede", [True, False])
def test_delayed_native_resume_reply_cannot_outlive_new_command_or_ticket(trial, supersede):
    t = trial
    t.action("arm", epoch="game")
    t.ready()
    async def delayed(*unused):
        if supersede:
            t.action("imu_pause", epoch="game")
        else:
            t.advance(18)  # IMU remains healthy, the explicit operator ticket expires.
    with pytest.raises(ValueError, match="expired or changed"):
        t.resume(delayed)
    assert not t.core.executor.armed and t.gate.hold_id


def test_peer_pause_and_ros_delivery_loss_allow_only_explicit_resume(trial):
    t = trial
    t.action("arm", epoch="game")
    t.ros -= .15
    t.path.sync_balance()
    assert t.gate.state == "paused" and not t.core.executor.armed
    t.advance(4)
    result = t.resume()
    assert result["imu"]["state"] == "healthy" and not t.releases
    assert t.action("imu_pause", epoch="new")["state"] == "paused"
    t.advance(4)
    assert t.gate.state == "ready_to_resume" and not t.core.executor.armed


@pytest.mark.parametrize("change", [dict(w0=7), dict(w2=100), dict(w22=4), dict(w9=1,w17=1),
                                    dict(w12=150_000_000,w20=150_000_000), dict(w9=20_000_000_000,w17=20_000_000_000)])
def test_malformed_or_expired_native_evidence_withholds_authority(trial, change):
    trial.advance(**change)
    assert trial.gate.state == "unknown"
    with pytest.raises(ValueError):
        trial.action("arm", epoch="game")


def test_preexisting_native_hold_needs_an_explicit_resume(trial):
    t = trial
    t.status, t.incident = 3, 1
    t.advance(3)
    reply = t.action("arm", epoch="game")
    assert reply["state"] == "paused" and not t.core.executor.armed and not t.releases
    t.advance(3)
    assert t.resume()["state"] == "armed" and t.releases == [(1, 1)]
