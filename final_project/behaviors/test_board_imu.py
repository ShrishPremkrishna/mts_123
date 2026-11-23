"""Strict board wire format and the runner's match handlers with deterministic IMU inputs; no hardware."""
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
        self.reader = imu.BoardReceipt(self.file, os.getpid(), "strict")
        self.ros = self.now[0]
        self.gate = imu.MatchImu(self.reader, lambda: (self.ros, 0., 0.), clock_ns=self.clock_ns)
        self.controls = OperatorDrive(self.core)
        self.controls.observe_path(True, self.path.last_walk_generation)
        self.gateway = SimpleNamespace(stop_capable=True, state={"instance_id": "bench"},
                                       stop_publisher=SimpleNamespace(get_subscription_count=lambda: 1))
        self.stops, self.edges, self.outputs = [], [], []
        self.owners = {"gemini_walk": "active", "trick_leg_kick": "inactive"}
        self.during_walker_check = None

        async def read_owners():
            if self.during_walker_check is not None:
                self.during_walker_check()
            return dict(self.owners)
        self.robot = SimpleNamespace(gateway_stop_anchor=lambda: self.gateway.state, stops=0, fallen=None,
                                     motion_block_reason=lambda: None, controller_states=read_owners,
                                     leg_controllers=tuple(self.owners), current=None,
                                     commands=SimpleNamespace(empty=lambda: True))
        self.runtime = SimpleNamespace(robot=self.robot, stop_reason=None, handoff=SimpleNamespace(active=None),
                                       velocity_sink=self.outputs.append)
        self.runtime.stop = lambda reason: (self.stops.append(reason), setattr(self.runtime, "stop_reason", reason))
        self.path.balance = self.gate
        self.path.sync_balance = lambda: runner.sync_match_balance(self.gate, self.core, self.path, self.runtime,
                                                                   self.controls, self.edges.append)
        self.seq, self.status = 0, imu.HEALTHY
        self.advance(3)

    def clock_ns(self):
        return round(self.now[0] * 1e9)

    def write(self, **edits):
        self.seq += 2
        n = self.clock_ns()
        row = [self.seq, imu.MAGIC, os.getpid(), 1_000_000_000, 1, 0, 0, self.status,
               n, n-1_000_000, n-1_000_000, 0, 0, self.seq % 256, self.seq % 256, 5, 2,
               n-1_000_000, n-1_000_000, 0, 0, 3, 1, 0]
        for key, value in edits.items():
            row[int(key[1:])] = value
        self.file.write_bytes(imu.RECORD.pack(*row))

    def advance(self, count=1, **edits):
        """15 ms of board, ROS and native feedback, then one runner refresh."""
        for _ in range(count):
            self.now[0] += .015
            self.ros = self.now[0]
            n = self.clock_ns()
            self.path.native["sampled_mono_ns"] = n
            self.gate.observe_owners(self.gate.hold_id, n, self.owners, self.robot.leg_controllers)
            self.write(**edits)
            self.path.sync_balance()

    def packet(self, action, **fields):
        # Issue on the robot clock, leaving the full normal ticket expiry intact.
        self.controls.next_issue = self.now[0]
        offer = self.controls.offer(self.now[0])
        return dict(command=action, ticket=offer["ticket"], robot_id=self.core.robot_id, **fields)

    def action(self, action, **fields):
        return runner.operator_action(self.packet(action, **fields), self.core, self.path, self.runtime,
                                      self.gateway, self.controls, balance=self.gate)

    def lose_ros_delivery(self):
        self.ros -= .15
        self.path.sync_balance()

    def ready(self):
        self.lose_ros_delivery()
        assert self.gate.state == "paused" and not self.core.executor.armed
        self.advance(3)
        assert self.gate.state == "ready_to_resume"

    def resume(self, **edits):
        fields = dict(epoch="new", previous_epoch="game", hold_id=self.gate.hold_id,
                      board_session=self.gate.session, generation=self.gate.generation)
        fields.update(edits)
        return asyncio.run(runner.imu_resume_action(self.packet("imu_resume", **fields), self.core,
                           self.path, self.runtime, self.gateway, self.controls, self.gate))


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
    assert t.core.executor.armed != manual
    t.controls.pending = object()
    t.ready()
    assert t.controls.pending is None and t.controls.match_epoch is None
    assert not any((t.outputs[-1].vx, t.outputs[-1].vy, t.outputs[-1].wz)) and not t.stops
    assert any(edge["imu"]["state"] == "paused" for edge in t.edges)
    assert t.action("imu_pause", epoch="game")["hold_id"] == t.gate.hold_id
    with pytest.raises(ValueError):
        t.action("arm", epoch="bypass")
    for wrong in (dict(previous_epoch="foreign"), dict(hold_id="earlier"), dict(generation=2),
                  dict(board_session="other"), dict(epoch="game"), dict(pitch={"length": 3, "width": 2})):
        with pytest.raises(ValueError):
            t.resume(**wrong)
    assert not t.core.executor.armed
    result = t.resume()
    assert result["state"] == "armed" and result["imu"] == {**result["imu"], "epoch": "new", "state": "healthy"}
    assert t.core.executor.armed != manual and t.core.executor.epoch == "new"
    if manual:
        assert t.controls.match_epoch == "new" and t.controls.needs_neutral
    previous = dict(event="intent", epoch="game", clock=t.core.clock_name, producer_id=t.core.producer_id,
                    robot_id=t.core.robot_id, seq=90, kind="attack", intent_id="old",
                    issued_at=t.now[0], valid_until=t.now[0]+.1)
    assert not t.core.ingest(previous)["accepted"]


def test_stale_sensor_receipt_pauses_despite_fresh_board_and_ros_heartbeats(trial):
    t = trial
    t.action("arm", epoch="game")
    for _ in range(12):
        t.advance(w10=t.clock_ns()-150_000_000, w18=t.clock_ns()-150_000_000)
        assert t.gate.state == "paused" and not t.core.executor.armed
    t.advance(3)
    assert t.gate.state == "ready_to_resume"


@pytest.mark.parametrize("change", [dict(w7=4, w5=1), dict(w5=1), dict(w4=2), dict(w7=5), dict(w7=0)])
def test_fault_deactivation_or_new_home_needs_supported_home(trial, change):
    t = trial
    t.action("arm", epoch="game")
    t.advance(**change)
    assert t.gate.state == "home_required" and not t.core.executor.armed
    t.advance(5)
    assert t.gate.state == "home_required"
    with pytest.raises(ValueError):
        t.resume()


def test_interrupted_clip_requests_existing_hard_stop(trial):
    t = trial
    t.action("arm", epoch="game")
    t.runtime.handoff.active = object()
    t.lose_ros_delivery()
    assert len(t.stops) == 1 and not t.core.executor.armed


def test_stop_generation_during_walker_check_blocks_resume(trial):
    t = trial
    t.action("arm", epoch="game")
    t.ready()
    t.during_walker_check = lambda: setattr(t.robot, "stops", t.robot.stops + 1)
    with pytest.raises(ValueError):
        t.resume()
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


def test_nonzero_native_command_cannot_outlive_original_imu_deadline(trial):
    t = trial
    t.action("arm", epoch="game")
    t.path.frame_deadline = t.now[0] + .3
    t.path.native["sampled_mono_ns"] = t.clock_ns()
    t.ros = t.now[0] - .08
    t.path.write(Command(.1, 0, 0, t.now[0]+.15, t.core.robot_id))
    packet = runner.protocol.decode(t.sent[-1])
    assert packet.vx == .1 and packet.expires_mono_ns <= round(t.ros*1e9) + imu.AGE_NS
    t.now[0] += .025
    t.path.write(Command(.1, 0, 0, t.now[0]+.15, t.core.robot_id))
    assert runner.protocol.decode(t.sent[-1]).is_zero and not t.core.executor.armed


@pytest.mark.parametrize("supersede", [True, False])
def test_walker_check_cannot_outlive_its_ticket_or_a_newer_request(trial, supersede):
    t = trial
    t.action("arm", epoch="game")
    t.ready()
    if supersede:
        t.during_walker_check = lambda: t.action("imu_pause", epoch="game")
    else:
        t.during_walker_check = lambda: t.advance(18)  # IMU stays healthy; the ticket expires.
    with pytest.raises(ValueError, match="expired or changed"):
        t.resume()
    assert not t.core.executor.armed and t.gate.hold_id


def test_operator_pause_and_ros_delivery_loss_allow_only_explicit_resume(trial):
    t = trial
    t.action("arm", epoch="game")
    t.lose_ros_delivery()
    assert t.gate.state == "paused" and not t.core.executor.armed
    t.advance(4)
    assert t.resume()["imu"]["state"] == "healthy"
    assert t.action("imu_pause", epoch="new")["state"] == "paused"
    t.advance(4)
    assert t.gate.state == "ready_to_resume" and not t.core.executor.armed


@pytest.mark.parametrize("change", [dict(w0=7), dict(w1=1), dict(w2=100), dict(w22=2), dict(w22=3), dict(w23=1),
                                    dict(w7=2), dict(w7=3), dict(w7=6), dict(w15=8, w16=7), dict(w9=1, w17=1),
                                    dict(w12=150_000_000, w20=150_000_000),
                                    dict(w9=20_000_000_000, w17=20_000_000_000)])
def test_malformed_unselected_or_expired_evidence_withholds_authority(trial, change):
    trial.advance(**change)
    assert trial.gate.state == "unknown"
    with pytest.raises(ValueError):
        trial.action("arm", epoch="game")


def test_missing_receipt_or_unselected_profile_is_never_ready(tmp_path):
    gate = imu.MatchImu(imu.BoardReceipt(tmp_path / "absent.bin", os.getpid()), lambda: (1., 0., 0.))
    assert gate.refresh()["state"] == "unknown"
    with pytest.raises(ValueError, match="strict"):
        imu.BoardReceipt(tmp_path / "absent.bin", os.getpid(), "field_redundant_imu")


def test_health_requires_three_advancing_pairs_before_first_arm(tmp_path):
    t = Trial.__new__(Trial)
    Trial.__init__(t, tmp_path)
    t.reader.close()
    fresh = imu.MatchImu(imu.BoardReceipt(t.file, os.getpid()), lambda: (t.ros, 0., 0.), clock_ns=t.clock_ns)
    for expected in ("unknown", "unknown", "healthy"):
        t.now[0] += .015
        t.ros = t.now[0]
        t.write()
        assert fresh.refresh()["state"] == expected
    fresh.reader.close()


def test_operator_arm_requires_current_ticket_in_either_mode(trial):
    t = trial
    c = t.controls
    arm = {"command": "arm", "epoch": "match", "robot_id": t.core.robot_id}
    with pytest.raises(ValueError, match="expired"):
        runner.operator_action(arm, t.core, t.path, t.runtime, t.gateway, c, balance=t.gate)
    packet = {**arm, "ticket": c.offer(t.now[0])["ticket"]}
    assert runner.operator_action(packet, t.core, t.path, t.runtime, t.gateway, c, balance=t.gate)["accepted"]
    with pytest.raises(ValueError, match="consumed"):
        runner.operator_action(packet, t.core, t.path, t.runtime, t.gateway, c, balance=t.gate)
    assert t.action("pause")["state"] == "standby"
    with pytest.raises(ValueError, match="advancing"):
        t.action("arm", epoch="too-soon")  # Ending a match discards its accumulated evidence.
    t.advance(3)
    t.path.guard = {"state": "ready"}
    assert t.action("mode", mode="teleop")["mode"] == "teleop"
    with pytest.raises(ValueError, match="kickoff"):
        t.action("drive", epoch="match", forward=1, left=0, turn=0)
    reply = t.action("arm", epoch="keyboard-game", mode="teleop")
    assert reply["accepted"] and c.match_epoch == "keyboard-game" and not t.core.executor.armed
    with pytest.raises(ValueError, match="kickoff"):
        t.action("drive", epoch="old-game", forward=1, left=0, turn=0)
    assert c.command(t.now[0]).vx == 0
    with pytest.raises(ValueError, match="goal reset"):
        t.action("resume")


def test_operator_end_is_standby_when_idle_and_hard_stop_during_a_native_job(trial):
    t = trial
    assert t.action("arm", epoch="game")["accepted"]
    assert t.action("pause")["state"] == "standby"
    assert not t.core.executor.armed and t.gate.epoch is None and not t.stops
    assert not any((t.outputs[-1].vx, t.outputs[-1].vy, t.outputs[-1].wz))
    t.advance(3)
    assert t.action("arm", epoch="second")["accepted"]
    t.robot.current = object()
    assert t.action("pause")["state"] == "hard_stop" and t.stops


@pytest.mark.parametrize("mask", [0, 1, 2])
def test_incomplete_spi_reply_revokes_match_authority(tmp_path, mask):
    trial = Trial(tmp_path)
    assert trial.gate.state == "healthy"
    trial.advance(w21=mask)
    assert trial.gate.state != "healthy"
    assert "SPI" in trial.gate.reason
    assert not trial.core.executor.armed
